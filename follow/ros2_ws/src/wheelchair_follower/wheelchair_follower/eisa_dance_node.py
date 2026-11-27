#!/usr/bin/env python3
"""エイサー（ダンス物まね）Gazebo 3Dシミュレーション制御ノード (eisa_dance_node)

機能:
  1. finger_dance コンテナ (pose_detect / pose_dance) から Webカメラやダンス動画の
     33関節骨格推定データ (/eisa_state または /dance_pose) をリアルタイム受信
     （UDP 5006番ポート ＆ rosbridge WebSocket 9091番ポートの両対応）
  2. エイサーの4動作（優先順位順）を Gazebo 3D空間の「人モデル (person)」と
     「電動車いす (wheelchair)」の両方で再現:
       (1) しこ立ち     : 後ろへ下がり、立ち上がると元の位置 (0.0m) へ戻る
       (2) 足上げ       : 上げた足の外側へ旋回し、足を下ろすと正面 (0.0°) へ戻る
       (3) ステップ     : 右足ステップで前進→復帰、左足ステップで後退→復帰
       (4) 手上げ       : 両手上げで前進キープ、片手上げで左右旋回キープ
  3. オドメトリ (/odom, /person/odom) を用いた閉ループ位置・角度制御により、
     何回ステップや旋回を繰り返しても初期位置・正面方向からドリフトしない
  4. カメラ・動画未接続時は自動でエイサー4動作のデモンストレーションを再生し、
     カメラ・動画が接続された瞬間にリアルタイム連動モードへ自動切替する
"""
import base64
import collections
import hashlib
import json
import math
import os
import socket
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from wheelchair_follower.pose_mapping import DanceMapper


def _get_default_gateway():
    try:
        with open('/proc/net/route', 'r') as f:
            for line in f:
                fields = line.strip().split()
                if len(fields) >= 3 and fields[1] == '00000000':
                    ip_hex = fields[2]
                    return socket.inet_ntoa(bytes.fromhex(ip_hex)[::-1])
    except Exception:
        pass
    return None


def _ws_send_text(sock, text):
    """RFC 6455 クライアント→サーバ マスク付きテキストフレーム送信"""
    payload = text.encode('utf-8')
    length = len(payload)
    header = bytearray([0x81])  # FIN=1, opcode=1 (text)
    mask_key = os.urandom(4)
    if length < 126:
        header.append(0x80 | length)
    elif length < 65536:
        header.append(0x80 | 126)
        header.extend(struct.pack('!H', length))
    else:
        header.append(0x80 | 127)
        header.extend(struct.pack('!Q', length))
    header.extend(mask_key)
    masked = bytearray(length)
    for i in range(length):
        masked[i] = payload[i] ^ mask_key[i & 3]
    sock.sendall(header + masked)


def _ws_recv_frame(sock):
    """RFC 6455 フレーム1つを受信して (opcode, payload_bytes) を返す"""
    def recvn(n):
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    b12 = recvn(2)
    if not b12:
        return None, None
    opcode = b12[0] & 0x0F
    masked = (b12[1] & 0x80) != 0
    length = b12[1] & 0x7F
    if length == 126:
        ext = recvn(2)
        if not ext:
            return None, None
        length = struct.unpack('!H', ext)[0]
    elif length == 127:
        ext = recvn(8)
        if not ext:
            return None, None
        length = struct.unpack('!Q', ext)[0]
    mask_key = recvn(4) if masked else None
    payload = recvn(length) if length > 0 else b''
    if payload is None:
        return None, None
    if masked and mask_key:
        payload = bytes(b ^ mask_key[i & 3] for i, b in enumerate(payload))
    return opcode, payload


# カメラ待機中に自動再生するエイサー基本演舞シーケンス
# (継続時間[s], アクション名, 目標向き[deg], 目標前後[m(WHILL基準)])
EISA_DEMO_SEQUENCE = [
    (1.2, '構え（待機）', 0.0, 0.0),
    (1.2, '1. しこ立ち（後ろへ0.12m下がる）', 0.0, -0.12),
    (1.2, '1. しこ立ち復帰（立ち上がる → 元の位置）', 0.0, 0.0),
    (1.2, '2. 右足上げ（右斜め前へ前進）', -15.0, 0.12),
    (1.0, '2. 右足上げ復帰（元の位置へ戻る）', 0.0, 0.0),
    (1.2, '2. 左足上げ（左斜め前へ前進）', 15.0, 0.12),
    (1.0, '2. 左足上げ復帰（元の位置へ戻る）', 0.0, 0.0),
    (0.8, '3. 右足ステップ（前へ0.12m進む）', 0.0, 0.12),
    (0.9, '3. 右足ステップ復帰（元の位置へ戻る）', 0.0, 0.0),
    (0.8, '3. 左足ステップ（後ろへ0.12m下がる）', 0.0, -0.12),
    (0.9, '3. 左足ステップ復帰（元の位置へ戻る）', 0.0, 0.0),
    (1.2, '4. 左turn（左へ1回転）', 360.0, 0.0),
    (1.2, '4. 右turn（右へ1回転）', -360.0, 0.0),
]


class OdomPoseTracker:
    """Gazeboの真値オドメトリから初期位置(x0, y0, yaw0)を基準とした前後距離・横ずれ・相対角度を計算"""

    def __init__(self, kp_lin=3.5, kp_ang=8.0, max_lin=0.80, max_ang=5.5):
        self.kp_lin = kp_lin
        self.kp_ang = kp_ang
        self.max_lin = max_lin
        self.max_ang = max_ang
        self.x0 = None
        self.y0 = None
        self.yaw0 = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_rel_yaw = 0.0
        self.unwrapped_rel_yaw = 0.0
        self.ready = False
        self.has_ground_truth = False

    def update_ground_truth(self, msg: Odometry):
        self.has_ground_truth = True
        self._apply_odom(msg)

    def update_odom(self, msg: Odometry):
        if not self.has_ground_truth:
            self._apply_odom(msg)

    def _apply_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        if self.x0 is None:
            self.x0 = p.x
            self.y0 = p.y
            self.yaw0 = yaw
            self.prev_rel_yaw = 0.0
            self.unwrapped_rel_yaw = 0.0
        self.x = p.x
        self.y = p.y
        self.yaw = yaw
        rel_yaw = math.atan2(math.sin(self.yaw - self.yaw0), math.cos(self.yaw - self.yaw0))
        d_yaw = math.atan2(math.sin(rel_yaw - self.prev_rel_yaw), math.cos(rel_yaw - self.prev_rel_yaw))
        self.unwrapped_rel_yaw += d_yaw
        self.prev_rel_yaw = rel_yaw
        self.ready = True

    def relative_pose(self):
        if not self.ready:
            return 0.0, 0.0, 0.0
        dx = self.x - self.x0
        dy = self.y - self.y0
        fwd = dx * math.cos(self.yaw0) + dy * math.sin(self.yaw0)
        lat = -dx * math.sin(self.yaw0) + dy * math.cos(self.yaw0)
        rel_yaw = math.atan2(math.sin(self.yaw - self.yaw0), math.cos(self.yaw - self.yaw0))
        return fwd, lat, rel_yaw

    def is_at_home(self):
        if not self.ready:
            return True
        fwd, lat, rel_yaw = self.relative_pose()
        return abs(fwd) < 0.015 and abs(lat) < 0.02 and abs(rel_yaw) < math.radians(1.0)

    def compute_cmd(self, target_fwd_m, target_heading_deg):
        cmd = Twist()
        if not self.ready:
            return cmd, 0.0, 0.0
        fwd, lat, rel_yaw = self.relative_pose()
        target_yaw_rad = math.radians(target_heading_deg)

        # 1回転ターンモード（target_heading_deg = ±360°）のときは累積角度で1回転し、
        # 通常モード（|target_heading_deg| <= 90°）に戻ったら正面（[-π, +π]）に正規化する
        if abs(target_heading_deg) >= 180.0:
            err_yaw = target_yaw_rad - self.unwrapped_rel_yaw
            v = 0.0
            w = self.kp_ang * err_yaw if abs(err_yaw) > math.radians(1.5) else 0.0
            w = max(-self.max_ang, min(self.max_ang, w))
            cmd.linear.x = 0.0
            cmd.angular.z = float(w)
            return cmd, fwd, math.degrees(self.unwrapped_rel_yaw)
        else:
            self.unwrapped_rel_yaw = rel_yaw

        err_fwd = target_fwd_m - fwd

        # 片手上げ（斜め前への移動：target_fwd_m > 0 かつ target_heading_deg != 0）のときは
        # 目標角度 (target_yaw_rad) へ向けながら同時に前進して斜め前へ進む
        if abs(target_fwd_m) > 0.01 and abs(target_heading_deg) > 1.0:
            err_yaw = math.atan2(math.sin(target_yaw_rad - rel_yaw), math.cos(target_yaw_rad - rel_yaw))
            v = self.kp_lin * err_fwd if abs(err_fwd) > 0.012 else 0.0
            w = self.kp_ang * err_yaw if abs(err_yaw) > math.radians(0.8) else 0.0
        elif abs(err_fwd) > 0.018:
            # 横ずれ (lat) がある場合は初期ライン (lat=0) へ戻る角度補正を加える
            move_dir = 1.0 if err_fwd >= 0.0 else -1.0
            steer_yaw = max(-0.25, min(0.25, -1.2 * lat * move_dir))
            align_err = math.atan2(math.sin(steer_yaw - rel_yaw), math.cos(steer_yaw - rel_yaw))

            if abs(rel_yaw) > math.radians(7.0):
                # まだ斜めを向いている場合はその場で正面へ向き直る（横ドリフト防止）
                v = 0.0
                w = self.kp_ang * align_err
            else:
                v = self.kp_lin * err_fwd
                w = self.kp_ang * align_err if abs(align_err) > math.radians(0.5) else 0.0
        else:
            v = 0.0
            err_yaw = math.atan2(math.sin(target_yaw_rad - rel_yaw), math.cos(target_yaw_rad - rel_yaw))
            w = self.kp_ang * err_yaw if abs(err_yaw) > math.radians(0.8) else 0.0

        v = max(-self.max_lin, min(self.max_lin, v))
        w = max(-self.max_ang, min(self.max_ang, w))

        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        return cmd, fwd, math.degrees(rel_yaw)


class EisaDanceNode(Node):

    def __init__(self):
        super().__init__('eisa_dance')

        self.delay = self.declare_parameter('delay', 0.0).value               # 遅れ[s] (0.0ならカメラと同時に車いすが踊る)
        self.dist_scale = self.declare_parameter('dist_scale', 2.25).value    # Gazebo空間で見やすくする前後倍率 (0.24m * 2.25 = 0.54m)
        self.angle_scale = self.declare_parameter('angle_scale', 1.0).value   # Gazebo空間で見やすくする角度倍率 (30° * 1.0 = 30°, 360° * 1.0 = 360°)
        self.auto_demo = self.declare_parameter('auto_demo', True).value      # カメラ未接続時にエイサーデモを自動再生するか
        self.move_person = self.declare_parameter('move_person', False).value # Gazebo内の3D人物モデルは描画・制御しない（軽量化）

        self.chair_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.person_pub = self.create_publisher(Twist, '/person/cmd_vel', 10)

        self.chair_tracker = OdomPoseTracker(kp_lin=3.5, kp_ang=8.0, max_lin=0.80, max_ang=5.5)
        self.person_tracker = OdomPoseTracker(kp_lin=4.0, kp_ang=8.0, max_lin=1.00, max_ang=5.5)

        self.create_subscription(Odometry, '/wheelchair/ground_truth', self.chair_tracker.update_ground_truth, 10)
        self.create_subscription(Odometry, '/odom', self.chair_tracker.update_odom, 10)
        self.create_subscription(Odometry, '/person/odom', self.person_tracker.update_odom, 10)

        # ローカル骨格マッパー (/dance_pose 直接受信時用)
        self.mapper = DanceMapper()
        self.last_pose_ts = None

        # ライブ受信状態
        self.live_lock = threading.Lock()
        self.last_live_time = 0.0
        self.last_eisa_state_time = 0.0
        self.live_ok = False
        self.live_action = '待機中'
        self.live_heading_deg = 0.0
        self.live_forward_m = 0.0
        self.live_source = None

        # 遅延再生用バッファ: deque of (wall_time, heading_deg, forward_m, action)
        self.history = collections.deque(maxlen=600)

        # デモ再生状態
        self.demo_idx = 0
        self.demo_step_start = time.monotonic()
        self.last_logged_action = None
        self.last_status_print = 0.0

        self.running = True
        self.udp_thread = threading.Thread(target=self._udp_listener_loop, daemon=True)
        self.udp_thread.start()
        self.ws_thread = threading.Thread(target=self._rosbridge_listener_loop, daemon=True)
        self.ws_thread.start()

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('================================================================')
        self.get_logger().info('🌺【チバリヨカー・エイサー 3Dシミュレーション起動】🌺')
        self.get_logger().info(f'・物まね遅延 (delay): {self.delay:.2f} 秒 | Gazebo表示倍率: 前後×{self.dist_scale:.1f}, 角度×{self.angle_scale:.1f}')
        self.get_logger().info('・Webカメラ (http://localhost:6081/camera.html?port=9091) または')
        self.get_logger().info('  finger_dance の pose_dance と自動接続してリアルタイムに踊ります！')
        if self.auto_demo:
            self.get_logger().info('・カメラ映像が来るまでの間は、Gazebo内でエイサー4動作デモを自動再生します')
        self.get_logger().info('================================================================')

    def _update_live(self, ok, action, heading_deg, forward_m, source):
        now = time.monotonic()
        with self.live_lock:
            if self.live_source is None or (now - self.last_live_time > 3.0):
                self.live_source = source
                self.get_logger().info(f'🎥【リアルタイム連動開始】接続元: {source}')
            self.last_live_time = now
            self.live_ok = bool(ok)
            self.live_action = action
            self.live_heading_deg = float(heading_deg)
            self.live_forward_m = float(forward_m)

    def _udp_listener_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('0.0.0.0', 5006))
            sock.settimeout(0.5)
        except Exception as e:
            self.get_logger().warn(f'UDP 5006 バインド失敗: {e}')
            return

        while self.running and rclpy.ok():
            try:
                data, _ = sock.recvfrom(4096)
                for line in data.decode('utf-8', errors='ignore').splitlines():
                    line = line.strip()
                    if not line:
                        continue
                    d = json.loads(line)
                    self.last_eisa_state_time = time.monotonic()
                    self._update_live(
                        d.get('ok', False),
                        d.get('action', '踊っている'),
                        d.get('heading_deg', 0.0),
                        d.get('forward_m', 0.0),
                        'UDP:5006 (pose_dance)'
                    )
            except socket.timeout:
                continue
            except Exception:
                continue
        sock.close()

    def _rosbridge_listener_loop(self):
        # コンテナの名前・Docker の外（host.docker.internal）・ゲートウェイ・自分の順に探す。
        # これで届かないときは、環境変数 CHIBARIYO_EXTRA_HOSTS に住所をカンマ区切りで渡す（リポジトリには書かない）
        hosts = ['finger_dance', 'host.docker.internal', '127.0.0.1']
        hosts[2:2] = [h for h in os.environ.get('CHIBARIYO_EXTRA_HOSTS', '').split(',') if h.strip()]
        gw = _get_default_gateway()
        if gw and gw not in hosts:
            hosts.insert(1, gw)

        while self.running and rclpy.ok():
            sock = None
            connected_host = None
            for h in hosts:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(1.0)
                    s.connect((h, 9091))
                    key = base64.b64encode(os.urandom(16)).decode('ascii')
                    req = (
                        f"GET / HTTP/1.1\r\n"
                        f"Host: {h}:9091\r\n"
                        f"Upgrade: websocket\r\n"
                        f"Connection: Upgrade\r\n"
                        f"Sec-WebSocket-Key: {key}\r\n"
                        f"Sec-WebSocket-Version: 13\r\n\r\n"
                    )
                    s.sendall(req.encode('ascii'))
                    resp = s.recv(1024).decode('latin1', errors='ignore')
                    if '101' in resp:
                        s.settimeout(1.0)
                        sock = s
                        connected_host = h
                        break
                    s.close()
                except Exception:
                    try:
                        s.close()
                    except Exception:
                        pass

            if sock is None:
                time.sleep(1.5)
                continue

            self.get_logger().info(f'✅ finger_dance rosbridge ({connected_host}:9091) に接続完了！カメラ入力を待機中...')
            try:
                _ws_send_text(sock, json.dumps({
                    'op': 'subscribe',
                    'topic': '/eisa_state',
                    'type': 'std_msgs/msg/String'
                }))
                _ws_send_text(sock, json.dumps({
                    'op': 'subscribe',
                    'topic': '/dance_pose',
                    'type': 'std_msgs/msg/String'
                }))
                while self.running and rclpy.ok():
                    try:
                        opcode, payload = _ws_recv_frame(sock)
                    except socket.timeout:
                        continue
                    if opcode is None or opcode == 0x8:  # 切断
                        break
                    if opcode == 0x9:  # Ping -> Pong
                        continue
                    if opcode != 0x1 or not payload:
                        continue
                    msg_obj = json.loads(payload.decode('utf-8', errors='ignore'))
                    topic = msg_obj.get('topic')
                    raw_data = msg_obj.get('msg', {}).get('data')
                    if not raw_data:
                        continue
                    d = json.loads(raw_data)
                    now = time.monotonic()
                    if topic == '/eisa_state':
                        self.last_eisa_state_time = now
                        self._update_live(
                            d.get('ok', False),
                            d.get('action', '踊っている'),
                            d.get('heading_deg', 0.0),
                            d.get('forward_m', 0.0),
                            f'rosbridge ({connected_host}:9091 /eisa_state)'
                        )
                    elif topic == '/dance_pose':
                        # /eisa_state (pose_dance) が動いていないときでも /dance_pose だけでエイサー判定を行う
                        if now - self.last_eisa_state_time < 0.5:
                            continue
                        dt = 0.1 if self.last_pose_ts is None else min(0.3, max(0.01, now - self.last_pose_ts))
                        self.last_pose_ts = now
                        if not d.get('ok'):
                            self._update_live(False, '体が見えない：停止', 0.0, 0.0, f'rosbridge ({connected_host}:9091 /dance_pose)')
                        else:
                            heading, forward = self.mapper.update(d, dt)
                            action = getattr(self.mapper, 'action_name', '踊っている')
                            self._update_live(True, action, heading, forward, f'rosbridge ({connected_host}:9091 /dance_pose)')
            except Exception:
                pass
            finally:
                try:
                    sock.close()
                except Exception:
                    pass
                time.sleep(1.0)

    def _get_current_eisa_command(self, now):
        """現在の (active, action_name, scaled_heading_deg, scaled_forward_m, is_demo) を返す"""
        with self.live_lock:
            live_age = now - self.last_live_time
            live_ok = self.live_ok
            live_action = self.live_action
            live_h = self.live_heading_deg
            live_f = self.live_forward_m

        # カメラ・動画から直近 1.2 秒以内にデータが届いている場合 → ライブ連動モード
        if live_age < 1.2:
            if not live_ok:
                return False, live_action, 0.0, 0.0, False
            if abs(live_h) >= 180.0:
                scaled_h = 360.0 if live_h > 0 else -360.0
            else:
                scaled_h = max(-60.0, min(60.0, live_h * self.angle_scale))
            scaled_f = max(-0.85, min(0.85, live_f * self.dist_scale))
            return True, f'🎥 {live_action}', scaled_h, scaled_f, False

        # カメラ未接続時で auto_demo=True の場合 → エイサー4動作デモを自動再生
        if self.auto_demo:
            dur, act_name, h_deg, f_m = EISA_DEMO_SEQUENCE[self.demo_idx]
            if now - self.demo_step_start >= dur:
                self.demo_idx = (self.demo_idx + 1) % len(EISA_DEMO_SEQUENCE)
                self.demo_step_start = now
                dur, act_name, h_deg, f_m = EISA_DEMO_SEQUENCE[self.demo_idx]
            scaled_h = h_deg * self.angle_scale
            scaled_f = f_m * self.dist_scale
            return True, f'🌺[自動演舞デモ] {act_name}', scaled_h, scaled_f, True

        return False, 'カメラ入力待機中', 0.0, 0.0, False

    def control_loop(self):
        now = time.monotonic()
        active, action_label, target_h, target_f, _ = self._get_current_eisa_command(now)

        if not active:
            self.history.clear()
            if not self.chair_tracker.is_at_home():
                # しゃがみ込み等で一時的にカメラから見切れても、必ず初期位置(0.0m, 0.0°)へ自動で戻す
                c_cmd, cur_f, cur_yaw = self.chair_tracker.compute_cmd(0.0, 0.0)
                self.chair_pub.publish(c_cmd)
                if now - self.last_status_print >= 1.0:
                    self.last_status_print = now
                    self.get_logger().info(
                        f'↩️ 初期位置へ自動復帰中 | 現在[前後:{cur_f:+.2f}m, 向き:{cur_yaw:+.0f}°] '
                        f'-> 速度[v:{c_cmd.linear.x:+.2f}m/s, w:{c_cmd.angular.z:+.2f}rad/s]'
                    )
            else:
                self.chair_pub.publish(Twist())
                if action_label != self.last_logged_action:
                    self.last_logged_action = action_label
                    self.get_logger().info(f'⏸️ {action_label}（初期位置 0.00m / 0° で待機中）')
            if self.move_person:
                self.person_pub.publish(Twist())
            return

        # 履歴に追加 (人の動きはリアルタイム、車いすは delay 秒前の動きを追従)
        self.history.append((now, target_h, target_f, action_label))

        # 1) 3D Person (人モデル): 遅れなしでエイサーを踊る
        if self.move_person:
            p_cmd, _, _ = self.person_tracker.compute_cmd(target_f, target_h)
            self.person_pub.publish(p_cmd)

        # 2) 3D Wheelchair (車いす): delay 秒前の目標値を取り出して物まね制御
        want_t = now - max(0.0, self.delay)
        chair_target = (now, target_h, target_f, action_label)
        for item in self.history:
            if item[0] <= want_t:
                chair_target = item
            else:
                break

        _, c_h, c_f, c_act = chair_target
        c_cmd, cur_f, cur_yaw = self.chair_tracker.compute_cmd(c_f, c_h)
        self.chair_pub.publish(c_cmd)

        # アクションが切り替わったとき、または1.5秒おきに状態を日本語ログ表示
        if c_act != self.last_logged_action or (now - self.last_status_print >= 1.5):
            self.last_logged_action = c_act
            self.last_status_print = now
            self.get_logger().info(
                f'{c_act} | 目標[前後:{c_f:+.2f}m, 向き:{c_h:+.0f}°] '
                f'現在[前後:{cur_f:+.2f}m, 向き:{cur_yaw:+.0f}°] '
                f'-> 速度[v:{c_cmd.linear.x:+.2f}m/s, w:{c_cmd.angular.z:+.2f}rad/s]'
            )

    def destroy_node(self):
        self.running = False
        try:
            if rclpy.ok():
                self.chair_pub.publish(Twist())
                self.person_pub.publish(Twist())
        except Exception:
            pass
        try:
            super().destroy_node()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = EisaDanceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
