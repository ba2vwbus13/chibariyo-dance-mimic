# ======================================================================
# 人のダンスに合わせて踊る（pose_dance）
#   /dance_pose（pose_detect が送る体の動き）から、ロボットの目標の向きと前後の位置を決め（pose_mapping）、
#   それに近づくように速度の指令を送る。手首を肩より上に上げると、その側の LED を点ける。
#     体の向き＋腰の左右の揺れ＋手足の動き → ロボットの向き（heading）
#     腰の上下のはずみ＋足上げ・しこ立ち・前後ステップ → ロボットの前後の位置（forward）
#   体を見失ったとき、/dance_pose が TIMEOUT 秒届かないときは止まる。
#   delay を 0 より大きくすると、その秒数だけ遅れて同じ動きをする（掛け合い）。
# 送り先：既定（robot:=whill）は /cmd_vel（Twist）、Raspberry Pi Mouse は /cmd_vel（TwistStamped）と /leds、
#         -p turtlesim:=true なら /turtle1/cmd_vel（Twist）
# ======================================================================

import collections
import csv
import datetime
import json
import math
import os
import socket

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool

from finger_dance.pose_mapping import DanceMapper, Follower

try:
    from raspimouse_msgs.msg import Leds
except ImportError:   # LED の型がない環境（turtlesim だけで使うとき）
    Leds = None

PERIOD = 0.05   # 指令を送る間隔 [s]

# ロボットごとの既定値（パラメータで個別に変えられる）
#   whill：WHILL Model CR（骨格座標の肩・腰・手・足の動きから /cmd_vel (Twist) を送り、whill/whill_bridge.py が受け取る）
PRESETS = {
    'raspimouse': dict(bounce_gain=0.1, heading_limit=60.0, forward_limit=0.02, linear_max=0.05, angular_max=1.0,
                       stamped=True, cmd_topic='/cmd_vel', motor_power=True, leds=True),
    # turtlesim の亀は画面の大きさが約 11 なので、前後の動きを 10 倍にして見えるようにする
    'turtlesim': dict(bounce_gain=5.0, heading_limit=60.0, forward_limit=1.5, linear_max=1.5, angular_max=2.0,
                      stamped=False, cmd_topic='/turtle1/cmd_vel', motor_power=False, leds=False),
    'whill': dict(bounce_gain=0.4, heading_limit=110.0, forward_limit=0.40, linear_max=0.60, angular_max=8.0,
                  stamped=False, cmd_topic='/cmd_vel', motor_power=False, leds=False),
}


class PoseDance(Node):

    def __init__(self):
        super().__init__('pose_dance')
        self.turtlesim = self.declare_parameter('turtlesim', False).value
        p = self.declare_parameter
        robot = p('robot', 'auto').value   # whill・raspimouse・turtlesim。auto なら turtlesim または whill
        if robot == 'auto' or self.turtlesim:
            robot = 'turtlesim' if self.turtlesim else 'whill'
        if robot not in PRESETS:
            raise ValueError(f'robot は {list(PRESETS)} のどれか: {robot}')
        self.robot = robot
        d = PRESETS[robot]
        self.mapper = DanceMapper(
            yaw_gain=p('yaw_gain', 1.0).value, sway_gain=p('sway_gain', 60.0).value,
            bounce_gain=p('bounce_gain', d['bounce_gain']).value,
            heading_limit=p('heading_limit', d['heading_limit']).value, forward_limit=p('forward_limit', d['forward_limit']).value)
        self.follower = Follower(
            heading_gain=p('heading_gain', 16.0).value, forward_gain=p('forward_gain', 12.0).value,
            linear_max=p('linear_max', d['linear_max']).value, angular_max=p('angular_max', d['angular_max']).value)
        self.timeout = p('timeout', 2.5).value
        self.delay = p('delay', 0.0).value
        self.stamped = p('stamped', d['stamped']).value
        cmd_topic = p('cmd_topic', d['cmd_topic']).value
        self.use_motor_power = p('motor_power', d['motor_power']).value
        self.use_leds = p('leds', d['leds']).value and Leds is not None

        self.pub = self.create_publisher(TwistStamped if self.stamped else Twist, cmd_topic, 10)
        # turtlesim:=true のときでも PowerShell の whill_bridge.py (/cmd_vel) が「指令なし」にならないよう /cmd_vel にも送る
        # （※ robot:=whill のときは /turtle1/cmd_vel には一切送らない）
        self.pub_whill_mirror = (
            self.create_publisher(Twist, '/cmd_vel', 10)
            if cmd_topic != '/cmd_vel' else None
        )
        self.pub_leds = self.create_publisher(Leds, '/leds', 10) if self.use_leds else None
        self.pub_eisa = self.create_publisher(String, '/eisa_state', 10)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_targets = []
        # Gazebo のコンテナ（wheelchair_follow）へ送る。届かないときは環境変数 CHIBARIYO_EXTRA_HOSTS に住所をカンマ区切りで渡す
        extra = [h.strip() for h in os.environ.get('CHIBARIYO_EXTRA_HOSTS', '').split(',') if h.strip()]
        for h_name in ('wheelchair_follow', 'host.docker.internal', *extra, '127.0.0.1'):
            try:
                ip = socket.gethostbyname(h_name)
                if (ip, 5006) not in self.udp_targets:
                    self.udp_targets.append((ip, 5006))
            except Exception:
                pass
        self.sub = self.create_subscription(String, '/dance_pose', self.pose_callback, 10)
        self.motor_client = self.create_client(SetBool, '/motor_power')
        self.timer = self.create_timer(PERIOD, self.timer_callback)
        self.targets = collections.deque()   # (時刻, 目標の向き, 目標の前後の位置, 左手, 右手, アクション名, 正規化前後m)
        self.last_time = None                 # 最後に体を検出した時刻
        self.last_pose_time = None            # 最後に /dance_pose を受け取った時刻
        self.last_pose_dict = None            # 最新の /dance_pose 辞書データ
        self.start_wall_t = None              # 最初のカメラフレームを受信した時刻
        self.last_csv_t = -1.0
        self.whill_front = 0.0
        self.whill_side = 0.0
        self.leds = None
        self.state = None

        # CSV自動記録の初期化（既定: /home/ubuntu/ros2_ws/analysis/live_camera_dance.csv および日時付きファイル）
        csv_dir = '/home/ubuntu/ros2_ws/analysis'
        os.makedirs(csv_dir, exist_ok=True)
        default_csv = os.path.join(csv_dir, 'live_camera_dance.csv')
        self.csv_path = p('csv', default_csv).value or default_csv
        ts_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_ts_path = os.path.join(csv_dir, f'camera_dance_{ts_str}.csv')
        self.csv_fields = [
            't', 'ok', 'action_name',
            'whill_front_pct', 'whill_side_pct', 'whill_front_raw', 'whill_side_raw',
            'v', 'w',
            'heading_target', 'forward_target', 'heading', 'forward',
            'yaw', 'dproj_a',
            'squat', 'l_knee_lift', 'r_knee_lift', 'r_step_fwd', 'l_step_back', 'l_up', 'r_up',
            'hip_x', 'hip_y',
            'r_knee_x', 'r_knee_y', 'l_knee_x', 'l_knee_y',
            'r_ankle_x', 'r_ankle_y', 'l_ankle_x', 'l_ankle_y',
        ]
        self.csv_files = []
        self.csv_writers = []
        for cpath in dict.fromkeys([self.csv_path, self.csv_ts_path]):
            try:
                f_obj = open(cpath, 'w', newline='', encoding='utf-8-sig')
                w_obj = csv.DictWriter(f_obj, fieldnames=self.csv_fields)
                w_obj.writeheader()
                f_obj.flush()
                self.csv_files.append(f_obj)
                self.csv_writers.append(w_obj)
            except Exception as e:
                self.get_logger().warn(f'CSVファイル作成失敗 ({cpath}): {e}')

        self.get_logger().info(f'ロボット {robot}、送り先 {cmd_topic}、向き ±{self.mapper.heading_limit:.0f}°、'
                               f'前後 ±{self.mapper.forward_limit:g}、遅れ {self.delay:g} 秒')
        self.get_logger().info(f'📊 CSVリアルタイム記録先: {self.csv_path} (および {os.path.basename(self.csv_ts_path)})')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def pose_callback(self, msg):
        d = json.loads(msg.data)
        t = self.now()
        if self.start_wall_t is None:
            self.start_wall_t = t
        dt = 0.1 if self.last_pose_time is None else min(0.3, max(0.01, t - self.last_pose_time))
        self.last_pose_time = t
        self.last_pose_dict = d
        if not d.get('ok'):
            # 一時的に人が検出できない間も、次に人が認識されるまでの動きに滑らかに繋げる
            if self.last_time is not None and (t - self.last_time) <= self.timeout:
                bridge_h = 8.0 * math.sin(t * 3.2)
                bridge_f = 0.04 * math.cos(t * 1.6)
                fwd_cmd = bridge_f * 10.0 if self.robot == 'turtlesim' else bridge_f
                self.targets.append((t, bridge_h, fwd_cmd, False, False, 'つなぎ動作（次動作へ補間中）', bridge_f))
            return
        if 'heading_target' in d and 'forward_target' in d:
            heading, forward = float(d['heading_target']), float(d['forward_target'])
            forward_norm = forward
            if self.robot == 'turtlesim':
                forward *= 10.0   # WHILL用の±0.12mをturtlesim画面(幅11)で見やすい±1.2に拡大
        else:
            heading, forward = self.mapper.update(d, dt)
            forward_norm = forward / 10.0 if self.robot == 'turtlesim' else forward
        action = d.get('action_name') or getattr(self.mapper, 'action_name', '待機中（初期位置）')
        self.targets.append((t, heading, forward, d.get('l_up', False), d.get('r_up', False), action, forward_norm))
        self.last_time = t

    def current_target(self):
        # delay 秒前の目標を使う（delay = 0 なら最新の目標）
        t_want = self.now() - self.delay
        target = None
        while self.targets and self.targets[0][0] <= t_want:
            target = self.targets.popleft()
        if target is not None:
            self.targets.appendleft(target)   # 次の周期でも使えるよう戻しておく
            while len(self.targets) > 1 and self.targets[1][0] <= t_want:
                self.targets.popleft()
        return target

    def timer_callback(self):
        lost = self.last_time is None or self.now() - self.last_time > self.timeout + self.delay
        target = None if lost else self.current_target()
        if target is None:
            if lost:
                self.mapper.reset()
                self.targets.clear()
            # しゃがみ込み等で一時的に見失った場合でも、初期位置(0.0m, 0.0°)へ戻り切るまで指令を出す
            if abs(self.follower.forward) >= 0.004 or abs(self.follower.heading) >= 0.4:
                v, w = self.follower.step(0.0, 0.0, PERIOD)
                state = '初期位置へ復帰中'
            else:
                v, w = 0.0, 0.0
                state = '体が見えない：停止' if lost else '待機中'
            eisa_payload = {'ok': False, 'action': state, 'heading_deg': 0.0, 'forward_m': 0.0, 'v': round(float(v), 4), 'w': round(float(w), 4)}
        else:
            v, w = self.follower.step(target[1], target[2], PERIOD)
            state = target[5] if len(target) > 5 else '踊っている'
            fwd_m = target[6] if len(target) > 6 else target[2]
            self.set_leds(target[3], target[4])
            eisa_payload = {
                'ok': True,
                'action': state,
                'heading_deg': round(float(target[1]), 2),
                'forward_m': round(float(fwd_m), 4),
                'l_up': bool(target[3]),
                'r_up': bool(target[4]),
                'v': round(float(v), 4),
                'w': round(float(w), 4),
            }
        self.send(v, w)
        self._record_csv_if_needed(v, w, state, target)
        try:
            payload_str = json.dumps(eisa_payload, ensure_ascii=False)
            self.pub_eisa.publish(String(data=payload_str))
            payload_bytes = (payload_str + '\n').encode('utf-8')
            for addr in self.udp_targets:
                try:
                    self.udp_sock.sendto(payload_bytes, addr)
                except Exception:
                    pass
        except Exception:
            pass
        if state != self.state:
            self.state = state
            self.get_logger().info(state)
        if lost:
            self.set_leds(False, False)

    def _record_csv_if_needed(self, v, w, state, target):
        if not self.csv_writers or self.start_wall_t is None:
            return
        now_t = self.now()
        elapsed = now_t - self.start_wall_t
        # 0.1秒 (10Hz) 間隔でCSVに1行記録
        if elapsed - self.last_csv_t < 0.095:
            return
        dt_csv = 0.1 if self.last_csv_t < 0 else min(0.3, max(0.05, elapsed - self.last_csv_t))
        self.last_csv_t = elapsed

        # turtlesimモードのときはWHILLスケール (1/10) に戻してWHILL値を算出
        v_whill = float(v) / 10.0 if self.robot == 'turtlesim' else float(v)
        w_whill = float(w)
        fwd_cur = self.follower.forward / 10.0 if self.robot == 'turtlesim' else self.follower.forward

        def _clip(x, lo, hi):
            return max(lo, min(hi, x))

        if target is None and abs(v_whill) < 1e-4 and abs(w_whill) < 1e-4:
            self.whill_front = 0.0
            self.whill_side = 0.0
        else:
            tf = _clip(v_whill / 0.35, -1.0, 1.0) * 55.0
            if tf < 0:
                tf = max(-70.0, tf * 1.3)
            ts = -1.0 * _clip(w_whill / 3.0, -1.0, 1.0) * 70.0
            r_step = 500.0 * dt_csv
            f_step = r_step * 1.5 if tf < 0 else r_step
            if tf * self.whill_front < 0 or abs(tf) < 1.0:
                self.whill_front = tf
            else:
                self.whill_front += _clip(tf - self.whill_front, -f_step, f_step) if abs(tf) > abs(self.whill_front) else (tf - self.whill_front)
            if ts * self.whill_side < 0 or abs(ts) < 1.0:
                self.whill_side = ts
            else:
                self.whill_side += _clip(ts - self.whill_side, -r_step, r_step) if abs(ts) > abs(self.whill_side) else (ts - self.whill_side)

        f_int = int(round(self.whill_front))
        s_int = int(round(self.whill_side))

        d = self.last_pose_dict if (self.last_pose_dict and (now_t - (self.last_pose_time or 0.0) <= self.timeout)) else {}
        ok_flag = 1 if d.get('ok') else 0
        iw = float(d.get('img_w', 320.0)) or 320.0
        ih = float(d.get('img_h', 240.0)) or 240.0

        def _norm_coord(key, denom):
            return round(float(d[key]) / denom, 4) if (ok_flag and key in d) else ''

        ht_val = round(float(target[1]), 2) if target is not None else 0.0
        ft_val = round(float(target[6] if len(target) > 6 else target[2]), 4) if target is not None else 0.0

        row = {
            't': round(elapsed, 2),
            'ok': ok_flag,
            'action_name': state,
            'whill_front_pct': f_int,
            'whill_side_pct': s_int,
            'whill_front_raw': round(self.whill_front, 2),
            'whill_side_raw': round(self.whill_side, 2),
            'v': round(v_whill, 4),
            'w': round(w_whill, 4),
            'heading_target': ht_val,
            'forward_target': ft_val,
            'heading': round(self.follower.heading, 2),
            'forward': round(fwd_cur, 4),
            'yaw': round(float(d['yaw']), 2) if (ok_flag and 'yaw' in d) else '',
            'dproj_a': round(float(d.get('dproj_a', 0.0)), 4) if ok_flag else '',
            'squat': int(bool(ok_flag and d.get('squat'))),
            'l_knee_lift': int(bool(ok_flag and d.get('l_knee_lift'))),
            'r_knee_lift': int(bool(ok_flag and d.get('r_knee_lift'))),
            'r_step_fwd': int(bool(ok_flag and d.get('r_step_fwd'))),
            'l_step_back': int(bool(ok_flag and d.get('l_step_back'))),
            'l_up': int(bool(ok_flag and d.get('l_up'))),
            'r_up': int(bool(ok_flag and d.get('r_up'))),
            'hip_x': _norm_coord('hip_x', iw),
            'hip_y': _norm_coord('hip_y', ih),
            'r_knee_x': _norm_coord('r_knee_x', iw),
            'r_knee_y': _norm_coord('r_knee_y', ih),
            'l_knee_x': _norm_coord('l_knee_x', iw),
            'l_knee_y': _norm_coord('l_knee_y', ih),
            'r_ankle_x': _norm_coord('r_ankle_x', iw),
            'r_ankle_y': _norm_coord('r_ankle_y', ih),
            'l_ankle_x': _norm_coord('l_ankle_x', iw),
            'l_ankle_y': _norm_coord('l_ankle_y', ih),
        }
        for w_obj, f_obj in zip(self.csv_writers, self.csv_files):
            try:
                w_obj.writerow(row)
                f_obj.flush()
            except Exception:
                pass

    def close_csv(self):
        for f_obj in getattr(self, 'csv_files', []):
            try:
                f_obj.close()
            except Exception:
                pass

    def set_leds(self, left, right):
        if not self.use_leds or (left, right) == self.leds:
            return
        self.leds = (left, right)
        msg = Leds()
        msg.led0 = msg.led1 = bool(right)   # 右手 → LED 0・1
        msg.led2 = msg.led3 = bool(left)    # 左手 → LED 2・3
        self.pub_leds.publish(msg)

    def send(self, linear, angular):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            twist = msg.twist
        else:
            msg = Twist()
            twist = msg
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.pub.publish(msg)
        if self.pub_whill_mirror is not None:
            w_msg = Twist()
            w_msg.linear.x = max(-0.45, min(0.45, float(linear) / 10.0))
            w_msg.angular.z = max(-5.5, min(5.5, float(angular)))
            self.pub_whill_mirror.publish(w_msg)

    def motor_power(self, on):
        if not self.use_motor_power:
            return
        if not self.motor_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/motor_power が見つかりません（ロボット側の raspimouse ノード、robot_connect、ROS_DOMAIN_ID を確認）')
            return
        future = self.motor_client.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl + C のあとでも停止の指令を送れるようにする
    node = PoseDance()
    node.motor_power(True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0)
        node.set_leds(False, False)
        node.motor_power(False)
        node.close_csv()
        node.destroy_node()
        rclpy.try_shutdown()
