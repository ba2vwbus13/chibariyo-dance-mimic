#!/usr/bin/env python3
"""YOLO + 色ヒストグラムRe-IDで「利用者本人」を追従するノード。

yolo_follower(最も近い人を追う)との違い:
  - ロックオン時に対象の服の色ヒストグラム(HSV)を記憶する
  - 以降は「色の類似度 + 位置の連続性」で本人を選ぶ(近いだけの別人は追わない)
  - 見失っても記憶は保持し、再登場した本人を色で再発見(Re-ID)する
  - 追従状態を /follower/status (JSON文字列)で配信し、metrics_loggerが記録する

アルゴリズム:
  1. RGB画像に対しYOLO(ultralytics)で人(COCOクラス0)を検出
  2. 各候補について 距離(深度中央値)・方位角・上半身の色ヒストグラム を計算
  3. 未ロックなら: 最も近い候補(lock_max_dist以内)を本人としてロックオン
  4. ロック済みなら: score = w_hist×色類似度 + w_pos×位置連続性 が最大の候補を選ぶ
     (score が accept_threshold 未満なら「本人不在」として見失い扱い)
  5. P制御で cmd_vel を出力。lost_timeout の間見失ったら探索(記憶は保持)
  6. 探索は最後に見えた側へのその場旋回。predict_search=True のときは、本人の位置
     (odom座標)と歩行速度から今いそうな位置を予測してそちらへ向き直る
"""
import json
import math
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class YoloReidFollowerNode(Node):

    def __init__(self):
        super().__init__('yolo_reid_follower')

        # 制御パラメータ(他方式と共通の考え方)
        self.declare_parameter('target_distance', 1.2)
        self.declare_parameter('stop_distance', 0.8)
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 1.2)
        self.declare_parameter('k_linear', 0.8)
        self.declare_parameter('k_angular', 1.8)
        self.declare_parameter('lost_timeout', 1.5)
        # 見失い時、最後に見えた方向へその場旋回して探索する
        # (全停止だと一度はぐれたきり復帰できず、計測が座礁で埋まるため)
        self.declare_parameter('search_angular', 0.5)  # 探索旋回速度 [rad/s]
        self.declare_parameter('hfov', 1.36)          # 水平FOV [rad]
        # YOLOパラメータ
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('imgsz', 320)
        # Re-IDパラメータ
        self.declare_parameter('lock_max_dist', 4.0)   # ロックオン許容距離 [m]
        # 追従中の距離上限。lock_max_distはロック時にしか効かず、
        # 追従中は背景を掴んだ候補(6m超)がそのまま採用されていた
        self.declare_parameter('track_max_dist', 5.0)   # 追従中の許容距離 [m]
        self.declare_parameter('w_hist', 0.6)          # 色類似度の重み
        self.declare_parameter('w_pos', 0.4)           # 位置連続性の重み
        self.declare_parameter('accept_threshold', 0.45)  # 本人と認める最小score
        # 色類似度の下限(これ未満は別人)。原本アンカー導入後の実測で
        # 本人は中央値0.90/p5 0.65、通行人は中央値0.45/p95 0.55に分かれた。
        # 0.55で本人の棄却2.3%、通行人の通過5.5%
        self.declare_parameter('hist_min_sim', 0.55)
        # Trueで見失い時間に応じて色の下限を最大1/4まで緩める(旧挙動)。
        # 記憶がずれて全候補を棄却し続ける状態から抜けるための措置だったが、
        # 追従中でも下限が0.46まで下がり、本人に重なった通行人を受理していた。
        # 記憶のずれ自体は anchor_update_sim で防ぐ
        self.declare_parameter('hist_relax', False)
        self.declare_parameter('hist_update_sim', 0.6) # これ以上似ていたらヒストグラム更新
        # 更新にはロックオン時の原本にもこれ以上似ていることを要求する。
        # 記憶だけで判定すると、本人に似た通行人を受理した瞬間に記憶がその人へ
        # 寄り始め、数秒後には本人の方が棄却される(実測で21秒見失った回の原因)
        self.declare_parameter('anchor_update_sim', 0.7)
        self.declare_parameter('depth_sync', True)  # Falseで旧挙動(最新の深度を使う)
        # 予測探索。見失った本人は歩き続けているので、最後に見えた方位へ
        # 一定速度で回るより、歩行先の予測位置へ向き直る方が早く視野に戻せる。
        # 実測では見失い時間の大半(全体の7-42%)は本人が視野外にいる時間だった。
        # ただし2026-10-20のn=3比較では一定旋回との差が出なかったため既定は無効
        self.declare_parameter('predict_search', False)  # Trueで予測探索
        self.declare_parameter('predict_horizon', 3.0)   # 等速予測を延ばす最大時間 [s]
        self.declare_parameter('predict_hold', 2.0)      # 予測位置を向いて待つ追加時間 [s]
        self.declare_parameter('max_target_speed', 1.0)  # 推定歩行速度の上限 [m/s]
        self.declare_parameter('search_linear_max', 0.3) # 予測探索中の最大前進速度 [m/s]
        self.declare_parameter('camera_x', 0.255)        # base中心からカメラまで [m]
        # 色を取る領域を床からの高さで決める。ボックス比率(上から15-55%)だと、
        # 近距離でボックス上端が画面外に切れたとき別の部位を見てしまい、
        # 本人の類似度が1m以内で0.47-0.67まで落ちて通行人(0.40-0.43)と重なっていた
        self.declare_parameter('torso_roi', True)        # Falseで旧挙動(ボックス比率)
        self.declare_parameter('camera_height', 0.67)    # カメラの床からの高さ [m]
        self.declare_parameter('torso_z_min', 0.95)      # 胴体帯の下端 [m]
        self.declare_parameter('torso_z_max', 1.35)      # 胴体帯の上端 [m]
        # ヒストグラムを色相・彩度の両方向にぼかす幅 [ビン] (0で旧挙動)。
        # 赤い服の彩度は照明や向きでフレームごとに平均172-194と動き、幅8の彩度ビンの
        # 境界をまたぐため、平滑化なしでは本人どうしの類似度が0.14まで落ちていた
        # (服装違いの検証で判明)。効いているのは彩度方向で、色相方向だけでは変わらない
        self.declare_parameter('hist_blur', 2)
        self.declare_parameter('hist_ema', 0.9)        # ヒストグラム更新の慣性(旧:新=0.9:0.1)
        self.declare_parameter('pos_angle_sigma', 0.35)  # 位置連続性: 方位のゆるさ [rad]
        self.declare_parameter('pos_dist_sigma', 0.8)    # 位置連続性: 距離のゆるさ [m]

        self.get_logger().info('YOLOモデルを読み込み中...')
        from ultralytics import YOLO  # import に時間がかかるためここで
        self.model = YOLO(self.get_parameter('model').value)
        self.get_logger().info('YOLOモデル読み込み完了')

        self.depth_image = None
        # RGBと同じ時刻の深度を使うため直近数フレームを保持する。
        # 最新の深度をそのまま使うと1フレームずれることがあり、旋回中は
        # 人のボックスに背景の深度が入って距離が2.6m⇔6.5mと交互に跳ねていた
        self.depth_buf = deque(maxlen=5)  # (stamp_ns, depth)
        # 対応する深度より先に届いたRGB。捨てると処理フレームが3割減るため、
        # 深度の到着を待って処理する(推論中に溜まったRGBと深度は届く順が不定)
        self.pending_rgb = None
        self.busy = False           # 推論中は新しいフレームをスキップ
        self.target_hist = None     # 本人の色ヒストグラム(ロックオン後に保持)
        self.anchor_hist = None     # ロックオン時の原本(更新しない)
        self.last_seen_time = None  # 最後に本人を見た時刻
        self.last_dist = None       # 最後に見た距離 [m]
        self.last_bearing = None    # 最後に見た方位 [rad]
        self.search_dir = 1.0       # 探索旋回の向き(最後に見えた側)
        self.odom_pose = None       # 車椅子の (x, y, yaw) 車輪odom座標
        self.target_track = deque(maxlen=15)  # 本人の (t, x, y) odom座標
        self.target_vel = (0.0, 0.0)          # 本人の推定速度 [m/s]

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/follower/status', 10)
        # 診断用: 毎フレームの候補と棄却理由(計測指標には影響しない)
        self.debug_pub = self.create_publisher(String, '/follower/debug', 10)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image_raw', self.rgb_callback, 1)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 1)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.watchdog)

        self.get_logger().info('YOLO Re-ID追従ノード起動')

    # ---------- 画像変換(cv_bridge非依存) ----------
    @staticmethod
    def image_to_bgr(msg: Image):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = buf.reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if msg.encoding == 'bgr8':
            return img.copy()
        return None

    @staticmethod
    def image_to_depth(msg: Image):
        if msg.encoding == '32FC1':
            return np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width)
        if msg.encoding == '16UC1':
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width)
            return raw.astype(np.float32) * 0.001
        return None

    # ---------- 色ヒストグラム ----------
    def bbox_histogram(self, bgr, x0, y0, x1, y1, dist):
        """ボックス胴体領域のHSVヒストグラム(H30×S32ビン、正規化済み)"""
        h_img, w_img = bgr.shape[:2]
        # 横はボックス中央60%
        bw = x1 - x0
        bh = y1 - y0
        rx0 = int(max(0, x0 + 0.2 * bw))
        rx1 = int(min(w_img, x1 - 0.2 * bw))
        if self.get_parameter('torso_roi').value:
            # 縦は床から torso_z_min〜max の帯が写る行(ピンホール投影)。
            # 画素は正方なので fy = fx = (幅/2)/tan(hfov/2)、光軸は画像中央
            f = (w_img / 2) / math.tan(self.get_parameter('hfov').value / 2)
            h_cam = self.get_parameter('camera_height').value
            v_top = h_img / 2 - f * (self.get_parameter('torso_z_max').value - h_cam) / dist
            v_bot = h_img / 2 - f * (self.get_parameter('torso_z_min').value - h_cam) / dist
            ry0 = int(max(0, y0, v_top))
            ry1 = int(min(h_img, y1, v_bot))
        else:
            # 旧挙動: 上から15%〜55%
            ry0 = int(max(0, y0 + 0.15 * bh))
            ry1 = int(min(h_img, y0 + 0.55 * bh))
        if rx1 - rx0 < 4 or ry1 - ry0 < 4:
            return None
        patch = bgr[ry0:ry1, rx0:rx1]
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [30, 32],
                            [0, 180, 0, 256])
        blur = self.get_parameter('hist_blur').value
        if blur > 0:
            # 色相(行)は循環するので、両端を反対側から補ってからぼかす
            pad = np.concatenate([hist[-blur:], hist, hist[:blur]], axis=0)
            hist = cv2.GaussianBlur(pad, (2 * blur + 1, 2 * blur + 1), 0)[blur:-blur]
            hist = np.ascontiguousarray(hist, dtype=np.float32)
        cv2.normalize(hist, hist, alpha=1.0, norm_type=cv2.NORM_L1)
        return hist

    @staticmethod
    def hist_similarity(h1, h2):
        """バタチャリヤ距離に基づく類似度(1=同一, 0=全く別)"""
        d = cv2.compareHist(h1, h2, cv2.HISTCMP_BHATTACHARYYA)
        return max(0.0, 1.0 - float(d))

    def pos_score(self, dist, bearing, t_lost):
        """位置連続性スコア。見失い時間が長いほどゲートを緩める"""
        if self.last_dist is None:
            return 0.0
        relax = 1.0 + min(3.0, t_lost)  # 最大4倍まで緩和
        s_ang = self.get_parameter('pos_angle_sigma').value * relax
        s_dst = self.get_parameter('pos_dist_sigma').value * relax
        # 予測位置と比べる。最後に見えた方位はカメラ座標なので、
        # 探索で旋回した後に比べると本人ほど離れて見えてしまう
        ref = self.predicted(t_lost)
        ref_d, ref_b = ref if ref is not None else (self.last_dist, self.last_bearing)
        da = math.atan2(math.sin(bearing - ref_b), math.cos(bearing - ref_b)) / s_ang
        dd = (dist - ref_d) / s_dst
        return math.exp(-(da * da + dd * dd))

    # ---------- コールバック ----------
    # ---------- 本人位置の予測(odom座標) ----------
    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (p.x, p.y, yaw)

    def camera_to_odom(self, dist, bearing):
        rx, ry, yaw = self.odom_pose
        cx = rx + self.get_parameter('camera_x').value * math.cos(yaw)
        cy = ry + self.get_parameter('camera_x').value * math.sin(yaw)
        return cx + dist * math.cos(yaw + bearing), cy + dist * math.sin(yaw + bearing)

    def odom_to_camera(self, x, y):
        rx, ry, yaw = self.odom_pose
        cx = rx + self.get_parameter('camera_x').value * math.cos(yaw)
        cy = ry + self.get_parameter('camera_x').value * math.sin(yaw)
        b = math.atan2(y - cy, x - cx) - yaw
        return math.hypot(x - cx, y - cy), math.atan2(math.sin(b), math.cos(b))

    def update_target_track(self, dist, bearing):
        if self.odom_pose is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        x, y = self.camera_to_odom(dist, bearing)
        self.target_track.append((t, x, y))
        # 1秒強の窓で速度を推定(1フレームの差分は深度の揺れで暴れる)
        old = [p for p in self.target_track if t - p[0] <= 1.5]
        t0, x0, y0 = old[0]
        if t - t0 >= 0.5:
            vx, vy = (x - x0) / (t - t0), (y - y0) / (t - t0)
            v = math.hypot(vx, vy)
            vmax = self.get_parameter('max_target_speed').value
            if v > vmax:
                vx, vy = vx * vmax / v, vy * vmax / v
            self.target_vel = (vx, vy)

    def predicted(self, t_lost):
        """見失ってt_lost秒後の本人の位置を、現在のカメラ座標の(距離, 方位)で返す"""
        if (not self.get_parameter('predict_search').value
                or self.odom_pose is None or not self.target_track):
            return None
        _, x, y = self.target_track[-1]
        h = min(t_lost, self.get_parameter('predict_horizon').value)
        x += self.target_vel[0] * h
        y += self.target_vel[1] * h
        return self.odom_to_camera(x, y)

    @staticmethod
    def stamp_ns(msg: Image):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def depth_callback(self, msg: Image):
        depth = self.image_to_depth(msg)
        if depth is None:
            return
        self.depth_buf.append((self.stamp_ns(msg), depth))
        if self.pending_rgb is not None and \
                abs(self.stamp_ns(self.pending_rgb) - self.stamp_ns(msg)) <= 50_000_000:
            rgb, self.pending_rgb = self.pending_rgb, None
            self.rgb_callback(rgb)

    def depth_for(self, msg: Image):
        """RGBと同じstampの深度。無ければ最も近いもの(50ms以内)"""
        if not self.depth_buf:
            return None
        if not self.get_parameter('depth_sync').value:
            return self.depth_buf[-1][1]
        t = self.stamp_ns(msg)
        stamp, depth = min(self.depth_buf, key=lambda d: abs(d[0] - t))
        if abs(stamp - t) > 50_000_000:
            return None
        return depth

    def rgb_callback(self, msg: Image):
        if self.busy:
            return
        self.depth_image = self.depth_for(msg)
        if self.depth_image is None:
            if self.get_parameter('depth_sync').value:
                self.pending_rgb = msg  # 深度が届いたら処理する
            return
        self.pending_rgb = None
        bgr = self.image_to_bgr(msg)
        if bgr is None:
            return
        if self.depth_image.shape[:2] != (msg.height, msg.width):
            return

        self.busy = True
        try:
            results = self.model.predict(
                bgr,
                classes=[0],  # person のみ
                conf=self.get_parameter('confidence').value,
                imgsz=self.get_parameter('imgsz').value,
                verbose=False,
            )
            boxes = results[0].boxes
            if boxes is None or len(boxes) == 0:
                return

            hfov = self.get_parameter('hfov').value
            candidates = []  # (dist, bearing, hist)
            for xyxy in boxes.xyxy.cpu().numpy():
                x0, y0, x1, y1 = xyxy
                dist = self.bbox_depth(x0, y0, x1, y1, msg.width, msg.height)
                if dist is None:
                    continue
                hist = self.bbox_histogram(bgr, x0, y0, x1, y1, dist)
                if hist is None:
                    continue
                # float()を忘れるとnumpy.float32がjson.dumpsで
                # TypeErrorになりノードが落ちるため必ずPython floatに変換する
                bearing = float((0.5 - (x0 + x1) / 2 / msg.width) * hfov)
                candidates.append((dist, bearing, hist))
            if not candidates:
                return

            if self.target_hist is None:
                self.lock_on(candidates)
                return

            self.track(candidates)
        finally:
            self.busy = False

    def lock_on(self, candidates):
        """未ロック時: 最も近い候補を本人として記憶"""
        dist, bearing, hist = min(candidates, key=lambda c: c[0])
        if dist > self.get_parameter('lock_max_dist').value:
            return
        self.target_hist = hist
        self.anchor_hist = hist.copy()
        self.target_track.clear()
        self.target_vel = (0.0, 0.0)
        self.mark_seen(dist, bearing, sim=1.0)
        self.get_logger().info(f'ロックオン: 距離{dist:.2f}m の人物を本人として記憶')

    def track(self, candidates):
        """ロック済み: 色類似度+位置連続性で本人を選ぶ"""
        now = self.get_clock().now()
        t_lost = 0.0
        if self.last_seen_time is not None:
            t_lost = (now - self.last_seen_time).nanoseconds * 1e-9

        relax = 1.0 + min(3.0, t_lost)  # 最大4倍まで緩和(pos_scoreと共通)
        w_h = self.get_parameter('w_hist').value
        w_p = self.get_parameter('w_pos').value
        best = None  # (score, sim, dist, bearing, hist)
        track_max = self.get_parameter('track_max_dist').value
        # hist_relax=True(旧挙動)では見失いが続くほど色の門を緩める。
        # 記憶がずれて全候補を棄却し続ける状態(追従率8%の回)への対策だったが、
        # 現在はずれを anchor_update_sim で防いでいるため既定では緩めない。
        min_sim = self.get_parameter('hist_min_sim').value
        if self.get_parameter('hist_relax').value:
            min_sim /= relax
        diag = []  # 候補ごとの [dist, bearing, sim, pos, score, 理由]
        for dist, bearing, hist in candidates:
            sim = self.hist_similarity(self.target_hist, hist)
            pos = self.pos_score(dist, bearing, t_lost)
            score = w_h * sim + w_p * pos
            diag.append([round(dist, 2), round(bearing, 3), round(sim, 3),
                         round(pos, 3), round(score, 3), ''])
            if dist > track_max:
                diag[-1][5] = 'far'
                continue  # 背景を掴んだ候補。本人がこの距離に居ることはない
            if sim < min_sim:
                diag[-1][5] = 'color'
                continue  # 色が違いすぎる → 別人
            if best is None or score > best[0]:
                best = (score, sim, dist, bearing, hist)

        accepted = (best is not None
                    and best[0] >= self.get_parameter('accept_threshold').value)
        self.publish_debug(t_lost, min_sim, diag, accepted)
        if not accepted:
            return  # 本人候補なし(見失い継続。watchdogが停止を担当)

        score, sim, dist, bearing, hist = best
        # 十分似ているときだけ記憶を少しずつ更新(照明変化に追随、別人には染まらない)
        if (sim > self.get_parameter('hist_update_sim').value
                and self.hist_similarity(self.anchor_hist, hist)
                > self.get_parameter('anchor_update_sim').value):
            ema = self.get_parameter('hist_ema').value
            self.target_hist = ema * self.target_hist + (1.0 - ema) * hist
            cv2.normalize(self.target_hist, self.target_hist,
                          alpha=1.0, norm_type=cv2.NORM_L1)
        self.mark_seen(dist, bearing, sim)

    def mark_seen(self, dist, bearing, sim):
        self.last_seen_time = self.get_clock().now()
        self.last_dist = dist
        self.last_bearing = bearing
        self.search_dir = 1.0 if bearing >= 0.0 else -1.0
        self.update_target_track(dist, bearing)
        self.publish_cmd(dist, bearing)
        self.publish_status('tracking', dist, bearing, sim)

    def bbox_depth(self, x0, y0, x1, y1, w, h):
        """ボックス中央1/3領域の深度[m]。人は背景より手前にあるので下位25%を取る。

        中央値だと、ボックスに背景が半分近く混ざったときに背景側へ倒れ、
        2m先の人を6m先(部屋の壁)と誤報して全速で突っ込む挙動になっていた。
        人体はボックス内で最も手前の面なので、低いパーセンタイルの方が頑健。
        """
        cx0 = int(max(0, x0 + (x1 - x0) / 3))
        cx1 = int(min(w, x1 - (x1 - x0) / 3))
        cy0 = int(max(0, y0 + (y1 - y0) / 3))
        cy1 = int(min(h, y1 - (y1 - y0) / 3))
        if cx1 <= cx0 or cy1 <= cy0:
            return None
        patch = self.depth_image[cy0:cy1, cx0:cx1]
        valid = patch[np.isfinite(patch) & (patch > 0.05)]
        if valid.size == 0:
            return None
        return float(np.percentile(valid, 25))

    # ---------- 制御 ----------
    def publish_cmd(self, dist, bearing):
        target_d = self.get_parameter('target_distance').value
        stop_d = self.get_parameter('stop_distance').value
        k_lin = self.get_parameter('k_linear').value
        k_ang = self.get_parameter('k_angular').value
        max_lin = self.get_parameter('max_linear').value
        max_ang = self.get_parameter('max_angular').value

        cmd = Twist()
        cmd.angular.z = max(-max_ang, min(max_ang, k_ang * bearing))
        if dist > stop_d:
            v = k_lin * (dist - target_d)
            v *= max(0.0, math.cos(bearing))
            cmd.linear.x = max(0.0, min(max_lin, v))
        self.cmd_pub.publish(cmd)

    def publish_status(self, state, dist=None, bearing=None, sim=None):
        msg = String()
        msg.data = json.dumps({
            'state': state,
            'distance': dist,
            'bearing': bearing,
            'similarity': sim,
        })
        self.status_pub.publish(msg)

    def publish_debug(self, t_lost, min_sim, diag, accepted):
        msg = String()
        msg.data = json.dumps({
            't_lost': round(t_lost, 2),
            'min_sim': round(min_sim, 3),
            'accepted': accepted,
            'cands': diag,
        })
        self.debug_pub.publish(msg)

    def watchdog(self):
        if self.last_seen_time is None:
            return
        timeout = self.get_parameter('lost_timeout').value
        elapsed = (self.get_clock().now() - self.last_seen_time).nanoseconds * 1e-9
        if elapsed > timeout:
            cmd = Twist()
            pred = self.predicted(elapsed)
            hold = (self.get_parameter('predict_horizon').value
                    + self.get_parameter('predict_hold').value)
            if pred is not None and elapsed < timeout + hold:
                # 予測位置へ向き直り、遠ければゆっくり近づく
                d, b = pred
                max_ang = self.get_parameter('max_angular').value
                cmd.angular.z = max(-max_ang, min(max_ang,
                                    self.get_parameter('k_angular').value * b))
                if abs(b) < 0.4 and d > self.get_parameter('target_distance').value:
                    cmd.linear.x = min(
                        self.get_parameter('search_linear_max').value,
                        self.get_parameter('k_linear').value
                        * (d - self.get_parameter('target_distance').value))
                self.search_dir = 1.0 if b >= 0.0 else -1.0
            else:
                cmd.angular.z = (self.get_parameter('search_angular').value
                                 * self.search_dir)
            self.cmd_pub.publish(cmd)  # 探索中(記憶は保持)
            self.publish_status('lost')


def main(args=None):
    rclpy.init(args=args)
    node = YoloReidFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
