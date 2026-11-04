#!/usr/bin/env python3
"""骨格(MediaPipe Pose)で人を追従するノード。深度カメラを使わない。

YOLO 方式との違い:
  - 人の枠ではなく骨格の点を取り、方向は腰の中心、距離は体の大きさ(胴の長さ・肩幅)から出す
  - 深度画像を使わないので、普通の RGB カメラ 1 台で動く(Pi Mouse・WHILL の実機向け)

アルゴリズム:
  1. RGB 画像に MediaPipe Pose Landmarker(VIDEO モード、最大 num_poses 人)をかける
  2. 各人の方向と距離を pose_follow_core.measure() で出す
  3. pose_follow_core.TargetTracker で、ロックオンした人を位置の連続性で選び続ける
  4. P 制御で cmd_vel を出力。見失ったら最後に見えた側へその場旋回して探す

カメラは高さ 1 m くらい・少し上向きにする(Gazebo なら sim.launch.py の
camera_z:=0.85 camera_pitch:=-0.17)。従来の 0.67 m・水平では、2 m より近いと頭が切れて
骨格が取れない。

体の実寸(torso_m, shoulder_m)の既定値は Gazebo の人モデルで測った値。実機では
calib_distance に人が立つ距離 [m] を入れると、ロックオンしたときに実寸を測り直す。

モデルファイル(pose_landmarker_full.task)は MediaPipe の配布物を model_path に置く。
model_path に無ければ、Docker のイメージに入れたもの(/opt/mediapipe/pose_landmarker_full.task)を使う。
追従状態を /follower/status (JSON文字列)で配信する(metrics_logger の計測用)。
"""
import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from wheelchair_follower.pose_follow_core import (
    TargetTracker, calibrate, follow_cmd, measure)



IMAGE_MODEL = '/opt/mediapipe/pose_landmarker_full.task'

class PoseFollowerNode(Node):

    def __init__(self):
        super().__init__('pose_follower')

        # 制御パラメータ(他方式と共通の考え方)
        self.declare_parameter('target_distance', 1.2)
        self.declare_parameter('stop_distance', 0.8)
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 1.2)
        self.declare_parameter('k_linear', 0.8)
        self.declare_parameter('k_angular', 1.8)
        self.declare_parameter('lost_timeout', 1.5)
        self.declare_parameter('search_angular', 0.5)  # 探索旋回速度 [rad/s]
        self.declare_parameter('hfov', 1.36)           # 水平FOV [rad]
        # 骨格のパラメータ
        self.declare_parameter('model_path', '~/mp/pose_landmarker_full.task')
        self.declare_parameter('num_poses', 2)
        self.declare_parameter('min_visibility', 0.5)
        self.declare_parameter('torso_m', 0.62)       # 胴の長さ(肩の中心〜腰の中心) [m]
        self.declare_parameter('shoulder_m', 0.42)    # 肩幅 [m]
        self.declare_parameter('calib_distance', 0.0)  # >0 ならロックオン時にこの距離で実寸を測る
        self.declare_parameter('lock_max_dist', 4.0)  # ロックオン許容距離 [m]
        self.declare_parameter('relock_max_dist', 6.0)  # 見失った後のロックオン許容距離 [m]
        self.declare_parameter('gate_m', 0.8)         # 前回位置からの許容ずれ [m]
        # 調べる用: 空でなければ、見失っている間の画像(検出結果つき)を 1 秒ごとに保存する
        self.declare_parameter('debug_dir', '')

        from mediapipe.tasks import python as mp_python  # import に時間がかかるためここで
        from mediapipe.tasks.python import vision
        import mediapipe as mp
        self.mp = mp
        path = os.path.expanduser(self.get_parameter('model_path').value)
        if not os.path.exists(path) and os.path.exists(IMAGE_MODEL):
            path = IMAGE_MODEL   # Docker のイメージに入れたモデル
        opts = vision.PoseLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=path),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=self.get_parameter('num_poses').value)
        self.detector = vision.PoseLandmarker.create_from_options(opts)
        self.t_ms = 0

        self.tracker = TargetTracker(
            lock_max_dist=self.get_parameter('lock_max_dist').value,
            relock_max_dist=self.get_parameter('relock_max_dist').value,
            gate_m=self.get_parameter('gate_m').value)
        self.torso_m = self.get_parameter('torso_m').value
        self.shoulder_m = self.get_parameter('shoulder_m').value
        self.last_seen_time = None
        self.search_dir = 1.0       # 探索旋回の向き(最後に見えた側)
        self.busy = False
        self.n_frames = 0
        self.t_infer = 0.0
        self.n_people = 0      # 人(骨格)が 1 人以上取れた枚数
        self.n_measured = 0    # 肩が見えて距離まで出せた枚数
        self.n_tracked = 0     # 追っている人を選べた枚数
        self.t_debug = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/follower/status', 10)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image_raw', self.rgb_callback, 1)
        self.timer = self.create_timer(0.1, self.watchdog)

        self.get_logger().info('骨格追従ノード起動')

    @staticmethod
    def image_to_rgb(msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            return np.ascontiguousarray(img)
        if msg.encoding == 'bgr8':
            return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return None

    def lost_seconds(self):
        if self.last_seen_time is None:
            return 0.0
        return (self.get_clock().now() - self.last_seen_time).nanoseconds * 1e-9

    def rgb_callback(self, msg: Image):
        if self.busy:
            return
        rgb = self.image_to_rgb(msg)
        if rgb is None:
            return
        self.busy = True
        try:
            # VIDEO モードは時刻が単調増加でないといけない
            self.t_ms = max(self.t_ms + 1, int(time.monotonic() * 1000))
            t0 = time.monotonic()
            r = self.detector.detect_for_video(
                self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), self.t_ms)
            self.t_infer += time.monotonic() - t0
            self.n_frames += 1
            if self.n_frames % 100 == 0:
                self.get_logger().info(
                    f'推論 平均 {1000 * self.t_infer / self.n_frames:.0f} ms/枚 '
                    f'(全 {self.n_frames} 枚: 人あり {self.n_people} / '
                    f'距離あり {self.n_measured} / 追従 {self.n_tracked})')

            hfov = self.get_parameter('hfov').value
            vis = self.get_parameter('min_visibility').value
            ms = [m for m in (measure(lm, msg.width, msg.height, hfov,
                                      self.torso_m, self.shoulder_m, vis)
                              for lm in r.pose_landmarks) if m is not None]
            locked = self.tracker.last is not None
            target = self.tracker.select(ms, self.lost_seconds())
            self.n_people += bool(r.pose_landmarks)
            self.n_measured += bool(ms)
            self.n_tracked += target is not None
            if target is None:
                self.save_debug(rgb, r.pose_landmarks, ms)
                return

            calib_d = self.get_parameter('calib_distance').value
            if not locked and calib_d > 0.0:
                t_m, s_m = calibrate(target.torso_px, target.shoulder_px, calib_d,
                                     msg.width, hfov)
                self.torso_m = t_m or self.torso_m
                self.shoulder_m = s_m or self.shoulder_m
                self.get_logger().info(
                    f'実寸を測り直した: 胴 {self.torso_m:.2f} m / 肩幅 {self.shoulder_m:.2f} m')
                target.distance = calib_d
                self.tracker.last = (calib_d, target.bearing)

            self.last_seen_time = self.get_clock().now()
            self.search_dir = 1.0 if target.bearing >= 0.0 else -1.0
            self.publish_cmd(target.distance, target.bearing)
            self.publish_status('tracking', target.distance, target.bearing)
        finally:
            self.busy = False

    def save_debug(self, rgb, poses, ms):
        out = os.path.expanduser(self.get_parameter('debug_dir').value)
        now = time.monotonic()
        if not out or now - self.t_debug < 1.0:
            return
        self.t_debug = now
        os.makedirs(out, exist_ok=True)
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        h, w = img.shape[:2]
        for lm in poses:
            for p in lm:
                c = (0, 255, 0) if p.visibility >= 0.5 else (0, 0, 255)
                cv2.circle(img, (int(p.x * w), int(p.y * h)), 3, c, -1)
        text = (f'lost {self.lost_seconds():.1f}s poses {len(poses)} measured '
                + ' '.join(f'{m.distance:.1f}m/{m.bearing:+.2f}' for m in ms))
        cv2.putText(img, text, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imwrite(os.path.join(out, f'lost_{self.n_frames:06d}.png'), img)

    def publish_cmd(self, dist, bearing):
        g = self.get_parameter
        v, w = follow_cmd(dist, bearing,
                          g('target_distance').value, g('stop_distance').value,
                          g('k_linear').value, g('k_angular').value,
                          g('max_linear').value, g('max_angular').value)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def publish_status(self, state, dist=None, bearing=None):
        """yolo_reid_follower と同じ書式で追従状態を配信(metrics_logger用)"""
        msg = String()
        msg.data = json.dumps({
            'state': state,
            'distance': dist,
            'bearing': bearing,
            'similarity': None,
        })
        self.status_pub.publish(msg)

    def watchdog(self):
        if self.last_seen_time is None:
            return
        if self.lost_seconds() > self.get_parameter('lost_timeout').value:
            cmd = Twist()
            cmd.angular.z = (self.get_parameter('search_angular').value
                             * self.search_dir)
            self.cmd_pub.publish(cmd)  # その場旋回で探索(前回位置の記憶は保持)
            self.publish_status('lost')


def main(args=None):
    rclpy.init(args=args)
    node = PoseFollowerNode()
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
