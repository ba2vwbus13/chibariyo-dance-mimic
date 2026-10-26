# ======================================================================
# 体の検出（pose_detect）
#   /image_raw/compressed（カメラページ、または video_source の動画）を受け取り、
#   MediaPipe の Pose Landmarker で体の 33 点を求め、踊りに使う動きを /dance_pose（String、JSON）で送る。
#     例：{"ok": true, "yaw": 12.3, "hip_x": 321.0, "hip_y": 260.5, "torso": 98.2, "l_up": false, "r_up": true}
#   体の点と体の向きを描いた画像を /pose_image（非圧縮。rqt_image_view で見る）と
#   /pose_image/compressed（JPEG。ros2 bag で記録する）で送る。
# パラメータ：model（モデルファイル）
# ======================================================================

import json
import math
import time

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from finger_dance.pose_mapping import features

MODEL = '/opt/finger_dance/pose_landmarker_full.task'
# 骨組みの線（肩・腕・胴・脚）
BONES = [(11, 12), (11, 13), (13, 15), (12, 14), (14, 16), (11, 23), (12, 24), (23, 24),
         (23, 25), (25, 27), (24, 26), (26, 28)]


class PoseDetect(Node):

    def __init__(self):
        super().__init__('pose_detect')
        model = self.declare_parameter('model', MODEL).value
        options = vision.PoseLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model),
            running_mode=vision.RunningMode.VIDEO, num_poses=1)
        self.landmarker = vision.PoseLandmarker.create_from_options(options)
        self.last_ts = 0
        latest_only = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, latest_only)
        self.pub_pose = self.create_publisher(String, '/dance_pose', 10)
        self.pub_image = self.create_publisher(Image, '/pose_image', 1)
        self.pub_jpeg = self.create_publisher(CompressedImage, '/pose_image/compressed', 10)
        self.last_ok = None
        self.get_logger().info('体の検出を開始（/image_raw/compressed を待っています）')

    def image_callback(self, msg):
        t0 = time.time()
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        h, w = img.shape[:2]
        ts = max(int(time.monotonic() * 1000), self.last_ts + 1)
        self.last_ts = ts
        res = self.landmarker.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), ts)
        f = None
        if res.pose_landmarks:
            pts = [(p.x, p.y, p.visibility) for p in res.pose_landmarks[0]]
            f = features(pts, [(p.x, p.y, p.z) for p in res.pose_world_landmarks[0]], w, h)
            self.draw(img, pts, f)
        out = {'ok': f is not None}
        if f:
            out.update({k: (round(v, 2) if isinstance(v, float) else v) for k, v in f.items()})
        self.pub_pose.publish(String(data=json.dumps(out)))
        label = f'yaw {f["yaw"]:+.0f}deg' if f else 'no body'
        cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 0) if f else (0, 0, 255), 2)
        if (f is not None) != self.last_ok:
            self.get_logger().info(f'体を{"検出" if f else "見失った"}　処理 {1000 * (time.time() - t0):.0f} ms')
            self.last_ok = f is not None
        raw = Image()
        raw.header = msg.header
        raw.height, raw.width = h, w
        raw.encoding = 'bgr8'
        raw.step = w * 3
        raw.data = img.tobytes()
        self.pub_image.publish(raw)
        jpeg = CompressedImage()
        jpeg.header = msg.header
        jpeg.format = 'jpeg'
        jpeg.data = cv2.imencode('.jpg', img)[1].tobytes()
        self.pub_jpeg.publish(jpeg)

    @staticmethod
    def draw(img, pts, f):
        h, w = img.shape[:2]
        P = [(int(x * w), int(y * h)) for x, y, _ in pts]
        for a, b in BONES:
            cv2.line(img, P[a], P[b], (0, 200, 0), 2, cv2.LINE_AA)
        for k in (11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28):
            cv2.circle(img, P[k], 4, (0, 255, 0), -1)
        if f:
            # 腰の中心から、体の向きを上から見た矢印で描く（上 = カメラの方、左右 = 本人の左右を画面の右左に）
            cx, cy = int(f['hip_x']), int(f['hip_y'])
            a = math.radians(f['yaw'])
            ex, ey = int(cx + 50 * math.sin(a)), int(cy + 50 * math.cos(a) * 0.4)
            cv2.arrowedLine(img, (cx, cy), (ex, ey), (0, 140, 255), 3, tipLength=0.3)
            for side, k in (('l_up', 15), ('r_up', 16)):
                if f[side]:
                    cv2.circle(img, P[k], 12, (0, 200, 255), 3)


def main():
    rclpy.init()
    node = PoseDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.landmarker.close()
        node.destroy_node()
        rclpy.try_shutdown()
