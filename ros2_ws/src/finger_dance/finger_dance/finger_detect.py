# ======================================================================
# 指の形と向きの検出（finger_detect）
#   ・/image_raw/compressed（PC のカメラの映像。camera.html から rosbridge 経由で届く）を受け取り、
#     MediaPipe の Hand Landmarker で手の21点を求める
#   ・手の形（point：人差し指だけ、peace：チョキ、fist：グー、open：パー、other：その他、none：手がない）と
#     人差し指の向き（度。0 = 真上、+ = 本人から見て左）を /finger_gesture（std_msgs/msg/String、JSON）で送る
#       例：{"shape": "point", "angle": 23.5, "score": 0.97}
#   ・手の点と判定結果を描いた画像を /finger_image（非圧縮。rqt_image_view で見る）と
#     /finger_image/compressed（JPEG。ros2 bag で記録する）で送る
# パラメータ：model（モデルファイル）、mirror（映像が左右反転しているとき true）、ext_ratio（指を伸ばしている判定の倍率）
# ======================================================================

import json
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

from finger_dance.hand_shape import EXT_RATIO, FINGERS, classify, index_angle

MODEL = '/opt/finger_dance/hand_landmarker.task'   # Dockerfile で入れる
COLORS = {'point': (0, 200, 0), 'peace': (255, 0, 255), 'fist': (0, 0, 255), 'open': (0, 0, 255),
          'other': (128, 128, 128), 'none': (128, 128, 128)}


class FingerDetect(Node):

    def __init__(self):
        super().__init__('finger_detect')
        model = self.declare_parameter('model', MODEL).value
        self.mirror = self.declare_parameter('mirror', False).value
        self.ext_ratio = self.declare_parameter('ext_ratio', EXT_RATIO).value
        options = vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model),
            running_mode=vision.RunningMode.VIDEO, num_hands=1,
            min_hand_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.landmarker = vision.HandLandmarker.create_from_options(options)
        self.last_ts = 0
        # 処理が画像の届く速さに追いつかないとき、古い画像を捨てて最新の1枚だけを処理する（遅れをためない）
        latest_only = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, latest_only)
        self.pub_gesture = self.create_publisher(String, '/finger_gesture', 10)
        self.pub_image = self.create_publisher(CompressedImage, '/finger_image/compressed', 10)
        self.pub_raw = self.create_publisher(Image, '/finger_image', 1)
        self.last_shape = None
        self.get_logger().info('指の検出を開始（/image_raw/compressed を待っています）')

    def image_callback(self, msg):
        t0 = time.time()
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        h, w = img.shape[:2]
        # VIDEO モードは時刻 [ms] が毎回増えている必要がある
        ts = max(int(time.monotonic() * 1000), self.last_ts + 1)
        self.last_ts = ts
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)   # MediaPipe は RGB の順（OpenCV は BGR の順）
        result = self.landmarker.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)

        shape, angle, score = 'none', 0.0, 0.0
        if result.hand_landmarks:
            # x, y は 0〜1 の割合なので画素に直す（縦横の比が 1:1 でないため、距離や角度はこちらで計算する）
            pts = [(p.x * w, p.y * h) for p in result.hand_landmarks[0]]
            shape = classify(pts, self.ext_ratio)
            angle = index_angle(pts, self.mirror)
            score = result.handedness[0][0].score if result.handedness else 0.0
            self.draw(img, pts, shape)
        label = f'{shape} {angle:+.0f}deg' if shape == 'point' else shape
        cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, COLORS[shape], 2)

        self.pub_gesture.publish(String(data=json.dumps(
            {'shape': shape, 'angle': round(angle, 1), 'score': round(score, 2)})))
        if shape != self.last_shape:   # 形が変わったときだけ表示する
            self.get_logger().info(f'手の形: {shape}　処理 {1000 * (time.time() - t0):.0f} ms')
            self.last_shape = shape

        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = cv2.imencode('.jpg', img)[1].tobytes()
        self.pub_image.publish(out)
        # 表示用の非圧縮の画像（PC の中だけで使う。ロボットは受け取らないので Wi-Fi には流れない）
        raw = Image()
        raw.header = msg.header
        raw.height, raw.width = h, w
        raw.encoding = 'bgr8'
        raw.step = w * 3
        raw.data = img.tobytes()
        self.pub_raw.publish(raw)

    @staticmethod
    def draw(img, pts, shape):
        for x, y in pts:
            cv2.circle(img, (int(x), int(y)), 3, (0, 255, 0), -1)
        if shape == 'point':
            # 人差し指の付け根から指先の向きに、指の2倍の長さの矢印を描く
            mcp, _, tip = FINGERS['index']
            x0, y0 = pts[mcp]
            x1, y1 = x0 + 2 * (pts[tip][0] - x0), y0 + 2 * (pts[tip][1] - y0)
            cv2.arrowedLine(img, (int(x0), int(y0)), (int(x1), int(y1)), COLORS['point'], 3, tipLength=0.2)


def main():
    rclpy.init()
    node = FingerDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.landmarker.close()
        node.destroy_node()
        rclpy.try_shutdown()
