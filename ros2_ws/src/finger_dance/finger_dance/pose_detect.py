# ======================================================================
# 体の検出（pose_detect） - 軽量・高速化版
#   /image_raw/compressed（カメラページ、または video_source の動画）を受け取り、
#   MediaPipe の Pose Landmarker (Liteモデル) で体の 33 点を求め、踊りに使う動きを /dance_pose（String、JSON）で送る。
#   体の点と体の向きを描いた画像を /pose_image と /pose_image/compressed で送る。
# ======================================================================

import json
import math
import os
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

FULL_MODEL_OPT = '/opt/finger_dance/pose_landmarker_full.task'
LITE_MODEL_WS = '/home/ubuntu/ros2_ws/models/pose_landmarker_lite.task'
LITE_MODEL_OPT = '/opt/finger_dance/pose_landmarker_lite.task'
MODEL = FULL_MODEL_OPT if os.path.exists(FULL_MODEL_OPT) else (LITE_MODEL_WS if os.path.exists(LITE_MODEL_WS) else LITE_MODEL_OPT)

# 骨組みの線（肩・腕・胴・脚・足先）
BONES = [(11, 12), (11, 13), (13, 15), (12, 14), (14, 16), (11, 23), (12, 24), (23, 24),
         (23, 25), (25, 27), (27, 29), (27, 31), (29, 31),
         (24, 26), (26, 28), (28, 30), (28, 32), (30, 32)]


class PoseDetect(Node):

    def __init__(self):
        super().__init__('pose_detect')
        model = self.declare_parameter('model', MODEL).value
        if not os.path.exists(model):
            self.get_logger().warn(f'指定されたモデル {model} が見つからないため、軽量モデル {MODEL} を使用します')
            model = MODEL
        options = vision.PoseLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.25,
            min_pose_presence_confidence=0.25,
            min_tracking_confidence=0.25)
        self.landmarker = vision.PoseLandmarker.create_from_options(options)
        self.last_ts = 0
        self.last_preview_time = 0.0
        self.is_processing = False
        latest_only = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, latest_only)
        self.pub_pose = self.create_publisher(String, '/dance_pose', 10)
        self.pub_image = self.create_publisher(Image, '/pose_image', 1)
        self.pub_jpeg = self.create_publisher(CompressedImage, '/pose_image/compressed', 1)
        self.last_ok = None
        self.get_logger().info(f'体の検出を開始（使用モデル: {os.path.basename(model)}）')

    def image_callback(self, msg):
        if self.is_processing:
            return
        self.is_processing = True
        try:
            t0 = time.time()
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                return
            h, w = img.shape[:2]

            # 推論を高速化するため、画像が大きい場合は 320x240 に縮小してから MediaPipe に入力する
            if w > 320:
                infer_img = cv2.resize(img, (320, int(h * 320 / w)), interpolation=cv2.INTER_NEAREST)
            else:
                infer_img = img

            ts = max(int(time.monotonic() * 1000), self.last_ts + 1)
            self.last_ts = ts
            res = self.landmarker.detect_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(infer_img, cv2.COLOR_BGR2RGB)), ts)
            f = None
            if res.pose_landmarks:
                pts = [(p.x, p.y, p.visibility) for p in res.pose_landmarks[0]]
                f = features(pts, [(p.x, p.y, p.z) for p in res.pose_world_landmarks[0]], w, h)

            out = {'ok': f is not None}
            if f:
                out.update({k: (round(v, 4) if isinstance(v, float) else v) for k, v in f.items()})
            self.pub_pose.publish(String(data=json.dumps(out)))

            if (f is not None) != self.last_ok:
                self.get_logger().info(f'体を{"検出" if f else "見失った"}　処理 {1000 * (time.time() - t0):.0f} ms')
                self.last_ok = f is not None

            # プレビュー画像の送信（負荷軽減のため最大約 8 fps に間引き、解像度とJPEG品質を軽量化）
            now = time.monotonic()
            need_image = self.pub_image.get_subscription_count() > 0
            need_jpeg = self.pub_jpeg.get_subscription_count() > 0 and (now - self.last_preview_time >= 0.12)
            if need_image or need_jpeg:
                self.last_preview_time = now
                if res.pose_landmarks and f is not None:
                    self.draw(img, pts, f)
                label = f'yaw {f["yaw"]:+.0f}deg' if f else 'no body'
                cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 0) if f else (0, 0, 255), 2)
                if f:
                    badges = []
                    if f.get('squat'): badges.append("SQUAT")
                    elif f.get('r_knee_lift'): badges.append("R_LIFT")
                    elif f.get('l_knee_lift'): badges.append("L_LIFT")
                    elif f.get('r_step_fwd'): badges.append("R_STEP")
                    elif f.get('l_step_back'): badges.append("L_STEP")
                    if abs(f.get('yaw', 0.0)) > 110: badges.append("HALF_TURN")
                    if badges:
                        cv2.putText(img, " | ".join(badges), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
                if need_image:
                    raw = Image()
                    raw.header = msg.header
                    raw.height, raw.width = h, w
                    raw.encoding = 'bgr8'
                    raw.step = w * 3
                    raw.data = img.tobytes()
                    self.pub_image.publish(raw)
                if need_jpeg:
                    prev_img = cv2.resize(img, (320, int(h * 320 / w)), interpolation=cv2.INTER_NEAREST) if w > 320 else img
                    jpeg = CompressedImage()
                    jpeg.header = msg.header
                    jpeg.format = 'jpeg'
                    jpeg.data = cv2.imencode('.jpg', prev_img, [int(cv2.IMWRITE_JPEG_QUALITY), 60])[1].tobytes()
                    self.pub_jpeg.publish(jpeg)
        finally:
            self.is_processing = False

    @staticmethod
    def draw(img, pts, f):
        h, w = img.shape[:2]
        P = [(int(x * w), int(y * h)) for x, y, _ in pts]
        for a, b in BONES:
            if pts[a][2] >= 0.2 and pts[b][2] >= 0.2:
                cv2.line(img, P[a], P[b], (0, 200, 0), 2, cv2.LINE_AA)
        for k in (11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32):
            if pts[k][2] >= 0.2:
                cv2.circle(img, P[k], 4, (0, 255, 0), -1)
        if f:
            cx, cy = int(max(10, min(w - 10, f['hip_x']))), int(max(10, min(h - 10, f['hip_y'])))
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
