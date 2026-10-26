# ======================================================================
# 動画ファイルを /image_raw/compressed に流す（video_source）
#   カメラページの代わりに、ダンスの動画を実際の時間どおりに流す。最後まで行くと最初に戻る（loop）。
#   画像は縦横の比を保ったまま 640 × 480 に収め、余白は黒にする。
# パラメータ：file（動画ファイル）、rate（1 秒に流す枚数、10）、loop（true）
# ======================================================================

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

WIDTH, HEIGHT = 640, 480


class VideoSource(Node):

    def __init__(self):
        super().__init__('video_source')
        self.file = os.path.expanduser(self.declare_parameter('file', '').value)
        self.rate = self.declare_parameter('rate', 10.0).value
        self.loop = self.declare_parameter('loop', True).value
        self.cap = cv2.VideoCapture(self.file)
        if not self.cap.isOpened():
            raise RuntimeError(f'動画を開けません: {self.file}')
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 25.0
        self.frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.pub = self.create_publisher(CompressedImage, '/image_raw/compressed', 1)
        self.start = time.monotonic()
        self.index = -1      # 最後に読んだフレームの番号
        self.frame = None
        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f'動画を流します: {os.path.basename(self.file)}（{self.frames / self.fps:.1f} 秒、{self.fps:.0f} fps）')

    def tick(self):
        # 動画の中の「今」の位置まで読み進める（流す枚数が動画より少なくても、実際の時間どおりに進む）
        want = int((time.monotonic() - self.start) * self.fps)
        while self.index < want:
            ok, img = self.cap.read()
            if not ok:
                if not self.loop:
                    self.get_logger().info('動画が終わりました')
                    self.timer.cancel()
                    return
                self.get_logger().info('動画の最初に戻ります')
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.start = time.monotonic()
                self.index = -1
                want = 0
                continue
            self.index += 1
            self.frame = img
        if self.frame is None:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'video'
        msg.format = 'jpeg'
        msg.data = cv2.imencode('.jpg', self.letterbox(self.frame), [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
        self.pub.publish(msg)

    @staticmethod
    def letterbox(img):
        h, w = img.shape[:2]
        s = min(WIDTH / w, HEIGHT / h)
        small = cv2.resize(img, (int(w * s), int(h * s)))
        out = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        y, x = (HEIGHT - small.shape[0]) // 2, (WIDTH - small.shape[1]) // 2
        out[y:y + small.shape[0], x:x + small.shape[1]] = small
        return out


def main():
    rclpy.init()
    node = VideoSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()
