"""色領域の切り抜きを真値ラベル付きで保存する収集ノード(一時利用)。
本番reidノードを継承し、bbox_histogram に渡った領域を、そのフレームでの本人/通行人の
真の相対位置と照合して person / person2 / ? のラベルを付ける。走行は本番ノードに任せる"""
import json, math, os, sys, cv2, numpy as np, rclpy
from collections import deque
from nav_msgs.msg import Odometry
from wheelchair_follower.yolo_reid_follower_node import YoloReidFollowerNode

OUT = sys.argv[1]

class Collect(YoloReidFollowerNode):
    def __init__(self):
        super().__init__()
        os.makedirs(OUT, exist_ok=True)
        self.log = open(f'{OUT}/log.jsonl', 'w')
        # 真値は推論中に受信が詰まるため、全件を書き出してラベル付けはオフラインで行う
        self.gtlog = open(f'{OUT}/gt.jsonl', 'w')
        self.gt = {}            # name -> deque of (stamp_ns, (x, y, yaw))
        self.img_stamp = None
        self.k = 0
        for name, topic in [('robot', '/wheelchair/ground_truth'), ('person', '/person/odom'), ('person2', '/person2/odom')]:
            self.create_subscription(Odometry, topic, lambda m, n=name: self.gt_cb(n, m), 10)
    def gt_cb(self, name, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        t = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        self.gt.setdefault(name, deque(maxlen=200)).append((t, (p.x, p.y, yaw)))
        self.gtlog.write(json.dumps([name, t, round(p.x, 4), round(p.y, 4), round(yaw, 4)]) + '\n')
    def gt_at(self, name):
        # 画像と同じ時刻の真値(推論中に車椅子が旋回しても方位がずれないように)
        return min(self.gt[name], key=lambda e: abs(e[0] - self.img_stamp))[1]
    def rgb_callback(self, msg):
        self.img_stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        super().rgb_callback(msg)
    def rel(self, name):
        rx, ry, yaw = self.gt_at('robot')
        cx, cy = rx + 0.255*math.cos(yaw), ry + 0.255*math.sin(yaw)
        x, y, _ = self.gt_at(name)
        b = math.atan2(y - cy, x - cx) - yaw
        return math.hypot(x - cx, y - cy), math.atan2(math.sin(b), math.cos(b))
    def bbox_histogram(self, bgr, x0, y0, x1, y1, dist):
        h = super().bbox_histogram(bgr, x0, y0, x1, y1, dist)
        if h is None or len(self.gt) < 3 or self.img_stamp is None:
            return h
        w = bgr.shape[1]
        bearing = (0.5 - (x0 + x1) / 2 / w) * self.get_parameter('hfov').value
        label = '?'
        for n in ('person', 'person2'):
            d, b = self.rel(n)
            if abs(b - bearing) < 0.12 and abs(d - 0.15 - dist) < 0.5:
                label = n if label == '?' else 'both'
        f = (w / 2) / math.tan(self.get_parameter('hfov').value / 2)
        hc = self.get_parameter('camera_height').value
        vt = int(max(0, y0, bgr.shape[0]/2 - f*(1.35-hc)/dist)); vb = int(min(bgr.shape[0], y1, bgr.shape[0]/2 - f*(0.95-hc)/dist))
        bw = x1 - x0; rx0 = int(max(0, x0 + 0.2*bw)); rx1 = int(min(w, x1 - 0.2*bw))
        if vb - vt < 4 or rx1 - rx0 < 4:
            return h
        self.k += 1
        name = f'{self.k:05d}'
        cv2.imwrite(f'{OUT}/{name}.png', bgr[vt:vb, rx0:rx1])
        self.log.write(json.dumps({'name': name, 'label': label, 'dist': round(dist, 2), 'bearing': round(bearing, 3),
                                   'box': [int(x0), int(y0), int(x1), int(y1)],
                                   'gt_person': [round(v, 3) for v in self.rel('person')],
                                   'gt_person2': [round(v, 3) for v in self.rel('person2')],
                                   'stamp_img': self.img_stamp, 'stamp_gt': self.gt['robot'][-1][0],
                                   'now': self.get_clock().now().nanoseconds,
                                   'yaw_img': round(self.gt_at('robot')[2], 3),
                                   'yaw_now': round(self.gt['robot'][-1][1][2], 3)}) + '\n')
        self.log.flush()
        return h

rclpy.init(args=sys.argv)
n = Collect()
try:
    rclpy.spin(n)
except Exception:
    pass
