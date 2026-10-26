# 手の21点（MediaPipe Hand Landmarker）から、手の形と人差し指の向きを求める。
# ROS 2 にも MediaPipe にも依存しない関数だけを置く（単体で試験できるようにするため）。
#
# 点の番号（MediaPipe の決まり）
#   0：手首
#   人差し指 5（付け根）・6（第2関節）・8（指先）　中指 9・10・12　薬指 13・14・16　小指 17・18・20
#   親指（1〜4）は、伸ばしているかの判定が向きによって不安定なので使わない。

import math

WRIST = 0
# (付け根 MCP, 第2関節 PIP, 指先 TIP)
FINGERS = {
    'index': (5, 6, 8),
    'middle': (9, 10, 12),
    'ring': (13, 14, 16),
    'pinky': (17, 18, 20),
}
EXT_RATIO = 1.1   # 手首から指先までの距離が、手首から第2関節までの距離のこの倍より長ければ「伸ばしている」

# 伸ばしている指（人差し指, 中指, 薬指, 小指）→ 手の形
SHAPES = {
    (True, False, False, False): 'point',   # 人差し指だけ
    (True, True, False, False): 'peace',    # チョキ
    (False, False, False, False): 'fist',   # グー
    (True, True, True, True): 'open',       # パー
}


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def finger_extended(pts, finger, ratio=EXT_RATIO):
    """pts：21点の (x, y)（画素単位）。手の向きに関係なく判定できるよう、手首からの距離で比べる。"""
    _, pip, tip = FINGERS[finger]
    return _dist(pts[WRIST], pts[tip]) > ratio * _dist(pts[WRIST], pts[pip])


def classify(pts, ratio=EXT_RATIO):
    """手の形を 'point'・'peace'・'fist'・'open'・'other' のどれかで返す。"""
    ext = tuple(finger_extended(pts, f, ratio) for f in ('index', 'middle', 'ring', 'pinky'))
    return SHAPES.get(ext, 'other')


def index_angle(pts, mirror=False):
    """人差し指（付け根 → 指先）の向き [度]。
    0 = 真上、+90 = 本人から見て左、-90 = 本人から見て右、±180 = 真下。
    カメラの映像は左右反転していない（本人の左が画像の右に写る）ものとする。
    映像が鏡のように反転している場合は mirror=True にする。
    """
    mcp, _, tip = FINGERS['index']
    vx = pts[tip][0] - pts[mcp][0]      # 画像の右が +
    vy = pts[tip][1] - pts[mcp][1]      # 画像の下が +
    angle = math.degrees(math.atan2(vx, -vy))   # 画像の右（＝本人の左）が +
    return -angle if mirror else angle
