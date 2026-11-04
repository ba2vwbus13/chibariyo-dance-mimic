"""骨格(MediaPipe Pose の 33 点)から、人の方向と距離を出す計算。

ROS に依存しない。Gazebo の車いすでも、Pi Mouse や WHILL の実機でも同じものを使う。

  方向: 腰の中心(見えなければ肩の中心)の横位置を、ピンホールカメラの式で角度にする
  距離: 体の大きさから出す。距離 = 焦点距離[px] × 体の実寸[m] / 画像上の長さ[px]
        - 胴の長さ(肩の中心〜腰の中心)は、人が横を向いても変わらない
        - 肩幅は、人が横を向くと縮む(=遠く見える)
        両方取れたときは近い方を使う(遠めに誤ると速く寄りすぎるので、安全側)

体の実寸は人によって違うので、実機では calibrate() で測り直す
(人に既知の距離に立ってもらい、そのときの画像上の長さから実寸を逆算する)。
"""
import math
from dataclasses import dataclass

# MediaPipe Pose の点の番号
L_SHOULDER, R_SHOULDER, L_HIP, R_HIP = 11, 12, 23, 24


@dataclass
class BodyMeasure:
    bearing: float          # 方位角 [rad] (左が正)
    distance: float         # 距離 [m]
    cx: float               # 体の中心の横位置 [px]
    torso_px: float = None  # 胴の長さ [px]
    shoulder_px: float = None  # 肩幅 [px]


def focal_px(width, hfov):
    """水平画角から焦点距離[px]を出す(正方画素を仮定)"""
    return (width / 2.0) / math.tan(hfov / 2.0)


def _visible(lm, i, min_vis):
    return lm[i].visibility >= min_vis


def measure(lm, width, height, hfov, torso_m, shoulder_m, min_vis=0.5):
    """1 人分の点(lm[i].x, .y は 0〜1、.visibility)から方向と距離を出す。

    横向きで奥の肩や腰が隠れていても、見えている側の肩と腰で胴の長さを測る。
    肩が 1 つも見えないとき、または胴も肩幅も測れないときは None。
    """
    def center(ids):
        ids = [i for i in ids if _visible(lm, i, min_vis)]
        if not ids:
            return None
        return (sum(lm[i].x for i in ids) / len(ids) * width,
                sum(lm[i].y for i in ids) / len(ids) * height)

    s = center((L_SHOULDER, R_SHOULDER))
    if s is None:
        return None
    h = center((L_HIP, R_HIP))
    f = focal_px(width, hfov)

    shoulder_px = None
    if _visible(lm, L_SHOULDER, min_vis) and _visible(lm, R_SHOULDER, min_vis):
        shoulder_px = math.hypot((lm[L_SHOULDER].x - lm[R_SHOULDER].x) * width,
                                 (lm[L_SHOULDER].y - lm[R_SHOULDER].y) * height)
    torso_px = math.hypot(h[0] - s[0], h[1] - s[1]) if h else None
    cx = h[0] if h else s[0]

    dists = []
    if torso_px and torso_px > 5:
        dists.append(f * torso_m / torso_px)
    if shoulder_px and shoulder_px > 5:
        dists.append(f * shoulder_m / shoulder_px)
    if not dists:
        return None
    bearing = math.atan2(width / 2.0 - cx, f)
    return BodyMeasure(bearing, min(dists), cx, torso_px, shoulder_px)


def calibrate(torso_px, shoulder_px, known_distance, width, hfov):
    """既知の距離に立った人の画像上の長さから、体の実寸 (torso_m, shoulder_m) を逆算する"""
    f = focal_px(width, hfov)
    return (torso_px * known_distance / f if torso_px else None,
            shoulder_px * known_distance / f if shoulder_px else None)


class TargetTracker:
    """複数人が映ったとき、追っている人(ロックオンした人)を選ぶ。

    ロック前: いちばん近い人(lock_max_dist 以内)にロックオン
    ロック後: 前回の位置(距離・方位)にいちばん近い人。gate を超えて離れていたら「いない」
    見失っても前回の位置は覚えておき、離れていた時間に応じて gate を広げて探す
    relock_after 秒以上見失ったら、探して旋回した後なので前回の位置は当てにならない。
    ロックを外し、いちばん近い人(relock_max_dist 以内)にロックオンし直す。
    探している間に人は歩いて離れるので、最初のロックオンより遠くまで許す
    """

    def __init__(self, lock_max_dist=4.0, gate_m=0.8, gate_growth=0.5, gate_max=2.5,
                 relock_after=5.0, relock_max_dist=6.0):
        self.lock_max_dist = lock_max_dist
        self.relock_max_dist = relock_max_dist
        self.ever_locked = False
        self.gate_m = gate_m
        self.gate_growth = gate_growth    # 見失っていた 1 秒ごとに広げる幅 [m]
        self.gate_max = gate_max
        self.relock_after = relock_after
        self.last = None                  # 前回の (distance, bearing)

    @staticmethod
    def _xy(d, b):
        return d * math.cos(b), d * math.sin(b)

    def select(self, measures, t_lost=0.0):
        """measures: BodyMeasure のリスト。追う人の BodyMeasure か None を返す"""
        if not measures:
            return None
        if t_lost > self.relock_after:
            self.last = None
        if self.last is None:
            best = min(measures, key=lambda m: m.distance)
            limit = self.relock_max_dist if self.ever_locked else self.lock_max_dist
            if best.distance > limit:
                return None
        else:
            lx, ly = self._xy(*self.last)
            gate = min(self.gate_max, self.gate_m + self.gate_growth * t_lost)

            def gap(m):
                x, y = self._xy(m.distance, m.bearing)
                return math.hypot(x - lx, y - ly)
            best = min(measures, key=gap)
            if gap(best) > gate:
                return None
        self.last = (best.distance, best.bearing)
        self.ever_locked = True
        return best


def follow_cmd(distance, bearing, target_distance=1.2, stop_distance=0.8,
               k_linear=0.8, k_angular=1.8, max_linear=0.7, max_angular=1.2):
    """距離と方位から (前進速度, 旋回速度) を出す P 制御(他の追従方式と同じ式)"""
    w = max(-max_angular, min(max_angular, k_angular * bearing))
    v = 0.0
    if distance > stop_distance:
        v = k_linear * (distance - target_distance) * max(0.0, math.cos(bearing))
        v = max(0.0, min(max_linear, v))
    return v, w
