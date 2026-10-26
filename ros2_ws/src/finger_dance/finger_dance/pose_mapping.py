# 人の体の動き → ロボットの目標の姿勢と速度（人のダンスに合わせて踊る。ROS 2 に依存しない）
#
# 体の 33 点（MediaPipe Pose Landmarker）から 3 つの動きを取り出し、ロボットの目標にする。
#   体の向き（両肩の奥行きの差）＋ 腰の左右の揺れ → ロボットの目標の向き
#   腰の上下のはずみ（しゃがむ・伸びる）       → ロボットの目標の前後の位置（小さく前後に揺れる）
#   手首を肩より上に上げる                     → ロボットの LED
# ロボットは、目標の向き・位置に近づくように前後の速さ v と旋回の速さ ω を決める（比例制御）。
# 向き・位置は目標に「追いつく」ように動くので、ロボットはその場から離れていかない。

import math

# 点の番号（MediaPipe の決まり。左右は本人の左右）
NOSE, L_SH, R_SH, L_WR, R_WR, L_HIP, R_HIP = 0, 11, 12, 15, 16, 23, 24


def _alpha(dt, tau):
    """時定数 tau [s] のなめらかさ（指数平均）の係数。フレームの間隔が変わっても同じ効き方にする"""
    return math.exp(-dt / tau) if tau > 0 else 0.0


def features(pts, world, width, height, min_visibility=0.5):
    """pts：画像上の 33 点 (x, y, visibility)。x, y は 0〜1 の割合。world：33 点の (x, y, z) [m]（腰が原点）。
    width, height：画像の大きさ [画素]。
    戻り値：体の向き yaw [度]（本人から見て左を向くと +）、腰の中心の位置 hip_x・hip_y [画素]、
    胴の長さ torso（肩の中心から腰の中心まで [画素]。揺れの大きさをこれで割って、写り方の大小によらなくする）、
    左右の手首を上げているか。両肩と腰が見えていないときは None を返す。
    """
    if min(pts[k][2] for k in (L_SH, R_SH, L_HIP, R_HIP)) < min_visibility:
        return None
    # 正面を向くと左肩は画像の右に写る。左を向くと左肩が奥（z が大きい）に下がる
    yaw = math.degrees(math.atan2(world[L_SH][2] - world[R_SH][2], world[L_SH][0] - world[R_SH][0]))
    hip_x = (pts[L_HIP][0] + pts[R_HIP][0]) / 2 * width
    hip_y = (pts[L_HIP][1] + pts[R_HIP][1]) / 2 * height
    sh_x = (pts[L_SH][0] + pts[R_SH][0]) / 2 * width
    sh_y = (pts[L_SH][1] + pts[R_SH][1]) / 2 * height
    torso = math.hypot(hip_x - sh_x, hip_y - sh_y)
    if torso < 5:
        return None
    return {
        'yaw': yaw, 'hip_x': hip_x, 'hip_y': hip_y, 'torso': torso,
        'l_up': pts[L_WR][1] < pts[L_SH][1] and pts[L_WR][2] >= min_visibility,
        'r_up': pts[R_WR][1] < pts[R_SH][1] and pts[R_WR][2] >= min_visibility,
    }


class DanceMapper:
    """体の動き → ロボットの目標の向き [度] と目標の前後の位置 [m]"""

    def __init__(self, yaw_gain=0.7, sway_gain=60.0, bounce_gain=0.1, heading_limit=60.0, forward_limit=0.02,
                 yaw_tau=0.08, sway_tau=1.3, bounce_tau=0.8, yaw_clip=90.0):
        self.yaw_gain = yaw_gain            # 体の向き 1° → ロボットの向き何度
        self.sway_gain = sway_gain          # 腰の左右のずれ（胴の長さに対する割合）1 → ロボットの向き何度
        self.bounce_gain = bounce_gain      # 腰の上下のずれ（胴の長さに対する割合）1 → ロボットの前後何 m
        self.heading_limit = heading_limit  # 目標の向きの範囲 ±[度]
        self.forward_limit = forward_limit  # 目標の前後の位置の範囲 ±[m]
        self.yaw_tau, self.sway_tau, self.bounce_tau = yaw_tau, sway_tau, bounce_tau
        self.yaw_clip = yaw_clip            # 背中を向けたときなど、向きの推定が飛ぶのでこの範囲に収める
        self.reset()

    def reset(self):
        self.yaw_s = None      # なめらかにした体の向き
        self.hip_x0 = None     # 腰の左右のゆっくりした平均（揺れの基準）
        self.hip_y0 = None     # 腰の上下のゆっくりした平均（はずみの基準）
        self.torso = None      # なめらかにした胴の長さ

    def update(self, f, dt):
        """f：features() の戻り値。dt：前の画像からの時間 [s]。戻り値：(目標の向き [度], 目標の前後の位置 [m])"""
        yaw = max(-self.yaw_clip, min(self.yaw_clip, f['yaw']))
        if self.yaw_s is None:
            self.yaw_s, self.hip_x0, self.hip_y0, self.torso = yaw, f['hip_x'], f['hip_y'], f['torso']
        a = _alpha(dt, self.yaw_tau)
        self.yaw_s = a * self.yaw_s + (1 - a) * yaw
        a = _alpha(dt, self.sway_tau)
        self.hip_x0 = a * self.hip_x0 + (1 - a) * f['hip_x']
        a = _alpha(dt, self.bounce_tau)
        self.hip_y0 = a * self.hip_y0 + (1 - a) * f['hip_y']
        a = _alpha(dt, self.sway_tau)
        self.torso = a * self.torso + (1 - a) * f['torso']
        # 画像の右へ動く（本人の左へ動く）と + 。しゃがむ（腰が画像の下へ動く）と後ろへ
        sway = (f['hip_x'] - self.hip_x0) / self.torso
        bounce = (f['hip_y'] - self.hip_y0) / self.torso
        heading = self.yaw_gain * self.yaw_s + self.sway_gain * sway
        forward = -self.bounce_gain * bounce
        heading = max(-self.heading_limit, min(self.heading_limit, heading))
        forward = max(-self.forward_limit, min(self.forward_limit, forward))
        return heading, forward


class Follower:
    """目標の向き・位置に近づくように、v と ω を決める。ロボットの向き・位置は、出した指令を足し合わせて推定する"""

    def __init__(self, heading_gain=3.0, forward_gain=4.0, linear_max=0.05, angular_max=1.0):
        self.heading_gain = heading_gain    # 向きのずれ 1 rad → ω [rad/s]
        self.forward_gain = forward_gain    # 位置のずれ 1 m → v [m/s]
        self.linear_max, self.angular_max = linear_max, angular_max
        self.heading = 0.0   # 推定したロボットの向き [度]
        self.forward = 0.0   # 推定したロボットの前後の位置 [m]

    def step(self, heading_target, forward_target, dt):
        w = self.heading_gain * math.radians(heading_target - self.heading)
        v = self.forward_gain * (forward_target - self.forward)
        w = max(-self.angular_max, min(self.angular_max, w))
        v = max(-self.linear_max, min(self.linear_max, v))
        self.heading += math.degrees(w * dt)
        self.forward += v * dt
        return v, w
