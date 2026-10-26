# 手の形・指の向き → ロボットの速度（前後 linear [m/s]、旋回 angular [rad/s]。angular は + が左回り）
# ROS 2 に依存しない関数だけを置く（単体で試験できるようにするため。チバリヨカーでもそのまま使う）。

import math

STRAIGHT_BAND = 10.0   # 真上・真下から ±この角度 [度] 以内なら旋回しない（まっすぐ前進・後退）
SIDE_BAND = 20.0       # 真横から ±この角度 [度] 以内なら前後に動かない（その場で旋回）


def point_command(angle, linear_max, angular_max, straight_band=STRAIGHT_BAND, side_band=SIDE_BAND):
    """人差し指の向き angle [度]（0 = 真上、+ = 本人から見て左、±180 = 真下）を、
    ジョイスティックのように速度に直す。
      前後 = linear_max × cos(angle)　（真上で前進、真下で後退）
      旋回 = angular_max × sin(angle)（傾けた向きに、傾けた角度に応じた速さで回る。真横で最大）
    """
    a = math.radians(angle)
    linear = linear_max * math.cos(a)
    angular = angular_max * math.sin(a)
    if abs(abs(angle) - 90.0) <= side_band:
        linear = 0.0
    if abs(angle) <= straight_band or abs(angle) >= 180.0 - straight_band:
        angular = 0.0
    return linear, angular


def dance_command(t, angular, period):
    """チョキの踊り：その場で左右に小刻みに振る。t は踊り始めてからの時間 [s]。
    最初の半周期だけ左に回り、その後は period ごとに右・左を入れ替える（向きが元の向きを中心に振れる）。
    """
    left = ((t / period) + 0.5) % 2.0 < 1.0
    return 0.0, (angular if left else -angular)
