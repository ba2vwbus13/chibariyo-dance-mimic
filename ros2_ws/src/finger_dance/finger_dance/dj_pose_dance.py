# ======================================================================
# DJダンシングマウス（dj_pose_dance）
#   3大新機能：
#     1. 正面キープ ＆ 斜めステップ（斜め前・斜め後ろ・S字サイドステップ）
#     2. DJブザー機能（動きの激しさ・手の高さ・腰の揺れに合わせてビートとシンセ音が鳴る）
#     3. 安全ブレーキ（光センサで前方の壁・障害物を検知して自動後退＆警告音）
# ======================================================================

import collections
import json
import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Int16, String
from std_srvs.srv import SetBool

from finger_dance.pose_mapping import DanceMapper, Follower

try:
    from raspimouse_msgs.msg import Leds, LightSensors
except ImportError:
    Leds = None
    LightSensors = None

PERIOD = 0.05   # 制御周期 [s] (20Hz)

# ペンタトニックスケール (ド・レ・ミ・ソ・ラ・ド)
PENTATONIC = [262, 294, 330, 392, 440, 523, 659, 784, 880, 1046]


class DJPoseDance(Node):

    def __init__(self):
        super().__init__('dj_pose_dance')
        self.turtlesim = self.declare_parameter('turtlesim', False).value
        sim = self.turtlesim
        p = self.declare_parameter

        # ダンスの目標生成器
        self.mapper = DanceMapper(
            yaw_gain=p('yaw_gain', 0.5).value,
            sway_gain=p('sway_gain', 45.0).value,
            bounce_gain=p('bounce_gain', 5.0 if sim else 0.12).value,
            heading_limit=p('heading_limit', 45.0).value,
            forward_limit=p('forward_limit', 1.0 if sim else 0.04).value,
        )

        # 追従制御器（キレのある動き）
        self.follower = Follower(
            heading_gain=p('heading_gain', 3.5).value,
            forward_gain=p('forward_gain', 4.5).value,
            linear_max=p('linear_max', 1.0 if sim else 0.08).value,
            angular_max=p('angular_max', 2.0 if sim else 1.2).value,
        )

        self.timeout = p('timeout', 0.5).value
        self.delay = p('delay', 0.0).value
        self.stamped = p('stamped', not sim).value
        cmd_topic = p('cmd_topic', '/turtle1/cmd_vel' if sim else '/cmd_vel').value
        self.use_motor_power = p('motor_power', not sim).value
        self.use_leds = p('leds', not sim).value and Leds is not None

        # 安全センサ設定
        self.safety_distance = p('safety_threshold', 1500).value  # 前方光センサしきい値
        self.safety_trigger = False
        self.safety_back_timer = 0.0

        # DJブザー設定
        self.enable_dj_sound = p('enable_sound', True).value
        self.current_note = 0
        self.sound_tick = 0

        # パブリッシャーとサブスクライバー
        self.pub = self.create_publisher(TwistStamped if self.stamped else Twist, cmd_topic, 10)
        self.pub_leds = self.create_publisher(Leds, '/leds', 10) if self.use_leds else None
        self.pub_buzzer = self.create_publisher(Int16, '/buzzer', 10)
        self.sub_pose = self.create_subscription(String, '/dance_pose', self.pose_callback, 10)

        # 光センサ購読（壁検知用）
        if LightSensors is not None:
            self.sub_lights = self.create_subscription(LightSensors, '/light_sensors', self.light_callback, 10)
        else:
            self.sub_lights = None

        self.motor_client = self.create_client(SetBool, '/motor_power')
        self.timer = self.create_timer(PERIOD, self.timer_callback)

        self.targets = collections.deque()
        self.last_time = None
        self.last_pose_time = None
        self.leds = None
        self.state = None
        self.latest_raw_pose = {}

        self.get_logger().info('🎧 DJダンシングマウス起動！ [斜め＆S字ステップ / DJブザー / 安全ブレーキ]')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def light_callback(self, msg):
        """光センサによる前方壁検知"""
        f_max = max(msg.forward_r, msg.forward_l)
        if f_max > self.safety_distance:
            if not self.safety_trigger:
                self.get_logger().warn(f'⚠️ 壁を検知しました！(センサ値: {f_max}) 安全ブレーキを作動して後退します')
            self.safety_trigger = True
            self.safety_back_timer = 0.6  # 0.6秒間後退回避
        elif self.safety_back_timer <= 0:
            self.safety_trigger = False

    def pose_callback(self, msg):
        d = json.loads(msg.data)
        t = self.now()
        dt = 0.1 if self.last_pose_time is None else min(0.3, max(0.01, t - self.last_pose_time))
        self.last_pose_time = t
        if not d.get('ok'):
            return
        self.latest_raw_pose = d
        heading, forward = self.mapper.update(d, dt)

        # 斜めステップ支援：腰が左右に大きく振られたら、自動的に前後のステップを同期して「斜め前/斜め後ろ」にスライド
        sway_ratio = abs(heading) / max(1.0, self.mapper.heading_limit)
        if sway_ratio > 0.4:
            # 腰の振りと前後の上下動を掛け合わせて、斜め方向のスライドステップを強化
            extra_slide = 0.02 * math.copysign(1.0, forward if abs(forward) > 0.005 else 1.0)
            forward += extra_slide

        self.targets.append((t, heading, forward, d.get('l_up', False), d.get('r_up', False)))
        self.last_time = t

    def current_target(self):
        t_want = self.now() - self.delay
        target = None
        while self.targets and self.targets[0][0] <= t_want:
            target = self.targets.popleft()
        if target is not None:
            self.targets.appendleft(target)
            while len(self.targets) > 1 and self.targets[1][0] <= t_want:
                self.targets.popleft()
        return target

    def timer_callback(self):
        lost = self.last_time is None or self.now() - self.last_time > self.timeout + self.delay

        # 安全ブレーキ最優先モード
        if self.safety_back_timer > 0:
            self.safety_back_timer -= PERIOD
            self.send(-0.04, 0.0)  # ゆっくり後退
            self.set_buzzer(1200)   # 警告ビープ音
            self.set_leds(True, True)
            return

        target = None if lost else self.current_target()
        if target is None:
            v, w = 0.0, 0.0
            state = '体が見えない：待機' if lost else '待機中'
            if lost:
                self.mapper.reset()
                self.targets.clear()
                self.set_buzzer(0)
                self.set_leds(False, False)
        else:
            v, w = self.follower.step(target[1], target[2], PERIOD)

            # S字サイドステップ補正（正面キープ機能）：
            # 旋回しすぎた場合は、正面（0°）に戻ろうとする復元トルクを加えることで
            # 「正面を向いたまま左右に平行移動する」カニ歩き風の揺れを演出
            heading_error = self.follower.heading
            restore_w = -0.8 * math.radians(heading_error)
            w += restore_w

            state = '🎵 DJダンシング中！'
            self.set_leds(target[3], target[4])
            self.update_dj_sound(v, w, target[3], target[4], target[1])

        self.send(v, w)
        if state != self.state:
            self.state = state
            self.get_logger().info(state)

    def update_dj_sound(self, v, w, l_up, r_up, heading_target):
        """激しさやポーズに合わせてDJサウンドを生成"""
        if not self.enable_dj_sound:
            return

        self.sound_tick += 1
        # 両手を上げたら「ファンファーレ高音」
        if l_up and r_up:
            freq = 1046  # C6 (High C)
        # 左手を上げたら「高音シンセ」
        elif l_up:
            freq = 784   # G5
        # 右手を上げたら「中音シンセ」
        elif r_up:
            freq = 523   # C5
        else:
            # 動いているときの「テルミン風シンセビート」
            motion_energy = abs(v) + abs(w) * 0.04
            if motion_energy > 0.015:
                # 左右の傾きに合わせてペンタトニックスケールの音を選ぶ
                idx = int((heading_target + self.mapper.heading_limit) / (2 * self.mapper.heading_limit) * (len(PENTATONIC) - 1))
                idx = max(0, min(len(PENTATONIC) - 1, idx))
                # 0.15秒ごとにビートを刻む（歯切れの良いリズム）
                if (self.sound_tick % 3) == 0:
                    freq = PENTATONIC[idx]
                else:
                    freq = 0
            else:
                freq = 0

        self.set_buzzer(freq)

    def set_buzzer(self, hz):
        msg = Int16()
        msg.data = int(hz)
        self.pub_buzzer.publish(msg)

    def set_leds(self, left, right):
        if not self.use_leds or (left, right) == self.leds:
            return
        self.leds = (left, right)
        msg = Leds()
        msg.led0 = msg.led1 = bool(right)
        msg.led2 = msg.led3 = bool(left)
        self.pub_leds.publish(msg)

    def send(self, linear, angular):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            twist = msg.twist
        else:
            msg = Twist()
            twist = msg
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.pub.publish(msg)

    def motor_power(self, on):
        if not self.use_motor_power:
            return
        if not self.motor_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/motor_power が見つかりません')
            return
        future = self.motor_client.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DJPoseDance()
    node.motor_power(True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0)
        node.set_buzzer(0)
        node.set_leds(False, False)
        node.motor_power(False)
        node.destroy_node()
        rclpy.try_shutdown()
