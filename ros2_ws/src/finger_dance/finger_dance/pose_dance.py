# ======================================================================
# 人のダンスに合わせて踊る（pose_dance）
#   /dance_pose（pose_detect が送る体の動き）から、ロボットの目標の向きと前後の位置を決め（pose_mapping）、
#   それに近づくように速度の指令を送る。手首を肩より上に上げると、その側の LED を点ける。
#     体の向き＋腰の左右の揺れ → ロボットの向き（±60° まで）
#     腰の上下のはずみ         → ロボットの前後の位置（±2 cm まで）
#   体を見失ったとき、/dance_pose が TIMEOUT 秒届かないときは止まる。
#   delay を 0 より大きくすると、その秒数だけ遅れて同じ動きをする（掛け合い）。
# 送り先：Raspberry Pi Mouse は /cmd_vel（TwistStamped）と /leds、-p turtlesim:=true なら /turtle1/cmd_vel（Twist）
# ======================================================================

import collections
import json

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool

from finger_dance.pose_mapping import DanceMapper, Follower

try:
    from raspimouse_msgs.msg import Leds
except ImportError:   # LED の型がない環境（turtlesim だけで使うとき）
    Leds = None

PERIOD = 0.05   # 指令を送る間隔 [s]


class PoseDance(Node):

    def __init__(self):
        super().__init__('pose_dance')
        self.turtlesim = self.declare_parameter('turtlesim', False).value
        sim = self.turtlesim
        p = self.declare_parameter
        self.mapper = DanceMapper(
            yaw_gain=p('yaw_gain', 0.7).value, sway_gain=p('sway_gain', 60.0).value,
            # turtlesim の亀は画面の大きさが約 11 なので、前後の動きを 50 倍にして見えるようにする
            bounce_gain=p('bounce_gain', 5.0 if sim else 0.1).value,
            heading_limit=p('heading_limit', 60.0).value, forward_limit=p('forward_limit', 1.0 if sim else 0.02).value)
        self.follower = Follower(
            heading_gain=p('heading_gain', 3.0).value, forward_gain=p('forward_gain', 4.0).value,
            linear_max=p('linear_max', 1.0 if sim else 0.05).value, angular_max=p('angular_max', 2.0 if sim else 1.0).value)
        self.timeout = p('timeout', 0.5).value
        self.delay = p('delay', 0.0).value
        self.stamped = p('stamped', not sim).value
        cmd_topic = p('cmd_topic', '/turtle1/cmd_vel' if sim else '/cmd_vel').value
        self.use_motor_power = p('motor_power', not sim).value
        self.use_leds = p('leds', not sim).value and Leds is not None

        self.pub = self.create_publisher(TwistStamped if self.stamped else Twist, cmd_topic, 10)
        self.pub_leds = self.create_publisher(Leds, '/leds', 10) if self.use_leds else None
        self.sub = self.create_subscription(String, '/dance_pose', self.pose_callback, 10)
        self.motor_client = self.create_client(SetBool, '/motor_power')
        self.timer = self.create_timer(PERIOD, self.timer_callback)
        self.targets = collections.deque()   # (時刻, 目標の向き, 目標の前後の位置, 左手, 右手)
        self.last_time = None                 # 最後に体を検出した時刻
        self.last_pose_time = None            # 最後に /dance_pose を受け取った時刻
        self.leds = None
        self.state = None
        self.get_logger().info(f'送り先 {cmd_topic}、向き ±{self.mapper.heading_limit:.0f}°、'
                               f'前後 ±{self.mapper.forward_limit:g}、遅れ {self.delay:g} 秒')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def pose_callback(self, msg):
        d = json.loads(msg.data)
        t = self.now()
        dt = 0.1 if self.last_pose_time is None else min(0.3, max(0.01, t - self.last_pose_time))
        self.last_pose_time = t
        if not d.get('ok'):
            return
        heading, forward = self.mapper.update(d, dt)
        self.targets.append((t, heading, forward, d.get('l_up', False), d.get('r_up', False)))
        self.last_time = t

    def current_target(self):
        # delay 秒前の目標を使う（delay = 0 なら最新の目標）
        t_want = self.now() - self.delay
        target = None
        while self.targets and self.targets[0][0] <= t_want:
            target = self.targets.popleft()
        if target is not None:
            self.targets.appendleft(target)   # 次の周期でも使えるよう戻しておく
            while len(self.targets) > 1 and self.targets[1][0] <= t_want:
                self.targets.popleft()
        return target

    def timer_callback(self):
        lost = self.last_time is None or self.now() - self.last_time > self.timeout + self.delay
        target = None if lost else self.current_target()
        if target is None:
            v, w = 0.0, 0.0
            state = '体が見えない：停止' if lost else '待機中'
            if lost:
                self.mapper.reset()
                self.targets.clear()
        else:
            v, w = self.follower.step(target[1], target[2], PERIOD)
            state = '踊っている'
            self.set_leds(target[3], target[4])
        self.send(v, w)
        if state != self.state:
            self.state = state
            self.get_logger().info(state)
        if lost:
            self.set_leds(False, False)

    def set_leds(self, left, right):
        if not self.use_leds or (left, right) == self.leds:
            return
        self.leds = (left, right)
        msg = Leds()
        msg.led0 = msg.led1 = bool(right)   # 右手 → LED 0・1
        msg.led2 = msg.led3 = bool(left)    # 左手 → LED 2・3
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
            self.get_logger().warn('/motor_power が見つかりません（ロボット側の raspimouse ノード、robot_connect、ROS_DOMAIN_ID を確認）')
            return
        future = self.motor_client.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl + C のあとでも停止の指令を送れるようにする
    node = PoseDance()
    node.motor_power(True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0)
        node.set_leds(False, False)
        node.motor_power(False)
        node.destroy_node()
        rclpy.try_shutdown()
