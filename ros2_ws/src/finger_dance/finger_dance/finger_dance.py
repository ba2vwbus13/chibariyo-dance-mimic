# ======================================================================
# 指の動きでロボットを踊らせる（finger_dance）
#   /finger_gesture（finger_detect が送る手の形と指の向き）を受け取り、対応表に従って速度の指令を送る。
#     人差し指だけ … 指の向きで走る（上 = 前進、左右に傾ける = その向きに旋回、下 = 後退）
#     チョキ       … その場で左右に小刻みに振る（踊り）
#     グー・パー・その他・手が映らない … 停止
#     /finger_gesture が TIMEOUT 秒届かない（カメラや検出ノードが止まった）… 停止
#   走り出す形（人差し指・チョキ）は CONFIRM 回続けて同じ形のときに切り替え、止まる形はすぐに切り替える。
# 送り先：Raspberry Pi Mouse は /cmd_vel（TwistStamped）、-p turtlesim:=true なら /turtle1/cmd_vel（Twist）
# ======================================================================

import json
import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool

from finger_dance.mapping import SIDE_BAND, STRAIGHT_BAND, dance_command, point_command

MOVING = ('point', 'peace')
MESSAGES = {
    'point': '人差し指：指の向きに走る',
    'peace': 'チョキ：踊る',
    'fist': 'グー：停止',
    'open': 'パー：停止',
    'other': 'その他の形：停止',
    'none': '手が映っていない：停止',
    'lost': '/finger_gesture が届かない：停止',
}


class FingerDance(Node):

    def __init__(self):
        super().__init__('finger_dance')
        self.turtlesim = self.declare_parameter('turtlesim', False).value
        sim = self.turtlesim
        # 速さの上限（実機はタイヤが滑るので 0.05 m/s 以下にする。専攻科実験 R1 と同じ）
        self.linear_max = self.declare_parameter('linear_max', 1.0 if sim else 0.05).value
        self.angular_max = self.declare_parameter('angular_max', 1.5 if sim else 1.0).value
        self.dance_angular = self.declare_parameter('dance_angular', 3.0 if sim else 1.0).value
        self.dance_period = self.declare_parameter('dance_period', 0.4).value
        self.straight_band = self.declare_parameter('straight_band', STRAIGHT_BAND).value
        self.side_band = self.declare_parameter('side_band', SIDE_BAND).value
        self.timeout = self.declare_parameter('timeout', 0.5).value
        self.confirm = self.declare_parameter('confirm', 2).value
        self.smoothing = self.declare_parameter('smoothing', 0.5).value   # 指の向きのなめらかさ（0 = そのまま、1 に近いほどなめらか）
        # チバリヨカーなどに使うとき：-p stamped:=false -p cmd_topic:=/cmd_vel -p motor_power:=false
        self.stamped = self.declare_parameter('stamped', not sim).value
        cmd_topic = self.declare_parameter('cmd_topic', '/turtle1/cmd_vel' if sim else '/cmd_vel').value
        self.use_motor_power = self.declare_parameter('motor_power', not sim).value

        self.pub = self.create_publisher(TwistStamped if self.stamped else Twist, cmd_topic, 10)
        self.sub = self.create_subscription(String, '/finger_gesture', self.gesture_callback, 10)
        self.motor_client = self.create_client(SetBool, '/motor_power')
        self.timer = self.create_timer(0.1, self.timer_callback)   # 0.1秒ごとに指令を送る

        self.shape = 'none'          # 今、従っている手の形
        self.pending, self.count = None, 0   # 走り出す形に切り替える前の確認
        self.vec = None              # 指の向き（なめらかにした単位ベクトル (sin, cos)）
        self.dance_start = 0.0
        self.last_time = None        # 最後に /finger_gesture を受け取った時刻
        self.state = None            # 表示用
        self.get_logger().info(f'送り先 {cmd_topic}（{"TwistStamped" if self.stamped else "Twist"}）、'
                               f'前後 最大 {self.linear_max} m/s、旋回 最大 {self.angular_max} rad/s')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def gesture_callback(self, msg):
        d = json.loads(msg.data)
        shape = d.get('shape', 'none')
        self.last_time = self.now()
        if shape in MOVING:
            if shape == self.shape:
                self.pending, self.count = None, 0
            elif shape == self.pending:
                self.count += 1
            else:
                self.pending, self.count = shape, 1
            if self.pending is not None and self.count >= self.confirm:
                self.set_shape(shape)
        else:
            self.pending, self.count = None, 0
            self.set_shape(shape)   # 止まる形はすぐに従う
        if shape == 'point':
            self.update_angle(d.get('angle', 0.0))

    def set_shape(self, shape):
        if shape == self.shape:
            return
        if shape == 'peace':
            self.dance_start = self.now()
        if shape != 'point':
            self.vec = None
        self.shape = shape
        self.pending, self.count = None, 0

    def update_angle(self, angle):
        # 角度のまま平均すると ±180° のところで飛ぶので、向きの単位ベクトルで平均する
        v = (math.sin(math.radians(angle)), math.cos(math.radians(angle)))
        if self.vec is None:
            self.vec = v
        else:
            k = self.smoothing
            self.vec = (k * self.vec[0] + (1 - k) * v[0], k * self.vec[1] + (1 - k) * v[1])

    def smoothed_angle(self):
        return math.degrees(math.atan2(self.vec[0], self.vec[1])) if self.vec else 0.0

    def timer_callback(self):
        shape = self.shape
        if self.last_time is None or self.now() - self.last_time > self.timeout:
            shape = 'lost'
        linear, angular = 0.0, 0.0
        if shape == 'point' and self.vec is not None:
            linear, angular = point_command(self.smoothed_angle(), self.linear_max, self.angular_max,
                                            self.straight_band, self.side_band)
        elif shape == 'peace':
            linear, angular = dance_command(self.now() - self.dance_start, self.dance_angular, self.dance_period)
        self.send(linear, angular)
        if shape != self.state:   # 形が変わったときだけ表示する
            self.state = shape
            self.get_logger().info(MESSAGES.get(shape, shape))

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
        # Raspberry Pi Mouse のモータの電源を入れる・切る（turtlesim では何もしない）
        if not self.use_motor_power:
            return
        if not self.motor_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/motor_power が見つかりません（ロボット側の raspimouse ノード、robot_connect、ROS_DOMAIN_ID を確認）')
            return
        future = self.motor_client.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl + C のあとでも停止の指令を送れるようにする
    node = FingerDance()
    node.motor_power(True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0)
        node.motor_power(False)
        node.destroy_node()
        rclpy.try_shutdown()
