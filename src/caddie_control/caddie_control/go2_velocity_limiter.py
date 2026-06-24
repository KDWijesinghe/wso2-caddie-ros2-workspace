import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class Go2VelocityLimiter(Node):
    """Rate-limited Twist adapter for simulation or Unitree SDK handoff.

    In Gazebo, publish the output topic to the simulated base controller. On
    hardware, this node is the right place to translate the same safe Twist into
    Unitree Go2 SportMode or low-level SDK commands.
    """

    def __init__(self):
        super().__init__('go2_velocity_limiter')
        self.declare_parameter('input_topic', '/cmd_vel_raw')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('gait_status_topic', '/go2/gait_status')
        self.declare_parameter('max_linear', 0.8)
        self.declare_parameter('max_angular', 1.4)
        self.declare_parameter('linear_accel_limit', 0.8)
        self.declare_parameter('angular_accel_limit', 1.8)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('gait_mode', 'trot')

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.linear_accel_limit = float(
            self.get_parameter('linear_accel_limit').value)
        self.angular_accel_limit = float(
            self.get_parameter('angular_accel_limit').value)
        self.command_timeout = float(
            self.get_parameter('command_timeout').value)
        self.gait_mode = str(self.get_parameter('gait_mode').value)

        self.target = Twist()
        self.current = Twist()
        self.last_command_time = 0.0
        self.last_update_time = time.monotonic()

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('output_topic').value), 10)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('gait_status_topic').value), 10)
        self.create_subscription(
            Twist,
            str(self.get_parameter('input_topic').value),
            self._cmd_cb,
            10)
        self.create_timer(0.02, self._timer_cb)
        self.create_timer(1.0, self._status_timer_cb)

        self.get_logger().info(
            'Go2VelocityLimiter ready. Remap Nav2/autonomy output to '
            '/cmd_vel_raw when you want this limiter in the command path.')

    def _cmd_cb(self, msg: Twist):
        self.target.linear.x = self._clamp(msg.linear.x, -self.max_linear, self.max_linear)
        self.target.linear.y = self._clamp(msg.linear.y, -0.2, 0.2)
        self.target.angular.z = self._clamp(
            msg.angular.z, -self.max_angular, self.max_angular)
        self.last_command_time = time.monotonic()

    def _timer_cb(self):
        now = time.monotonic()
        dt = max(0.0, now - self.last_update_time)
        self.last_update_time = now

        if now - self.last_command_time > self.command_timeout:
            target_linear = 0.0
            target_angular = 0.0
        else:
            target_linear = self.target.linear.x
            target_angular = self.target.angular.z

        self.current.linear.x = self._approach(
            self.current.linear.x,
            target_linear,
            self.linear_accel_limit * dt)
        self.current.angular.z = self._approach(
            self.current.angular.z,
            target_angular,
            self.angular_accel_limit * dt)

        out = Twist()
        out.linear.x = self.current.linear.x
        out.linear.y = self.target.linear.y
        out.angular.z = self.current.angular.z
        self.cmd_pub.publish(out)

    def _status_timer_cb(self):
        speed = math.hypot(self.current.linear.x, self.current.linear.y)
        state = 'standing' if speed < 0.02 and abs(self.current.angular.z) < 0.02 else self.gait_mode
        msg = String()
        msg.data = (
            f'gait={state}; vx={self.current.linear.x:.2f}; '
            f'wz={self.current.angular.z:.2f}')
        self.status_pub.publish(msg)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, float(value)))

    @staticmethod
    def _approach(current: float, target: float, max_delta: float) -> float:
        if target > current:
            return min(target, current + max_delta)
        return max(target, current - max_delta)


def main(args=None):
    rclpy.init(args=args)
    node = Go2VelocityLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
