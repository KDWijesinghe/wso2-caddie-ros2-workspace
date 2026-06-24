import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from unitree_api.msg import Request


ROBOT_SPORT_API_ID_STOPMOVE = 1003
ROBOT_SPORT_API_ID_STANDUP = 1004
ROBOT_SPORT_API_ID_MOVE = 1008


class UnitreeSportModeBridge(Node):
    """Bridge ROS Twist commands into Unitree Go2 SportMode requests.

    This follows Unitree's official unitree_ros2 SportClient request shape:
    publish unitree_api/msg/Request to /api/sport/request. Move requests use
    api_id=1008 and JSON {"x": vx, "y": vy, "z": vyaw}; StopMove uses 1003.
    """

    def __init__(self):
        super().__init__('unitree_sportmode_bridge')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('sport_request_topic', '/api/sport/request')
        self.declare_parameter('status_topic', '/go2/sportmode_bridge_status')
        self.declare_parameter('max_linear_x', 0.6)
        self.declare_parameter('max_linear_y', 0.2)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('command_timeout', 0.35)
        self.declare_parameter('publish_standup_on_start', False)
        self.declare_parameter('publish_period', 0.05)

        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_linear_y = float(self.get_parameter('max_linear_y').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)

        self.last_cmd = Twist()
        self.last_cmd_time = 0.0
        self.stop_sent = True

        self.req_pub = self.create_publisher(
            Request, str(self.get_parameter('sport_request_topic').value), 10)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            self._cmd_cb,
            10)
        self.create_timer(
            max(0.02, float(self.get_parameter('publish_period').value)),
            self._timer_cb)

        if bool(self.get_parameter('publish_standup_on_start').value):
            self._publish_request(ROBOT_SPORT_API_ID_STANDUP, {})

        self._publish_status(
            'Unitree SportMode bridge ready; /cmd_vel -> /api/sport/request.')

    def _cmd_cb(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()
        self.stop_sent = False

    def _timer_cb(self):
        if time.monotonic() - self.last_cmd_time > self.command_timeout:
            if not self.stop_sent:
                self._publish_request(ROBOT_SPORT_API_ID_STOPMOVE, {})
                self._publish_status('Published Unitree StopMove request.')
                self.stop_sent = True
            return

        vx = self._clamp(self.last_cmd.linear.x, -self.max_linear_x, self.max_linear_x)
        vy = self._clamp(self.last_cmd.linear.y, -self.max_linear_y, self.max_linear_y)
        wz = self._clamp(self.last_cmd.angular.z, -self.max_yaw_rate, self.max_yaw_rate)
        self._publish_request(ROBOT_SPORT_API_ID_MOVE, {'x': vx, 'y': vy, 'z': wz})

    def _publish_request(self, api_id: int, params: dict):
        req = Request()
        req.header.identity.api_id = int(api_id)
        req.parameter = json.dumps(params, separators=(',', ':')) if params else ''
        self.req_pub.publish(req)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, float(value)))


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeSportModeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_request(ROBOT_SPORT_API_ID_STOPMOVE, {})
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
