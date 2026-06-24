import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HiddenWheelJointStatePublisher(Node):
    """Publish TF-only joint states for Gazebo's hidden drive wheels."""

    def __init__(self):
        super().__init__('hidden_wheel_joint_state_publisher')
        self.declare_parameter('publish_rate', 30.0)

        self.joint_names = [
            'left_front_wheel_joint',
            'left_rear_wheel_joint',
            'right_front_wheel_joint',
            'right_rear_wheel_joint',
        ]
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        period = 1.0 / max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(period, self._timer_cb)
        self.get_logger().info('Publishing hidden wheel joint states for RViz TF.')

    def _timer_cb(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HiddenWheelJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
