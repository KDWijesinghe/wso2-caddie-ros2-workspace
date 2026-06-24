import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan


class SensorFrameNormalizer(Node):
    """Republish Gazebo sensor topics with URDF frame IDs.

    Gazebo Sim often scopes sensor frame IDs by model/link/sensor name. That is
    useful internally, but ROS navigation expects frames that exist in the URDF
    TF tree. This node keeps the message data untouched and normalizes only the
    header frame IDs.
    """

    def __init__(self):
        super().__init__('sensor_frame_normalizer')
        self.declare_parameter('scan_in', '/gz/scan')
        self.declare_parameter('scan_out', '/scan')
        self.declare_parameter('scan_frame', 'lidar_link')
        self.declare_parameter('imu_in', '/gz/imu')
        self.declare_parameter('imu_out', '/imu')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('rgb_in', '/gz/camera/image')
        self.declare_parameter('rgb_out', '/camera/image_raw')
        self.declare_parameter('rgb_frame', 'camera_rgb_optical_frame')
        self.declare_parameter('depth_in', '/gz/camera/depth_image')
        self.declare_parameter('depth_out', '/camera/depth/image_raw')
        self.declare_parameter('depth_frame', 'camera_depth_optical_frame')
        self.declare_parameter('info_in', '/gz/camera/camera_info')
        self.declare_parameter('info_out', '/camera/camera_info')
        self.declare_parameter('info_frame', 'camera_rgb_optical_frame')

        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        self.scan_pub = self.create_publisher(
            LaserScan, str(self.get_parameter('scan_out').value), sensor_qos)
        self.imu_pub = self.create_publisher(
            Imu, str(self.get_parameter('imu_out').value), sensor_qos)
        self.rgb_pub = self.create_publisher(
            Image, str(self.get_parameter('rgb_out').value), sensor_qos)
        self.depth_pub = self.create_publisher(
            Image, str(self.get_parameter('depth_out').value), sensor_qos)
        self.info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter('info_out').value), sensor_qos)

        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_in').value),
            self._scan_cb, sensor_qos)
        self.create_subscription(
            Imu, str(self.get_parameter('imu_in').value),
            self._imu_cb, sensor_qos)
        self.create_subscription(
            Image, str(self.get_parameter('rgb_in').value),
            self._rgb_cb, sensor_qos)
        self.create_subscription(
            Image, str(self.get_parameter('depth_in').value),
            self._depth_cb, sensor_qos)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('info_in').value),
            self._info_cb, sensor_qos)

        self.get_logger().info(
            'SensorFrameNormalizer ready: /gz sensor topics -> ROS URDF frames.')

    def _scan_cb(self, msg: LaserScan):
        msg.header.frame_id = str(self.get_parameter('scan_frame').value)
        self.scan_pub.publish(msg)

    def _imu_cb(self, msg: Imu):
        msg.header.frame_id = str(self.get_parameter('imu_frame').value)
        self.imu_pub.publish(msg)

    def _rgb_cb(self, msg: Image):
        msg.header.frame_id = str(self.get_parameter('rgb_frame').value)
        self.rgb_pub.publish(msg)

    def _depth_cb(self, msg: Image):
        msg.header.frame_id = str(self.get_parameter('depth_frame').value)
        self.depth_pub.publish(msg)

    def _info_cb(self, msg: CameraInfo):
        msg.header.frame_id = str(self.get_parameter('info_frame').value)
        self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFrameNormalizer()
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
