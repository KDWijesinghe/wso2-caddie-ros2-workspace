import math
import time
from dataclasses import dataclass

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass(frozen=True)
class SurfaceProfile:
    name: str
    speed_scale: float
    stride_scale: float
    lift_scale: float
    crouch: float
    stance_width: float


class Go2GaitAnimator(Node):
    """Animate the simulated Go2 legs from velocity commands.

    Gazebo still uses hidden drive wheels for stable Nav2 odometry. This node
    drives the visible Unitree leg joints through ros2_control so the robot
    looks like it is walking when /cmd_vel is active. The gait is a visual
    layer, but it adapts stride, lift, and stance from odometry/IMU so sand,
    rough, green, and slope zones look different in simulation.
    """

    JOINTS = [
        'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
        'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
        'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    ]

    def __init__(self):
        super().__init__('go2_gait_animator')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('course_origin_x', -15.0)
        self.declare_parameter('course_origin_y', -7.0)
        self.declare_parameter(
            'trajectory_topic', '/go2_leg_controller/joint_trajectory')
        self.declare_parameter('status_topic', '/go2/gait_status')
        self.declare_parameter('publish_rate', 18.0)
        self.declare_parameter('command_timeout', 1.0)
        self.declare_parameter('stand_thigh', 0.78)
        self.declare_parameter('stand_calf', -1.55)
        self.declare_parameter('max_thigh_swing', 0.055)
        self.declare_parameter('max_calf_swing', 0.040)
        self.declare_parameter('max_hip_swing', 0.010)
        self.declare_parameter('min_step_frequency', 0.32)
        self.declare_parameter('max_step_frequency', 0.62)
        self.declare_parameter('command_filter_tau', 0.70)
        self.declare_parameter('gait_blend_tau', 1.00)
        self.declare_parameter('surface_filter_tau', 0.85)
        self.declare_parameter('trajectory_time', 0.60)
        self.declare_parameter('slope_pitch_gain', 0.18)
        self.declare_parameter('slope_roll_gain', 0.14)

        self.cmd = Twist()
        self.last_cmd_time = 0.0
        self.last_update_time = time.monotonic()
        self.last_state = ''
        self.last_surface = ''
        self.phase = 0.0
        self.filtered_linear = 0.0
        self.filtered_lateral = 0.0
        self.filtered_turn = 0.0
        self.gait_blend = 0.0
        self.x = -15.0
        self.y = -7.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.course_origin_x = float(
            self.get_parameter('course_origin_x').value)
        self.course_origin_y = float(
            self.get_parameter('course_origin_y').value)
        self.surface_name = 'tee'
        self.surface_speed_scale = 0.85
        self.surface_stride_scale = 0.80
        self.surface_lift_scale = 0.90
        self.surface_crouch = 0.015
        self.surface_stance_width = 0.005
        self.slope_amount = 0.0

        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.stand_thigh = float(self.get_parameter('stand_thigh').value)
        self.stand_calf = float(self.get_parameter('stand_calf').value)
        self.max_thigh_swing = float(self.get_parameter('max_thigh_swing').value)
        self.max_calf_swing = float(self.get_parameter('max_calf_swing').value)
        self.max_hip_swing = float(self.get_parameter('max_hip_swing').value)
        self.min_step_frequency = float(
            self.get_parameter('min_step_frequency').value)
        self.max_step_frequency = float(
            self.get_parameter('max_step_frequency').value)
        self.command_filter_tau = float(
            self.get_parameter('command_filter_tau').value)
        self.gait_blend_tau = float(self.get_parameter('gait_blend_tau').value)
        self.surface_filter_tau = float(
            self.get_parameter('surface_filter_tau').value)
        self.trajectory_time = float(self.get_parameter('trajectory_time').value)
        self.slope_pitch_gain = float(
            self.get_parameter('slope_pitch_gain').value)
        self.slope_roll_gain = float(self.get_parameter('slope_roll_gain').value)

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter('trajectory_topic').value),
            10)
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10)
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            self._cmd_cb,
            10)

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_cb,
            10)
        self.create_subscription(
            Imu,
            str(self.get_parameter('imu_topic').value),
            self._imu_cb,
            sensor_qos)

        period = 1.0 / max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(period, self._timer_cb)
        self.create_timer(1.0, self._status_timer_cb)
        self.add_on_set_parameters_callback(self._parameters_cb)
        self.get_logger().info(
            'Go2GaitAnimator ready: surface-aware /cmd_vel leg animation.')

    def _cmd_cb(self, msg: Twist):
        self.cmd = msg
        self.last_cmd_time = time.monotonic()

    def _odom_cb(self, msg: Odometry):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        _, _, self.yaw = self._quat_to_rpy(msg.pose.pose.orientation)

    def _imu_cb(self, msg: Imu):
        self.roll, self.pitch, _ = self._quat_to_rpy(msg.orientation)

    def _timer_cb(self):
        now = time.monotonic()
        dt = self._bounded_dt(now)
        active = self._command_active(now)
        self._update_filtered_motion(dt, active)
        self._update_surface_context(dt)
        positions = (
            self._gait_positions(dt)
            if self.gait_blend > 0.02 else
            self._stand_positions()
        )

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.JOINTS

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = self._duration_from_seconds(self.trajectory_time)
        trajectory.points.append(point)
        self.trajectory_pub.publish(trajectory)

    def _command_active(self, now: float) -> bool:
        if now - self.last_cmd_time > self.command_timeout:
            return False
        return (
            abs(self.cmd.linear.x) > 0.03 or
            abs(self.cmd.linear.y) > 0.03 or
            abs(self.cmd.angular.z) > 0.05
        )

    def _stand_positions(self):
        positions = []
        for _, _, side_sign, front_sign in self._leg_sequence():
            hip_bias, thigh_bias, calf_bias = self._stance_biases(
                side_sign, front_sign)
            positions.extend([
                self._clamp(hip_bias, -0.85, 0.85),
                self._clamp(self.stand_thigh + thigh_bias, -0.8, 2.0),
                self._clamp(self.stand_calf + calf_bias, -2.55, -0.9),
            ])
        return positions

    def _update_filtered_motion(self, dt: float, active: bool):
        target_linear = self.cmd.linear.x if active else 0.0
        target_lateral = self.cmd.linear.y if active else 0.0
        target_turn = self.cmd.angular.z if active else 0.0

        cmd_alpha = self._low_pass_alpha(dt, self.command_filter_tau)
        self.filtered_linear += cmd_alpha * (target_linear - self.filtered_linear)
        self.filtered_lateral += cmd_alpha * (target_lateral - self.filtered_lateral)
        self.filtered_turn += cmd_alpha * (target_turn - self.filtered_turn)

        linear = math.hypot(self.filtered_linear, self.filtered_lateral)
        turn = abs(self.filtered_turn)
        target_blend = self._clamp(max(linear / 0.55, turn / 1.1), 0.0, 1.0)
        blend_alpha = self._low_pass_alpha(dt, self.gait_blend_tau)
        self.gait_blend += blend_alpha * (target_blend - self.gait_blend)

    def _update_surface_context(self, dt: float):
        course_x = self.x + self.course_origin_x
        course_y = self.y + self.course_origin_y
        profile = self._surface_profile_for_position(course_x, course_y)
        slope = math.hypot(self.roll, self.pitch)
        slope_blend = self._clamp(slope / 0.22, 0.0, 1.0)

        self.surface_name = (
            'slope' if slope_blend > 0.45 and profile.name == 'fairway'
            else profile.name
        )
        target_speed = min(profile.speed_scale, 1.0 - 0.18 * slope_blend)
        target_stride = min(profile.stride_scale, 1.0 - 0.28 * slope_blend)
        target_lift = max(profile.lift_scale, 1.0 + 0.55 * slope_blend)
        target_crouch = profile.crouch + 0.045 * slope_blend
        target_width = profile.stance_width + 0.020 * slope_blend

        alpha = self._low_pass_alpha(dt, self.surface_filter_tau)
        self.surface_speed_scale += alpha * (target_speed - self.surface_speed_scale)
        self.surface_stride_scale += alpha * (target_stride - self.surface_stride_scale)
        self.surface_lift_scale += alpha * (target_lift - self.surface_lift_scale)
        self.surface_crouch += alpha * (target_crouch - self.surface_crouch)
        self.surface_stance_width += alpha * (
            target_width - self.surface_stance_width)
        self.slope_amount += alpha * (slope_blend - self.slope_amount)

    def _gait_positions(self, dt: float):
        frequency = (
            self.min_step_frequency +
            (self.max_step_frequency - self.min_step_frequency) * self.gait_blend)
        frequency *= self.surface_speed_scale
        self.phase = (self.phase + 2.0 * math.pi * frequency * dt) % (2.0 * math.pi)

        thigh_amp = (
            self.max_thigh_swing * self.gait_blend * self.surface_stride_scale)
        calf_amp = (
            self.max_calf_swing * self.gait_blend * self.surface_stride_scale)
        hip_amp = self.max_hip_swing * self.gait_blend
        turn_bias = self._clamp(self.filtered_turn, -1.0, 1.0) * 0.025
        direction = 1.0 if self.filtered_linear >= 0.0 else -1.0

        positions = []
        for _, diagonal_phase, side_sign, front_sign in self._leg_sequence():
            leg_phase = self.phase + diagonal_phase
            swing = math.sin(leg_phase) * direction
            lift = max(0.0, swing)
            terrain_lift = (
                0.020 + 0.040 * (self.surface_lift_scale - 1.0)
            ) * self.gait_blend * lift
            hip_bias, thigh_bias, calf_bias = self._stance_biases(
                side_sign, front_sign)
            hip = (
                hip_bias +
                side_sign * hip_amp * math.sin(leg_phase + math.pi / 2.0) +
                side_sign * turn_bias)
            thigh = (
                self.stand_thigh + thigh_bias +
                thigh_amp * swing +
                terrain_lift * 0.65)
            calf = (
                self.stand_calf + calf_bias -
                calf_amp * swing -
                terrain_lift)
            positions.extend([
                self._clamp(hip, -0.85, 0.85),
                self._clamp(thigh, -0.8, 2.0),
                self._clamp(calf, -2.55, -0.9),
            ])
        return positions

    def _status_timer_cb(self):
        state = 'walk' if self.gait_blend > 0.05 else 'stand'
        if state == self.last_state and self.surface_name == self.last_surface:
            return
        self.last_state = state
        self.last_surface = self.surface_name
        msg = String()
        msg.data = (
            f'gait={state}; surface={self.surface_name}; '
            f'slope={math.degrees(self.slope_amount * 0.22):.1f}deg; '
            f'blend={self.gait_blend:.2f}; animated_legs=true')
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def _parameters_cb(self, params):
        numeric_params = {
            'command_timeout': 'command_timeout',
            'stand_thigh': 'stand_thigh',
            'stand_calf': 'stand_calf',
            'max_thigh_swing': 'max_thigh_swing',
            'max_calf_swing': 'max_calf_swing',
            'max_hip_swing': 'max_hip_swing',
            'min_step_frequency': 'min_step_frequency',
            'max_step_frequency': 'max_step_frequency',
            'command_filter_tau': 'command_filter_tau',
            'gait_blend_tau': 'gait_blend_tau',
            'surface_filter_tau': 'surface_filter_tau',
            'trajectory_time': 'trajectory_time',
            'slope_pitch_gain': 'slope_pitch_gain',
            'slope_roll_gain': 'slope_roll_gain',
            'course_origin_x': 'course_origin_x',
            'course_origin_y': 'course_origin_y',
        }
        for param in params:
            attr = numeric_params.get(param.name)
            if attr is not None:
                setattr(self, attr, float(param.value))
        return SetParametersResult(successful=True)

    def _stance_biases(self, side_sign: float, front_sign: float):
        pitch = self._clamp(self.pitch, -0.35, 0.35)
        roll = self._clamp(self.roll, -0.35, 0.35)
        pitch_bias = -front_sign * pitch * self.slope_pitch_gain
        roll_bias = side_sign * roll * self.slope_roll_gain
        thigh_bias = self.surface_crouch + pitch_bias + roll_bias
        calf_bias = -0.45 * self.surface_crouch - 0.30 * abs(
            pitch_bias + roll_bias)
        hip_bias = side_sign * (self.surface_stance_width + 0.035 * roll)
        return hip_bias, thigh_bias, calf_bias

    def _surface_profile_for_position(self, x: float, y: float) -> SurfaceProfile:
        if self._in_circle(x, y, 14.0, 0.0, 5.0):
            return SurfaceProfile('green', 0.68, 0.55, 0.80, 0.000, 0.000)
        if self._in_rotated_box(x, y, -15.0, -7.0, 5.5, 3.0, 0.15):
            return SurfaceProfile('tee', 0.85, 0.80, 0.90, 0.015, 0.005)
        if self._in_rotated_box(x, y, 4.0, 7.0, 6.0, 3.5, -0.25):
            return SurfaceProfile('sand', 0.52, 0.55, 1.55, 0.050, 0.020)
        if self._in_rotated_box(x, y, 10.0, -6.0, 5.0, 3.0, 0.30):
            return SurfaceProfile('sand', 0.52, 0.55, 1.55, 0.050, 0.020)
        if self._in_rotated_box(x, y, -5.0, 4.7, 10.0, 2.6, 0.20):
            return SurfaceProfile('rough', 0.68, 0.72, 1.30, 0.030, 0.014)
        if self._in_rotated_box(x, y, -3.5, -1.2, 8.0, 4.0, 0.05):
            return SurfaceProfile('mound', 0.70, 0.66, 1.35, 0.035, 0.018)
        return SurfaceProfile('fairway', 1.00, 1.00, 1.00, 0.000, 0.000)

    @staticmethod
    def _leg_sequence():
        return (
            ('FL', 0.0, 1.0, 1.0),
            ('FR', math.pi, -1.0, 1.0),
            ('RL', math.pi, 1.0, -1.0),
            ('RR', 0.0, -1.0, -1.0),
        )

    def _bounded_dt(self, now: float) -> float:
        dt = now - self.last_update_time
        self.last_update_time = now
        return self._clamp(dt, 0.001, 0.10)

    @staticmethod
    def _quat_to_rpy(q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def _low_pass_alpha(dt: float, tau: float) -> float:
        tau = max(0.001, float(tau))
        return 1.0 - math.exp(-dt / tau)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, float(value)))

    @staticmethod
    def _in_circle(x: float, y: float, cx: float, cy: float, radius: float) -> bool:
        return math.hypot(x - cx, y - cy) <= radius

    @staticmethod
    def _in_rotated_box(
        x: float,
        y: float,
        cx: float,
        cy: float,
        length: float,
        width: float,
        yaw: float,
    ) -> bool:
        dx = x - cx
        dy = y - cy
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        local_x = dx * cos_yaw - dy * sin_yaw
        local_y = dx * sin_yaw + dy * cos_yaw
        return abs(local_x) <= length * 0.5 and abs(local_y) <= width * 0.5

    @staticmethod
    def _duration_from_seconds(value: float) -> Duration:
        value = max(0.05, float(value))
        sec = int(value)
        nanosec = int((value - sec) * 1_000_000_000)
        return Duration(sec=sec, nanosec=nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = Go2GaitAnimator()
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
