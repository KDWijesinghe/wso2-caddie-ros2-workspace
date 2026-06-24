import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Go2GaitAnimator(Node):
    """Animate the simulated Go2 legs from velocity commands.

    Gazebo still uses hidden drive wheels for stable Nav2 odometry. This node
    drives the visible Unitree leg joints through ros2_control so the robot
    looks like it is walking when /cmd_vel is active.
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
        self.declare_parameter(
            'trajectory_topic', '/go2_leg_controller/joint_trajectory')
        self.declare_parameter('status_topic', '/go2/gait_status')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('command_timeout', 0.8)
        self.declare_parameter('stand_thigh', 0.78)
        self.declare_parameter('stand_calf', -1.55)
        self.declare_parameter('max_thigh_swing', 0.07)
        self.declare_parameter('max_calf_swing', 0.055)
        self.declare_parameter('max_hip_swing', 0.012)
        self.declare_parameter('min_step_frequency', 0.45)
        self.declare_parameter('max_step_frequency', 0.85)
        self.declare_parameter('command_filter_tau', 0.50)
        self.declare_parameter('gait_blend_tau', 0.75)
        self.declare_parameter('trajectory_time', 0.45)

        self.cmd = Twist()
        self.last_cmd_time = 0.0
        self.last_update_time = time.monotonic()
        self.last_state = ''
        self.phase = 0.0
        self.filtered_linear = 0.0
        self.filtered_lateral = 0.0
        self.filtered_turn = 0.0
        self.gait_blend = 0.0

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
        self.trajectory_time = float(self.get_parameter('trajectory_time').value)

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

        period = 1.0 / max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(period, self._timer_cb)
        self.create_timer(1.0, self._status_timer_cb)
        self.add_on_set_parameters_callback(self._parameters_cb)
        self.get_logger().info(
            'Go2GaitAnimator ready: /cmd_vel -> /go2_leg_controller/joint_trajectory.')

    def _cmd_cb(self, msg: Twist):
        self.cmd = msg
        self.last_cmd_time = time.monotonic()

    def _timer_cb(self):
        now = time.monotonic()
        dt = self._bounded_dt(now)
        active = self._command_active(now)
        self._update_filtered_motion(dt, active)
        positions = self._gait_positions(dt) if self.gait_blend > 0.02 else self._stand_positions()

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
        return [
            0.0, self.stand_thigh, self.stand_calf,
            0.0, self.stand_thigh, self.stand_calf,
            0.0, self.stand_thigh, self.stand_calf,
            0.0, self.stand_thigh, self.stand_calf,
        ]

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

    def _gait_positions(self, dt: float):
        frequency = (
            self.min_step_frequency +
            (self.max_step_frequency - self.min_step_frequency) * self.gait_blend)
        self.phase = (self.phase + 2.0 * math.pi * frequency * dt) % (2.0 * math.pi)

        thigh_amp = self.max_thigh_swing * self.gait_blend
        calf_amp = self.max_calf_swing * self.gait_blend
        hip_amp = self.max_hip_swing * self.gait_blend
        turn_bias = self._clamp(self.filtered_turn, -1.0, 1.0) * 0.025

        positions = []
        for leg_name, diagonal_phase, side_sign in (
            ('FL', 0.0, 1.0),
            ('FR', math.pi, -1.0),
            ('RL', math.pi, 1.0),
            ('RR', 0.0, -1.0),
        ):
            leg_phase = self.phase + diagonal_phase
            swing = math.sin(leg_phase)
            lift = max(0.0, swing)
            hip = side_sign * hip_amp * math.sin(leg_phase + math.pi / 2.0) + side_sign * turn_bias
            thigh = self.stand_thigh + thigh_amp * swing
            calf = self.stand_calf - calf_amp * swing - 0.02 * self.gait_blend * lift
            positions.extend([
                self._clamp(hip, -0.85, 0.85),
                self._clamp(thigh, -0.8, 2.0),
                self._clamp(calf, -2.55, -0.9),
            ])
        return positions

    def _status_timer_cb(self):
        state = 'walk' if self.gait_blend > 0.05 else 'stand'
        if state == self.last_state:
            return
        self.last_state = state
        msg = String()
        msg.data = f'gait={state}; blend={self.gait_blend:.2f}; animated_legs=true'
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
            'trajectory_time': 'trajectory_time',
        }
        for param in params:
            attr = numeric_params.get(param.name)
            if attr is not None:
                setattr(self, attr, float(param.value))
        return SetParametersResult(successful=True)

    def _bounded_dt(self, now: float) -> float:
        dt = now - self.last_update_time
        self.last_update_time = now
        return self._clamp(dt, 0.001, 0.10)

    @staticmethod
    def _low_pass_alpha(dt: float, tau: float) -> float:
        tau = max(0.001, float(tau))
        return 1.0 - math.exp(-dt / tau)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, float(value)))

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
