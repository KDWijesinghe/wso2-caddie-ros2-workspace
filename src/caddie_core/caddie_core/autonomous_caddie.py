import json
import math
from typing import Any

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from visualization_msgs.msg import Marker

import tf2_ros

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def pose_stamped(x: float, y: float, yaw: float, frame_id: str, stamp) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw * 0.5)
    pose.pose.orientation.w = math.cos(yaw * 0.5)
    return pose


class AutonomousCaddie(Node):
    """Main high-level orchestrator for the simulated caddie."""

    def __init__(self):
        super().__init__('autonomous_caddie')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('status_topic', '/caddie/status')
        self.declare_parameter('ball_detections_topic', '/caddie/ball_detections')
        self.declare_parameter('voice_command_topic', '/caddie/voice_command')
        self.declare_parameter('llm_command_topic', '/caddie/llm_command')
        self.declare_parameter('object_standoff', 0.75)
        self.declare_parameter('goal_tolerance', 0.28)
        self.declare_parameter('fallback_max_linear', 0.35)
        self.declare_parameter('fallback_max_angular', 0.8)
        self.declare_parameter('fallback_enabled', True)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.object_standoff = float(self.get_parameter('object_standoff').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.fallback_max_linear = float(
            self.get_parameter('fallback_max_linear').value)
        self.fallback_max_angular = float(
            self.get_parameter('fallback_max_angular').value)
        self.fallback_enabled = bool(
            self.get_parameter('fallback_enabled').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.target_marker_pub = self.create_publisher(
            Marker, '/caddie/target_marker', 10)

        self.create_subscription(
            String,
            str(self.get_parameter('voice_command_topic').value),
            self._command_cb,
            10)
        self.create_subscription(
            String,
            str(self.get_parameter('llm_command_topic').value),
            self._command_cb,
            10)
        self.create_subscription(
            String,
            str(self.get_parameter('ball_detections_topic').value),
            self._ball_detections_cb,
            10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 20)

        self.nav_client = (
            ActionClient(self, NavigateToPose, 'navigate_to_pose')
            if NavigateToPose is not None else None
        )
        self.active_goal_handle = None

        self.latest_odom: Odometry | None = None
        self.home_pose: tuple[float, float, float] | None = None
        self.mapping_active = False
        self.balls: dict[str, dict[str, Any]] = {}
        self.last_ball_frame = self.map_frame
        self.fallback_target: tuple[float, float, float, str, str] | None = None

        self.course_waypoints = {
            'tee_box': (-15.0, -7.0, 0.20),
            'fairway': (2.0, 0.0, 0.0),
            'green': (14.0, 0.0, 0.0),
            'clubhouse': (-20.0, 11.0, -0.2),
        }

        self.create_timer(0.10, self._fallback_control_timer)
        self.create_timer(2.0, self._heartbeat_timer)

        self._publish_status(
            'Autonomous caddie online. Say or publish: start mapping, '
            'retrieve nearest ball, return home, list balls, analyze shot.')

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def _ball_detections_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Ignoring malformed ball detection JSON.')
            return

        frame_id = payload.get('frame_id', self.map_frame)
        self.last_ball_frame = frame_id
        balls = {}
        for item in payload.get('balls', []):
            label = str(item.get('label', f'ball_{len(balls) + 1}'))
            balls[label] = {
                'label': label,
                'frame_id': frame_id,
                'x': float(item.get('x', 0.0)),
                'y': float(item.get('y', 0.0)),
                'z': float(item.get('z', 0.0)),
                'confidence': float(item.get('confidence', 0.0)),
                'observations': int(item.get('observations', 0)),
            }
        self.balls = balls

    def _command_cb(self, msg: String):
        command = msg.data.strip().lower()
        if not command:
            return

        self.get_logger().info(f'Command received: {command}')
        verb, _, arg = command.partition(' ')

        if verb == 'start_mapping':
            self._start_mapping()
        elif verb == 'stop_mapping':
            self._stop_mapping()
        elif verb == 'retrieve_ball':
            self._retrieve_ball(arg.strip() or 'nearest')
        elif verb == 'return_home':
            self._return_home()
        elif verb == 'go_to_waypoint':
            self._go_to_waypoint(arg.strip())
        elif verb == 'list_balls':
            self._list_balls()
        elif verb == 'analyze_shot':
            self._analyze_shot()
        elif verb == 'follow_golfer':
            self._follow_golfer()
        elif verb == 'stop':
            self._stop_motion('Stopped by command.')
        else:
            self._publish_status(f'Unknown caddie command: {command}')

    def _start_mapping(self):
        pose = self._current_pose(self.map_frame)
        if pose is not None:
            self.home_pose = pose
        elif self.latest_odom is not None:
            odom_pose = self.latest_odom.pose.pose
            self.home_pose = (
                odom_pose.position.x,
                odom_pose.position.y,
                yaw_from_quat(odom_pose.orientation),
            )
        self.mapping_active = True
        self.balls.clear()
        home_text = (
            f' Home recorded at ({self.home_pose[0]:.2f}, '
            f'{self.home_pose[1]:.2f}).'
            if self.home_pose is not None else ''
        )
        self._publish_status(f'Mapping mode started.{home_text}')

    def _stop_mapping(self):
        self.mapping_active = False
        self._publish_status(
            f'Mapping mode stopped. Stored {len(self.balls)} golf ball track(s).')

    def _retrieve_ball(self, target: str):
        ball = self._select_ball(target)
        if ball is None:
            self._publish_status('No matching golf ball is available yet.')
            return

        transformed = self._ball_in_map(ball)
        if transformed is None:
            self._publish_status(
                f'Cannot transform {ball["label"]} into the map frame yet.')
            return

        bx, by, bz = transformed
        goal = self._goal_near_object(bx, by)
        if goal is None:
            self._publish_status('Robot pose is not available for ball retrieval.')
            return

        gx, gy, gyaw = goal
        self._publish_status(
            f'Retrieving {ball["label"]}: ball at ({bx:.2f}, {by:.2f}), '
            f'goal ({gx:.2f}, {gy:.2f}).')
        self._publish_target_marker(gx, gy, bz, ball['label'])
        self._navigate_to(gx, gy, gyaw, self.map_frame, ball['label'])

    def _return_home(self):
        if self.home_pose is None:
            self._publish_status('Home pose is not recorded yet; start mapping first.')
            return
        hx, hy, hyaw = self.home_pose
        self._publish_status(f'Returning home to ({hx:.2f}, {hy:.2f}).')
        self._publish_target_marker(hx, hy, 0.1, 'home')
        self._navigate_to(hx, hy, hyaw, self.map_frame, 'home')

    def _go_to_waypoint(self, waypoint: str):
        if waypoint not in self.course_waypoints:
            self._publish_status(
                f'Unknown waypoint "{waypoint}". Known: {sorted(self.course_waypoints)}')
            return
        x, y, yaw = self.course_waypoints[waypoint]
        self._publish_status(f'Navigating to {waypoint} at ({x:.2f}, {y:.2f}).')
        self._publish_target_marker(x, y, 0.1, waypoint)
        self._navigate_to(x, y, yaw, self.map_frame, waypoint)

    def _list_balls(self):
        if not self.balls:
            self._publish_status('Detected golf balls: none.')
            return
        parts = []
        for label, ball in sorted(self.balls.items()):
            parts.append(
                f'{label}=({ball["x"]:.2f},{ball["y"]:.2f}) '
                f'conf={ball["confidence"]:.2f}')
        self._publish_status('Detected golf balls: ' + '; '.join(parts))

    def _analyze_shot(self):
        if not self.balls:
            self._publish_status(
                'Shot analytics waiting for ball detections.')
            return

        ball = self._select_ball('nearest')
        transformed = self._ball_in_map(ball) if ball is not None else None
        current = self._current_pose(self.map_frame)
        if ball is None or transformed is None or current is None:
            self._publish_status(
                'Shot analytics waiting for map pose and ball transform.')
            return

        bx, by, _ = transformed
        rx, ry, ryaw = current
        distance = math.hypot(bx - rx, by - ry)
        bearing = math.degrees(normalize_angle(math.atan2(by - ry, bx - rx) - ryaw))
        self._publish_status(
            f'Shot analytics: nearest ball {ball["label"]} is {distance:.1f} m '
            f'away at {bearing:+.0f} deg relative bearing. '
            'Use retrieve nearest ball for autonomous pickup approach.')

    def _follow_golfer(self):
        self._publish_status(
            'Follow-golfer mode requested. Simulation hook is ready; add a '
            'person/golfer tracker topic to close the loop.')

    def _select_ball(self, target: str) -> dict[str, Any] | None:
        if not self.balls:
            return None
        target = target.strip().replace(' ', '_')
        if not target or target == 'nearest':
            current = self._current_pose(self.map_frame)
            if current is None:
                return next(iter(sorted(self.balls.values(), key=lambda b: b['label'])))
            rx, ry, _ = current

            def distance_to_robot(ball):
                transformed = self._ball_in_map(ball)
                if transformed is None:
                    return float('inf')
                bx, by, _ = transformed
                return math.hypot(bx - rx, by - ry)

            return min(self.balls.values(), key=distance_to_robot)
        if target in self.balls:
            return self.balls[target]
        if target.isdigit() and f'ball_{target}' in self.balls:
            return self.balls[f'ball_{target}']
        matches = [
            ball for label, ball in self.balls.items()
            if label.startswith(target) or target.startswith(label)
        ]
        if len(matches) == 1:
            return matches[0]
        return None

    def _ball_in_map(self, ball: dict[str, Any]) -> tuple[float, float, float] | None:
        frame_id = ball.get('frame_id', self.map_frame)
        if frame_id == self.map_frame:
            return (ball['x'], ball['y'], ball['z'])

        point = PointStamped()
        point.header.frame_id = frame_id
        point.header.stamp = Time().to_msg()
        point.point.x = float(ball['x'])
        point.point.y = float(ball['y'])
        point.point.z = float(ball['z'])
        try:
            transformed = self.tf_buffer.transform(
                point, self.map_frame, timeout=Duration(seconds=0.2))
            p = transformed.point
            return (float(p.x), float(p.y), float(p.z))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def _goal_near_object(self, obj_x: float, obj_y: float) -> tuple[float, float, float] | None:
        pose = self._current_pose(self.map_frame)
        if pose is None:
            return None
        rx, ry, _ = pose
        dx = obj_x - rx
        dy = obj_y - ry
        distance = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx) if distance > 0.01 else 0.0
        if distance <= self.object_standoff:
            return (rx, ry, yaw)
        scale = (distance - self.object_standoff) / distance
        return (rx + dx * scale, ry + dy * scale, yaw)

    def _current_pose(self, frame_id: str) -> tuple[float, float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id, self.base_frame, Time(), timeout=Duration(seconds=0.15))
            t = transform.transform.translation
            yaw = yaw_from_quat(transform.transform.rotation)
            return (float(t.x), float(t.y), float(yaw))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            if frame_id == 'odom' and self.latest_odom is not None:
                pose = self.latest_odom.pose.pose
                return (
                    float(pose.position.x),
                    float(pose.position.y),
                    yaw_from_quat(pose.orientation),
                )
            return None

    def _navigate_to(self, x: float, y: float, yaw: float, frame_id: str, label: str):
        self.fallback_target = None
        if self.nav_client is not None and self.nav_client.server_is_ready():
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose_stamped(
                x, y, yaw, frame_id, self.get_clock().now().to_msg())
            future = self.nav_client.send_goal_async(goal_msg)
            future.add_done_callback(
                lambda f: self._nav_goal_response_cb(f, label))
            return

        if self.fallback_enabled:
            self._publish_status(
                f'Nav2 action server is not ready; using fallback controller for {label}.')
            self.fallback_target = (x, y, yaw, frame_id, label)
        else:
            self._publish_status(
                f'Nav2 action server is not ready; cannot navigate to {label}.')

    def _nav_goal_response_cb(self, future, label: str):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._publish_status(f'Nav2 rejected goal for {label}.')
            return
        self.active_goal_handle = goal_handle
        self._publish_status(f'Nav2 accepted goal for {label}.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._nav_result_cb(f, label))

    def _nav_result_cb(self, future, label: str):
        result = future.result().result
        self.active_goal_handle = None
        self._publish_status(
            f'Nav2 result for {label}: error_code={getattr(result, "error_code", 0)}.')

    def _fallback_control_timer(self):
        if self.fallback_target is None:
            return

        x, y, yaw, frame_id, label = self.fallback_target
        pose = self._current_pose(frame_id)
        if pose is None:
            self.cmd_pub.publish(Twist())
            return

        rx, ry, ryaw = pose
        dx = x - rx
        dy = y - ry
        distance = math.hypot(dx, dy)
        if distance <= self.goal_tolerance:
            self.cmd_pub.publish(Twist())
            self.fallback_target = None
            self._publish_status(f'Fallback controller reached {label}.')
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(desired_yaw - ryaw)
        heading_scale = max(0.0, 1.0 - abs(yaw_error) / 1.2)

        cmd = Twist()
        cmd.linear.x = min(self.fallback_max_linear, 0.55 * distance) * heading_scale
        if abs(yaw_error) > 0.85:
            cmd.linear.x = 0.0
        cmd.angular.z = max(
            -self.fallback_max_angular,
            min(self.fallback_max_angular, 1.6 * yaw_error),
        )
        self.cmd_pub.publish(cmd)

    def _stop_motion(self, reason: str):
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None
        self.fallback_target = None
        self.cmd_pub.publish(Twist())
        self._publish_status(reason)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _publish_target_marker(self, x: float, y: float, z: float, label: str):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'caddie_target'
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = max(0.04, float(z))
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.36
        marker.color.b = 0.02
        marker.color.a = 0.75
        marker.text = label
        self.target_marker_pub.publish(marker)

    def _heartbeat_timer(self):
        if self.mapping_active:
            self.get_logger().info(
                f'Mapping active; tracking {len(self.balls)} ball(s).',
                throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousCaddie()
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
