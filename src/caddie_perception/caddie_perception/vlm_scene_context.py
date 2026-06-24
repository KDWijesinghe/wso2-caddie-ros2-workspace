import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VlmSceneContext(Node):
    """Lightweight VLM integration point for course scene summaries.

    This node deliberately keeps the transport local and simple. In simulation it
    turns structured detections into a scene summary. A real VLM backend can
    replace _summarize_scene while keeping the same ROS topic contract.
    """

    def __init__(self):
        super().__init__('vlm_scene_context')
        self.declare_parameter('input_topic', '/caddie/ball_detections')
        self.declare_parameter('output_topic', '/caddie/scene_context')
        self.declare_parameter('golfer_reference_frame', 'base_footprint')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self.pub = self.create_publisher(String, output_topic, 10)
        self.create_subscription(String, input_topic, self._detections_cb, 10)
        self.get_logger().info(
            f'VLM scene-context adapter listening on {input_topic}.')

    def _detections_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Ignoring malformed ball detection JSON.')
            return

        summary = self._summarize_scene(payload)
        out = String()
        out.data = json.dumps(summary, sort_keys=True)
        self.pub.publish(out)

    def _summarize_scene(self, payload: dict) -> dict:
        balls = payload.get('balls', [])
        frame_id = payload.get('frame_id', '')
        if not balls:
            return {
                'frame_id': frame_id,
                'summary': 'No golf balls are currently visible.',
                'recommended_command': 'scan course',
                'ball_count': 0,
            }

        ordered = sorted(
            balls,
            key=lambda b: math.hypot(float(b.get('x', 0.0)), float(b.get('y', 0.0))))
        nearest = ordered[0]
        distance = math.hypot(float(nearest['x']), float(nearest['y']))
        side = 'left' if float(nearest['y']) > 0.25 else 'right' if float(nearest['y']) < -0.25 else 'ahead'

        return {
            'frame_id': frame_id,
            'summary': (
                f'{len(balls)} golf ball(s) detected. Nearest is '
                f'{nearest["label"]} {distance:.1f} m {side}.'
            ),
            'recommended_command': f'retrieve {nearest["label"]}',
            'ball_count': len(balls),
            'nearest_ball': nearest,
        }


def main(args=None):
    rclpy.init(args=args)
    node = VlmSceneContext()
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
