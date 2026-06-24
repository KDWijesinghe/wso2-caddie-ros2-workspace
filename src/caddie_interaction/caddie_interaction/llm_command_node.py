import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from caddie_interaction.command_utils import canonicalize_command, normalize_text


class LlmCommandNode(Node):
    """Conversational command router with a deterministic local fallback.

    The node accepts free-form text on /caddie/conversation_text and publishes
    canonical commands on /caddie/llm_command. If a cloud LLM is added later,
    keep this ROS contract and replace _classify_locally with the API call.
    """

    def __init__(self):
        super().__init__('llm_command_node')
        self.declare_parameter('input_topic', '/caddie/conversation_text')
        self.declare_parameter('scene_context_topic', '/caddie/scene_context')
        self.declare_parameter('output_topic', '/caddie/llm_command')
        self.declare_parameter('enable_cloud_llm', False)

        self.scene_context = {}
        self.output_pub = self.create_publisher(
            String, str(self.get_parameter('output_topic').value), 10)
        self.create_subscription(
            String,
            str(self.get_parameter('input_topic').value),
            self._text_cb,
            10)
        self.create_subscription(
            String,
            str(self.get_parameter('scene_context_topic').value),
            self._scene_cb,
            10)

        self.get_logger().info(
            'LLM command router ready. Publish natural language to '
            '/caddie/conversation_text.')

    def _scene_cb(self, msg: String):
        try:
            self.scene_context = json.loads(msg.data)
        except json.JSONDecodeError:
            self.scene_context = {}

    def _text_cb(self, msg: String):
        text = normalize_text(msg.data)
        if not text:
            return

        command = self._classify(text)
        if command is None:
            self.get_logger().info(f'No command matched conversation: "{text}"')
            return

        out = String()
        out.data = command
        self.output_pub.publish(out)
        self.get_logger().info(f'Conversation "{text}" -> {command}')

    def _classify(self, text: str) -> str | None:
        if bool(self.get_parameter('enable_cloud_llm').value):
            command = self._classify_with_optional_cloud(text)
            if command is not None:
                return command
        return self._classify_locally(text)

    def _classify_with_optional_cloud(self, text: str) -> str | None:
        api_key = os.environ.get('OPENAI_API_KEY', '')
        if not api_key:
            self.get_logger().warn(
                'enable_cloud_llm is true but OPENAI_API_KEY is not set; '
                'using local command rules.',
                throttle_duration_sec=10.0)
            return None
        self.get_logger().warn(
            'Cloud LLM hook is intentionally left as a project integration '
            'point; using local command rules.',
            throttle_duration_sec=10.0)
        return None

    def _classify_locally(self, text: str) -> str | None:
        direct = canonicalize_command(text)
        if direct is not None:
            return direct

        if any(word in text for word in ('recommend', 'suggest', 'what should')):
            recommended = self.scene_context.get('recommended_command')
            if isinstance(recommended, str):
                return canonicalize_command(recommended)

        if 'how many' in text and 'ball' in text:
            return 'list_balls'
        if 'where' in text and 'ball' in text:
            return 'list_balls'
        if 'lost ball' in text:
            return 'retrieve_ball nearest'
        if 'carry' in text or 'bag' in text:
            return 'follow_golfer'

        return None


def main(args=None):
    rclpy.init(args=args)
    node = LlmCommandNode()
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
