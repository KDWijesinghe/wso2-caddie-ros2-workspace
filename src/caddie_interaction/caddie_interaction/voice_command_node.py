import json
import os
import queue
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from caddie_interaction.command_utils import canonicalize_command, normalize_text

try:
    import sounddevice as sd
except ImportError:
    sd = None

try:
    from vosk import KaldiRecognizer, Model
except ImportError:
    KaldiRecognizer = None
    Model = None


class VoiceCommandNode(Node):
    """Offline Vosk voice pipeline adapted for caddie commands."""

    def __init__(self):
        super().__init__('voice_command_node')
        self.declare_parameter('model_path', os.environ.get('VOSK_MODEL_PATH', ''))
        self.declare_parameter('enable_microphone', True)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('block_size', 4000)
        self.declare_parameter('device', os.environ.get('VOICE_INPUT_DEVICE', ''))
        self.declare_parameter('command_topic', '/caddie/voice_command')
        self.declare_parameter('transcript_topic', '/caddie/transcript')
        self.declare_parameter('text_command_topic', '/caddie/text_command')

        self.command_pub = self.create_publisher(
            String, str(self.get_parameter('command_topic').value), 10)
        self.transcript_pub = self.create_publisher(
            String, str(self.get_parameter('transcript_topic').value), 10)
        self.create_subscription(
            String,
            str(self.get_parameter('text_command_topic').value),
            self._text_command_cb,
            10)

        self.audio_queue: queue.Queue[bytes] = queue.Queue()
        self.recognizer = None
        self.stream = None
        self.stream_thread = None

        self._setup_vosk()
        self.create_timer(0.05, self._audio_timer_cb)

        self.get_logger().info(
            'VoiceCommandNode ready. Text fallback: publish std_msgs/String '
            'to /caddie/text_command.')

    def _setup_vosk(self):
        if not bool(self.get_parameter('enable_microphone').value):
            self.get_logger().info('Microphone disabled by parameter.')
            return
        if sd is None or Model is None or KaldiRecognizer is None:
            self.get_logger().warn(
                'sounddevice/vosk is not installed; microphone voice input disabled.')
            return

        model_path = str(self.get_parameter('model_path').value)
        if not model_path:
            self.get_logger().warn(
                'VOSK_MODEL_PATH/model_path is empty; microphone voice input disabled.')
            return
        if not os.path.isdir(model_path):
            self.get_logger().warn(
                f'Vosk model path does not exist: {model_path}. '
                'Text command fallback remains available.')
            return

        try:
            model = Model(model_path)
            grammar = json.dumps([
                'start mapping',
                'scan course',
                'stop mapping',
                'retrieve ball',
                'retrieve nearest ball',
                'find ball',
                'return home',
                'follow me',
                'list balls',
                'analyze shot',
                'go to tee',
                'go to fairway',
                'go to green',
                'stop',
                '[unk]',
            ])
            sample_rate = int(self.get_parameter('sample_rate').value)
            self.recognizer = KaldiRecognizer(model, sample_rate, grammar)
            self.recognizer.SetWords(True)
            self.stream_thread = threading.Thread(
                target=self._microphone_thread, daemon=True)
            self.stream_thread.start()
            self.get_logger().info(f'Vosk microphone pipeline active: {model_path}')
        except Exception as exc:
            self.recognizer = None
            self.get_logger().warn(
                f'Could not start Vosk voice input ({exc}). '
                'Text command fallback remains available.')

    def _microphone_thread(self):
        sample_rate = int(self.get_parameter('sample_rate').value)
        block_size = int(self.get_parameter('block_size').value)
        device_param = str(self.get_parameter('device').value).strip()
        device = None
        if device_param:
            try:
                device = int(device_param)
            except ValueError:
                device = device_param

        def callback(indata, frames, time_info, status):
            if status:
                self.get_logger().debug(str(status))
            self.audio_queue.put(bytes(indata))

        try:
            with sd.RawInputStream(
                    samplerate=sample_rate,
                    blocksize=block_size,
                    dtype='int16',
                    channels=1,
                    device=device,
                    callback=callback):
                while rclpy.ok():
                    threading.Event().wait(0.25)
        except Exception as exc:
            self.get_logger().error(f'Microphone stream failed: {exc}')

    def _audio_timer_cb(self):
        if self.recognizer is None:
            return
        for _ in range(4):
            try:
                data = self.audio_queue.get_nowait()
            except queue.Empty:
                return
            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                text = normalize_text(result.get('text', ''))
                if text:
                    self._process_text(text)

    def _text_command_cb(self, msg: String):
        self._process_text(msg.data)

    def _process_text(self, text: str):
        text = normalize_text(text)
        if not text:
            return

        transcript = String()
        transcript.data = text
        self.transcript_pub.publish(transcript)

        command = canonicalize_command(text)
        if command is None:
            self.get_logger().info(f'Heard "{text}" but no caddie command matched.')
            return

        out = String()
        out.data = command
        self.command_pub.publish(out)
        self.get_logger().info(f'Heard "{text}" -> {command}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
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
