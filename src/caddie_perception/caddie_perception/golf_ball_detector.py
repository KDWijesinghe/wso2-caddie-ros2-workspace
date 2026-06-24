import json
import math
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseArray
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_geometry_msgs  # noqa: F401 - registers geometry message transforms
import tf2_ros

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


@dataclass
class PixelDetection:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_name: str
    source: str
    source_id: int | None = None

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) * 0.5, (self.y1 + self.y2) * 0.5)


def image_to_bgr_array(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    channels_by_encoding = {
        'bgr8': 3,
        'rgb8': 3,
        'bgra8': 4,
        'rgba8': 4,
        'mono8': 1,
        '8uc1': 1,
        '8uc3': 3,
        '8uc4': 4,
    }
    if encoding not in channels_by_encoding:
        raise ValueError(f'unsupported RGB image encoding: {msg.encoding}')

    channels = channels_by_encoding[encoding]
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if channels == 1:
        img = raw.reshape(msg.height, msg.step)[:, :msg.width]
        return np.dstack((img, img, img)).copy()

    row_pixels = msg.step // channels
    img = raw.reshape(msg.height, row_pixels, channels)[:, :msg.width, :]
    if encoding in {'rgb8', 'rgba8'}:
        return img[:, :, :3][:, :, ::-1].copy()
    return img[:, :, :3].copy()


def depth_image_to_meters(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    if encoding == '32fc1':
        dtype = np.dtype(np.float32)
        scale = 1.0
    elif encoding in {'16uc1', 'mono16'}:
        dtype = np.dtype(np.uint16)
        scale = 0.001
    else:
        raise ValueError(f'unsupported depth image encoding: {msg.encoding}')

    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    row_items = msg.step // dtype.itemsize
    raw = np.frombuffer(bytes(msg.data), dtype=dtype)
    depth = raw.reshape(msg.height, row_items)[:, :msg.width].astype(np.float32)
    depth *= scale
    depth[~np.isfinite(depth)] = np.nan
    depth[depth <= 0.0] = np.nan
    return np.ascontiguousarray(depth, dtype=np.float32)


class GolfBallDetector(Node):
    """RGB-D golf ball detector with YOLO tracking and OpenCV fallback."""

    def __init__(self):
        super().__init__('golf_ball_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_frame', 'camera_rgb_optical_frame')
        self.declare_parameter('detector_backend', 'auto')
        self.declare_parameter('yolo_model', '')
        self.declare_parameter('tracker_cfg', 'botsort.yaml')
        self.declare_parameter('yolo_imgsz', 640)
        self.declare_parameter('yolo_conf', 0.25)
        self.declare_parameter(
            'ball_class_names', ['golf ball', 'sports ball', 'ball'])
        self.declare_parameter('detection_rate', 5.0)
        self.declare_parameter('depth_radius_px', 6)
        self.declare_parameter('dedup_distance', 0.35)
        self.declare_parameter('track_ttl', 20.0)
        self.declare_parameter('cv_min_area_px', 10.0)
        self.declare_parameter('cv_max_area_px', 2200.0)
        self.declare_parameter('cv_min_circularity', 0.55)
        self.declare_parameter('cv_white_value_min', 155)
        self.declare_parameter('cv_white_saturation_max', 95)
        self.declare_parameter('publish_annotated_image', False)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.detector_backend = str(
            self.get_parameter('detector_backend').value).lower()
        self.yolo_model_name = str(self.get_parameter('yolo_model').value)
        self.tracker_cfg = str(self.get_parameter('tracker_cfg').value)
        self.yolo_imgsz = int(self.get_parameter('yolo_imgsz').value)
        self.yolo_conf = float(self.get_parameter('yolo_conf').value)
        self.ball_class_names = {
            str(name).lower()
            for name in self.get_parameter('ball_class_names').value
        }
        self.detection_rate = max(
            0.2, float(self.get_parameter('detection_rate').value))
        self.depth_radius_px = max(
            1, int(self.get_parameter('depth_radius_px').value))
        self.dedup_distance = max(
            0.05, float(self.get_parameter('dedup_distance').value))
        self.track_ttl = max(1.0, float(self.get_parameter('track_ttl').value))
        self.cv_min_area_px = float(
            self.get_parameter('cv_min_area_px').value)
        self.cv_max_area_px = float(
            self.get_parameter('cv_max_area_px').value)
        self.cv_min_circularity = float(
            self.get_parameter('cv_min_circularity').value)
        self.cv_white_value_min = int(
            self.get_parameter('cv_white_value_min').value)
        self.cv_white_saturation_max = int(
            self.get_parameter('cv_white_saturation_max').value)
        self.publish_annotated_image = bool(
            self.get_parameter('publish_annotated_image').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._rgb_lock = threading.Lock()
        self._depth_lock = threading.Lock()
        self._inference_lock = threading.Lock()
        self._latest_rgb_msg: Image | None = None
        self._latest_depth: np.ndarray | None = None
        self._last_inference_time = 0.0

        self.fx = 525.0
        self.fy = 525.0
        self.cx = 319.5
        self.cy = 239.5
        self.camera_info_received = False

        self.tracks: dict[str, dict] = {}
        self.next_track_id = 1

        self.model = self._load_yolo()

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(
            Image, self.image_topic, self._image_cb, sensor_qos)
        self.create_subscription(
            Image, self.depth_topic, self._depth_cb, sensor_qos)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_cb,
            sensor_qos)

        self.pose_pub = self.create_publisher(
            PoseArray, '/caddie/ball_poses', 10)
        self.json_pub = self.create_publisher(
            String, '/caddie/ball_detections', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/caddie/ball_markers', 10)
        self.annotated_pub = self.create_publisher(
            Image, '/caddie/perception/annotated_image', 1)

        self.create_timer(0.03, self._detection_timer_cb)
        self.create_timer(1.0, self._publish_tracks)

        active = 'YOLO' if self.model is not None else 'OpenCV'
        self.get_logger().info(
            f'GolfBallDetector ready: image={self.image_topic}, '
            f'depth={self.depth_topic}, backend={active}, '
            f'rate={self.detection_rate:.1f} Hz.')

    def _load_yolo(self):
        wants_yolo = self.detector_backend in {'auto', 'yolo'}
        if not wants_yolo:
            return None
        if not self.yolo_model_name:
            self.get_logger().info(
                'No yolo_model parameter supplied; using OpenCV golf-ball fallback.')
            return None
        if YOLO is None:
            self.get_logger().warn(
                'ultralytics is not installed; using OpenCV golf-ball fallback.')
            return None

        try:
            model = YOLO(self.yolo_model_name)
            model.track(
                np.zeros((480, 640, 3), dtype=np.uint8),
                tracker=self.tracker_cfg,
                persist=True,
                imgsz=self.yolo_imgsz,
                conf=self.yolo_conf,
                show=False,
                save=False,
                save_txt=False,
                verbose=False)
            self.get_logger().info(
                f'YOLO warmed up: model={self.yolo_model_name}, '
                f'tracker={self.tracker_cfg}, conf={self.yolo_conf:.2f}.')
            return model
        except Exception as exc:
            self.get_logger().warn(
                f'YOLO could not start ({exc}); using OpenCV fallback.')
            return None

    def _image_cb(self, msg: Image):
        with self._rgb_lock:
            self._latest_rgb_msg = msg

    def _depth_cb(self, msg: Image):
        try:
            depth = depth_image_to_meters(msg)
        except Exception as exc:
            self.get_logger().warn(
                f'Depth conversion failed: {exc}', throttle_duration_sec=5.0)
            return
        with self._depth_lock:
            self._latest_depth = depth

    def _camera_info_cb(self, msg: CameraInfo):
        if self.camera_info_received:
            return
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id
        self.camera_info_received = True
        self.get_logger().info(
            f'Camera info received: fx={self.fx:.1f}, fy={self.fy:.1f}, '
            f'cx={self.cx:.1f}, cy={self.cy:.1f}, frame={self.camera_frame}.')

    def _detection_timer_cb(self):
        now = time.monotonic()
        if now - self._last_inference_time < (1.0 / self.detection_rate):
            return
        if not self._inference_lock.acquire(blocking=False):
            return

        try:
            with self._rgb_lock:
                msg = self._latest_rgb_msg
                self._latest_rgb_msg = None
            if msg is None:
                return

            self._last_inference_time = now
            self._run_detection(msg)
        finally:
            self._inference_lock.release()

    def _run_detection(self, msg: Image):
        try:
            bgr = image_to_bgr_array(msg)
        except Exception as exc:
            self.get_logger().warn(
                f'RGB conversion failed: {exc}', throttle_duration_sec=5.0)
            return

        detections: list[PixelDetection] = []
        if self.model is not None:
            detections = self._detect_yolo(bgr)
        if not detections and self.detector_backend in {'auto', 'opencv', 'cv'}:
            detections = self._detect_opencv(bgr)

        if detections and self.publish_annotated_image:
            self._publish_annotated(msg, bgr, detections)

        for det in detections:
            point = self._pixel_to_world(det, msg.header.stamp, msg.header.frame_id)
            if point is None:
                continue
            frame_id, x, y, z = point
            self._upsert_track(det, frame_id, x, y, z)

        if detections:
            self._publish_tracks()

    def _detect_yolo(self, bgr: np.ndarray) -> list[PixelDetection]:
        try:
            results = self.model.track(
                bgr,
                tracker=self.tracker_cfg,
                persist=True,
                imgsz=self.yolo_imgsz,
                conf=self.yolo_conf,
                show=False,
                save=False,
                save_txt=False,
                verbose=False)
        except Exception as exc:
            self.get_logger().warn(
                f'YOLO inference failed: {exc}', throttle_duration_sec=5.0)
            return []

        if not results or results[0].boxes is None:
            return []

        boxes = results[0].boxes
        xyxy = boxes.xyxy.cpu().numpy()
        cls_ids = boxes.cls.cpu().numpy().astype(int)
        confs = boxes.conf.cpu().numpy() if boxes.conf is not None else np.ones(len(xyxy))
        track_ids = (
            boxes.id.cpu().numpy().astype(int)
            if boxes.id is not None else [None] * len(xyxy)
        )
        names = results[0].names
        out = []

        for box, cls_id, conf, track_id in zip(xyxy, cls_ids, confs, track_ids):
            if isinstance(names, dict):
                class_name = str(names.get(int(cls_id), cls_id))
            else:
                class_name = str(names[int(cls_id)])
            if not self._is_ball_class(class_name):
                continue
            x1, y1, x2, y2 = [float(v) for v in box]
            out.append(PixelDetection(
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
                confidence=float(conf),
                class_name=class_name,
                source='yolo',
                source_id=None if track_id is None else int(track_id),
            ))
        return out

    def _is_ball_class(self, class_name: str) -> bool:
        lower = class_name.lower().replace('_', ' ')
        return any(name in lower for name in self.ball_class_names)

    def _detect_opencv(self, bgr: np.ndarray) -> list[PixelDetection]:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 0, self.cv_white_value_min], dtype=np.uint8)
        upper = np.array(
            [179, self.cv_white_saturation_max, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections: list[PixelDetection] = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.cv_min_area_px or area > self.cv_max_area_px:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0.0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < self.cv_min_circularity:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if h <= 0:
                continue
            aspect = w / float(h)
            if aspect < 0.55 or aspect > 1.8:
                continue

            radius = 0.5 * max(w, h)
            confidence = min(0.99, max(0.30, circularity * min(1.0, radius / 12.0)))
            detections.append(PixelDetection(
                x1=float(x),
                y1=float(y),
                x2=float(x + w),
                y2=float(y + h),
                confidence=confidence,
                class_name='golf_ball',
                source='opencv',
                source_id=None,
            ))

        detections.sort(key=lambda det: det.confidence, reverse=True)
        return detections[:12]

    def _pixel_to_world(
            self, det: PixelDetection, stamp, msg_frame: str
    ) -> tuple[str, float, float, float] | None:
        with self._depth_lock:
            depth = None if self._latest_depth is None else self._latest_depth.copy()
        if depth is None:
            self.get_logger().debug(
                'Depth image not available yet; skipping 3D ball projection.',
                throttle_duration_sec=2.0)
            return None

        u, v = det.center
        h, w = depth.shape
        ui = int(round(u))
        vi = int(round(v))
        if ui < 0 or ui >= w or vi < 0 or vi >= h:
            return None

        r = self.depth_radius_px
        u1, u2 = max(0, ui - r), min(w, ui + r + 1)
        v1, v2 = max(0, vi - r), min(h, vi + r + 1)
        patch = depth[v1:v2, u1:u2]
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size == 0:
            return None
        z = float(np.median(valid))

        x_cam = (u - self.cx) * z / self.fx
        y_cam = (v - self.cy) * z / self.fy

        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = msg_frame or self.camera_frame
        if point.header.frame_id.endswith('optical_frame'):
            point.point.x = float(x_cam)
            point.point.y = float(y_cam)
            point.point.z = float(z)
        else:
            point.point.x = float(z)
            point.point.y = float(-x_cam)
            point.point.z = float(-y_cam)

        for target_frame in (self.map_frame, self.base_frame, point.header.frame_id):
            if not target_frame:
                continue
            if target_frame == point.header.frame_id:
                return (target_frame, point.point.x, point.point.y, point.point.z)
            try:
                transformed = self.tf_buffer.transform(
                    point, target_frame, timeout=Duration(seconds=0.12))
                p = transformed.point
                return (target_frame, float(p.x), float(p.y), float(p.z))
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue

        self.get_logger().debug(
            f'No TF available from {point.header.frame_id} to '
            f'{self.map_frame}/{self.base_frame}.',
            throttle_duration_sec=2.0)
        return None

    def _upsert_track(
            self, det: PixelDetection, frame_id: str, x: float, y: float, z: float):
        now = self.get_clock().now().nanoseconds * 1e-9
        best_key = None
        best_dist = float('inf')

        if det.source_id is not None:
            for key, track in self.tracks.items():
                if track.get('source') == det.source and track.get('source_id') == det.source_id:
                    best_key = key
                    break

        if best_key is None:
            for key, track in self.tracks.items():
                if track.get('frame_id') != frame_id:
                    continue
                dist = math.hypot(track['x'] - x, track['y'] - y)
                if dist < self.dedup_distance and dist < best_dist:
                    best_key = key
                    best_dist = dist

        if best_key is None:
            best_key = f'ball_{self.next_track_id}'
            self.next_track_id += 1
            self.tracks[best_key] = {
                'label': best_key,
                'observations': 0,
                'first_seen': now,
            }
            self.get_logger().info(
                f'New golf ball track {best_key}: '
                f'{frame_id}({x:.2f}, {y:.2f}, {z:.2f})')

        track = self.tracks[best_key]
        observations = int(track.get('observations', 0))
        alpha = 1.0 / float(min(observations + 1, 8))
        if observations == 0 or track.get('frame_id') != frame_id:
            track['x'] = x
            track['y'] = y
            track['z'] = z
        else:
            track['x'] = (1.0 - alpha) * track['x'] + alpha * x
            track['y'] = (1.0 - alpha) * track['y'] + alpha * y
            track['z'] = (1.0 - alpha) * track['z'] + alpha * z
        track['frame_id'] = frame_id
        track['confidence'] = max(float(det.confidence), track.get('confidence', 0.0))
        track['class_name'] = det.class_name
        track['source'] = det.source
        track['source_id'] = det.source_id
        track['observations'] = observations + 1
        track['last_seen'] = now

    def _publish_tracks(self):
        now_ros = self.get_clock().now()
        now = now_ros.nanoseconds * 1e-9
        stale = [
            key for key, track in self.tracks.items()
            if now - float(track.get('last_seen', now)) > self.track_ttl
        ]
        for key in stale:
            del self.tracks[key]

        frame_id = self.map_frame
        if self.tracks:
            frame_id = next(iter(self.tracks.values())).get('frame_id', self.map_frame)

        pose_array = PoseArray()
        pose_array.header.stamp = now_ros.to_msg()
        pose_array.header.frame_id = frame_id

        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        payload = {
            'stamp': now,
            'frame_id': frame_id,
            'count': 0,
            'balls': [],
        }

        for index, (label, track) in enumerate(sorted(self.tracks.items())):
            if track.get('frame_id') != frame_id:
                continue
            pose = Pose()
            pose.position.x = float(track['x'])
            pose.position.y = float(track['y'])
            pose.position.z = float(track['z'])
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            payload['balls'].append({
                'label': label,
                'x': float(track['x']),
                'y': float(track['y']),
                'z': float(track['z']),
                'confidence': float(track.get('confidence', 0.0)),
                'observations': int(track.get('observations', 0)),
                'source': track.get('source', ''),
            })

            marker = Marker()
            marker.header = pose_array.header
            marker.ns = 'golf_balls'
            marker.id = index * 2
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.85
            marker.color.a = 0.95
            markers.markers.append(marker)

            text = Marker()
            text.header = pose_array.header
            text.ns = 'golf_ball_labels'
            text.id = index * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pose.position.x
            text.pose.position.y = pose.position.y
            text.pose.position.z = pose.position.z + 0.22
            text.pose.orientation.w = 1.0
            text.scale.z = 0.18
            text.color.r = 1.0
            text.color.g = 0.45
            text.color.b = 0.02
            text.color.a = 1.0
            text.text = label
            markers.markers.append(text)

        payload['count'] = len(payload['balls'])

        self.pose_pub.publish(pose_array)
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.json_pub.publish(msg)
        self.marker_pub.publish(markers)

    def _publish_annotated(
            self, original_msg: Image, bgr: np.ndarray,
            detections: list[PixelDetection]):
        annotated = bgr.copy()
        for det in detections:
            x1, y1, x2, y2 = map(int, (det.x1, det.y1, det.x2, det.y2))
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 140, 255), 2)
            label = f'{det.class_name} {det.confidence:.2f}'
            cv2.putText(
                annotated, label, (x1, max(16, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 140, 255), 1,
                cv2.LINE_AA)
        out = Image()
        out.header = original_msg.header
        out.height = int(annotated.shape[0])
        out.width = int(annotated.shape[1])
        out.encoding = 'bgr8'
        out.is_bigendian = False
        out.step = int(annotated.shape[1] * 3)
        out.data = annotated.tobytes()
        self.annotated_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GolfBallDetector()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
