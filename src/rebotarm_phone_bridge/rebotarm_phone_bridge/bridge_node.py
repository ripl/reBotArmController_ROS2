from __future__ import annotations

from collections import deque
import math
import queue
import socket
import threading
import time

from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import PhoneButtonEvent, PhoneTrackingStatus
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .calibration import (
    calibrate_world_to_base,
    matrix_to_quaternion_xyzw,
    normalize_quaternion_xyzw,
    transform_pose_world_to_base,
)
from .osc_decoder import decode_packet, OscDecodeError, OscMessage
from .pose_filter import PoseLowPassFilter


class PhoneTrackingBridge(Node):
    def __init__(self) -> None:
        super().__init__("phone_tracking_bridge")

        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("port", 9000)
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("world_frame_id", "phone_ar_world")
        self.declare_parameter("phone_frame_id", "phone_camera")
        self.declare_parameter("pose_timeout", 0.5)
        self.declare_parameter("pose_filter.time_constant", 0.1)
        self.declare_parameter("calibration.num_samples", 100)
        self.declare_parameter("calibration.min_inlier_samples", 50)
        self.declare_parameter("calibration.outlier_threshold_deg", 3.0)

        self._bind_host = str(self.get_parameter("bind_host").value)
        self._port = int(self.get_parameter("port").value)
        self._base_frame = str(self.get_parameter("base_frame_id").value)
        self._world_frame = str(self.get_parameter("world_frame_id").value)
        self._phone_frame = str(self.get_parameter("phone_frame_id").value)
        self._pose_timeout = float(self.get_parameter("pose_timeout").value)
        self._pose_filter_time_constant = float(
            self.get_parameter("pose_filter.time_constant").value
        )
        self._sample_target = int(
            self.get_parameter("calibration.num_samples").value
        )
        self._min_inliers = int(
            self.get_parameter("calibration.min_inlier_samples").value
        )
        threshold_deg = float(
            self.get_parameter("calibration.outlier_threshold_deg").value
        )
        self._outlier_threshold_rad = math.radians(threshold_deg)
        self._validate_parameters()

        pose_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._world_pose_publisher = self.create_publisher(
            PoseStamped, "/phone_tracking/pose_world", pose_qos
        )
        self._base_pose_publisher = self.create_publisher(
            PoseStamped, "/phone_tracking/pose_base", pose_qos
        )
        self._button_publisher = self.create_publisher(
            PhoneButtonEvent, "/phone_tracking/button_event", 10
        )
        self._status_publisher = self.create_publisher(
            PhoneTrackingStatus, "/phone_tracking/status", latched_qos
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._calibration_service = self.create_service(
            Trigger,
            "/phone_tracking/calibrate",
            self._start_calibration,
        )

        self._state = PhoneTrackingStatus.DISCONNECTED
        self._status_message = "waiting for phone pose stream"
        self._session_id: str | None = None
        self._session_started_monotonic: float | None = None
        self._last_pose_monotonic: float | None = None
        self._pending_position: np.ndarray | None = None
        self._R_BW: np.ndarray | None = None
        self._calibration_samples: list[np.ndarray] = []
        self._pose_filter = PoseLowPassFilter(
            self._pose_filter_time_constant
        )
        self._seen_button_sequences: deque[int] = deque(maxlen=256)
        self._seen_button_sequence_set: set[int] = set()
        self._invalid_packet_count = 0

        self._event_queue: queue.SimpleQueue[OscMessage | Exception] = (
            queue.SimpleQueue()
        )
        self._stop_receiver = threading.Event()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.settimeout(0.2)
        self._socket.bind((self._bind_host, self._port))
        self._receiver_thread = threading.Thread(
            target=self._receive_loop,
            name="phone-osc-receiver",
            daemon=True,
        )
        self._receiver_thread.start()

        self._timer = self.create_timer(0.01, self._tick)
        self._publish_status()
        self.get_logger().info(
            f"phone bridge listening on udp://{self._bind_host}:{self._port}"
        )

    def _validate_parameters(self) -> None:
        if not 1 <= self._port <= 65535:
            raise ValueError("port must be in [1, 65535]")
        if self._pose_timeout <= 0.0:
            raise ValueError("pose_timeout must be positive")
        if (
            not math.isfinite(self._pose_filter_time_constant)
            or self._pose_filter_time_constant <= 0.0
        ):
            raise ValueError("pose_filter.time_constant must be positive and finite")
        if self._sample_target <= 0:
            raise ValueError("calibration.num_samples must be positive")
        if not 1 <= self._min_inliers <= self._sample_target:
            raise ValueError(
                "calibration.min_inlier_samples must be in "
                "[1, calibration.num_samples]"
            )
        if self._outlier_threshold_rad <= 0.0:
            raise ValueError(
                "calibration.outlier_threshold_deg must be positive"
            )
        frames = (
            self._base_frame,
            self._world_frame,
            self._phone_frame,
        )
        if any(not frame for frame in frames):
            raise ValueError("frame IDs must not be empty")

    def _receive_loop(self) -> None:
        while not self._stop_receiver.is_set():
            try:
                packet, _address = self._socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                for message in decode_packet(packet):
                    self._event_queue.put(message)
            except OscDecodeError as exc:
                self._event_queue.put(exc)

    def _tick(self) -> None:
        self._check_timeout()
        for _ in range(512):
            try:
                event = self._event_queue.get_nowait()
            except queue.Empty:
                break
            if isinstance(event, Exception):
                self._reject_packet(str(event))
            else:
                self._handle_message(event)

    def _check_timeout(self) -> None:
        timeout_reference = (
            self._last_pose_monotonic
            if self._last_pose_monotonic is not None
            else self._session_started_monotonic
        )
        if (
            self._session_id is not None
            and timeout_reference is not None
            and time.monotonic() - timeout_reference > self._pose_timeout
        ):
            self._invalidate_session("phone pose stream timed out")

    def _handle_message(self, message: OscMessage) -> None:
        handlers = {
            "/phone/camera/session_id": self._handle_session_id,
            "/lota/camera/position": self._handle_position,
            "/lota/camera/rotation": self._handle_rotation,
            "/phone/input/button": self._handle_button,
        }
        handler = handlers.get(message.address)
        if handler is not None:
            handler(message.arguments)

    def _handle_session_id(self, arguments: tuple[object, ...]) -> None:
        if (
            len(arguments) != 1
            or not isinstance(arguments[0], str)
            or not arguments[0]
            or len(arguments[0]) > 128
        ):
            self._reject_packet("invalid /phone/camera/session_id")
            return
        session_id = arguments[0]
        if session_id == self._session_id:
            return

        previous = self._session_id
        self._clear_session_data()
        self._session_id = session_id
        self._session_started_monotonic = time.monotonic()
        self._state = PhoneTrackingStatus.UNCALIBRATED
        if previous is None:
            message = f"phone AR session detected: {session_id}"
        else:
            message = f"phone AR session changed; calibration cleared: {session_id}"
        self._status_message = message
        self._publish_status()
        self.get_logger().info(message)

    def _handle_position(self, arguments: tuple[object, ...]) -> None:
        if self._session_id is None:
            return
        try:
            position = np.asarray(arguments, dtype=np.float64)
        except (TypeError, ValueError):
            self._reject_packet("invalid /lota/camera/position")
            return
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            self._reject_packet("invalid /lota/camera/position")
            return
        self._pending_position = position

    def _handle_rotation(self, arguments: tuple[object, ...]) -> None:
        if self._session_id is None:
            return
        if self._pending_position is None:
            self._reject_packet("rotation arrived without a matching position")
            return
        position = self._pending_position
        self._pending_position = None
        try:
            quaternion = normalize_quaternion_xyzw(arguments)
        except (TypeError, ValueError) as exc:
            self._reject_packet(str(exc))
            return

        was_stream_valid = self._stream_valid()
        received_monotonic = time.monotonic()
        filtered_position, filtered_quaternion = self._pose_filter.update(
            position,
            quaternion,
            received_monotonic,
        )
        self._last_pose_monotonic = received_monotonic
        stamp = self.get_clock().now().to_msg()
        self._publish_world_pose(
            filtered_position,
            filtered_quaternion,
            stamp,
        )

        if self._state == PhoneTrackingStatus.COLLECTING:
            self._calibration_samples.append(quaternion)
            if len(self._calibration_samples) >= self._sample_target:
                self._finish_calibration()
            else:
                self._status_message = "collecting calibration samples"
                self._publish_status()

        if self._R_BW is not None:
            position_base, quaternion_base = transform_pose_world_to_base(
                self._R_BW,
                filtered_position,
                filtered_quaternion,
            )
            self._publish_base_pose(position_base, quaternion_base, stamp)
            self._broadcast_calibration(stamp)
        elif (
            not was_stream_valid
            and self._state != PhoneTrackingStatus.COLLECTING
        ):
            self._status_message = "pose stream active; calibration required"
            self._publish_status()

    def _handle_button(self, arguments: tuple[object, ...]) -> None:
        if self._session_id is None or len(arguments) != 4:
            return
        sequence, button, gesture, source_timestamp = arguments
        if (
            not isinstance(sequence, int)
            or not isinstance(button, int)
            or not isinstance(gesture, int)
        ):
            self._reject_packet("invalid /phone/input/button integer fields")
            return
        try:
            source_timestamp = float(source_timestamp)
        except (TypeError, ValueError):
            self._reject_packet("invalid /phone/input/button timestamp")
            return
        if (
            button not in (1, 2, 3, 4)
            or gesture not in (1, 2)
            or not math.isfinite(source_timestamp)
        ):
            self._reject_packet("invalid /phone/input/button values")
            return
        if sequence in self._seen_button_sequence_set:
            return
        if len(self._seen_button_sequences) == self._seen_button_sequences.maxlen:
            expired = self._seen_button_sequences.popleft()
            self._seen_button_sequence_set.remove(expired)
        self._seen_button_sequences.append(sequence)
        self._seen_button_sequence_set.add(sequence)

        message = PhoneButtonEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.sequence = sequence
        message.button = button
        message.gesture = gesture
        message.source_timestamp = source_timestamp
        self._button_publisher.publish(message)

    def _publish_world_pose(
        self,
        position: np.ndarray,
        quaternion: np.ndarray,
        stamp,
    ) -> None:
        message = self._make_pose(self._world_frame, position, quaternion, stamp)
        self._world_pose_publisher.publish(message)
        self._broadcast_pose(message, self._phone_frame)

    def _publish_base_pose(
        self,
        position: np.ndarray,
        quaternion: np.ndarray,
        stamp,
    ) -> None:
        message = self._make_pose(self._base_frame, position, quaternion, stamp)
        self._base_pose_publisher.publish(message)

    def _make_pose(
        self,
        frame_id: str,
        position: np.ndarray,
        quaternion: np.ndarray,
        stamp,
    ) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = float(quaternion[0])
        message.pose.orientation.y = float(quaternion[1])
        message.pose.orientation.z = float(quaternion[2])
        message.pose.orientation.w = float(quaternion[3])
        return message

    def _broadcast_pose(self, pose: PoseStamped, child_frame_id: str) -> None:
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = child_frame_id
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

    def _broadcast_calibration(self, stamp) -> None:
        assert self._R_BW is not None
        quaternion = matrix_to_quaternion_xyzw(self._R_BW)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._base_frame
        transform.child_frame_id = self._world_frame
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self._tf_broadcaster.sendTransform(transform)

    def _start_calibration(self, _request, response):
        if self._session_id is None or not self._stream_valid():
            response.success = False
            response.message = "cannot calibrate without a fresh phone pose"
            return response
        self._R_BW = None
        self._calibration_samples.clear()
        self._state = PhoneTrackingStatus.COLLECTING
        self._status_message = "collecting calibration samples"
        self._publish_status()
        response.success = True
        response.message = "calibration collection started"
        self.get_logger().info(response.message)
        return response

    def _finish_calibration(self) -> None:
        try:
            result = calibrate_world_to_base(
                self._calibration_samples,
                outlier_threshold_rad=self._outlier_threshold_rad,
                min_inlier_samples=self._min_inliers,
            )
        except (ValueError, np.linalg.LinAlgError) as exc:
            self._R_BW = None
            self._state = PhoneTrackingStatus.ERROR
            self._status_message = f"calibration failed: {exc}"
            self._publish_status()
            self.get_logger().error(self._status_message)
            return

        self._R_BW = result.R_BW.copy()
        self._state = PhoneTrackingStatus.CALIBRATED
        self._status_message = (
            "calibration complete: "
            f"{result.inlier_count} inliers, {result.outlier_count} outliers"
        )
        self._publish_status()
        self.get_logger().info(self._status_message)
        self.get_logger().info(f"R_BW:\n{self._R_BW}")

    def _stream_valid(self) -> bool:
        return (
            self._last_pose_monotonic is not None
            and time.monotonic() - self._last_pose_monotonic
            <= self._pose_timeout
        )

    def _clear_session_data(self) -> None:
        self._R_BW = None
        self._pending_position = None
        self._last_pose_monotonic = None
        self._session_started_monotonic = None
        self._pose_filter.reset()
        self._calibration_samples.clear()
        self._seen_button_sequences.clear()
        self._seen_button_sequence_set.clear()

    def _invalidate_session(self, reason: str) -> None:
        self._clear_session_data()
        self._session_id = None
        self._state = PhoneTrackingStatus.DISCONNECTED
        self._status_message = reason
        self._publish_status()
        self.get_logger().warn(f"{reason}; calibration cleared")

    def _reject_packet(self, reason: str) -> None:
        self._invalid_packet_count += 1
        if self._invalid_packet_count == 1 or self._invalid_packet_count % 100 == 0:
            self.get_logger().warn(
                f"discarded invalid OSC packet/message: {reason} "
                f"(count={self._invalid_packet_count})"
            )

    def _publish_status(self) -> None:
        status = PhoneTrackingStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.state = self._state
        status.session_id = self._session_id or ""
        status.stream_valid = self._stream_valid()
        status.calibration_valid = self._R_BW is not None
        status.calibration_samples = len(self._calibration_samples)
        status.calibration_target_samples = self._sample_target
        status.message = self._status_message
        self._status_publisher.publish(status)

    def destroy_node(self):
        self._stop_receiver.set()
        self._socket.close()
        if self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PhoneTrackingBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
