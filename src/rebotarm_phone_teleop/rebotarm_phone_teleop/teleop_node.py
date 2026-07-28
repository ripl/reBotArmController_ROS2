from __future__ import annotations

import math
import time

from geometry_msgs.msg import PoseStamped, TransformStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.msg import ArmStatus, PhoneButtonEvent, PhoneTrackingStatus
from rebotarm_msgs.srv import GripperCommand
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .mapping import (
    map_relative_orientation,
    map_relative_position,
    normalize_quaternion,
)


class PhoneEefTeleop(Node):
    MOVING_TO_READY = "MOVING_TO_READY"
    READY_FAILED = "READY_FAILED"
    INACTIVE = "INACTIVE"
    ENABLING = "ENABLING"
    ACTIVE = "ACTIVE"
    DISABLING = "DISABLING"

    def __init__(self) -> None:
        super().__init__("phone_eef_teleop")

        self.declare_parameter("command_enabled", False)
        self.declare_parameter("enable_orientation", True)
        self.declare_parameter("position_scale", 0.3)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("phone_pose_timeout", 0.15)
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("eef_frame_id", "end_link")

        self._command_enabled = bool(
            self.get_parameter("command_enabled").value
        )
        self._enable_orientation = bool(
            self.get_parameter("enable_orientation").value
        )
        self._position_scale = float(
            self.get_parameter("position_scale").value
        )
        self._publish_rate = float(
            self.get_parameter("publish_rate").value
        )
        self._phone_pose_timeout = float(
            self.get_parameter("phone_pose_timeout").value
        )
        self._base_frame = str(
            self.get_parameter("base_frame_id").value
        )
        self._eef_frame = str(self.get_parameter("eef_frame_id").value)
        self._validate_parameters()

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._phone_subscription = self.create_subscription(
            PoseStamped,
            "/phone_tracking/pose_base",
            self._phone_pose_callback,
            sensor_qos,
        )
        self._button_subscription = self.create_subscription(
            PhoneButtonEvent,
            "/phone_tracking/button_event",
            self._button_callback,
            10,
        )
        self._tracking_subscription = self.create_subscription(
            PhoneTrackingStatus,
            "/phone_tracking/status",
            self._tracking_status_callback,
            latched_qos,
        )
        self._arm_subscription = self.create_subscription(
            ArmStatus,
            "/rebotarm/arm_status",
            self._arm_status_callback,
            latched_qos,
        )
        self._target_publisher = self.create_publisher(
            PoseStamped,
            "/rebotarm/eef_target_pose",
            sensor_qos,
        )
        self._preview_publisher = self.create_publisher(
            PoseStamped,
            "/phone_teleop/eef_target_preview",
            sensor_qos,
        )
        self._streaming_client = self.create_client(
            SetBool,
            "/rebotarm/eef_streaming/enable",
        )
        self._gripper_open_client = self.create_client(
            GripperCommand,
            "/rebotarm/gripper/open",
        )
        self._gripper_close_client = self.create_client(
            GripperCommand,
            "/rebotarm/gripper/close",
        )
        self._ready_pose_client = ActionClient(
            self,
            MoveToPose,
            "/rebotarm/move_to_pose",
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._state = (
            self.MOVING_TO_READY
            if self._command_enabled
            else self.INACTIVE
        )
        self._ready_goal_sent = False
        self._ready_goal_handle = None
        self._tracking_valid = False
        self._session_id = ""
        self._active_session_id = ""
        self._arm_state: str | None = None
        self._latest_phone_position: tuple[float, float, float] | None = None
        self._latest_phone_orientation: tuple[float, float, float, float] | None = None
        self._latest_phone_monotonic: float | None = None
        self._initial_phone_position: tuple[float, float, float] | None = None
        self._initial_phone_orientation: tuple[float, float, float, float] | None = None
        self._initial_eef_position: tuple[float, float, float] | None = None
        self._initial_eef_orientation: tuple[float, float, float, float] | None = None
        self._cancel_enable = False
        self._gripper_open = False
        self._gripper_request_in_flight = False

        self._timer = self.create_timer(
            1.0 / self._publish_rate,
            self._publish_target,
        )
        self._ready_pose_timer = self.create_timer(
            0.1,
            self._move_to_ready_pose,
        )
        mode = "COMMAND" if self._command_enabled else "PREVIEW"
        if self._command_enabled:
            self.get_logger().info(
                "phone EEF teleop started in COMMAND mode; "
                "waiting to move to teleop_ready_pose"
            )
        else:
            self.get_logger().info(
                f"phone EEF teleop ready in {mode} mode; "
                "Volume Up single-click toggles teleop"
            )

    def _validate_parameters(self) -> None:
        if not math.isfinite(self._position_scale) or self._position_scale <= 0.0:
            raise ValueError("position_scale must be positive and finite")
        if self._publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        if self._phone_pose_timeout <= 0.0:
            raise ValueError("phone_pose_timeout must be positive")
        if not self._base_frame or not self._eef_frame:
            raise ValueError("frame IDs must not be empty")

    def _phone_pose_callback(self, message: PoseStamped) -> None:
        if message.header.frame_id != self._base_frame:
            self.get_logger().warn(
                f"ignoring phone pose in frame {message.header.frame_id!r}; "
                f"expected {self._base_frame!r}"
            )
            return
        position = (
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        )
        if not all(math.isfinite(value) for value in position):
            self.get_logger().warn("ignoring invalid phone position")
            return
        try:
            orientation = normalize_quaternion(
                (
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                )
            )
        except ValueError:
            self.get_logger().warn("ignoring invalid phone orientation")
            return
        self._latest_phone_position = position
        self._latest_phone_orientation = orientation
        self._latest_phone_monotonic = time.monotonic()

    def _tracking_status_callback(
        self,
        message: PhoneTrackingStatus,
    ) -> None:
        previous_session = self._session_id
        self._session_id = message.session_id
        self._tracking_valid = (
            message.state == PhoneTrackingStatus.CALIBRATED
            and message.stream_valid
            and message.calibration_valid
        )
        if self._state in (self.ENABLING, self.ACTIVE):
            if not self._tracking_valid:
                self._stop(f"phone tracking invalid: {message.message}")
            elif previous_session and message.session_id != self._active_session_id:
                self._stop("phone AR session changed")

    def _arm_status_callback(self, message: ArmStatus) -> None:
        self._arm_state = message.state_machine
        if (
            self._command_enabled
            and self._state == self.ACTIVE
            and message.state_machine != "EEF_STREAMING"
        ):
            self._stop(
                f"arm left EEF_STREAMING: {message.state_machine}",
            )

    def _button_callback(self, message: PhoneButtonEvent) -> None:
        if (
            message.button == PhoneButtonEvent.VOLUME_UP
            and message.gesture == PhoneButtonEvent.DOUBLE
        ):
            self._return_to_ready_pose()
            return
        if message.gesture != PhoneButtonEvent.SINGLE:
            return
        if message.button == PhoneButtonEvent.VOLUME_UP:
            if self._state == self.INACTIVE:
                self._begin_activation()
            elif self._state == self.ENABLING:
                self._cancel_enable = True
                self.get_logger().info("teleop pause requested while enabling")
            elif self._state == self.ACTIVE:
                self._begin_deactivation("teleop paused by Volume Up")
        elif message.button == PhoneButtonEvent.VOLUME_DOWN:
            self._toggle_gripper()

    def _toggle_gripper(self) -> None:
        if (
            not self._command_enabled
            or self._state not in (self.INACTIVE, self.ACTIVE)
        ):
            return
        if self._gripper_request_in_flight:
            self.get_logger().warn("ignoring Volume Down; gripper command in progress")
            return

        target_open = not self._gripper_open
        client = (
            self._gripper_open_client
            if target_open
            else self._gripper_close_client
        )
        label = "open" if target_open else "close"
        if not client.service_is_ready():
            self.get_logger().warn(f"gripper {label} service is unavailable")
            return

        self._gripper_open = target_open
        self._gripper_request_in_flight = True
        future = client.call_async(GripperCommand.Request())
        future.add_done_callback(
            lambda result: self._gripper_command_done(result, label)
        )
        self.get_logger().info(f"requesting gripper {label}")

    def _gripper_command_done(self, future, label: str) -> None:
        self._gripper_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"gripper {label} failed: {exc}")
            return
        if response.success:
            self.get_logger().info(
                f"gripper {label} complete at "
                f"{response.reached_position:.3f} rad"
            )
        else:
            self.get_logger().warn(
                f"gripper {label} command sent but target not reached: "
                f"{response.message}; current={response.reached_position:.3f} rad"
            )

    def _return_to_ready_pose(self) -> None:
        if not self._command_enabled:
            self.get_logger().warn(
                "ignoring Volume Up double-click in preview mode"
            )
            return
        if self._state == self.ACTIVE:
            self._begin_deactivation(
                "returning to teleop_ready_pose",
                return_to_ready=True,
            )
        elif self._state in (self.INACTIVE, self.READY_FAILED):
            if self._arm_state != "IDLE":
                self.get_logger().warn(
                    "cannot return to teleop_ready_pose: "
                    f"arm state is {self._arm_state or 'unknown'}, expected IDLE"
                )
                return
            self._begin_move_to_ready_pose()

    def _begin_move_to_ready_pose(self) -> None:
        self._clear_latched_poses()
        self._state = self.MOVING_TO_READY
        self._ready_goal_sent = False
        self._ready_goal_handle = None
        self._ready_pose_timer.reset()
        self._move_to_ready_pose()

    def _move_to_ready_pose(self) -> None:
        if (
            not self._command_enabled
            or self._state != self.MOVING_TO_READY
            or self._ready_goal_sent
            or not self._ready_pose_client.server_is_ready()
        ):
            return

        goal = MoveToPose.Goal()
        goal.target_pose.position.x = 0.3
        goal.target_pose.position.y = 0.0
        goal.target_pose.position.z = 0.3
        goal.target_pose.orientation.w = 1.0
        goal.duration = 2.0
        self._ready_goal_sent = True
        self._ready_pose_timer.cancel()
        future = self._ready_pose_client.send_goal_async(goal)
        future.add_done_callback(self._ready_goal_response)
        self.get_logger().info("moving EEF to teleop_ready_pose")

    def _ready_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._ready_pose_failed(str(exc))
            return
        if not goal_handle.accepted:
            self._ready_pose_failed("goal rejected")
            return
        self._ready_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._ready_pose_result)

    def _ready_pose_result(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:
            self._ready_pose_failed(str(exc))
            return
        self._ready_goal_handle = None
        if not result.success:
            self._ready_pose_failed(result.message)
            return
        self._state = self.INACTIVE
        self.get_logger().info(
            "teleop_ready_pose reached; "
            "Volume Up single-click toggles teleop, double-click returns to ready"
        )

    def _ready_pose_failed(self, reason: str) -> None:
        self._ready_goal_handle = None
        self._state = self.READY_FAILED
        self.get_logger().error(f"failed to reach teleop_ready_pose: {reason}")

    def _begin_activation(self) -> None:
        error = self._preflight_error()
        if error is not None:
            self.get_logger().warn(f"cannot start teleop: {error}")
            return
        if not self._capture_initial_poses():
            return

        self._active_session_id = self._session_id
        if not self._command_enabled:
            self._state = self.ACTIVE
            self.get_logger().info(
                "teleop preview active; initial phone and EEF poses captured"
            )
            return

        if not self._streaming_client.service_is_ready():
            self._clear_latched_poses()
            self.get_logger().warn(
                "cannot start teleop: EEF streaming service is unavailable"
            )
            return
        self._clear_latched_poses()
        self._state = self.ENABLING
        self._cancel_enable = False
        request = SetBool.Request()
        request.data = True
        future = self._streaming_client.call_async(request)
        future.add_done_callback(self._enable_done)
        self.get_logger().info("requesting EEF streaming enable")

    def _enable_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._state = self.INACTIVE
            self.get_logger().error(f"EEF streaming enable failed: {exc}")
            return
        if not response.success:
            self._state = self.INACTIVE
            self.get_logger().warn(
                f"EEF streaming enable rejected: {response.message}"
            )
            return
        if self._cancel_enable:
            self._begin_deactivation("teleop enable cancelled")
            return

        error = self._preflight_error(allowed_arm_states=("IDLE", "EEF_STREAMING"))
        if error is not None or not self._capture_initial_poses():
            self._begin_deactivation(
                f"teleop activation aborted: {error or 'EEF TF unavailable'}"
            )
            return
        self._active_session_id = self._session_id
        self._state = self.ACTIVE
        self.get_logger().info(
            "teleop active; initial phone and EEF poses captured"
        )
        self._publish_target()

    def _preflight_error(
        self,
        *,
        allowed_arm_states: tuple[str, ...] = ("IDLE",),
    ) -> str | None:
        if not self._tracking_valid or not self._session_id:
            return "phone stream is not calibrated and valid"
        if not self._phone_pose_is_fresh():
            return "fresh phone pose is unavailable"
        if self._arm_state not in allowed_arm_states:
            return f"arm state is {self._arm_state or 'unknown'}, expected IDLE"
        return None

    def _capture_initial_poses(self) -> bool:
        assert self._latest_phone_position is not None
        assert self._latest_phone_orientation is not None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._eef_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"cannot start teleop: EEF TF unavailable: {exc}"
            )
            return False
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        eef_position = (
            float(translation.x),
            float(translation.y),
            float(translation.z),
        )
        eef_orientation = (
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )
        if not all(
            math.isfinite(value)
            for value in (*eef_position, *eef_orientation)
        ):
            self.get_logger().warn("cannot start teleop: EEF TF is invalid")
            return False
        self._initial_phone_position = self._latest_phone_position
        self._initial_phone_orientation = self._latest_phone_orientation
        self._initial_eef_position = eef_position
        self._initial_eef_orientation = eef_orientation
        return True

    def _phone_pose_is_fresh(self) -> bool:
        return (
            self._latest_phone_position is not None
            and self._latest_phone_orientation is not None
            and self._latest_phone_monotonic is not None
            and time.monotonic() - self._latest_phone_monotonic
            <= self._phone_pose_timeout
        )

    def _publish_target(self) -> None:
        if self._state != self.ACTIVE:
            return
        if not self._phone_pose_is_fresh():
            self._stop("phone pose timed out")
            return
        if not self._tracking_valid or self._session_id != self._active_session_id:
            self._stop("phone tracking or AR session became invalid")
            return

        assert self._latest_phone_position is not None
        assert self._latest_phone_orientation is not None
        assert self._initial_phone_position is not None
        assert self._initial_phone_orientation is not None
        assert self._initial_eef_position is not None
        assert self._initial_eef_orientation is not None
        position = map_relative_position(
            self._initial_phone_position,
            self._latest_phone_position,
            self._initial_eef_position,
            self._position_scale,
        )
        orientation = self._initial_eef_orientation
        if self._enable_orientation:
            orientation = map_relative_orientation(
                self._initial_phone_orientation,
                self._latest_phone_orientation,
                self._initial_eef_orientation,
            )
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._base_frame
        message.pose.position.x = position[0]
        message.pose.position.y = position[1]
        message.pose.position.z = position[2]
        (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ) = orientation
        self._preview_publisher.publish(message)
        if self._command_enabled:
            self._target_publisher.publish(message)
        self._broadcast_target(message)

    def _broadcast_target(self, pose: PoseStamped) -> None:
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = "phone_teleop_eef_target"
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

    def _stop(self, reason: str) -> None:
        if self._state == self.ENABLING:
            self._cancel_enable = True
            self.get_logger().warn(reason)
        elif self._state == self.ACTIVE:
            self._begin_deactivation(reason, warning=True)

    def _begin_deactivation(
        self,
        reason: str,
        *,
        warning: bool = False,
        return_to_ready: bool = False,
    ) -> None:
        self._clear_latched_poses()
        if not self._command_enabled:
            self._state = self.INACTIVE
            if warning:
                self.get_logger().warn(reason)
            else:
                self.get_logger().info(reason)
            return

        self._state = self.DISABLING
        if not self._streaming_client.service_is_ready():
            self._state = self.INACTIVE
            self.get_logger().error(
                f"{reason}; EEF streaming disable service is unavailable"
            )
            return
        request = SetBool.Request()
        request.data = False
        future = self._streaming_client.call_async(request)
        future.add_done_callback(
            lambda result: self._disable_done(
                result,
                reason,
                warning,
                return_to_ready,
            )
        )

    def _disable_done(
        self,
        future,
        reason: str,
        warning: bool,
        return_to_ready: bool,
    ) -> None:
        self._state = self.INACTIVE
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"{reason}; EEF streaming disable failed: {exc}"
            )
            return
        if response.success:
            if warning:
                self.get_logger().warn(reason)
            else:
                self.get_logger().info(reason)
            if return_to_ready:
                self._begin_move_to_ready_pose()
        else:
            self.get_logger().error(
                f"{reason}; EEF streaming disable rejected: {response.message}"
            )

    def _clear_latched_poses(self) -> None:
        self._active_session_id = ""
        self._initial_phone_position = None
        self._initial_phone_orientation = None
        self._initial_eef_position = None
        self._initial_eef_orientation = None

    def stop_for_shutdown(self):
        self._clear_latched_poses()
        if self._ready_goal_handle is not None:
            return self._ready_goal_handle.cancel_goal_async()
        if self._state in (self.MOVING_TO_READY, self.READY_FAILED):
            return None
        if (
            not self._command_enabled
            or self._state == self.INACTIVE
            or not self._streaming_client.service_is_ready()
        ):
            return None
        self._state = self.DISABLING
        request = SetBool.Request()
        request.data = False
        return self._streaming_client.call_async(request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhoneEefTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        future = node.stop_for_shutdown()
        if future is not None and rclpy.ok():
            rclpy.spin_until_future_complete(node, future, timeout_sec=0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
