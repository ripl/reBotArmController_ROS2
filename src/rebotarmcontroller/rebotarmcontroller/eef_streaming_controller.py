from __future__ import annotations

import math
import threading
import time

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool


class EefStreamingController:
    """Tracks the latest base-frame end-effector pose setpoint."""

    def __init__(self, node, hardware, namespace: str) -> None:
        self._node = node
        self._hardware = hardware
        self._lock = threading.Lock()
        self._control_group = MutuallyExclusiveCallbackGroup()
        self._active = False
        self._latest_target: np.ndarray | None = None
        self._latest_target_time: float | None = None
        self._q_command: np.ndarray | None = None
        self._pose_command: np.ndarray | None = None

        self._rate_hz = self._positive_scalar("eef_streaming.control_rate_hz")
        self._timeout = self._positive_scalar("eef_streaming.target_timeout")
        self._max_linear_velocity = self._positive_scalar(
            "eef_streaming.max_linear_velocity"
        )
        self._max_angular_velocity = self._positive_scalar(
            "eef_streaming.max_angular_velocity"
        )
        self._joint_velocity_limits = self._positive_vector(
            "eef_streaming.joint_velocity_limits", len(hardware.joint_names)
        )
        self._workspace_min = self._vector("eef_streaming.workspace_min", 3)
        self._workspace_max = self._vector("eef_streaming.workspace_max", 3)
        if np.any(self._workspace_min >= self._workspace_max):
            raise ValueError("eef streaming workspace_min must be below workspace_max")

        margin = self._positive_or_zero_scalar("eef_streaming.joint_limit_margin")
        lower, upper = hardware.joint_position_limits
        self._joint_lower = np.where(np.isfinite(lower), lower + margin, -np.inf)
        self._joint_upper = np.where(np.isfinite(upper), upper - margin, np.inf)
        if np.any(self._joint_lower >= self._joint_upper):
            raise ValueError("eef streaming joint_limit_margin is too large")

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._subscription = node.create_subscription(
            PoseStamped,
            f"/{namespace}/eef_target_pose",
            self._target_callback,
            qos,
            callback_group=node.reentrant_group,
        )
        self._enable_service = node.create_service(
            SetBool,
            f"/{namespace}/eef_streaming/enable",
            self._enable_callback,
            callback_group=self._control_group,
        )
        self._timer = node.create_timer(
            1.0 / self._rate_hz,
            self._control_tick,
            callback_group=self._control_group,
        )

    def _positive_scalar(self, name: str) -> float:
        value = float(self._node.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f"{name} must be positive")
        return value

    def _positive_or_zero_scalar(self, name: str) -> float:
        value = float(self._node.get_parameter(name).value)
        if value < 0.0:
            raise ValueError(f"{name} must be non-negative")
        return value

    def _vector(self, name: str, size: int) -> np.ndarray:
        values = np.asarray(self._node.get_parameter(name).value, dtype=np.float64)
        if values.shape != (size,) or not np.all(np.isfinite(values)):
            raise ValueError(f"{name} must contain {size} finite values")
        return values

    def _positive_vector(self, name: str, size: int) -> np.ndarray:
        values = self._vector(name, size)
        if np.any(values <= 0.0):
            raise ValueError(f"{name} must contain positive values")
        return values

    def _target_callback(self, msg: PoseStamped) -> None:
        target = self._pose_to_array(msg.pose)
        if target is None:
            self._node.get_logger().warn("ignoring invalid EEF target pose")
            return
        with self._lock:
            self._latest_target = target
            self._latest_target_time = time.monotonic()

    def _enable_callback(self, request, response):
        if request.data:
            try:
                q_current = self._hardware.begin_eef_streaming()
                pose_current = self._hardware.pose_from_joint_positions(q_current)
            except RuntimeError as exc:
                response.success = False
                response.message = str(exc)
                return response
            self._q_command = q_current
            self._pose_command = self._pose_to_array(pose_current)
            self._active = True
            response.success = True
            response.message = "EEF streaming enabled"
        else:
            self._stop_streaming()
            response.success = True
            response.message = "EEF streaming disabled"
        self._node.publish_arm_status()
        return response

    def _control_tick(self) -> None:
        if not self._active:
            return
        with self._lock:
            target = None if self._latest_target is None else self._latest_target.copy()
            target_time = self._latest_target_time
        if target is None or target_time is None:
            return
        if time.monotonic() - target_time > self._timeout:
            self._stop_streaming()
            self._node.get_logger().warn("EEF streaming target timed out; holding position")
            return
        if np.any(target[:3] < self._workspace_min) or np.any(
            target[:3] > self._workspace_max
        ):
            self._stop_streaming()
            self._node.get_logger().warn("EEF streaming target is outside the workspace")
            return

        assert self._q_command is not None and self._pose_command is not None
        limited_pose = self._limit_pose(target)
        try:
            q_ik = self._hardware.solve_eef_ik(limited_pose, self._q_command)
        except RuntimeError:
            self._active = False
            return
        if q_ik is None or np.any(q_ik < self._joint_lower) or np.any(
            q_ik > self._joint_upper
        ):
            self._stop_streaming()
            self._node.get_logger().warn("EEF streaming target has no safe IK solution")
            return

        period = 1.0 / self._rate_hz
        max_delta = self._joint_velocity_limits * period
        q_next = self._q_command + np.clip(q_ik - self._q_command, -max_delta, max_delta)
        try:
            self._hardware.set_eef_streaming_target(q_next, self._joint_velocity_limits)
            pose_next = self._hardware.pose_from_joint_positions(q_next)
        except RuntimeError:
            self._active = False
            return
        self._q_command = q_next
        self._pose_command = self._pose_to_array(pose_next)

    def _limit_pose(self, target: np.ndarray) -> Pose:
        assert self._pose_command is not None
        period = 1.0 / self._rate_hz
        position = self._pose_command[:3]
        delta = target[:3] - position
        distance = float(np.linalg.norm(delta))
        max_distance = self._max_linear_velocity * period
        if distance > max_distance:
            delta *= max_distance / distance
        orientation = self._slerp_limited(
            self._pose_command[3:],
            target[3:],
            self._max_angular_velocity * period,
        )

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position + delta)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
            float(v) for v in orientation
        )
        return pose

    @staticmethod
    def _pose_to_array(pose: Pose) -> np.ndarray | None:
        values = np.array(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(values[3:]))
        if not np.all(np.isfinite(values)) or norm < 1e-6:
            return None
        values[3:] /= norm
        return values

    @staticmethod
    def _slerp_limited(
        start: np.ndarray,
        target: np.ndarray,
        max_angle: float,
    ) -> np.ndarray:
        end = target.copy()
        dot = float(np.dot(start, end))
        if dot < 0.0:
            end *= -1.0
            dot *= -1.0
        dot = max(-1.0, min(1.0, dot))
        angle = 2.0 * math.acos(dot)
        if angle <= max_angle:
            return end
        ratio = max_angle / angle
        half_angle = math.acos(dot)
        if half_angle < 1e-6:
            return start
        scale_start = math.sin((1.0 - ratio) * half_angle) / math.sin(half_angle)
        scale_end = math.sin(ratio * half_angle) / math.sin(half_angle)
        return scale_start * start + scale_end * end

    def _stop_streaming(self) -> None:
        if not self._active:
            return
        self._hardware.stop_eef_streaming()
        self._active = False
        self._node.publish_arm_status()
