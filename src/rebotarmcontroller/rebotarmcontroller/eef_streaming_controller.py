from __future__ import annotations

import math
import threading
import time

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from .streaming_diagnostics import StreamingDiagnostics


class EefStreamingController:
    """Tracks the latest base-frame end-effector pose setpoint."""

    def __init__(
        self,
        node,
        hardware,
        namespace: str,
        diagnostics: StreamingDiagnostics | None = None,
    ) -> None:
        self._node = node
        self._hardware = hardware
        self._diagnostics = diagnostics
        self._target_callback_diagnostics: StreamingDiagnostics | None = None
        self._frame_id = str(node.get_parameter("frame_id").value)
        self._target_tf_broadcaster = TransformBroadcaster(node)
        self._lock = threading.Lock()
        self._control_group = MutuallyExclusiveCallbackGroup()
        self._active = False
        self._latest_target: np.ndarray | None = None
        self._latest_target_time: float | None = None
        self._q_command: np.ndarray | None = None
        self._pose_command: np.ndarray | None = None
        self._last_target_callback_ns: int | None = None
        self._last_control_tick_ns: int | None = None
        self._control_stop = threading.Event()
        self._control_thread: threading.Thread | None = None
        self._stream_start_ns: int | None = None

        self._rate_hz = self._positive_scalar("eef_streaming.control_rate_hz")
        self._publish_target_tf = self._bool_param("eef_streaming.publish_target_tf")
        self._diagnostics_detail = self._bool_param("eef_streaming.diagnostics_detail")
        self._target_callback_diagnostics_enabled = self._bool_param(
            "eef_streaming.target_callback_diagnostics_enabled"
        )
        self._internal_target_enabled = self._bool_param(
            "eef_streaming.internal_target_enabled"
        )
        self._internal_target_x = float(
            node.get_parameter("eef_streaming.internal_target_x").value
        )
        self._internal_target_y_start = float(
            node.get_parameter("eef_streaming.internal_target_y_start").value
        )
        self._internal_target_y_end = float(
            node.get_parameter("eef_streaming.internal_target_y_end").value
        )
        self._internal_target_z = float(
            node.get_parameter("eef_streaming.internal_target_z").value
        )
        self._internal_target_duration = self._positive_scalar(
            "eef_streaming.internal_target_duration"
        )
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

        self._subscription = None
        target_topic = f"/{namespace}/eef_target_pose"
        if not self._internal_target_enabled:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self._subscription = node.create_subscription(
                PoseStamped,
                target_topic,
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
        if (
            self._target_callback_diagnostics_enabled
        ):
            self._target_callback_diagnostics = StreamingDiagnostics(
                node.get_logger()
            )
            self._target_callback_diagnostics.start(
                mode="target_callback_only",
                target_topic=target_topic,
                publish_target_tf=self._publish_target_tf,
                diagnostics_detail=self._diagnostics_detail,
                internal_target_enabled=self._internal_target_enabled,
            )

    def _positive_scalar(self, name: str) -> float:
        value = float(self._node.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f"{name} must be positive")
        return value

    def _bool_param(self, name: str) -> bool:
        return bool(self._node.get_parameter(name).value)

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
        callback_ns = time.monotonic_ns()
        previous_callback_ns = self._last_target_callback_ns
        self._last_target_callback_ns = callback_ns
        target = self._pose_to_array(msg.pose)
        if target is None:
            self._node.get_logger().warn("ignoring invalid EEF target pose")
            return
        with self._lock:
            self._latest_target = target
            self._latest_target_time = callback_ns * 1e-9
        values = {
            "callback_interval_ns": (
                None
                if previous_callback_ns is None
                else callback_ns - previous_callback_ns
            )
        }
        if self._diagnostics_detail:
            values["target"] = target.tolist()
        self._record_diagnostic(
            "target_received",
            monotonic_ns=callback_ns,
            **values,
        )
        if self._target_callback_diagnostics is not None:
            self._target_callback_diagnostics.record(
                "target_received",
                monotonic_ns=callback_ns,
                **values,
            )
        if not self._publish_target_tf:
            return
        transform = TransformStamped()
        transform.header.stamp = self._node.get_clock().now().to_msg()
        transform.header.frame_id = self._frame_id
        transform.child_frame_id = "eef_target"
        transform.transform.translation.x = float(target[0])
        transform.transform.translation.y = float(target[1])
        transform.transform.translation.z = float(target[2])
        transform.transform.rotation.x = float(target[3])
        transform.transform.rotation.y = float(target[4])
        transform.transform.rotation.z = float(target[5])
        transform.transform.rotation.w = float(target[6])
        self._target_tf_broadcaster.sendTransform(transform)

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
            with self._lock:
                self._latest_target = None
                self._latest_target_time = None
            self._stop_target_callback_diagnostics("streaming_enable")
            if self._diagnostics is not None:
                if self._diagnostics.active:
                    self._diagnostics.stop("streaming_enable")
                self._diagnostics.start(
                    control_rate_hz=self._rate_hz,
                    target_timeout_s=self._timeout,
                    max_linear_velocity=self._max_linear_velocity,
                    max_angular_velocity=self._max_angular_velocity,
                    joint_velocity_limits=self._joint_velocity_limits.tolist(),
                    q_initial=q_current.tolist(),
                    pose_initial=self._pose_command.tolist(),
                    publish_target_tf=self._publish_target_tf,
                    diagnostics_detail=self._diagnostics_detail,
                    internal_target_enabled=self._internal_target_enabled,
                )
            self._last_target_callback_ns = None
            self._last_control_tick_ns = None
            self._stream_start_ns = time.monotonic_ns()
            self._active = True
            self._start_control_thread()
            response.success = True
            response.message = "EEF streaming enabled"
        else:
            self._stop_streaming("service_disable")
            response.success = True
            response.message = "EEF streaming disabled"
        self._node.publish_arm_status()
        return response

    def shutdown(self) -> None:
        if self._active:
            self._stop_streaming("shutdown")
        else:
            self._stop_control_thread()
            if self._target_callback_diagnostics_enabled:
                self._stop_target_callback_diagnostics("shutdown")

    def _start_control_thread(self) -> None:
        if self._control_thread is not None and self._control_thread.is_alive():
            return
        self._control_stop.clear()
        self._control_thread = threading.Thread(
            target=self._control_loop,
            name="eef_streaming_control",
            daemon=True,
        )
        self._control_thread.start()

    def _stop_control_thread(self) -> None:
        self._control_stop.set()
        thread = self._control_thread
        if thread is None or thread is threading.current_thread():
            return
        thread.join(timeout=1.0)
        if thread.is_alive():
            self._node.get_logger().warn("EEF streaming control thread did not stop")
            return
        self._control_thread = None

    def _control_loop(self) -> None:
        period_ns = int(1_000_000_000 / self._rate_hz)
        next_tick_ns = time.monotonic_ns()
        self._record_diagnostic(
            "control_loop",
            monotonic_ns=next_tick_ns,
            outcome="started",
            rate_hz=self._rate_hz,
            period_ns=period_ns,
        )
        stop_reason = "stop_requested"
        try:
            while not self._control_stop.is_set():
                if not self._active:
                    stop_reason = "inactive"
                    return

                now_ns = time.monotonic_ns()
                wait_ns = next_tick_ns - now_ns
                if wait_ns > 0:
                    if self._control_stop.wait(wait_ns * 1e-9):
                        return

                wake_ns = time.monotonic_ns()
                lateness_ns = max(0, wake_ns - next_tick_ns)
                self._record_diagnostic(
                    "control_loop_tick",
                    monotonic_ns=wake_ns,
                    scheduled_monotonic_ns=next_tick_ns,
                    wake_lateness_ns=lateness_ns,
                    period_ns=period_ns,
                    outcome="late" if lateness_ns > period_ns else "scheduled",
                )
                self._control_tick()

                tick_end_ns = time.monotonic_ns()
                next_tick_ns += period_ns
                if next_tick_ns < tick_end_ns:
                    self._record_diagnostic(
                        "control_loop_resync",
                        monotonic_ns=tick_end_ns,
                        behind_ns=tick_end_ns - next_tick_ns,
                        period_ns=period_ns,
                    )
                    next_tick_ns = tick_end_ns + period_ns
        finally:
            self._record_diagnostic(
                "control_loop",
                outcome=stop_reason,
                period_ns=period_ns,
            )

    def _control_tick(self) -> None:
        if not self._active:
            return
        tick_start_ns = time.monotonic_ns()
        previous_tick_ns = self._last_control_tick_ns
        self._last_control_tick_ns = tick_start_ns
        tick_interval_ns = (
            None if previous_tick_ns is None else tick_start_ns - previous_tick_ns
        )
        if self._internal_target_enabled:
            target = self._internal_target(tick_start_ns)
            target_time = tick_start_ns * 1e-9
        else:
            with self._lock:
                target = None if self._latest_target is None else self._latest_target.copy()
                target_time = self._latest_target_time
        if target is None or target_time is None:
            self._record_diagnostic(
                "control_tick",
                monotonic_ns=tick_start_ns,
                tick_interval_ns=tick_interval_ns,
                outcome="no_target",
                total_duration_ns=time.monotonic_ns() - tick_start_ns,
            )
            return
        target_age_ns = tick_start_ns - int(target_time * 1e9)
        if target_age_ns * 1e-9 > self._timeout:
            self._record_diagnostic(
                "control_tick",
                monotonic_ns=tick_start_ns,
                tick_interval_ns=tick_interval_ns,
                target_age_ns=target_age_ns,
                outcome="target_timeout",
                total_duration_ns=time.monotonic_ns() - tick_start_ns,
            )
            self._stop_streaming("target_timeout")
            self._node.get_logger().warn("EEF streaming target timed out; holding position")
            return
        if np.any(target[:3] < self._workspace_min) or np.any(
            target[:3] > self._workspace_max
        ):
            self._record_diagnostic(
                "control_tick",
                monotonic_ns=tick_start_ns,
                tick_interval_ns=tick_interval_ns,
                target_age_ns=target_age_ns,
                outcome="outside_workspace",
                total_duration_ns=time.monotonic_ns() - tick_start_ns,
            )
            self._stop_streaming("outside_workspace")
            self._node.get_logger().warn("EEF streaming target is outside the workspace")
            return

        assert self._q_command is not None and self._pose_command is not None
        q_command = self._q_command.copy() if self._diagnostics_detail else None
        pose_command = self._pose_command.copy() if self._diagnostics_detail else None
        limited_pose = self._limit_pose(target)
        limited_pose_array = (
            self._pose_to_array(limited_pose) if self._diagnostics_detail else None
        )
        solve_start_ns = time.monotonic_ns()
        try:
            q_ik = self._hardware.solve_eef_ik(limited_pose, self._q_command)
        except RuntimeError:
            self._active = False
            self._record_diagnostic(
                "control_tick",
                monotonic_ns=tick_start_ns,
                tick_interval_ns=tick_interval_ns,
                target_age_ns=target_age_ns,
                outcome="streaming_inactive_during_ik",
                total_duration_ns=time.monotonic_ns() - tick_start_ns,
            )
            self._stop_diagnostics("streaming_inactive_during_ik")
            return
        solve_end_ns = time.monotonic_ns()
        if q_ik is None or np.any(q_ik < self._joint_lower) or np.any(
            q_ik > self._joint_upper
        ):
            self._record_diagnostic(
                "control_tick",
                monotonic_ns=tick_start_ns,
                tick_interval_ns=tick_interval_ns,
                target_age_ns=target_age_ns,
                solve_duration_ns=solve_end_ns - solve_start_ns,
                outcome="unsafe_ik",
                total_duration_ns=time.monotonic_ns() - tick_start_ns,
            )
            self._stop_streaming("unsafe_ik")
            self._node.get_logger().warn("EEF streaming target has no safe IK solution")
            return

        period = 1.0 / self._rate_hz
        max_delta = self._joint_velocity_limits * period
        q_next = self._q_command + np.clip(q_ik - self._q_command, -max_delta, max_delta)
        set_target_start_ns = time.monotonic_ns()
        try:
            command_sequence = self._hardware.set_eef_streaming_target(
                q_next,
                self._joint_velocity_limits,
            )
            set_target_end_ns = time.monotonic_ns()
            fk_start_ns = set_target_end_ns
            pose_next = self._hardware.pose_from_joint_positions(q_next)
        except RuntimeError:
            self._active = False
            self._record_diagnostic(
                "control_tick",
                monotonic_ns=tick_start_ns,
                tick_interval_ns=tick_interval_ns,
                target_age_ns=target_age_ns,
                outcome="streaming_inactive_during_command",
                total_duration_ns=time.monotonic_ns() - tick_start_ns,
            )
            self._stop_diagnostics("streaming_inactive_during_command")
            return
        fk_end_ns = time.monotonic_ns()
        self._q_command = q_next
        self._pose_command = self._pose_to_array(pose_next)
        values = {
            "tick_interval_ns": tick_interval_ns,
            "target_age_ns": target_age_ns,
            "command_sequence": command_sequence,
            "solve_duration_ns": solve_end_ns - solve_start_ns,
            "set_target_duration_ns": set_target_end_ns - set_target_start_ns,
            "fk_duration_ns": fk_end_ns - fk_start_ns,
            "outcome": "commanded",
            "total_duration_ns": fk_end_ns - tick_start_ns,
        }
        if self._diagnostics_detail:
            assert q_command is not None and pose_command is not None
            values.update(
                target=target.tolist(),
                pose_command=pose_command.tolist(),
                limited_pose=(
                    None if limited_pose_array is None else limited_pose_array.tolist()
                ),
                q_command=q_command.tolist(),
                q_ik=q_ik.tolist(),
                q_next=q_next.tolist(),
            )
        self._record_diagnostic(
            "control_tick",
            monotonic_ns=tick_start_ns,
            **values,
        )

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

    def _internal_target(self, now_ns: int) -> np.ndarray:
        start_ns = self._stream_start_ns if self._stream_start_ns is not None else now_ns
        elapsed = max(0.0, (now_ns - start_ns) * 1e-9)
        progress = min(1.0, elapsed / self._internal_target_duration)
        sine_progress = 0.5 - 0.5 * math.cos(math.pi * progress)
        y = self._internal_target_y_start + (
            self._internal_target_y_end - self._internal_target_y_start
        ) * sine_progress
        return np.array(
            [
                self._internal_target_x,
                y,
                self._internal_target_z,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
            dtype=np.float64,
        )

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

    def _stop_streaming(self, reason: str) -> None:
        self._stop_control_thread()
        if not self._active:
            self._stop_diagnostics(reason)
            return
        self._hardware.stop_eef_streaming()
        self._active = False
        self._node.publish_arm_status()
        self._stop_diagnostics(reason)

    def _record_diagnostic(
        self,
        event: str,
        *,
        monotonic_ns: int | None = None,
        **values,
    ) -> None:
        if self._diagnostics is not None:
            self._diagnostics.record(event, monotonic_ns=monotonic_ns, **values)

    def _stop_diagnostics(self, reason: str) -> None:
        if self._diagnostics is not None:
            self._diagnostics.stop(reason)

    def _stop_target_callback_diagnostics(self, reason: str) -> None:
        if self._target_callback_diagnostics is not None:
            self._target_callback_diagnostics.stop(reason)
            self._target_callback_diagnostics = None
