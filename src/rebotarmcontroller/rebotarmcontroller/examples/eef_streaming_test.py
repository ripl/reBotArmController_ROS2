#!/usr/bin/env python3
"""Move to a ready pose, then stream a deterministic lateral EEF trajectory."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.msg import ArmStatus
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

_X = 0.3
_Y_START = 0.0
_Y_END = 0.4
_Z = 0.3
_MOVE_DURATION = 2.0
_READY_DURATION = 2.0
_PUBLISH_RATE_HZ = 50.0
_FINAL_HOLD_DURATION = 0.5


def profile_progress(profile: str, normalized_time: float) -> float:
    """Return normalized position for the requested time profile."""
    t = max(0.0, min(1.0, float(normalized_time)))
    if profile == "linear":
        return t
    if profile == "sine":
        return 0.5 - 0.5 * math.cos(math.pi * t)
    raise ValueError(f"unsupported profile: {profile}")


class EefStreamingTest(Node):
    def __init__(self, profile: str) -> None:
        super().__init__("eef_streaming_test")
        self._profile = profile
        self._stop_requested = False
        self._streaming_enabled = False
        self._phase = "initializing"
        self._events: list[dict[str, object]] = []
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self._log_path = Path(
            f"/tmp/rebotarm_eef_streaming_{profile}_{timestamp}.jsonl"
        )
        self._move_to_pose = ActionClient(
            self,
            MoveToPose,
            "/rebotarm/move_to_pose",
        )
        self._streaming = self.create_client(
            SetBool,
            "/rebotarm/eef_streaming/enable",
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._target = self.create_publisher(
            PoseStamped,
            "/rebotarm/eef_target_pose",
            qos,
        )
        self.create_subscription(
            PoseStamped,
            "/rebotarm/eef_target_pose",
            self._target_loopback_callback,
            qos,
        )
        joint_state_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            JointState,
            "/rebotarm/joint_states",
            self._joint_state_callback,
            joint_state_qos,
        )
        self.create_subscription(
            ArmStatus,
            "/rebotarm/arm_status",
            self._arm_status_callback,
            10,
        )
        self._record("program_start", profile=profile)

    def request_stop(self) -> None:
        self._stop_requested = True
        self._record("stop_requested")

    def run(self) -> bool:
        self._set_phase("move_to_ready")
        if not self._move_to_ready_pose():
            return False
        self._set_phase("ready")
        self._set_phase("streaming_enable")
        if self._stop_requested or not self._set_streaming(True):
            return False

        self._set_phase("streaming")
        self.get_logger().info(
            f"streaming {self._profile} profile: "
            f"y={_Y_START:.3f} -> {_Y_END:.3f} m in {_MOVE_DURATION:.1f} s "
            f"at {_PUBLISH_RATE_HZ:.0f} Hz"
        )
        started = time.monotonic()
        if not self._publish_trajectory():
            return False
        elapsed = time.monotonic() - started
        self.get_logger().info(
            f"streaming trajectory complete in {elapsed:.3f} s"
        )
        return True

    def cleanup(self) -> None:
        if self._streaming_enabled:
            try:
                self._set_phase("streaming_disable")
                self._set_streaming(False)
            except Exception as exc:
                self._record("cleanup_error", message=str(exc))
                self.get_logger().error(f"failed to disable streaming: {exc}")
        self._set_phase("done")

    def save_log(self) -> None:
        metadata = {
            "event": "metadata",
            "format_version": 1,
            "profile": self._profile,
            "x": _X,
            "y_start": _Y_START,
            "y_end": _Y_END,
            "z": _Z,
            "move_duration_s": _MOVE_DURATION,
            "ready_duration_s": _READY_DURATION,
            "target_publish_rate_hz": _PUBLISH_RATE_HZ,
            "final_hold_duration_s": _FINAL_HOLD_DURATION,
        }
        with self._log_path.open("w", encoding="utf-8") as stream:
            stream.write(json.dumps(metadata, separators=(",", ":")) + "\n")
            for event in self._events:
                stream.write(json.dumps(event, separators=(",", ":")) + "\n")
        self.get_logger().info(f"diagnostic log saved to {self._log_path}")

    def record_error(self, message: str) -> None:
        self._record("program_error", message=message)

    def _move_to_ready_pose(self) -> bool:
        if not self._move_to_pose.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("move_to_pose action not available")
            return False

        goal = MoveToPose.Goal()
        goal.target_pose.position.x = _X
        goal.target_pose.position.y = _Y_START
        goal.target_pose.position.z = _Z
        goal.target_pose.orientation.w = 1.0
        goal.duration = _READY_DURATION

        self.get_logger().info(
            "moving to ready pose: "
            f"x={_X:.3f}, y={_Y_START:.3f}, z={_Z:.3f}, identity orientation"
        )
        self._record(
            "action_goal",
            x=_X,
            y=_Y_START,
            z=_Z,
            duration_s=_READY_DURATION,
        )
        send_future = self._move_to_pose.send_goal_async(goal)
        if not self._wait_for_future(send_future, 5.0):
            self._record("action_goal_timeout")
            self.get_logger().error("move_to_pose goal request timed out")
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._record("action_goal_rejected")
            self.get_logger().error("move_to_pose goal rejected")
            return False
        self._record("action_goal_accepted")

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, _READY_DURATION + 5.0):
            cancel_future = goal_handle.cancel_goal_async()
            self._wait_for_future(cancel_future, 1.0, allow_stop=True)
            self._record("action_result_timeout")
            self.get_logger().error(
                "move_to_pose result timed out or was interrupted"
            )
            return False
        result_response = result_future.result()
        result = None if result_response is None else result_response.result
        if result is None or not result.success:
            message = "no result" if result is None else result.message
            self._record("action_result", success=False, message=message)
            self.get_logger().error(f"move_to_pose failed: {message}")
            return False
        self._record("action_result", success=True, message=result.message)
        self.get_logger().info("ready pose reached")
        return True

    def _set_streaming(self, enabled: bool) -> bool:
        label = "enable" if enabled else "disable"
        if not self._streaming.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("EEF streaming service not available")
            return False
        request = SetBool.Request()
        request.data = enabled
        future = self._streaming.call_async(request)
        if not self._wait_for_future(future, 5.0, allow_stop=not enabled):
            self.get_logger().error(f"EEF streaming {label} timed out")
            return False
        response = future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            self._record(
                "streaming_service",
                enabled=enabled,
                success=False,
                message=message,
            )
            self.get_logger().error(f"EEF streaming {label} failed: {message}")
            return False
        self._streaming_enabled = enabled
        self._record(
            "streaming_service",
            enabled=enabled,
            success=True,
            message=response.message,
        )
        self.get_logger().info(f"EEF streaming {label}d")
        return True

    def _publish_trajectory(self) -> bool:
        period = 1.0 / _PUBLISH_RATE_HZ
        sample_count = int(math.ceil(_MOVE_DURATION * _PUBLISH_RATE_HZ))
        start = time.monotonic()

        for index in range(sample_count + 1):
            deadline = start + min(index * period, _MOVE_DURATION)
            if not self._wait_until(deadline):
                return False
            trajectory_time = min(index * period, _MOVE_DURATION)
            normalized_time = trajectory_time / _MOVE_DURATION
            progress = profile_progress(self._profile, normalized_time)
            y = _Y_START + (_Y_END - _Y_START) * progress
            self._publish_pose(
                y,
                trajectory_time_s=trajectory_time,
                progress=progress,
                scheduled_monotonic_ns=int(deadline * 1e9),
            )

        self._set_phase("final_hold")
        hold_end = time.monotonic() + _FINAL_HOLD_DURATION
        next_publish = time.monotonic() + period
        while next_publish < hold_end:
            if not self._wait_until(next_publish):
                return False
            self._publish_pose(
                _Y_END,
                trajectory_time_s=_MOVE_DURATION,
                progress=1.0,
                scheduled_monotonic_ns=int(next_publish * 1e9),
            )
            next_publish += period
        return True

    def _publish_pose(
        self,
        y: float,
        *,
        trajectory_time_s: float,
        progress: float,
        scheduled_monotonic_ns: int,
    ) -> None:
        actual_monotonic_ns = time.monotonic_ns()
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.pose.position.x = _X
        message.pose.position.y = float(y)
        message.pose.position.z = _Z
        message.pose.orientation.w = 1.0
        self._target.publish(message)
        self._record(
            "target_publish",
            monotonic_ns=actual_monotonic_ns,
            scheduled_monotonic_ns=scheduled_monotonic_ns,
            schedule_error_ns=actual_monotonic_ns - scheduled_monotonic_ns,
            message_stamp_ns=self._stamp_to_ns(message.header.stamp),
            trajectory_time_s=trajectory_time_s,
            progress=progress,
            x=_X,
            y=float(y),
            z=_Z,
        )

    def _wait_until(self, deadline: float) -> bool:
        while not self._stop_requested:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return True
            rclpy.spin_once(self, timeout_sec=min(remaining, 0.005))
        return False

    def _wait_for_future(
        self,
        future,
        timeout_sec: float,
        *,
        allow_stop: bool = False,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                return False
            if self._stop_requested and not allow_stop:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def _target_loopback_callback(self, message: PoseStamped) -> None:
        self._record(
            "target_loopback",
            message_stamp_ns=self._stamp_to_ns(message.header.stamp),
            frame_id=message.header.frame_id,
            x=float(message.pose.position.x),
            y=float(message.pose.position.y),
            z=float(message.pose.position.z),
            qx=float(message.pose.orientation.x),
            qy=float(message.pose.orientation.y),
            qz=float(message.pose.orientation.z),
            qw=float(message.pose.orientation.w),
        )

    def _joint_state_callback(self, message: JointState) -> None:
        self._record(
            "joint_state",
            message_stamp_ns=self._stamp_to_ns(message.header.stamp),
            names=list(message.name),
            position=[float(value) for value in message.position],
            velocity=[float(value) for value in message.velocity],
            effort=[float(value) for value in message.effort],
        )

    def _arm_status_callback(self, message: ArmStatus) -> None:
        self._record(
            "arm_status",
            message_stamp_ns=self._stamp_to_ns(message.header.stamp),
            mode=message.mode,
            enabled=bool(message.enabled),
            control_loop_active=bool(message.control_loop_active),
            state_machine=message.state_machine,
            error_codes=list(message.error_codes),
        )

    def _set_phase(self, phase: str) -> None:
        self._phase = phase
        self._record("phase")

    def _record(
        self,
        event: str,
        *,
        monotonic_ns: int | None = None,
        **values,
    ) -> None:
        self._events.append(
            {
                "event": event,
                "phase": self._phase,
                "monotonic_ns": (
                    time.monotonic_ns()
                    if monotonic_ns is None
                    else monotonic_ns
                ),
                **values,
            }
        )

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        choices=("linear", "sine"),
        default="linear",
        help="linear or half-cosine position profile",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = EefStreamingTest(args.profile)

    def request_stop(_signum, _frame) -> None:
        node.request_stop()

    old_sigint = signal.getsignal(signal.SIGINT)
    old_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        ok = node.run()
    except Exception as exc:
        node.record_error(str(exc))
        node.get_logger().error(str(exc))
        ok = False
    finally:
        node.cleanup()
        node.save_log()
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
