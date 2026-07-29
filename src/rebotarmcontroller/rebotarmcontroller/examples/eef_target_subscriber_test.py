#!/usr/bin/env python3
"""Subscribe to EEF target poses without connecting robot hardware."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import statistics
import time

from geometry_msgs.msg import PoseStamped, TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from tf2_ros import TransformBroadcaster


class EefTargetSubscriberTest(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("eef_target_subscriber_test")
        self._args = args
        self._stop_requested = False
        self._events: list[dict[str, object]] = []
        self._receive_times_ns: list[int] = []
        self._last_callback_ns: int | None = None
        self._target_tf_broadcaster = TransformBroadcaster(self)
        namespace = args.namespace.strip("/")
        topic = f"/{namespace}/eef_target_pose"
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseStamped, topic, self._target_callback, qos)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self._log_path = Path(args.output_dir) / (
            f"eef_target_subscriber_{timestamp}.jsonl"
        )
        self._record(
            "program_start",
            topic=topic,
            duration_s=args.duration,
            publish_target_tf=args.publish_target_tf,
        )

    def request_stop(self) -> None:
        self._stop_requested = True
        self._record("stop_requested")

    def run(self) -> bool:
        deadline = time.monotonic() + self._args.duration
        while rclpy.ok() and not self._stop_requested:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return True
            rclpy.spin_once(self, timeout_sec=min(remaining, 0.05))
        return not self._stop_requested

    def save_log(self) -> None:
        self._log_path.parent.mkdir(parents=True, exist_ok=True)
        metadata = {
            "event": "metadata",
            "format_version": 1,
            "duration_s": self._args.duration,
            "publish_target_tf": self._args.publish_target_tf,
        }
        with self._log_path.open("w", encoding="utf-8") as stream:
            stream.write(json.dumps(metadata, separators=(",", ":")) + "\n")
            for event in self._events:
                stream.write(json.dumps(event, separators=(",", ":")) + "\n")
        self.get_logger().info(f"diagnostic log saved to {self._log_path}")

    def print_summary(self) -> None:
        count = len(self._receive_times_ns)
        if count < 2:
            print(f"received targets: {count}")
            return
        intervals = [
            self._receive_times_ns[index] - self._receive_times_ns[index - 1]
            for index in range(1, count)
        ]
        elapsed_s = (self._receive_times_ns[-1] - self._receive_times_ns[0]) * 1e-9
        actual_rate = (count - 1) / elapsed_s if elapsed_s > 0.0 else 0.0
        print(f"received targets: {count}")
        print(f"actual receive rate: {actual_rate:.2f} Hz")
        print(
            "receive interval: "
            f"mean={statistics.fmean(intervals) * 1e-6:.3f}ms "
            f"p95={self._percentile_ms(intervals, 0.95):.3f}ms "
            f"max={max(intervals) * 1e-6:.3f}ms"
        )
        print(
            "large gaps: "
            f">25ms={sum(value > 25_000_000 for value in intervals)} "
            f">40ms={sum(value > 40_000_000 for value in intervals)} "
            f">100ms={sum(value > 100_000_000 for value in intervals)}"
        )

    def record_error(self, message: str) -> None:
        self._record("program_error", message=message)

    def _target_callback(self, message: PoseStamped) -> None:
        callback_ns = time.monotonic_ns()
        previous_callback_ns = self._last_callback_ns
        self._last_callback_ns = callback_ns
        self._receive_times_ns.append(callback_ns)
        message_stamp_ns = self._stamp_to_ns(message.header.stamp)
        self._record(
            "target_received",
            monotonic_ns=callback_ns,
            callback_interval_ns=(
                None
                if previous_callback_ns is None
                else callback_ns - previous_callback_ns
            ),
            message_stamp_ns=message_stamp_ns,
            receive_wall_time_ns=time.time_ns(),
            frame_id=message.header.frame_id,
            x=float(message.pose.position.x),
            y=float(message.pose.position.y),
            z=float(message.pose.position.z),
            qx=float(message.pose.orientation.x),
            qy=float(message.pose.orientation.y),
            qz=float(message.pose.orientation.z),
            qw=float(message.pose.orientation.w),
        )
        if self._args.publish_target_tf:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self._args.frame_id
            transform.child_frame_id = "eef_target_subscriber_test"
            transform.transform.translation.x = float(message.pose.position.x)
            transform.transform.translation.y = float(message.pose.position.y)
            transform.transform.translation.z = float(message.pose.position.z)
            transform.transform.rotation = message.pose.orientation
            self._target_tf_broadcaster.sendTransform(transform)

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

    @staticmethod
    def _percentile_ms(values: list[int], percentile: float) -> float:
        if not values:
            return 0.0
        ordered = sorted(values)
        index = min(len(ordered) - 1, math.ceil(len(ordered) * percentile) - 1)
        return ordered[index] * 1e-6


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--namespace", default="rebotarm")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--output-dir", default="diagnostic_logs")
    parser.add_argument(
        "--publish-target-tf",
        action="store_true",
        help="publish a TF frame from each received target callback",
    )
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    return args


def main() -> None:
    args = _parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = EefTargetSubscriberTest(args)

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
        node.print_summary()
        node.save_log()
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
