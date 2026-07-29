#!/usr/bin/env python3
"""Publish EEF target poses without enabling controller-side EEF streaming."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import statistics
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


def profile_progress(profile: str, normalized_time: float) -> float:
    t = max(0.0, min(1.0, float(normalized_time)))
    if profile == "linear":
        return t
    if profile == "sine":
        return 0.5 - 0.5 * math.cos(math.pi * t)
    raise ValueError(f"unsupported profile: {profile}")


class EefTargetPublisherTest(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("eef_target_publisher_test")
        self._args = args
        self._stop_requested = False
        self._events: list[dict[str, object]] = []
        self._publish_times_ns: list[int] = []
        namespace = args.namespace.strip("/")
        topic = f"/{namespace}/eef_target_pose"
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = self.create_publisher(PoseStamped, topic, qos)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self._log_path = Path(args.output_dir) / (
            f"eef_target_publisher_{args.profile}_{timestamp}.jsonl"
        )
        self._record(
            "program_start",
            topic=topic,
            rate_hz=args.rate,
            duration_s=args.duration,
            profile=args.profile,
        )

    def request_stop(self) -> None:
        self._stop_requested = True
        self._record("stop_requested")

    def run(self) -> bool:
        period = 1.0 / self._args.rate
        sample_count = int(math.ceil(self._args.duration * self._args.rate))
        start = time.monotonic()
        for index in range(sample_count + 1):
            deadline = start + min(index * period, self._args.duration)
            if not self._wait_until(deadline):
                return False
            trajectory_time = min(index * period, self._args.duration)
            progress = profile_progress(
                self._args.profile,
                trajectory_time / self._args.duration,
            )
            y = self._args.y_start + (self._args.y_end - self._args.y_start) * progress
            self._publish_pose(
                x=self._args.x,
                y=y,
                z=self._args.z,
                trajectory_time_s=trajectory_time,
                progress=progress,
                scheduled_monotonic_ns=int(deadline * 1e9),
            )
        return True

    def save_log(self) -> None:
        self._log_path.parent.mkdir(parents=True, exist_ok=True)
        metadata = {
            "event": "metadata",
            "format_version": 1,
            "rate_hz": self._args.rate,
            "duration_s": self._args.duration,
            "profile": self._args.profile,
            "x": self._args.x,
            "y_start": self._args.y_start,
            "y_end": self._args.y_end,
            "z": self._args.z,
        }
        with self._log_path.open("w", encoding="utf-8") as stream:
            stream.write(json.dumps(metadata, separators=(",", ":")) + "\n")
            for event in self._events:
                stream.write(json.dumps(event, separators=(",", ":")) + "\n")
        self.get_logger().info(f"diagnostic log saved to {self._log_path}")

    def print_summary(self) -> None:
        count = len(self._publish_times_ns)
        if count < 2:
            print(f"published targets: {count}")
            return
        intervals = [
            self._publish_times_ns[index] - self._publish_times_ns[index - 1]
            for index in range(1, count)
        ]
        elapsed_s = (self._publish_times_ns[-1] - self._publish_times_ns[0]) * 1e-9
        actual_rate = (count - 1) / elapsed_s if elapsed_s > 0.0 else 0.0
        print(f"published targets: {count}")
        print(f"actual publish rate: {actual_rate:.2f} Hz")
        print(
            "publish interval: "
            f"mean={statistics.fmean(intervals) * 1e-6:.3f}ms "
            f"max={max(intervals) * 1e-6:.3f}ms"
        )

    def record_error(self, message: str) -> None:
        self._record("program_error", message=message)

    def _publish_pose(
        self,
        *,
        x: float,
        y: float,
        z: float,
        trajectory_time_s: float,
        progress: float,
        scheduled_monotonic_ns: int,
    ) -> None:
        actual_monotonic_ns = time.monotonic_ns()
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._args.frame_id
        message.pose.position.x = float(x)
        message.pose.position.y = float(y)
        message.pose.position.z = float(z)
        message.pose.orientation.w = 1.0
        self._publisher.publish(message)
        self._publish_times_ns.append(actual_monotonic_ns)
        self._record(
            "target_publish",
            monotonic_ns=actual_monotonic_ns,
            scheduled_monotonic_ns=scheduled_monotonic_ns,
            schedule_error_ns=actual_monotonic_ns - scheduled_monotonic_ns,
            message_stamp_ns=self._stamp_to_ns(message.header.stamp),
            trajectory_time_s=trajectory_time_s,
            progress=progress,
            x=float(x),
            y=float(y),
            z=float(z),
        )

    def _wait_until(self, deadline: float) -> bool:
        while not self._stop_requested:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return True
            rclpy.spin_once(self, timeout_sec=min(remaining, 0.005))
        return False

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


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--profile", choices=("linear", "sine"), default="sine")
    parser.add_argument("--x", type=float, default=0.3)
    parser.add_argument("--y-start", type=float, default=0.0)
    parser.add_argument("--y-end", type=float, default=0.4)
    parser.add_argument("--z", type=float, default=0.3)
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--namespace", default="rebotarm")
    parser.add_argument("--output-dir", default="diagnostic_logs")
    args = parser.parse_args()
    if args.rate <= 0.0:
        parser.error("--rate must be positive")
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    return args


def main() -> None:
    args = _parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = EefTargetPublisherTest(args)

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
