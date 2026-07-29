#!/usr/bin/env python3
"""Benchmark the low-level hardware send loop at requested rates."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import time

import numpy as np

from rebotarmcontroller.hardware_manager import HardwareManager


@dataclass
class _RunState:
    rate_hz: float
    started_ns: int
    sequence: int = 0
    last_tick_ns: int | None = None
    error: str | None = None


class HardwareSendRateBenchmark:
    def __init__(
        self,
        hardware: HardwareManager,
        *,
        output_dir: Path,
        send_gripper: bool,
        motion: str,
        amplitude: float,
        frequency: float,
        joint_index: int,
    ) -> None:
        self._hardware = hardware
        self._send_gripper = send_gripper
        self._motion = motion
        self._amplitude = float(amplitude)
        self._frequency = float(frequency)
        self._joint_index = int(joint_index)
        self._events: list[dict[str, object]] = []
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = output_dir / f"hardware_send_rate_{timestamp}.jsonl"
        self._gripper_hold = 0.0
        self._q_center: np.ndarray | None = None

    def run(self, rates: list[float], duration_s: float) -> None:
        self._q_center = self._hardware.get_joint_positions(request=True).copy()
        if self._joint_index != -1 and not 0 <= self._joint_index < len(self._q_center):
            raise ValueError(
                f"joint-index must be -1 or in 0..{len(self._q_center) - 1}, "
                f"got {self._joint_index}"
            )
        self._events.append(
            {
                "event": "session_start",
                "monotonic_ns": time.monotonic_ns(),
                "wall_time_ns": time.time_ns(),
                "rates_hz": rates,
                "duration_s": duration_s,
                "send_gripper": self._send_gripper,
                "motion": self._motion,
                "amplitude_rad": self._amplitude,
                "frequency_hz": self._frequency,
                "joint_index": self._joint_index,
                "q_center": self._q_center.tolist(),
            }
        )
        if self._hardware.has_gripper:
            self._gripper_hold = float(self._hardware.get_gripper_state()[0])

        for rate in rates:
            self._run_one_rate(float(rate), duration_s)

        self._events.append(
            {
                "event": "session_stop",
                "monotonic_ns": time.monotonic_ns(),
                "wall_time_ns": time.time_ns(),
            }
        )
        self._write_log()

    def _run_one_rate(self, rate_hz: float, duration_s: float) -> None:
        state = _RunState(rate_hz=rate_hz, started_ns=time.monotonic_ns())
        self._events.append(
            {
                "event": "rate_start",
                "monotonic_ns": state.started_ns,
                "rate_hz": rate_hz,
                "duration_s": duration_s,
            }
        )
        with self._hardware._cmd_lock:
            self._hardware._robot.stop_control_loop()
            self._hardware._control_output_enabled = True
            self._hardware._endpos_ctrl._running = True
        self._hardware._robot.start_control_loop(
            lambda robot, dt: self._tick(robot, dt, state),
            rate=rate_hz,
        )
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline and state.error is None:
            time.sleep(0.02)
        self._hardware._robot.stop_control_loop()
        summary = self._summarize_rate(rate_hz)
        self._events.append(
            {
                "event": "rate_stop",
                "monotonic_ns": time.monotonic_ns(),
                "rate_hz": rate_hz,
                "error": state.error,
                **summary,
            }
        )
        _print_summary(rate_hz, summary, state.error)

    def _tick(self, _robot, dt: float, state: _RunState) -> None:
        del _robot, dt
        tick_ns = time.monotonic_ns()
        interval_ns = None if state.last_tick_ns is None else tick_ns - state.last_tick_ns
        state.last_tick_ns = tick_ns
        state.sequence += 1

        arm_send_duration_ns = None
        gripper_send_duration_ns = None
        outcome = "sent"
        try:
            with self._hardware._cmd_lock:
                q_target = self._q_target_for_tick(tick_ns, state)
                self._hardware._endpos_ctrl._q_target[:] = q_target
                qd_target = np.array(
                    self._hardware._endpos_ctrl._qd_target,
                    dtype=np.float64,
                    copy=True,
                )
                velocity_limits = np.asarray(
                    (
                        self._hardware._endpos_ctrl._vlim_override
                        if self._hardware._endpos_ctrl._vlim_override is not None
                        else getattr(self._hardware._arm_group, "_pv_vlim")
                    ),
                    dtype=np.float64,
                ).copy()

            arm_start_ns = time.monotonic_ns()
            self._hardware._send_endpos_arm_command(
                q_target,
                qd_target,
                velocity_limits,
            )
            arm_send_duration_ns = time.monotonic_ns() - arm_start_ns

            if self._send_gripper and self._hardware.has_gripper:
                gripper_start_ns = time.monotonic_ns()
                self._hardware._gripper_group.send_mit(
                    np.array([self._gripper_hold], dtype=np.float64),
                    vel=np.zeros(1, dtype=np.float64),
                    kp=getattr(self._hardware._gripper_group, "_mit_kp"),
                    kd=getattr(self._hardware._gripper_group, "_mit_kd"),
                    tau=np.zeros(1, dtype=np.float64),
                )
                gripper_send_duration_ns = time.monotonic_ns() - gripper_start_ns
        except Exception as exc:
            outcome = "error"
            state.error = repr(exc)

        self._events.append(
            {
                "event": "hardware_send_tick",
                "monotonic_ns": tick_ns,
                "rate_hz": state.rate_hz,
                "sequence": state.sequence,
                "tick_interval_ns": interval_ns,
                "arm_send_duration_ns": arm_send_duration_ns,
                "gripper_send_duration_ns": gripper_send_duration_ns,
                "send_duration_ns": (
                    None
                    if arm_send_duration_ns is None
                    else arm_send_duration_ns + (gripper_send_duration_ns or 0)
                ),
                "q_target": None if outcome == "error" else q_target.tolist(),
                "outcome": outcome,
                "error": state.error,
            }
        )

    def _q_target_for_tick(self, tick_ns: int, state: _RunState) -> np.ndarray:
        assert self._q_center is not None
        q_target = self._q_center.copy()
        if self._motion == "sine":
            elapsed_s = (tick_ns - state.started_ns) * 1e-9
            offset = self._amplitude * math.sin(
                2.0 * math.pi * self._frequency * elapsed_s
            )
            if self._joint_index == -1:
                q_target += offset
            else:
                q_target[self._joint_index] += offset
        return q_target

    def _summarize_rate(self, rate_hz: float) -> dict[str, object]:
        ticks = [
            event
            for event in self._events
            if event.get("event") == "hardware_send_tick"
            and event.get("rate_hz") == rate_hz
        ]
        if len(ticks) < 2:
            return {"ticks": len(ticks)}
        span_s = (
            int(ticks[-1]["monotonic_ns"]) - int(ticks[0]["monotonic_ns"])
        ) * 1e-9
        actual_hz = (len(ticks) - 1) / span_s if span_s > 0.0 else 0.0
        period_ns = int(1_000_000_000 / rate_hz)
        intervals = [
            int(event["tick_interval_ns"])
            for event in ticks
            if event.get("tick_interval_ns") is not None
        ]
        arm = [
            int(event["arm_send_duration_ns"])
            for event in ticks
            if event.get("arm_send_duration_ns") is not None
        ]
        gripper = [
            int(event["gripper_send_duration_ns"])
            for event in ticks
            if event.get("gripper_send_duration_ns") is not None
        ]
        total = [
            int(event["send_duration_ns"])
            for event in ticks
            if event.get("send_duration_ns") is not None
        ]
        return {
            "ticks": len(ticks),
            "actual_rate_hz": actual_hz,
            "period_ns": period_ns,
            "interval": _stats_ns(intervals),
            "arm_send": _stats_ns(arm),
            "gripper_send": _stats_ns(gripper),
            "total_send": _stats_ns(total),
            "interval_over_period": sum(value > period_ns for value in intervals),
            "total_send_over_period": sum(value > period_ns for value in total),
            "total_send_over_20ms": sum(value > 20_000_000 for value in total),
            "total_send_over_50ms": sum(value > 50_000_000 for value in total),
        }

    def _write_log(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        with self.log_path.open("w", encoding="utf-8") as stream:
            for event in self._events:
                stream.write(json.dumps(event, separators=(",", ":")) + "\n")


def _stats_ns(values: list[int]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "mean_ms": statistics.mean(values) / 1e6,
        "p50_ms": _percentile(values, 50.0) / 1e6,
        "p95_ms": _percentile(values, 95.0) / 1e6,
        "p99_ms": _percentile(values, 99.0) / 1e6,
        "max_ms": max(values) / 1e6,
    }


def _percentile(values: list[int], percentile: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    index = (len(ordered) - 1) * percentile / 100.0
    low = math.floor(index)
    high = math.ceil(index)
    if low == high:
        return float(ordered[low])
    return float(ordered[low] * (high - index) + ordered[high] * (index - low))


def _print_summary(rate_hz: float, summary: dict[str, object], error: str | None) -> None:
    print(f"\nrequested rate: {rate_hz:.1f} Hz")
    if error is not None:
        print(f"error: {error}")
    print(f"ticks: {summary.get('ticks', 0)}")
    if "actual_rate_hz" in summary:
        print(f"actual rate: {float(summary['actual_rate_hz']):.2f} Hz")
        print(f"interval: {_format_stats(summary['interval'])}")
        print(f"arm send: {_format_stats(summary['arm_send'])}")
        print(f"gripper send: {_format_stats(summary['gripper_send'])}")
        print(f"total send: {_format_stats(summary['total_send'])}")
        print(
            "overruns: "
            f"interval>{summary['period_ns']}ns={summary['interval_over_period']}, "
            f"send>period={summary['total_send_over_period']}, "
            f"send>20ms={summary['total_send_over_20ms']}, "
            f"send>50ms={summary['total_send_over_50ms']}"
        )


def _format_stats(value: object) -> str:
    stats = value if isinstance(value, dict) else {}
    if not stats or stats.get("count", 0) == 0:
        return "count=0"
    return (
        f"count={stats['count']} "
        f"mean={stats['mean_ms']:.3f}ms "
        f"p50={stats['p50_ms']:.3f}ms "
        f"p95={stats['p95_ms']:.3f}ms "
        f"p99={stats['p99_ms']:.3f}ms "
        f"max={stats['max_ms']:.3f}ms"
    )


def _move_to_ready(
    hardware: HardwareManager,
    *,
    x: float,
    y: float,
    z: float,
    duration: float,
) -> None:
    ok = hardware.move_to_pose_traj(x, y, z, 0.0, 0.0, 0.0, duration)
    if not ok:
        raise RuntimeError("failed to plan ready pose trajectory")

    deadline = time.monotonic() + max(duration, 0.0) + 2.0
    while hardware.motion_active():
        if hardware.state_machine == "SAFE_HOMING":
            hardware.stop_motion()
            raise RuntimeError("ready pose preempted by safe_home")
        if time.monotonic() > deadline:
            hardware.stop_motion()
            hardware.hold_current_position()
            raise RuntimeError("ready pose timed out")
        time.sleep(0.02)

    hardware.set_state_machine("IDLE")
    hardware.start_endpos_control()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark reBotArm low-level hardware send timing.",
    )
    parser.add_argument("--rates", nargs="+", type=float, default=[50.0, 100.0, 200.0, 500.0])
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--hardware-config", default="")
    parser.add_argument("--model", default="")
    parser.add_argument("--channel", default="")
    parser.add_argument("--output-dir", default="diagnostic_logs")
    parser.add_argument("--send-gripper", action="store_true")
    parser.add_argument("--motion", choices=["hold", "sine"], default="hold")
    parser.add_argument("--amplitude", type=float, default=0.05)
    parser.add_argument("--frequency", type=float, default=0.5)
    parser.add_argument("--joint-index", type=int, default=0, help="-1 moves all arm joints")
    parser.add_argument("--ready", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--ready-x", type=float, default=0.3)
    parser.add_argument("--ready-y", type=float, default=0.0)
    parser.add_argument("--ready-z", type=float, default=0.3)
    parser.add_argument("--ready-duration", type=float, default=2.0)
    args = parser.parse_args()

    hardware = HardwareManager(
        hardware_config=args.hardware_config or None,
        model=args.model,
        channel=args.channel,
    )
    benchmark = HardwareSendRateBenchmark(
        hardware,
        output_dir=Path(args.output_dir),
        send_gripper=bool(args.send_gripper),
        motion=str(args.motion),
        amplitude=float(args.amplitude),
        frequency=float(args.frequency),
        joint_index=int(args.joint_index),
    )
    try:
        hardware.connect()
        if args.ready:
            print(
                "moving to ready pose: "
                f"x={args.ready_x:.3f}, y={args.ready_y:.3f}, z={args.ready_z:.3f}"
            )
            _move_to_ready(
                hardware,
                x=float(args.ready_x),
                y=float(args.ready_y),
                z=float(args.ready_z),
                duration=float(args.ready_duration),
            )
        benchmark.run(args.rates, max(float(args.duration), 0.1))
    finally:
        hardware.shutdown()
    print(f"\ndiagnostic log saved to {benchmark.log_path}")


if __name__ == "__main__":
    main()
