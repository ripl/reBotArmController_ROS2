import threading
import time
import unittest
from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import Pose

from rebotarmcontroller.eef_streaming_controller import EefStreamingController


class _Diagnostics:
    def __init__(self) -> None:
        self.events = []

    def record(self, event: str, **values) -> None:
        self.events.append({"event": event, **values})


class EefControllerDiagnosticsTest(unittest.TestCase):
    def test_control_loop_records_tick_and_resync(self) -> None:
        diagnostics = _Diagnostics()
        controller = EefStreamingController.__new__(EefStreamingController)
        controller._diagnostics = diagnostics
        controller._active = True
        controller._rate_hz = 1000.0
        controller._control_stop = threading.Event()
        calls = 0

        def control_tick() -> None:
            nonlocal calls
            calls += 1
            time.sleep(0.003)
            controller._control_stop.set()

        controller._control_tick = control_tick

        controller._control_loop()

        self.assertEqual(calls, 1)
        events = [event["event"] for event in diagnostics.events]
        self.assertIn("control_loop_tick", events)
        self.assertIn("control_loop_resync", events)

    def test_commanded_tick_records_target_and_joint_command(self) -> None:
        diagnostics = _Diagnostics()
        target = np.array([0.3, 0.01, 0.3, 0.0, 0.0, 0.0, 1.0])
        q_ik = np.full(6, 0.01)
        pose_result = Pose()
        pose_result.position.x = 0.3
        pose_result.position.y = 0.01
        pose_result.position.z = 0.3
        pose_result.orientation.w = 1.0
        hardware = SimpleNamespace(
            solve_eef_ik=lambda _pose, _seed: q_ik,
            set_eef_streaming_target=lambda _q, _limits: 12,
            pose_from_joint_positions=lambda _q: pose_result,
        )
        controller = EefStreamingController.__new__(EefStreamingController)
        controller._node = SimpleNamespace()
        controller._hardware = hardware
        controller._diagnostics = diagnostics
        controller._active = True
        controller._lock = threading.Lock()
        controller._latest_target = target
        controller._latest_target_time = time.monotonic()
        controller._last_control_tick_ns = None
        controller._q_command = np.zeros(6)
        controller._pose_command = np.array(
            [0.3, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]
        )
        controller._diagnostics_detail = True
        controller._timeout = 1.0
        controller._workspace_min = np.full(3, -1.0)
        controller._workspace_max = np.full(3, 1.0)
        controller._rate_hz = 50.0
        controller._max_linear_velocity = 1.6
        controller._max_angular_velocity = 3.0
        controller._joint_velocity_limits = np.full(6, 7.5)
        controller._joint_lower = np.full(6, -10.0)
        controller._joint_upper = np.full(6, 10.0)

        controller._control_tick()

        self.assertEqual(len(diagnostics.events), 1)
        event = diagnostics.events[0]
        self.assertEqual(event["event"], "control_tick")
        self.assertEqual(event["outcome"], "commanded")
        self.assertEqual(event["command_sequence"], 12)
        np.testing.assert_allclose(event["target"], target)
        np.testing.assert_allclose(event["q_ik"], q_ik)
        np.testing.assert_allclose(event["q_next"], q_ik)
        self.assertGreaterEqual(event["target_age_ns"], 0)
        self.assertGreater(event["total_duration_ns"], 0)


if __name__ == "__main__":
    unittest.main()
