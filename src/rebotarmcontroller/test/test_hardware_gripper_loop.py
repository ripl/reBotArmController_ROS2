import threading
import time
import unittest
from types import SimpleNamespace

import numpy as np

from rebotarmcontroller.gripper_control import (
    GripperControl,
    GripperControlConfig,
)
from rebotarmcontroller.hardware_manager import HardwareManager


class _FakeMotor:
    def __init__(self, position: float, velocity: float) -> None:
        self.state = SimpleNamespace(
            pos=position,
            vel=velocity,
            torq=0.0,
            status_code=1,
        )

    def get_state(self):
        return self.state


class _FakeGripperGroup:
    def __init__(self) -> None:
        self.commands = []
        self.send_entered = None
        self.send_release = None

    def send_mit(self, position, *, vel, kp, kd, tau) -> None:
        if self.send_entered is not None:
            self.send_entered.set()
            self.send_release.wait(timeout=1.0)
        self.commands.append(
            (
                float(position[0]),
                float(vel[0]),
                float(kp[0]),
                float(kd[0]),
                float(tau[0]),
            )
        )


class _FakeArmGroup:
    def __init__(self) -> None:
        self.joint_names = [f"joint{index}" for index in range(1, 7)]
        self._pv_vlim = np.full(6, 1.0)
        self._mit_kp = np.full(6, 10.0)
        self._mit_kd = np.full(6, 1.0)
        self.commands = []
        self.mit_commands = []
        self.feedback_requests = []
        self.send_entered = None
        self.send_release = None

    def send_pos_vel(self, position, *, vlim) -> None:
        if self.send_entered is not None:
            self.send_entered.set()
            self.send_release.wait(timeout=1.0)
        self.commands.append(
            (np.array(position, copy=True), np.array(vlim, copy=True))
        )

    def get_positions(self, *, request_feedback) -> np.ndarray:
        self.feedback_requests.append(request_feedback)
        return np.zeros(6)

    def send_mit(self, position, *, vel, kp, kd, tau) -> None:
        self.mit_commands.append(
            tuple(
                np.array(value, copy=True)
                for value in (position, vel, kp, kd, tau)
            )
        )


class _FakeEndPose:
    def __init__(self) -> None:
        self._has_gripper = False
        self._q_target = np.zeros(6)
        self._qd_target = np.zeros(6)
        self._vlim_override = None


class HardwareGripperLoopTest(unittest.TestCase):
    def setUp(self) -> None:
        motor = _FakeMotor(-4.5, 0.2)
        self.arm_group = _FakeArmGroup()
        self.group = _FakeGripperGroup()
        self.endpose = _FakeEndPose()
        self.manager = HardwareManager.__new__(HardwareManager)
        self.manager._cmd_lock = threading.RLock()
        self.manager._control_output_enabled = True
        self.manager._robot = SimpleNamespace(
            has_gripper=True,
            _motor_map={"gripper": motor},
        )
        self.manager._gripper_name = "gripper"
        self.manager._arm_group = self.arm_group
        self.manager._arm_control_mode = "posvel"
        self.manager._gripper_group = self.group
        self.manager._endpos_ctrl = self.endpose
        self.manager._eef_command_sequence = 0
        self.manager._diagnostics = None
        self.manager._gripper_grasp_event = threading.Event()
        self.manager._last_gripper_state = (-5.0, 0.0, 0.0, 1)
        self.manager._gripper_control = GripperControl(
            open_position=-5.0,
            close_position=0.0,
            goal_tolerance=0.12,
            config=GripperControlConfig(
                close_torque=0.30,
                hold_torque=0.15,
                torque_limit=0.50,
                move_kp=5.0,
                move_kd=1.0,
                close_kd=0.5,
                stall_velocity=0.05,
                stall_duration=0.10,
                startup_distance=0.30,
            ),
        )
        self.manager._gripper_control.start_grasp(-5.0, 0.0)

    def test_endpose_tick_sends_one_torque_limited_gripper_command(self) -> None:
        self.manager._endpos_loop_cb(None, 0.002)

        self.assertEqual(len(self.arm_group.commands), 1)
        self.assertEqual(len(self.group.commands), 1)
        _, _, kp, kd, torque = self.group.commands[0]
        self.assertEqual(kp, 0.0)
        self.assertAlmostEqual(kd, 0.5)
        self.assertAlmostEqual(torque, 0.30)

    def test_contact_transition_sets_completion_event(self) -> None:
        motor = self.manager._robot._motor_map["gripper"]
        motor.state.vel = 0.01
        self.manager._gripper_control._stall_since = time.monotonic() - 0.11

        self.manager._endpos_loop_cb(None, 0.002)

        self.assertEqual(
            self.manager._gripper_control.result,
            GripperControl.CONTACT,
        )
        self.assertTrue(self.manager._gripper_grasp_event.is_set())
        self.assertAlmostEqual(self.group.commands[-1][4], 0.15)

    def test_endpose_tick_records_hardware_output_diagnostics(self) -> None:
        events = []
        self.endpose._q_target = np.arange(6, dtype=np.float64)
        self.endpose._vlim_override = np.full(6, 2.0)
        self.manager._eef_command_sequence = 7
        self.manager._diagnostics = SimpleNamespace(
            active=True,
            record=lambda event, **values: events.append(
                {"event": event, **values}
            ),
        )

        self.manager._endpos_loop_cb(None, 0.002)

        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertEqual(event["event"], "hardware_output")
        self.assertTrue(event["lock_acquired"])
        self.assertEqual(event["command_sequence"], 7)
        np.testing.assert_array_equal(event["q_target"], np.arange(6))
        np.testing.assert_array_equal(event["velocity_limits"], np.full(6, 2.0))
        self.assertGreater(event["send_duration_ns"], 0)
        self.assertGreater(event["snapshot_duration_ns"], 0)

    def test_endpose_tick_preserves_mit_gravity_feedforward(self) -> None:
        self.manager._arm_control_mode = "mit"
        self.endpose._use_gravity_ff = True
        self.manager._gc_model = object()
        self.manager._gc_data = object()
        self.manager._pad_q_for_model = lambda _model, q, _count: q
        self.manager._gc_compute_generalized_gravity = (
            lambda _model, _q, _data: np.arange(6, dtype=np.float64)
        )

        self.manager._endpos_loop_cb(None, 0.002)

        self.assertEqual(len(self.arm_group.mit_commands), 1)
        self.assertEqual(self.arm_group.feedback_requests, [False])
        *_, torque = self.arm_group.mit_commands[0]
        np.testing.assert_array_equal(
            torque,
            np.array([0.0, 1.55, 3.10, 3.0, 4.0, 5.0]),
        )

    def test_endpose_tick_does_not_hold_command_lock_during_arm_send(self) -> None:
        entered = threading.Event()
        release = threading.Event()
        self.arm_group.send_entered = entered
        self.arm_group.send_release = release

        thread = threading.Thread(
            target=lambda: self.manager._endpos_loop_cb(None, 0.002)
        )
        thread.start()
        self.assertTrue(entered.wait(timeout=1.0))
        acquired = self.manager._cmd_lock.acquire(blocking=False)
        if acquired:
            self.manager._cmd_lock.release()
        release.set()
        thread.join(timeout=1.0)

        self.assertTrue(acquired)
        self.assertFalse(thread.is_alive())

    def test_endpose_tick_does_not_hold_command_lock_during_gripper_send(self) -> None:
        entered = threading.Event()
        release = threading.Event()
        self.group.send_entered = entered
        self.group.send_release = release

        thread = threading.Thread(
            target=lambda: self.manager._endpos_loop_cb(None, 0.002)
        )
        thread.start()
        self.assertTrue(entered.wait(timeout=1.0))
        acquired = self.manager._cmd_lock.acquire(blocking=False)
        if acquired:
            self.manager._cmd_lock.release()
        release.set()
        thread.join(timeout=1.0)

        self.assertTrue(acquired)
        self.assertFalse(thread.is_alive())


if __name__ == "__main__":
    unittest.main()
