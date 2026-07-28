import threading
import time
import unittest
from types import SimpleNamespace

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

    def send_mit(self, position, *, vel, kp, kd, tau) -> None:
        self.commands.append(
            (
                float(position[0]),
                float(vel[0]),
                float(kp[0]),
                float(kd[0]),
                float(tau[0]),
            )
        )


class _FakeEndPose:
    def __init__(self) -> None:
        self.calls = 0
        self._has_gripper = False

    def _loop_cb(self, _robot, _dt) -> None:
        self.calls += 1


class HardwareGripperLoopTest(unittest.TestCase):
    def setUp(self) -> None:
        motor = _FakeMotor(-4.5, 0.2)
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
        self.manager._gripper_group = self.group
        self.manager._endpos_ctrl = self.endpose
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

        self.assertEqual(self.endpose.calls, 1)
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


if __name__ == "__main__":
    unittest.main()
