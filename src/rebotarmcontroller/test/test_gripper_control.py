import unittest

from rebotarmcontroller.gripper_control import (
    GripperControl,
    GripperControlConfig,
)


class GripperControlTest(unittest.TestCase):
    def setUp(self) -> None:
        self.control = GripperControl(
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

    def test_grasp_closes_with_limited_torque(self) -> None:
        self.control.start_grasp(-5.0, 0.0)

        command = self.control.tick(-4.5, 0.2, 1.0)

        self.assertEqual(self.control.state, GripperControl.CLOSING)
        self.assertEqual(command.kp, 0.0)
        self.assertAlmostEqual(command.kd, 0.5)
        self.assertAlmostEqual(command.torque, 0.30)

    def test_sustained_stall_transitions_to_low_torque_hold(self) -> None:
        self.control.start_grasp(-5.0, 0.0)

        self.control.tick(-4.5, 0.01, 1.0)
        command = self.control.tick(-4.5, 0.01, 1.11)

        self.assertEqual(self.control.state, GripperControl.HOLDING)
        self.assertEqual(self.control.result, GripperControl.CONTACT)
        self.assertAlmostEqual(command.position, -4.5)
        self.assertAlmostEqual(command.kp, 5.0)
        self.assertAlmostEqual(command.torque, 0.15)

    def test_transient_low_velocity_does_not_report_contact(self) -> None:
        self.control.start_grasp(-5.0, 0.0)

        self.control.tick(-4.5, 0.01, 1.0)
        self.control.tick(-4.3, 0.2, 1.05)
        self.control.tick(-4.0, 0.01, 1.20)

        self.assertEqual(self.control.state, GripperControl.CLOSING)
        self.assertIsNone(self.control.result)

    def test_reaching_close_target_reports_no_contact(self) -> None:
        self.control.start_grasp(-5.0, 0.0)

        command = self.control.tick(-0.05, 0.01, 1.0)

        self.assertEqual(self.control.state, GripperControl.POSITION)
        self.assertEqual(self.control.result, GripperControl.REACHED_TARGET)
        self.assertAlmostEqual(command.position, 0.0)
        self.assertEqual(command.torque, 0.0)

    def test_requested_effort_is_clamped_and_limits_hold(self) -> None:
        self.control.start_grasp(-5.0, 0.0, max_effort=0.10)

        closing = self.control.tick(-4.5, 0.2, 1.0)
        self.control.tick(-4.5, 0.01, 1.1)
        holding = self.control.tick(-4.5, 0.01, 1.21)

        self.assertAlmostEqual(closing.torque, 0.10)
        self.assertAlmostEqual(holding.torque, 0.10)

    def test_timeout_cancels_grasp_and_holds_current_position(self) -> None:
        self.control.start_grasp(-5.0, 0.0)

        self.control.cancel(-4.2, result=GripperControl.TIMEOUT)
        command = self.control.tick(-4.2, 0.0, 1.0)

        self.assertEqual(self.control.state, GripperControl.POSITION)
        self.assertEqual(self.control.result, GripperControl.TIMEOUT)
        self.assertAlmostEqual(command.position, -4.2)
        self.assertEqual(command.torque, 0.0)

    def test_position_command_replaces_active_grasp(self) -> None:
        self.control.start_grasp(-5.0, 0.0)

        target = self.control.set_position(-5.0)
        command = self.control.tick(-4.5, 0.0, 1.0)

        self.assertEqual(target, -5.0)
        self.assertEqual(self.control.state, GripperControl.POSITION)
        self.assertIsNone(self.control.result)
        self.assertAlmostEqual(command.position, -5.0)
        self.assertEqual(command.torque, 0.0)

    def test_rs_direction_uses_negative_closing_torque(self) -> None:
        control = GripperControl(
            open_position=5.0,
            close_position=0.0,
            goal_tolerance=0.12,
            config=GripperControlConfig(
                close_torque=1.0,
                hold_torque=0.3,
                torque_limit=1.5,
                move_kp=5.0,
                move_kd=1.0,
                close_kd=0.5,
                stall_velocity=0.05,
                stall_duration=0.10,
                startup_distance=0.30,
            ),
        )
        control.start_grasp(5.0, 0.0)

        command = control.tick(4.5, -0.2, 1.0)

        self.assertAlmostEqual(command.torque, -1.0)
        self.assertTrue(control.is_closing_target(4.5, 0.0))
        self.assertFalse(control.is_closing_target(4.5, 5.0))


if __name__ == "__main__":
    unittest.main()
