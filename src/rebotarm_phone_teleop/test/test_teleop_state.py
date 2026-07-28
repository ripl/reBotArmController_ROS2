from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock

from rebotarm_msgs.msg import PhoneButtonEvent

from rebotarm_phone_teleop.teleop_node import PhoneEefTeleop


def make_node(**attributes):
    states = {
        name: getattr(PhoneEefTeleop, name)
        for name in (
            "MOVING_TO_READY",
            "READY_FAILED",
            "INACTIVE",
            "ENABLING",
            "ACTIVE",
            "DISABLING",
        )
    }
    return SimpleNamespace(**states, **attributes)


class TeleopStateTest(unittest.TestCase):
    def test_volume_up_double_click_uses_return_flow(self):
        node = make_node(
            _return_to_ready_pose=MagicMock(),
            _begin_activation=MagicMock(),
            _state=PhoneEefTeleop.INACTIVE,
        )
        message = SimpleNamespace(
            button=PhoneButtonEvent.VOLUME_UP,
            gesture=PhoneButtonEvent.DOUBLE,
        )

        PhoneEefTeleop._button_callback(node, message)

        node._return_to_ready_pose.assert_called_once_with()
        node._begin_activation.assert_not_called()

    def test_active_return_disables_streaming_first(self):
        node = make_node(
            _command_enabled=True,
            _state=PhoneEefTeleop.ACTIVE,
            _begin_deactivation=MagicMock(),
            _begin_move_to_ready_pose=MagicMock(),
        )

        PhoneEefTeleop._return_to_ready_pose(node)

        node._begin_deactivation.assert_called_once_with(
            "returning to teleop_ready_pose",
            return_to_ready=True,
        )
        node._begin_move_to_ready_pose.assert_not_called()

    def test_inactive_return_requires_idle_arm(self):
        node = make_node(
            _command_enabled=True,
            _state=PhoneEefTeleop.INACTIVE,
            _arm_state="EEF_STREAMING",
            _begin_deactivation=MagicMock(),
            _begin_move_to_ready_pose=MagicMock(),
            get_logger=MagicMock(),
        )

        PhoneEefTeleop._return_to_ready_pose(node)

        node._begin_move_to_ready_pose.assert_not_called()
        node.get_logger.return_value.warn.assert_called_once()

    def test_inactive_return_moves_when_arm_is_idle(self):
        node = make_node(
            _command_enabled=True,
            _state=PhoneEefTeleop.INACTIVE,
            _arm_state="IDLE",
            _begin_deactivation=MagicMock(),
            _begin_move_to_ready_pose=MagicMock(),
        )

        PhoneEefTeleop._return_to_ready_pose(node)

        node._begin_move_to_ready_pose.assert_called_once_with()

    def test_disable_success_continues_to_ready_pose(self):
        response = SimpleNamespace(success=True, message="")
        future = SimpleNamespace(result=MagicMock(return_value=response))
        node = make_node(
            _state=PhoneEefTeleop.DISABLING,
            _begin_move_to_ready_pose=MagicMock(),
            get_logger=MagicMock(),
        )

        PhoneEefTeleop._disable_done(
            node,
            future,
            "returning to teleop_ready_pose",
            False,
            True,
        )

        self.assertEqual(node._state, PhoneEefTeleop.INACTIVE)
        node._begin_move_to_ready_pose.assert_called_once_with()

    def test_disable_rejection_does_not_move_to_ready_pose(self):
        response = SimpleNamespace(success=False, message="rejected")
        future = SimpleNamespace(result=MagicMock(return_value=response))
        node = make_node(
            _state=PhoneEefTeleop.DISABLING,
            _begin_move_to_ready_pose=MagicMock(),
            get_logger=MagicMock(),
        )

        PhoneEefTeleop._disable_done(
            node,
            future,
            "returning to teleop_ready_pose",
            False,
            True,
        )

        self.assertEqual(node._state, PhoneEefTeleop.INACTIVE)
        node._begin_move_to_ready_pose.assert_not_called()


if __name__ == "__main__":
    unittest.main()
