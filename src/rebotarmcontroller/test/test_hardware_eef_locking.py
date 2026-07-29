import threading
import unittest
from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import Pose

from rebotarmcontroller.hardware_manager import HardwareManager


class HardwareEefLockingTest(unittest.TestCase):
    def setUp(self) -> None:
        self.manager = HardwareManager.__new__(HardwareManager)
        self.manager._cmd_lock = threading.RLock()
        self.manager._state_machine = "EEF_STREAMING"
        self.manager._arm_group = SimpleNamespace(
            joint_names=[f"joint{index}" for index in range(1, 7)]
        )
        self.manager._pad_q_for_model = lambda _model, q, _count: q

    def _command_lock_is_available_during(self, operation, entered, release) -> bool:
        errors = []

        def run() -> None:
            try:
                operation()
            except Exception as exc:
                errors.append(exc)

        thread = threading.Thread(target=run)
        thread.start()
        self.assertTrue(entered.wait(timeout=1.0))
        acquired = self.manager._cmd_lock.acquire(blocking=False)
        if acquired:
            self.manager._cmd_lock.release()
        release.set()
        thread.join(timeout=1.0)
        self.assertFalse(thread.is_alive())
        if errors:
            raise errors[0]
        return acquired

    def test_solve_eef_ik_does_not_wait_for_command_lock(self) -> None:
        entered = threading.Event()
        release = threading.Event()

        def solve(*_args, **_kwargs):
            entered.set()
            self.assertTrue(release.wait(timeout=1.0))
            return SimpleNamespace(success=True, q=np.zeros(6))

        self.manager._endpos_ctrl = SimpleNamespace(
            _model=object(),
            _data=object(),
            _end_frame_id=0,
        )
        self.manager._pos_rot_to_se3 = lambda *_args, **_kwargs: object()
        self.manager._solve_ik = solve
        self.manager._eef_streaming_ik_solver_params = object()
        pose = Pose()
        pose.orientation.w = 1.0

        errors = []

        def run() -> None:
            try:
                self.manager.solve_eef_ik(pose, np.zeros(6))
            except Exception as exc:
                errors.append(exc)

        self.manager._cmd_lock.acquire()
        thread = threading.Thread(target=run)
        thread.start()
        entered_without_command_lock = entered.wait(timeout=1.0)
        self.manager._cmd_lock.release()
        release.set()
        thread.join(timeout=1.0)

        self.assertTrue(entered_without_command_lock)
        self.assertFalse(thread.is_alive())
        if errors:
            raise errors[0]

    def test_pose_from_joint_positions_does_not_hold_command_lock(self) -> None:
        entered = threading.Event()
        release = threading.Event()

        def compute_fk(_model, _q):
            entered.set()
            self.assertTrue(release.wait(timeout=1.0))
            return np.zeros(3), np.eye(3), np.eye(4)

        self.manager._gc_model = object()
        self.manager._compute_fk = compute_fk

        acquired = self._command_lock_is_available_during(
            lambda: self.manager.pose_from_joint_positions(np.zeros(6)),
            entered,
            release,
        )

        self.assertTrue(acquired)


if __name__ == "__main__":
    unittest.main()
