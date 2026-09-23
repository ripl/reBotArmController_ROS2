"""MIT streaming state and control-loop output, with recording fakes instead of hardware."""
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from rebotarmcontroller.hardware_manager import HardwareManager


class Group:
    def __init__(self, joint_names=(), lock=None):
        self.sent, self.joint_names, self.lock, self.lock_free = [], list(joint_names), lock, []
        self.measured = np.zeros(len(self.joint_names))
        self._mit_kp, self._mit_kd = np.array([2.5]), np.array([1.0])

    def get_positions(self, request_feedback=False):
        return self.measured.copy()

    def send_mit(self, pos, vel=None, kp=None, kd=None, tau=None):
        self.sent.append(dict(pos=pos, vel=vel, kp=kp, kd=kd, tau=tau))
        if self.lock is not None:                    # can another thread take the lock during the send?
            free = []

            def probe():
                free.append(self.lock.acquire(blocking=False))
                if free[0]:
                    self.lock.release()
            t = threading.Thread(target=probe)
            t.start()
            t.join()
            self.lock_free.append(free[0])


@pytest.fixture
def hw():
    hw = HardwareManager.__new__(HardwareManager)
    hw._cmd_lock = threading.RLock()
    hw._mit_stream = None
    hw._mit_stream_stopped = False
    hw._state_machine = "IDLE"
    hw._arm_control_mode = "mit"
    hw._enabled, hw._gravity_comp_active, hw._control_output_enabled = True, False, True
    hw._arm_group = Group((f"joint{i}" for i in range(1, 7)), lock=hw._cmd_lock)
    hw._robot = SimpleNamespace(stop_control_loop=lambda: None, control_loop_active=True, has_gripper=True)
    hw.sdk_loop_calls = 0

    def sdk_loop(robot, dt):
        hw.sdk_loop_calls += 1
    hw._endpos_ctrl = SimpleNamespace(_q_target=np.zeros(6), _qd_target=np.zeros(6), _loop_cb=sdk_loop,
                                      _has_gripper=True, _gripper_group=Group(), _gripper_target=0.3, _running=True)
    hw.enable_calls = 0

    def start_endpos_control():
        hw.enable_calls += 1
        hw.set_state_machine("IDLE")
    hw.start_endpos_control = start_endpos_control
    return hw


def command(scale=1., pos=None):
    pos = np.radians(np.arange(6.) * .1 * scale) if pos is None else np.radians(pos)   # near the measured zeros
    return [pos, np.full(6, .1), np.full(6, 80.), np.full(6, 5.), np.full(6, .2)]


def test_stream_enters_state_once_and_loop_sends_latest_command(hw):
    assert hw.stream_mit(*command()) is True
    assert hw.stream_mit(*command(2.)) is False
    assert hw.enable_calls == 1 and hw.state_machine == "MIT_STREAMING"
    hw._endpos_loop_cb(None, .002)
    sent = hw._arm_group.sent[-1]
    for name, expected in zip(("pos", "vel", "kp", "kd", "tau"), command(2.)):
        np.testing.assert_array_equal(sent[name], expected)
    assert hw.sdk_loop_calls == 0
    assert hw._arm_group.lock_free == [True]            # the CAN send does not hold the command lock
    gripper = hw._endpos_ctrl._gripper_group.sent[-1]
    np.testing.assert_array_equal(gripper["pos"], [.3])


def test_leaving_stream_holds_last_streamed_pose(hw):
    hw.stream_mit(*command(2.))
    hw.set_state_machine("IDLE")
    hw._endpos_loop_cb(None, .002)
    assert hw.sdk_loop_calls == 1 and hw._mit_stream is None
    np.testing.assert_array_equal(hw._endpos_ctrl._q_target, command(2.)[0])
    np.testing.assert_array_equal(hw._endpos_ctrl._qd_target, command(2.)[1])


@pytest.mark.parametrize("bad", [np.zeros(5), np.array([0., 0., np.nan, 0., 0., 0.])])
def test_malformed_command_rejected_without_state_change(hw, bad):
    args = command()
    args[2] = bad
    with pytest.raises(ValueError):
        hw.stream_mit(*args)
    assert hw.state_machine == "IDLE" and hw.enable_calls == 0


def test_rejected_outside_mit_mode_and_while_busy(hw):
    hw._arm_control_mode = "posvel"
    with pytest.raises(RuntimeError):
        hw.stream_mit(*command())
    hw._arm_control_mode = "mit"
    hw._state_machine = "TRAJ_RUNNING"
    with pytest.raises(RuntimeError):
        hw.stream_mit(*command())
    assert hw.enable_calls == 0


def test_streaming_blocks_commands_that_would_stop_or_override_it(hw):
    hw.stream_mit(*command())
    with pytest.raises(RuntimeError):
        hw._require_idle("trajectory stream")        # follow_joint_trajectory, move_to_pose, IK
    with pytest.raises(RuntimeError):
        hw._begin_lowlevel_streaming("mit")          # per-joint passthrough stops the loop
    hw._gripper_name, hw._homing_thread = "gripper", None
    with pytest.raises(RuntimeError):
        hw._begin_gripper_command()                  # stops the control loop
    hw._begin_gripper_command(allow_endpos=True)      # keeps the loop running: allowed
    assert hw.state_machine == "MIT_STREAMING"


def test_step_guard_holds_last_target_until_reset(hw):
    hw.stream_mit(*command(pos=np.full(6, 1.)))
    hw.stream_mit(*command(pos=np.full(6, 2.9)))                   # 1.9 deg step: accepted
    with pytest.raises(RuntimeError, match="target step"):
        hw.stream_mit(*command(pos=[2.9, 2.9, 5.0, 2.9, 2.9, 2.9]))  # 2.1 deg on one joint
    with pytest.raises(RuntimeError, match="stopped by a guard"):
        hw.stream_mit(*command(pos=np.full(6, 2.9)))               # latched: even a small step is refused
    hw._endpos_loop_cb(None, .002)
    held = hw._arm_group.sent[-1]
    np.testing.assert_allclose(held["pos"], np.radians(np.full(6, 2.9)))
    np.testing.assert_array_equal(held["vel"], np.zeros(6))
    np.testing.assert_array_equal(held["kp"], np.full(6, 80.))
    assert hw.state_machine == "MIT_STREAMING"                     # arm goals stay rejected
    hw.set_state_machine("IDLE")                                   # enable / disable / safe_home reset it
    assert hw.stream_mit(*command(pos=np.full(6, 2.9))) is True


def test_gap_guard_mid_stream_holds_last_target(hw):
    hw.stream_mit(*command(pos=np.full(6, 1.)))
    hw._arm_group.measured = np.radians([0., 0., -14.5, 0., 0., 0.])   # arm pushed away
    with pytest.raises(RuntimeError, match="target-to-measured gap"):
        hw.stream_mit(*command(pos=np.full(6, 1.)))                 # 15.5 deg from measured on joint3
    hw._endpos_loop_cb(None, .002)
    np.testing.assert_allclose(hw._arm_group.sent[-1]["pos"], np.radians(np.full(6, 1.)))


def test_first_command_skips_step_guard_but_not_gap_guard(hw):
    hw._arm_group.measured = np.radians(np.full(6, 10.))
    assert hw.stream_mit(*command(pos=np.full(6, 10.))) is True     # no previous target to compare with
    hw.set_state_machine("IDLE")
    with pytest.raises(RuntimeError, match="stream not started"):
        hw.stream_mit(*command(pos=np.full(6, 26.)))                # 16 deg from measured
    assert hw.state_machine == "IDLE" and hw.enable_calls == 1
