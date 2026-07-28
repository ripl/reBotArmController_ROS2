from __future__ import annotations

from dataclasses import replace
import functools
import threading
import time

import numpy as np

from .conversions import fk_to_pose, pose_to_xyz_rpy
from .gripper_control import GripperControl, GripperControlConfig
from .hardware_config import resolve_hardware_config

_GRIPPER_GOAL_TOLERANCE_RAD = 0.12


def _locked(method):
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        with self._cmd_lock:
            return method(self, *args, **kwargs)

    return wrapper


class HardwareManager:
    """ROS-facing adapter for the new grouped reBotArm SDK."""

    def __init__(
        self,
        hardware_config: str | None = None,
        model: str = "",
        channel: str = "",
        eef_streaming_ik_max_iter: int = 20,
    ) -> None:
        self._cmd_lock = threading.RLock()
        hardware_config_path, hardware_data = resolve_hardware_config(
            hardware_config,
            model,
            channel,
        )

        from reBotArm_control_py.actuator import RebotArm
        from reBotArm_control_py.controllers import RebotArmEndPose
        from reBotArm_control_py.dynamics import compute_generalized_gravity
        from reBotArm_control_py.kinematics import (
            compute_fk,
            load_robot_model,
            pad_q_for_model,
            pos_rot_to_se3,
            solve_ik,
        )

        self._compute_fk = compute_fk
        self._pad_q_for_model = pad_q_for_model
        self._pos_rot_to_se3 = pos_rot_to_se3
        self._solve_ik = solve_ik

        runtime_config = hardware_data["_runtime"]
        control_runtime = runtime_config["control"]
        self._arm_control_mode = control_runtime["arm_control_mode"]

        self._robot = RebotArm(hw_yaml=str(hardware_config_path))
        self._arm_group = self._robot.groups.get("arm")
        if self._arm_group is None:
            raise ValueError("hardware config must define groups.arm")
        self._gripper_group = self._robot.groups.get("gripper")
        self._robot_get_state = self._robot.get_state
        self._robot.get_state = self._get_arm_state
        self._arm_mit_kp = np.array(control_runtime["mit_kp"], dtype=np.float64)
        self._arm_mit_kd = np.array(control_runtime["mit_kd"], dtype=np.float64)
        self._endpos_ctrl = RebotArmEndPose(
            self._robot,
            arm_control_mode=self._arm_control_mode,
        )
        if eef_streaming_ik_max_iter <= 0:
            raise ValueError("eef streaming IK max_iter must be positive")
        # Keep a separate iteration budget so streaming does not reduce the
        # endpoint IK budget used by move_to_pose and move_to_pose_ik.
        self._eef_streaming_ik_solver_params = replace(
            self._endpos_ctrl._ik_solver_params,
            max_iter=int(eef_streaming_ik_max_iter),
        )

        self._gripper_name = (
            self._gripper_group.joint_names[0]
            if self.has_gripper and self._gripper_group.joint_names
            else ""
        )
        gripper_data = hardware_data.get("gripper", {})
        gripper_limits = gripper_data.get("position_limits", {})
        self.gripper_open_position = float(gripper_limits.get("open", 0.0))
        self.gripper_close_position = float(gripper_limits.get("close", 0.0))
        self._gripper_target_position: float | None = None
        self._gripper_grasp_event = threading.Event()
        self._last_gripper_state = (0.0, 0.0, 0.0, 0)
        self._gripper_control: GripperControl | None = None
        if self.has_gripper:
            grasp = gripper_data.get("grasp", {})
            self._gripper_control = GripperControl(
                self.gripper_open_position,
                self.gripper_close_position,
                _GRIPPER_GOAL_TOLERANCE_RAD,
                GripperControlConfig(
                    close_torque=float(grasp.get("close_torque", 0.30)),
                    hold_torque=float(grasp.get("hold_torque", 0.15)),
                    torque_limit=float(grasp.get("torque_limit", 0.50)),
                    move_kp=float(grasp.get("move_kp", 5.0)),
                    move_kd=float(grasp.get("move_kd", 1.0)),
                    close_kd=float(grasp.get("close_kd", 0.5)),
                    stall_velocity=float(grasp.get("stall_velocity", 0.05)),
                    stall_duration=float(grasp.get("stall_duration", 0.10)),
                    startup_distance=float(grasp.get("startup_distance", 0.30)),
                ),
            )
            # HardwareManager owns all gripper output so the SDK loop cannot
            # overwrite torque-limited commands with its position target.
            self._endpos_ctrl._has_gripper = False

        self._gc_model = load_robot_model()
        self._gc_data = self._gc_model.createData()
        self._joint_lower_limits = np.asarray(
            self._gc_model.lowerPositionLimit[: len(self.joint_names)],
            dtype=np.float64,
        )
        self._joint_upper_limits = np.asarray(
            self._gc_model.upperPositionLimit[: len(self.joint_names)],
            dtype=np.float64,
        )
        self._gc_compute_generalized_gravity = compute_generalized_gravity
        gc_runtime = runtime_config["gravity_compensation"]
        self._gravity_comp_kp = np.array(gc_runtime["kp"], dtype=np.float64)
        self._gravity_comp_kd = np.array(gc_runtime["kd"], dtype=np.float64)
        self._gravity_comp_joint_direction = np.array(
            gc_runtime["joint_direction"],
            dtype=np.float64,
        )
        self._gravity_comp_tau_scale = np.array(
            gc_runtime["tau_scale"],
            dtype=np.float64,
        )

        self._connected = False
        self._enabled = False
        self._control_output_enabled = False
        self._state_machine = "IDLE"
        self._error_codes: list[str] = []
        self._gravity_comp_active = False
        self._gravity_comp_q_last: np.ndarray | None = None
        self._homing_thread: int | None = None

    # ------------------------------------------------------------------
    # state
    # ------------------------------------------------------------------

    @property
    def joint_names(self) -> list[str]:
        return list(self._arm_group.joint_names)

    @property
    def mode(self) -> str:
        return str(self._arm_group.mode)

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def control_loop_active(self) -> bool:
        return bool(self._robot.control_loop_active)

    @property
    def has_gripper(self) -> bool:
        return bool(self._robot.has_gripper)

    @property
    def state_machine(self) -> str:
        return self._state_machine

    @property
    def error_codes(self) -> list[str]:
        return list(self._error_codes)

    @_locked
    def set_state_machine(self, state: str) -> None:
        if state not in (
            "IDLE",
            "TRAJ_RUNNING",
            "LOWLEVEL_STREAMING",
            "EEF_STREAMING",
            "GRAVITY_COMP",
            "SAFE_HOMING",
        ):
            raise ValueError(f"unsupported state machine value: {state}")
        self._state_machine = state

    # ------------------------------------------------------------------
    # arm
    # ------------------------------------------------------------------

    @_locked
    def connect(self) -> None:
        if self._connected:
            return
        try:
            self._robot.connect()
            if self.has_gripper:
                position = self.get_gripper_state()[0]
                self._gripper_target_position = position
                self._gripper_control.set_position(position)
            self._start_endpos_loop()
            self._connected = True
            self._enabled = True
        except Exception:
            self._control_output_enabled = False
            self._endpos_ctrl._running = False
            try:
                self._robot.stop_control_loop()
                self._robot.disconnect()
            finally:
                self._connected = False
                self._enabled = False
            raise

    def shutdown(self, disable_after_safe_home: bool = True) -> None:
        if not self._connected:
            return
        try:
            self.safe_home()
            with self._cmd_lock:
                self._robot.stop_control_loop()

            if disable_after_safe_home:
                self.disable()

            with self._cmd_lock:
                self._robot.disconnect()
        finally:
            self._connected = False
            self._enabled = False
            self._control_output_enabled = False
            self.set_state_machine("IDLE")

    def get_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self._get_arm_state()

    def _get_arm_state(
        self,
        request_feedback: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        pos, vel, torq = self._robot_get_state(request_feedback=request_feedback)
        n = len(self.joint_names)
        return pos[:n], vel[:n], torq[:n]

    def get_joint_positions(self, request: bool = False) -> np.ndarray:
        return self._arm_group.get_positions(request_feedback=request)

    def get_joint_velocities(self, request: bool = False) -> np.ndarray:
        return self._arm_group.get_velocities(request_feedback=request)

    @_locked
    def hold_current_position(self) -> np.ndarray:
        current = self.get_joint_positions(request=True).copy()
        if self._state_machine != "SAFE_HOMING":
            self._endpos_ctrl._q_target[:] = current
            self._endpos_ctrl._qd_target[:] = 0.0
        return current

    @_locked
    def set_joint_position_target(self, positions) -> None:
        if self._state_machine == "SAFE_HOMING":
            raise RuntimeError("rejecting joint target during safe home")
        target = np.asarray(positions, dtype=np.float64).reshape(-1)
        if len(target) != len(self.joint_names):
            raise ValueError(
                f"expected {len(self.joint_names)} joint targets, got {len(target)}"
            )
        self.stop_motion()
        self._endpos_ctrl._q_target[:] = target
        self._endpos_ctrl._qd_target[:] = 0.0

    @_locked
    def start_endpos_control(self) -> None:
        if self._gravity_comp_active:
            raise RuntimeError("stop gravity compensation before starting endpos control")
        if self._state_machine in ("SAFE_HOMING", "TRAJ_RUNNING"):
            raise RuntimeError(f"rejecting endpos control in state {self._state_machine}")
        if self._state_machine == "EEF_STREAMING":
            return

        if self.control_loop_active:
            self.set_state_machine("IDLE")
            return

        self._robot.stop_control_loop()
        self._start_endpos_loop()
        self._enabled = True
        self.set_state_machine("IDLE")

    def enable(self) -> None:
        self.start_endpos_control()

    @_locked
    def disable(self) -> None:
        if self._gravity_comp_active:
            raise RuntimeError("stop gravity compensation before disable")
        self._control_output_enabled = False
        self.stop_motion()
        self._endpos_ctrl._running = False
        self._robot.stop_control_loop()
        self._robot.disable_all()
        self._enabled = False
        self.set_state_machine("IDLE")

    def safe_home(self) -> None:
        with self._cmd_lock:
            self.stop_motion()
            self.set_state_machine("IDLE")
            if self._gravity_comp_active:
                if self._arm_control_mode == "mit" and self.control_loop_active:
                    hold_target = (
                        self._gravity_comp_q_last.copy()
                        if self._gravity_comp_q_last is not None
                        else self._arm_group.get_positions(request_feedback=False).copy()
                    )
                    self._arm_group._mit_kp = self._arm_mit_kp.copy()
                    self._arm_group._mit_kd = self._arm_mit_kd.copy()
                    self._endpos_ctrl._q_target[:] = hold_target
                    self._endpos_ctrl._qd_target[:] = 0.0
                    self._endpos_ctrl._running = True
                    self._control_output_enabled = True
                    self._send_endpos_hold_once()
                    self._robot._ctrl_fn = self._endpos_loop_cb
                    self._gravity_comp_active = False
                    self._gravity_comp_q_last = None
                else:
                    self.stop_gravity_compensation()
            else:
                self.start_endpos_control()
            self.set_state_machine("SAFE_HOMING")
            self._homing_thread = threading.get_ident()
        try:
            if self.has_gripper:
                self.grasp_gripper(
                    target_position=self.gripper_close_position,
                    timeout=3.0,
                )
            self._endpos_ctrl.safe_home()
        finally:
            self._homing_thread = None
            self.set_state_machine("IDLE")

    @_locked
    def set_zero(self, joint_name: str = "") -> bool:
        self.stop_motion()
        self._robot.stop_control_loop()
        self._endpos_ctrl._running = False
        if joint_name:
            if joint_name not in self._robot._motor_map:
                raise KeyError(f"unknown joint: {joint_name}")
            self._set_zero_single(joint_name)
        else:
            self._robot.set_zero()
        self._enabled = False
        self.set_state_machine("IDLE")
        return True

    @_locked
    def send_joint_mit_cmd(
        self,
        joint_name: str,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        tau: float,
    ) -> None:
        index = self._joint_index(joint_name)
        self._begin_lowlevel_streaming("mit")
        q = self._arm_group.get_positions(request_feedback=True)
        target_pos = np.array(q, dtype=np.float64, copy=True)
        target_vel = np.zeros(len(self.joint_names), dtype=np.float64)
        target_tau = np.zeros(len(self.joint_names), dtype=np.float64)
        target_kp = np.array(self._arm_mit_kp, dtype=np.float64, copy=True)
        target_kd = np.array(self._arm_mit_kd, dtype=np.float64, copy=True)
        target_pos[index] = float(pos)
        target_vel[index] = float(vel)
        target_kp[index] = float(kp)
        target_kd[index] = float(kd)
        target_tau[index] = float(tau)
        self._arm_group.send_mit(
            target_pos,
            vel=target_vel,
            kp=target_kp,
            kd=target_kd,
            tau=target_tau,
        )
        self.set_state_machine("LOWLEVEL_STREAMING")

    @_locked
    def send_joint_pos_vel_cmd(
        self,
        joint_name: str,
        pos: float,
        vlim: float,
    ) -> None:
        index = self._joint_index(joint_name)
        self._begin_lowlevel_streaming("pos_vel")
        q = self._arm_group.get_positions(request_feedback=True)
        target_pos = np.array(q, dtype=np.float64, copy=True)
        target_vlim = np.array(
            getattr(self._arm_group, "_pv_vlim"), dtype=np.float64, copy=True
        )
        target_pos[index] = float(pos)
        target_vlim[index] = float(vlim)
        self._arm_group.send_pos_vel(target_pos, vlim=target_vlim)
        self.set_state_machine("LOWLEVEL_STREAMING")

    def current_pose(self):
        q, _, _ = self.get_joint_state()
        return self.pose_from_joint_positions(q)

    @_locked
    def pose_from_joint_positions(self, positions):
        q = np.asarray(positions, dtype=np.float64).reshape(-1)
        if len(q) != len(self.joint_names):
            raise ValueError(f"expected {len(self.joint_names)} joint positions")
        q_padded = self._pad_q_for_model(self._gc_model, q, len(self.joint_names))
        position, rotation, _ = self._compute_fk(self._gc_model, q_padded)
        return fk_to_pose(position, rotation)

    @property
    def joint_position_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return self._joint_lower_limits.copy(), self._joint_upper_limits.copy()

    def _require_idle(self, what: str) -> None:
        state = self._state_machine
        if state in ("TRAJ_RUNNING", "EEF_STREAMING", "GRAVITY_COMP", "SAFE_HOMING"):
            raise RuntimeError(f"rejecting {what} in state {state}")

    @_locked
    def begin_trajectory_stream(self) -> None:
        self._require_idle("trajectory stream")
        self.start_endpos_control()
        self.set_state_machine("TRAJ_RUNNING")

    def move_to_pose_traj(
        self, x, y, z, roll, pitch, yaw, duration: float
    ) -> bool:
        self.begin_trajectory_stream()
        ok = False
        try:
            ok = bool(
                self._endpos_ctrl.move_to_traj(x, y, z, roll, pitch, yaw, duration)
            )
        finally:
            if not ok and self._state_machine == "TRAJ_RUNNING":
                self.set_state_machine("IDLE")
        return ok

    @_locked
    def move_to_pose_ik(self, x, y, z, roll, pitch, yaw) -> tuple[bool, list[float]]:
        self._require_idle("IK target")
        self.start_endpos_control()
        ok = self._endpos_ctrl.move_to_ik(x, y, z, roll, pitch, yaw)
        return bool(ok), [float(v) for v in self._endpos_ctrl._q_target]

    @_locked
    def begin_eef_streaming(self) -> np.ndarray:
        if self._state_machine != "IDLE":
            raise RuntimeError(
                f"rejecting EEF streaming in state {self._state_machine}"
            )
        self.start_endpos_control()
        self._endpos_ctrl._vlim_override = None
        self.set_state_machine("EEF_STREAMING")
        return self.get_joint_positions(request=True).copy()

    @_locked
    def solve_eef_ik(self, pose, q_seed) -> np.ndarray | None:
        if self._state_machine != "EEF_STREAMING":
            raise RuntimeError("EEF streaming is not active")
        x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(pose)
        q_padded = self._pad_q_for_model(
            self._endpos_ctrl._model,
            np.asarray(q_seed, dtype=np.float64),
            len(self.joint_names),
        )
        target = self._pos_rot_to_se3(
            np.array([x, y, z]),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )
        result = self._solve_ik(
            self._endpos_ctrl._model,
            self._endpos_ctrl._data,
            self._endpos_ctrl._end_frame_id,
            target,
            q_padded,
            self._eef_streaming_ik_solver_params,
            controlled_joints=len(self.joint_names),
        )
        if not result.success:
            return None
        return np.asarray(result.q[: len(self.joint_names)], dtype=np.float64)

    @_locked
    def set_eef_streaming_target(self, positions, velocity_limits) -> None:
        if self._state_machine != "EEF_STREAMING":
            raise RuntimeError("EEF streaming is not active")
        q_target = np.asarray(positions, dtype=np.float64).reshape(-1)
        vlim = np.asarray(velocity_limits, dtype=np.float64).reshape(-1)
        if len(q_target) != len(self.joint_names) or len(vlim) != len(self.joint_names):
            raise ValueError("streaming targets must match the arm joint count")
        self._endpos_ctrl._q_target[:] = q_target
        self._endpos_ctrl._qd_target[:] = 0.0
        self._endpos_ctrl._vlim_override = vlim.copy()

    @_locked
    def stop_eef_streaming(self) -> None:
        if self._state_machine != "EEF_STREAMING":
            return
        self.hold_current_position()
        self._endpos_ctrl._vlim_override = None
        self.set_state_machine("IDLE")

    def get_joint_status_codes(self) -> list[int]:
        codes: list[int] = []
        for name in self.joint_names:
            try:
                st = self._robot._motor_map[name].get_state()
                codes.append(int(st.status_code if st is not None else 0))
            except Exception:
                codes.append(0)
        return codes

    # ------------------------------------------------------------------
    # gravity compensation
    # ------------------------------------------------------------------

    @_locked
    def start_gravity_compensation(self) -> None:
        if self._state_machine in ("TRAJ_RUNNING", "EEF_STREAMING", "SAFE_HOMING"):
            raise RuntimeError(
                f"rejecting gravity compensation in state {self._state_machine}"
            )
        self.stop_gravity_compensation()
        if not self._enabled:
            self._arm_group.enable()
            if self.has_gripper:
                self._gripper_group.enable()
            self._enabled = True
        self.stop_motion()
        self._robot.stop_control_loop()
        self._endpos_ctrl._running = False

        self._gravity_comp_q_last = self._arm_group.get_positions(
            request_feedback=True,
        ).copy()
        self._arm_group.mode_mit(
            kp=self._gravity_comp_kp,
            kd=self._gravity_comp_kd,
        )
        self._robot.disable_all()
        time.sleep(0.1)
        self._robot.enable_all()
        self._enabled = True
        self._gravity_comp_active = True
        arm_rate = float(getattr(self._robot, "_rate", 500.0))
        self._gravity_comp_tick(self._robot, 1.0 / arm_rate)
        self._robot.start_control_loop(self._gravity_comp_tick, rate=arm_rate)
        self.set_state_machine("GRAVITY_COMP")

    @_locked
    def stop_gravity_compensation(self) -> None:
        if not self._gravity_comp_active:
            return
        hold_target = (
            self._gravity_comp_q_last.copy()
            if self._gravity_comp_q_last is not None
            else None
        )
        self._robot.stop_control_loop()
        self._gravity_comp_active = False
        self._gravity_comp_q_last = None
        if self._enabled:
            self._start_endpos_hold(target=hold_target)
        self.set_state_machine("IDLE")

    def gravity_compensation_active(self) -> bool:
        return self._gravity_comp_active

    @staticmethod
    def _angles_near_reference(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
        delta = values - reference
        delta = (delta + np.pi) % (2.0 * np.pi) - np.pi
        return reference + delta

    def _read_gravity_comp_positions(
        self,
        *,
        request: bool = False,
        reference: np.ndarray | None = None,
    ) -> np.ndarray:
        q = self._arm_group.get_positions(request_feedback=request)
        ref = reference if reference is not None else self._gravity_comp_q_last
        if ref is not None:
            q = self._angles_near_reference(q, ref)
        self._gravity_comp_q_last = np.array(q, dtype=np.float64, copy=True)
        return self._gravity_comp_q_last.copy()

    def _gravity_comp_tick(self, _robot, dt: float) -> None:
        del dt
        if not self._cmd_lock.acquire(blocking=False):
            return
        try:
            if not self._gravity_comp_active:
                return

            q = self._read_gravity_comp_positions(request=True)
            q_for_model = q * self._gravity_comp_joint_direction
            q_model = self._pad_q_for_model(
                self._gc_model, q_for_model, len(self.joint_names)
            )
            tau_model = self._gc_compute_generalized_gravity(
                self._gc_model,
                q_model,
                self._gc_data,
            )[: len(self.joint_names)]
            tau_motor = (
                tau_model
                * self._gravity_comp_joint_direction
                * self._gravity_comp_tau_scale
            )

            self._arm_group.send_mit(
                q,
                vel=np.zeros(len(self.joint_names)),
                kp=self._gravity_comp_kp,
                kd=self._gravity_comp_kd,
                tau=tau_motor,
            )
        finally:
            self._cmd_lock.release()

    # ------------------------------------------------------------------
    # gripper
    # ------------------------------------------------------------------

    @_locked
    def set_gripper_target(self, position: float) -> None:
        self._begin_gripper_command(allow_endpos=True)
        target = self._gripper_control.set_position(position)
        self._gripper_grasp_event.clear()
        self._endpos_ctrl.set_gripper_target(target)
        self._gripper_target_position = target
        self._send_gripper_control_locked()

    def wait_gripper_target(self, timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.gripper_reached_target():
                return True
            time.sleep(0.02)
        return False

    def set_gripper_position(
        self,
        position: float,
        timeout: float = 3.0,
    ) -> tuple[bool, float]:
        self.set_gripper_target(position)
        reached = self.wait_gripper_target(timeout)
        return reached, self.get_gripper_state()[0]

    @_locked
    def start_gripper_grasp(
        self,
        target_position: float | None = None,
        max_effort: float | None = None,
    ) -> None:
        self._begin_gripper_command(allow_endpos=True)
        if self._gripper_control.state == GripperControl.CLOSING:
            raise RuntimeError("gripper grasp is already running")
        position = self.get_gripper_state()[0]
        target = (
            self.gripper_close_position
            if target_position is None
            else float(target_position)
        )
        if not self._gripper_control.is_closing_target(position, target):
            raise ValueError("gripper grasp target must move in the closing direction")
        self._gripper_grasp_event.clear()
        self._gripper_control.start_grasp(position, target, max_effort)
        self._gripper_target_position = None
        self._send_gripper_control_locked()
        if self._gripper_control.result is not None:
            self._gripper_grasp_event.set()

    def grasp_gripper(
        self,
        target_position: float | None = None,
        max_effort: float | None = None,
        timeout: float = 3.0,
    ) -> tuple[str, float]:
        if self._gripper_control is None:
            raise RuntimeError("gripper is not initialized")
        target = (
            self.gripper_close_position
            if target_position is None
            else float(target_position)
        )
        position = self.get_gripper_state()[0]
        if not self._gripper_control.is_closing_target(position, target):
            reached, final_position = self.set_gripper_position(target, timeout)
            result = (
                GripperControl.REACHED_TARGET if reached else GripperControl.TIMEOUT
            )
            return result, final_position

        self.start_gripper_grasp(target, max_effort)
        deadline = time.monotonic() + max(float(timeout), 0.0)
        while time.monotonic() < deadline:
            if self._gripper_grasp_event.wait(timeout=0.02):
                break
            self.get_gripper_state()

        result = self.gripper_grasp_result()
        if result is None:
            self.cancel_gripper_grasp(GripperControl.TIMEOUT)
            result = GripperControl.TIMEOUT
        return result, self.get_gripper_state()[0]

    @_locked
    def cancel_gripper_grasp(
        self,
        result: str = GripperControl.CANCELED,
    ) -> None:
        position = self.get_gripper_state()[0]
        self._gripper_control.cancel(position, result=result)
        self._gripper_target_position = position
        self._endpos_ctrl.set_gripper_target(position)
        self._send_gripper_control_locked()
        self._gripper_grasp_event.set()

    @_locked
    def gripper_grasp_result(self) -> str | None:
        return self._gripper_control.result if self._gripper_control is not None else None

    @_locked
    def gripper_grasp_active(self) -> bool:
        return (
            self._gripper_control is not None
            and self._gripper_control.state == GripperControl.CLOSING
        )

    @_locked
    def gripper_target_is_closing(self, target: float) -> bool:
        if self._gripper_control is None:
            return False
        position = self._read_gripper_state_cached()[0]
        return self._gripper_control.is_closing_target(position, target)

    def get_gripper_state(self) -> tuple[float, float, float, int]:
        if not self.has_gripper or not self._gripper_name:
            return 0.0, 0.0, 0.0, 0
        pos = float(self._gripper_group.get_positions()[0])
        vel = float(self._gripper_group.get_velocities(request_feedback=False)[0])
        status = 0
        torque = 0.0
        try:
            st = self._robot._motor_map[self._gripper_name].get_state()
            if st is not None:
                torque = float(st.torq)
                status = int(st.status_code)
        except Exception:
            status = 0
        self._last_gripper_state = (float(pos), float(vel), float(torque), status)
        return self._last_gripper_state

    def _read_gripper_state_cached(self) -> tuple[float, float, float, int]:
        try:
            st = self._robot._motor_map[self._gripper_name].get_state()
            if st is not None:
                self._last_gripper_state = (
                    float(st.pos),
                    float(st.vel),
                    float(st.torq),
                    int(st.status_code),
                )
        except Exception:
            pass
        return self._last_gripper_state

    def gripper_reached_target(self) -> bool:
        if self._gripper_target_position is None:
            return True
        pos = self.get_gripper_state()[0]
        return abs(pos - self._gripper_target_position) < _GRIPPER_GOAL_TOLERANCE_RAD

    def _send_gripper_control_locked(self) -> None:
        if self._gripper_control is None:
            return
        position, velocity, _, _ = self._read_gripper_state_cached()
        previous_result = self._gripper_control.result
        command = self._gripper_control.tick(position, velocity, time.monotonic())
        self._gripper_group.send_mit(
            np.array([command.position], dtype=np.float64),
            vel=np.array([command.velocity], dtype=np.float64),
            kp=np.array([command.kp], dtype=np.float64),
            kd=np.array([command.kd], dtype=np.float64),
            tau=np.array([command.torque], dtype=np.float64),
        )
        if (
            self._gripper_control.result is not None
            and self._gripper_control.result != previous_result
        ):
            self._gripper_grasp_event.set()

    @_locked
    def send_gripper_mit_cmd(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        tau: float,
    ) -> None:
        self._begin_gripper_command()
        self._begin_gripper_lowlevel("mit")
        self._gripper_group.send_mit(
            np.array([float(pos)], dtype=np.float64),
            vel=np.array([float(vel)], dtype=np.float64),
            kp=np.array([float(kp)], dtype=np.float64),
            kd=np.array([float(kd)], dtype=np.float64),
            tau=np.array([float(tau)], dtype=np.float64),
        )
        self._gripper_target_position = None

    @_locked
    def send_gripper_pos_vel_cmd(self, pos: float, vlim: float) -> None:
        self._begin_gripper_command()
        self._begin_gripper_lowlevel("pos_vel")
        self._gripper_group.send_pos_vel(
            np.array([float(pos)], dtype=np.float64),
            vlim=np.array([float(vlim)], dtype=np.float64),
        )
        self._gripper_target_position = None

    def _begin_gripper_command(self, *, allow_endpos: bool = False) -> None:
        if not self._enabled:
            raise RuntimeError("rejecting gripper command while arm is disabled")
        if self._gravity_comp_active or self.state_machine == "GRAVITY_COMP":
            raise RuntimeError("rejecting gripper command during gravity compensation")
        if (
            self.state_machine == "SAFE_HOMING"
            and threading.get_ident() != self._homing_thread
        ):
            raise RuntimeError("rejecting gripper command during safe home")
        if self.state_machine == "TRAJ_RUNNING":
            raise RuntimeError("rejecting gripper command while trajectory is running")
        if not self.has_gripper or not self._gripper_name:
            raise RuntimeError("gripper is not initialized")
        if not allow_endpos and self.control_loop_active:
            if self._gripper_control.state == GripperControl.CLOSING:
                position = self._read_gripper_state_cached()[0]
                self._gripper_control.cancel(position)
                self._gripper_grasp_event.set()
            self.stop_motion()
            self._robot.stop_control_loop()
            self._endpos_ctrl._running = False

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _begin_lowlevel_streaming(self, required_mode: str) -> None:
        if not self._enabled:
            raise RuntimeError("rejecting low-level command while arm is disabled")
        if self._gravity_comp_active or self.state_machine == "GRAVITY_COMP":
            raise RuntimeError("rejecting low-level command during gravity compensation")
        if self.state_machine == "EEF_STREAMING":
            raise RuntimeError("rejecting low-level command during EEF streaming")
        if self.state_machine == "SAFE_HOMING":
            raise RuntimeError("rejecting low-level command during safe home")
        if self.state_machine == "TRAJ_RUNNING":
            self.stop_motion()
        self._robot.stop_control_loop()
        self._endpos_ctrl._running = False

        if required_mode != self.mode:
            self._enter_mode(
                self._arm_group,
                required_mode,
                "arm",
                kp=self._arm_mit_kp,
                kd=self._arm_mit_kd,
            )
        self.set_state_machine("LOWLEVEL_STREAMING")

    def _begin_gripper_lowlevel(self, required_mode: str) -> None:
        self._enter_mode(self._gripper_group, required_mode, "gripper")
        self.set_state_machine("LOWLEVEL_STREAMING")

    @staticmethod
    def _enter_mode(group, required_mode: str, label: str, **mit_gains) -> None:
        if required_mode == "mit":
            ok = group.mode_mit(**mit_gains)
        elif required_mode == "pos_vel":
            ok = group.mode_pos_vel()
        else:
            raise ValueError(f"unsupported low-level mode: {required_mode}")
        if not ok:
            raise RuntimeError(f"{label} did not enter {required_mode} mode")

    def _start_endpos_hold(self, target: np.ndarray | None = None) -> None:
        if self.control_loop_active:
            return
        self._start_endpos_loop(target)

    def _start_endpos_loop(self, target: np.ndarray | None = None) -> None:
        self._configure_groups_for_endpos()
        if self.has_gripper:
            gripper_position = self.get_gripper_state()[0]
            self._gripper_control.set_position(gripper_position)
            self._gripper_target_position = gripper_position
            self._gripper_grasp_event.clear()
        if target is None:
            self.hold_current_position()
        else:
            self._endpos_ctrl._q_target[:] = np.asarray(target, dtype=np.float64)
            self._endpos_ctrl._qd_target[:] = 0.0
        self._control_output_enabled = True
        self._robot.start_control_loop(self._endpos_loop_cb)
        self._endpos_ctrl._running = True

    def _configure_groups_for_endpos(self) -> None:
        if self._arm_control_mode == "mit":
            self._arm_group.mode_mit(
                kp=self._arm_mit_kp,
                kd=self._arm_mit_kd,
            )
        else:
            self._arm_group.mode_pos_vel()
        self._arm_group.enable()
        if self.has_gripper:
            if not self._gripper_group.mode_mit():
                raise RuntimeError("gripper did not enter mit mode")
            self._gripper_group.enable()

    @_locked
    def stop_motion(self) -> None:
        self._endpos_ctrl._stop_send.set()
        if self._endpos_ctrl._send_thread is not None:
            self._endpos_ctrl._send_thread.join(timeout=5.0)
        self._endpos_ctrl._moving = False
        self._endpos_ctrl._stop_send.clear()

    def motion_active(self) -> bool:
        return bool(self._endpos_ctrl._moving)

    def _endpos_loop_cb(self, robot, dt: float) -> None:
        del dt
        if not self._cmd_lock.acquire(blocking=False):
            return
        try:
            if not self._control_output_enabled:
                return
            self._endpos_ctrl._loop_cb(robot, 0.0)
            if self.has_gripper:
                self._send_gripper_control_locked()
        finally:
            self._cmd_lock.release()

    def _send_endpos_hold_once(self) -> None:
        if self._arm_control_mode == "mit":
            self._arm_group.send_mit(
                self._endpos_ctrl._q_target,
                vel=self._endpos_ctrl._qd_target,
                kp=getattr(self._arm_group, "_mit_kp"),
                kd=getattr(self._arm_group, "_mit_kd"),
            )
        else:
            self._arm_group.send_pos_vel(
                self._endpos_ctrl._q_target,
                vlim=getattr(self._arm_group, "_pv_vlim"),
            )

    def _joint_index(self, joint_name: str) -> int:
        try:
            return self.joint_names.index(joint_name)
        except ValueError as exc:
            raise KeyError(f"unknown joint: {joint_name}") from exc

    def _set_zero_single(self, joint_name: str) -> None:
        self._robot.disable_all()
        time.sleep(0.3)
        motor = self._robot._motor_map[joint_name]
        ctrl = None
        for joint in self._robot._all_joints:
            if joint.name == joint_name:
                ctrl = self._robot._ctrl_map[str(joint.vendor)]
                break
        if ctrl is None:
            raise KeyError(f"unknown joint: {joint_name}")
        for _ in range(200):
            try:
                motor.request_feedback()
                ctrl.poll_feedback_once()
            except Exception:
                pass
            st = motor.get_state()
            if st is not None and st.status_code == 0:
                break
            time.sleep(0.05)
        motor.set_zero_position()
