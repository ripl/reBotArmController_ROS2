from __future__ import annotations

import threading
import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from sensor_msgs.msg import JointState

from .streaming_diagnostics import StreamingDiagnostics

_GRIPPER_MAX_WIDTH = 0.09


def _gripper_motor_to_joint_position(
    position: float,
    open_position: float,
    close_position: float,
) -> float:
    span = open_position - close_position
    ratio = 0.0 if span == 0.0 else (position - close_position) / span
    return max(0.0, min(_GRIPPER_MAX_WIDTH * 0.5, ratio * _GRIPPER_MAX_WIDTH * 0.5))


class JointStatePublisher:
    def __init__(
        self,
        node,
        hardware,
        namespace: str,
        rate_hz: float,
        diagnostics: StreamingDiagnostics | None = None,
    ) -> None:
        self._node = node
        self._hardware = hardware
        self._diagnostics = diagnostics
        self._publish_state_lock = threading.Lock()
        self._active_publishes = 0
        self._publish_sequence = 0
        self._publisher = node.create_publisher(
            JointState,
            f"/{namespace}/joint_states",
            node.sensor_qos,
            callback_group=node.reentrant_group,
        )
        self._joint_state_publishers = {
            name: node.create_publisher(
                JointMotorState,
                f"/{namespace}/joints/{name}/state",
                node.sensor_qos,
                callback_group=node.reentrant_group,
            )
            for name in hardware.joint_names
        }
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_publisher = node.create_publisher(
            ArmStatus,
            f"/{namespace}/arm_status",
            latched_qos,
            callback_group=node.reentrant_group,
        )
        self._gripper_state_publisher = None
        if hardware.has_gripper:
            self._gripper_state_publisher = node.create_publisher(
                JointMotorState,
                f"/{namespace}/gripper/state",
                node.sensor_qos,
                callback_group=node.reentrant_group,
            )
        period = 1.0 / max(float(rate_hz), 1.0)
        self._timer = node.create_timer(
            period,
            self.publish,
            callback_group=node.reentrant_group,
        )
        self.publish_status()

    def publish(self) -> None:
        callback_start_ns = time.monotonic_ns()
        with self._publish_state_lock:
            self._publish_sequence += 1
            sequence = self._publish_sequence
            self._active_publishes += 1
            concurrent_callbacks = self._active_publishes
        read_start_ns = time.monotonic_ns()
        pos = vel = effort = None
        message_stamp_ns = None
        outcome = "read_failed"
        try:
            try:
                pos, vel, effort = self._hardware.get_joint_state()
            except Exception as exc:
                self._node.get_logger().warn(f"joint state read failed: {exc}")
                return
            read_end_ns = time.monotonic_ns()

            msg = JointState()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            message_stamp_ns = (
                int(msg.header.stamp.sec) * 1_000_000_000
                + int(msg.header.stamp.nanosec)
            )
            status_codes = self._hardware.get_joint_status_codes()
            for i, name in enumerate(self._hardware.joint_names):
                motor_msg = JointMotorState()
                motor_msg.header = msg.header
                motor_msg.joint_name = name
                motor_msg.position = float(pos[i])
                motor_msg.velocity = float(vel[i])
                motor_msg.torque = float(effort[i])
                motor_msg.status_code = int(status_codes[i])
                self._joint_state_publishers[name].publish(motor_msg)

            msg.name = self._hardware.joint_names
            msg.position = [float(v) for v in pos]
            msg.velocity = [float(v) for v in vel]
            msg.effort = [float(v) for v in effort]

            if self._gripper_state_publisher is not None:
                g_pos, g_vel, g_torque, g_status = self._hardware.get_gripper_state()
                joint_pos = _gripper_motor_to_joint_position(
                    float(g_pos),
                    self._hardware.gripper_open_position,
                    self._hardware.gripper_close_position,
                )
                joint_vel = (
                    _gripper_motor_to_joint_position(
                        float(g_pos + g_vel),
                        self._hardware.gripper_open_position,
                        self._hardware.gripper_close_position,
                    )
                    - joint_pos
                )
                msg.name.extend(["gripper_joint1", "gripper_joint2"])
                msg.position.extend([joint_pos, joint_pos])
                msg.velocity.extend([joint_vel, joint_vel])
                msg.effort.extend([float(g_torque), float(g_torque)])

                gripper_msg = JointMotorState()
                gripper_msg.header = msg.header
                gripper_msg.joint_name = "gripper"
                gripper_msg.position = float(g_pos)
                gripper_msg.velocity = float(g_vel)
                gripper_msg.torque = float(g_torque)
                gripper_msg.status_code = int(g_status)
                self._gripper_state_publisher.publish(gripper_msg)

            self._publisher.publish(msg)
            outcome = "published"
        finally:
            callback_end_ns = time.monotonic_ns()
            with self._publish_state_lock:
                self._active_publishes -= 1
            if self._diagnostics is not None and self._diagnostics.active:
                self._diagnostics.record(
                    "joint_state_publish",
                    monotonic_ns=callback_start_ns,
                    sequence=sequence,
                    concurrent_callbacks=concurrent_callbacks,
                    read_duration_ns=(
                        callback_end_ns - read_start_ns
                        if pos is None
                        else read_end_ns - read_start_ns
                    ),
                    total_duration_ns=callback_end_ns - callback_start_ns,
                    message_stamp_ns=message_stamp_ns,
                    outcome=outcome,
                    position=None if pos is None else pos.tolist(),
                    velocity=None if vel is None else vel.tolist(),
                    effort=None if effort is None else effort.tolist(),
                )

    def publish_status(self, *, read_hardware: bool = True) -> None:
        msg = ArmStatus()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.mode = self._hardware.mode
        msg.enabled = self._hardware.enabled
        msg.control_loop_active = self._hardware.control_loop_active
        msg.state_machine = self._hardware.state_machine
        msg.joint_names = self._hardware.joint_names
        if read_hardware:
            msg.per_joint_status_code = self._hardware.get_joint_status_codes()
        else:
            msg.per_joint_status_code = [0 for _ in self._hardware.joint_names]
        msg.error_codes = self._hardware.error_codes
        self._status_publisher.publish(msg)
