from __future__ import annotations

import threading

from rclpy.qos import QoSProfile, ReliabilityPolicy
from .hardware_manager import MitStreamGuardTripped, MitStreamRejected
from rebotarm_msgs.msg import (
    ArmMitCmd,
    JointMitCmd,
    JointPosVelCmd,
)


class MotorPassthrough:
    def __init__(self, node, hardware, namespace: str, arbitration: str) -> None:
        self._node = node
        self._hardware = hardware
        self._arbitration = arbitration
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._subscriptions = []

        joint_commands = (
            (
                JointMitCmd,
                "cmd/mit",
                lambda hw, name, msg: hw.send_joint_mit_cmd(
                    name,
                    msg.pos,
                    msg.vel,
                    msg.kp,
                    msg.kd,
                    msg.tau,
                ),
            ),
            (
                JointPosVelCmd,
                "cmd/pos_vel",
                lambda hw, name, msg: hw.send_joint_pos_vel_cmd(
                    name,
                    msg.pos,
                    msg.vlim,
                ),
            ),
        )
        gripper_commands = (
            (
                JointMitCmd,
                "cmd/mit",
                lambda hw, msg: hw.send_gripper_mit_cmd(
                    msg.pos,
                    msg.vel,
                    msg.kp,
                    msg.kd,
                    msg.tau,
                ),
            ),
            (
                JointPosVelCmd,
                "cmd/pos_vel",
                lambda hw, msg: hw.send_gripper_pos_vel_cmd(msg.pos, msg.vlim),
            ),
        )

        for joint_name in hardware.joint_names:
            for msg_type, label, command in joint_commands:
                self._subscribe(
                    msg_type,
                    f"/{namespace}/joints/{joint_name}/{label}",
                    self._make_joint_callback(
                        joint_name,
                        label,
                        command,
                    ),
                    qos,
                )
        # Latest command wins: the control loop resends it every cycle.
        self._subscribe(
            ArmMitCmd,
            f"/{namespace}/arm/stream/mit",
            self._arm_mit_stream_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        if hardware.has_gripper:
            for msg_type, label, command in gripper_commands:
                self._subscribe(
                    msg_type,
                    f"/{namespace}/gripper/{label}",
                    self._make_gripper_callback(label, command),
                    qos,
                )

    def _subscribe(self, msg_type, topic: str, callback, qos: QoSProfile) -> None:
        self._subscriptions.append(
            self._node.create_subscription(
                msg_type,
                topic,
                callback,
                qos,
                callback_group=self._node.reentrant_group,
            )
        )

    def _arm_mit_stream_callback(self, msg) -> None:
        try:
            started = self._hardware.stream_mit(msg.pos, msg.vel, msg.kp, msg.kd, msg.tau)
        except MitStreamGuardTripped as exc:
            self._node.get_logger().warn(f"arm MIT stream command rejected: {exc}")
            threading.Thread(target=self._home_after_guard, daemon=True).start()
            return
        except MitStreamRejected as exc:  # refusals are routine; any other error crashes
            self._node.get_logger().warn(f"arm MIT stream command rejected: {exc}")
            return
        if started:
            self._node.publish_arm_status()

    def _home_after_guard(self) -> None:
        self._hardware.safe_home(on_started=self._node.publish_arm_status)   # SAFE_HOMING tells the streamer
        self._hardware.disable()                                           # at rest at home: torque off
        self._node.publish_arm_status()

    def _make_joint_callback(self, joint_name: str, label: str, command) -> object:
        def _callback(msg) -> None:
            if not self._can_send_lowlevel(
                f"/joints/{joint_name}/{label}",
                allow_preempt=True,
            ):
                return

            try:
                command(self._hardware, joint_name, msg)
            except Exception as exc:
                self._node.get_logger().warn(
                    f"joint {label} failed for {joint_name}: {exc}"
                )
            finally:
                self._node.publish_arm_status()

        return _callback

    def _make_gripper_callback(self, label: str, command) -> object:
        def _callback(msg) -> None:
            if not self._can_send_lowlevel(
                f"/gripper/{label}",
                allow_preempt=False,
            ):
                return

            try:
                command(self._hardware, msg)
            except Exception as exc:
                self._node.get_logger().warn(f"gripper {label} failed: {exc}")
            finally:
                self._node.publish_arm_status()

        return _callback

    def _can_send_lowlevel(self, label: str, *, allow_preempt: bool) -> bool:
        state = self._hardware.state_machine
        if state in ("GRAVITY_COMP", "SAFE_HOMING"):
            self._node.get_logger().warn(f"rejecting {label} in state {state}")
            return False
        if state == "TRAJ_RUNNING":
            if self._arbitration == "reject" or not allow_preempt:
                self._node.get_logger().warn(
                    f"rejecting {label} while trajectory is running"
                )
                return False
            self._node.get_logger().warn(
                f"preempting trajectory for {label}"
            )
            self._hardware.stop_motion()
        return True
