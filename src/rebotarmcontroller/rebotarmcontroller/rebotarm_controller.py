from __future__ import annotations

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rclpy.qos import qos_profile_sensor_data

from .hardware_manager import HardwareManager
from .eef_streaming_controller import EefStreamingController
from .motor_passthrough import MotorPassthrough
from .ros_actions import ArmActions
from .ros_publishers import JointStatePublisher
from .ros_services import ArmServices
from .streaming_diagnostics import StreamingDiagnostics


class reBotArmController(Node):
    def __init__(self) -> None:
        super().__init__("reBotArmController")

        self.reentrant_group = ReentrantCallbackGroup()
        self.slow_group = MutuallyExclusiveCallbackGroup()
        self.sensor_qos = qos_profile_sensor_data

        self.declare_parameter("hardware_config", "")
        self.declare_parameter("model", "")
        self.declare_parameter("channel", "")
        self.declare_parameter("joint_state_rate", 100.0)
        self.declare_parameter("joint_state_enabled", True)
        self.declare_parameter("hardware_connect_enabled", True)
        self.declare_parameter("hardware_output_loop_enabled", True)
        self.declare_parameter("controller_executor_threads", 1)
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("cmd_arbitration", "reject")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ee_frame_id", "end_link")
        self.declare_parameter("disable_after_safe_home", True)
        self.declare_parameter("eef_streaming.control_rate_hz", 50.0)
        self.declare_parameter("eef_streaming.target_timeout", 0.2)
        self.declare_parameter("eef_streaming.ik_max_iter", 20)
        self.declare_parameter("eef_streaming.max_linear_velocity", 0.10)
        self.declare_parameter("eef_streaming.max_angular_velocity", 0.60)
        self.declare_parameter(
            "eef_streaming.joint_velocity_limits",
            [0.4, 0.4, 0.4, 0.6, 0.6, 0.6],
        )
        self.declare_parameter("eef_streaming.workspace_min", [0.15, -0.35, 0.05])
        self.declare_parameter("eef_streaming.workspace_max", [0.55, 0.35, 0.55])
        self.declare_parameter("eef_streaming.joint_limit_margin", 0.05)
        self.declare_parameter("eef_streaming.diagnostics_enabled", True)
        self.declare_parameter("eef_streaming.publish_target_tf", True)
        self.declare_parameter("eef_streaming.diagnostics_detail", True)
        self.declare_parameter(
            "eef_streaming.target_callback_diagnostics_enabled", False
        )
        self.declare_parameter("eef_streaming.internal_target_enabled", False)
        self.declare_parameter("eef_streaming.internal_target_x", 0.3)
        self.declare_parameter("eef_streaming.internal_target_y_start", 0.0)
        self.declare_parameter("eef_streaming.internal_target_y_end", 0.4)
        self.declare_parameter("eef_streaming.internal_target_z", 0.3)
        self.declare_parameter("eef_streaming.internal_target_duration", 2.0)

        hardware_config = self.get_parameter("hardware_config").value or None
        model = str(self.get_parameter("model").value or "")
        channel = str(self.get_parameter("channel").value or "")
        eef_streaming_ik_max_iter = int(
            self.get_parameter("eef_streaming.ik_max_iter").value
        )
        self.arm_namespace = str(self.get_parameter("arm_namespace").value or "rebotarm").strip("/")
        joint_state_rate = float(self.get_parameter("joint_state_rate").value)
        joint_state_enabled = bool(self.get_parameter("joint_state_enabled").value)
        hardware_connect_enabled = bool(
            self.get_parameter("hardware_connect_enabled").value
        )
        hardware_output_loop_enabled = bool(
            self.get_parameter("hardware_output_loop_enabled").value
        )
        cmd_arbitration = str(self.get_parameter("cmd_arbitration").value or "reject")
        self.disable_after_safe_home = bool(
            self.get_parameter("disable_after_safe_home").value
        )
        diagnostics_enabled = bool(
            self.get_parameter("eef_streaming.diagnostics_enabled").value
        )
        if cmd_arbitration not in ("reject", "preempt"):
            self.get_logger().warn(
                f"unsupported cmd_arbitration={cmd_arbitration!r}; using 'reject'"
            )
            cmd_arbitration = "reject"

        self.streaming_diagnostics = (
            StreamingDiagnostics(self.get_logger()) if diagnostics_enabled else None
        )
        if hardware_connect_enabled:
            self.hardware = HardwareManager(
                hardware_config=hardware_config,
                model=model,
                channel=channel,
                eef_streaming_ik_max_iter=eef_streaming_ik_max_iter,
                output_loop_enabled=hardware_output_loop_enabled,
                diagnostics=self.streaming_diagnostics,
            )
            self.hardware.connect()
        else:
            self.hardware = _DisconnectedHardware()

        self.joint_state_publisher = JointStatePublisher(
            self,
            self.hardware,
            self.arm_namespace,
            joint_state_rate,
            enabled=joint_state_enabled,
            diagnostics=self.streaming_diagnostics,
        )
        self.arm_services = ArmServices(self, self.hardware, self.arm_namespace)
        self.arm_actions = ArmActions(self, self.hardware, self.arm_namespace)
        self.eef_streaming = EefStreamingController(
            self,
            self.hardware,
            self.arm_namespace,
            diagnostics=self.streaming_diagnostics,
        )
        self.motor_passthrough = MotorPassthrough(
            self,
            self.hardware,
            self.arm_namespace,
            cmd_arbitration,
        )

        self.get_logger().info(
            f"reBotArmController started: namespace=/{self.arm_namespace}, "
            f"joints={self.hardware.joint_names}, "
            f"hardware_connect_enabled={hardware_connect_enabled}"
        )

    def publish_arm_status(self, *, read_hardware: bool = True) -> None:
        self.joint_state_publisher.publish_status(read_hardware=read_hardware)

    def shutdown(self) -> None:
        self.eef_streaming.shutdown()
        self.hardware.shutdown(
            disable_after_safe_home=self.disable_after_safe_home,
        )


class _DisconnectedHardware:
    joint_names = [f"joint{index}" for index in range(1, 7)]
    has_gripper = False
    gripper_open_position = 0.0
    gripper_close_position = 0.0

    @property
    def mode(self) -> str:
        return "disconnected"

    @property
    def enabled(self) -> bool:
        return False

    @property
    def control_loop_active(self) -> bool:
        return False

    @property
    def state_machine(self) -> str:
        return "DISCONNECTED"

    @property
    def error_codes(self) -> list[str]:
        return ["hardware_connect_disabled"]

    @property
    def joint_position_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return np.full(6, -np.inf), np.full(6, np.inf)

    def connect(self) -> None:
        return

    def shutdown(self, disable_after_safe_home: bool = True) -> None:
        del disable_after_safe_home
        return

    def get_joint_status_codes(self) -> list[int]:
        return [0 for _ in self.joint_names]

    def get_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        zeros = np.zeros(len(self.joint_names), dtype=np.float64)
        return zeros.copy(), zeros.copy(), zeros.copy()

    def current_pose(self) -> Pose:
        pose = Pose()
        pose.orientation.w = 1.0
        return pose

    def gripper_grasp_active(self) -> bool:
        return False

    def set_state_machine(self, _state: str) -> None:
        return

    def motion_active(self) -> bool:
        return False

    def __getattr__(self, name: str):
        def _disabled(*_args, **_kwargs):
            raise RuntimeError(f"hardware is disconnected: {name}")

        return _disabled


def main(args=None) -> None:
    rclpy.init(args=args)
    node = reBotArmController()
    executor_threads = int(node.get_parameter("controller_executor_threads").value)
    executor = (
        SingleThreadedExecutor()
        if executor_threads <= 1
        else MultiThreadedExecutor(num_threads=executor_threads)
    )
    node.get_logger().info(
        f"reBotArmController executor threads={max(executor_threads, 1)}"
    )
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
