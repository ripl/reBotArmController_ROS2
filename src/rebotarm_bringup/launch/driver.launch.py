from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    driver_params = PathJoinSubstitution([bringup_share, "config", "driver_params.yaml"])
    hardware_config = LaunchConfiguration("hardware_config")
    model = LaunchConfiguration("model")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    joint_state_enabled = LaunchConfiguration("joint_state_enabled")
    hardware_connect_enabled = LaunchConfiguration("hardware_connect_enabled")
    hardware_output_loop_enabled = LaunchConfiguration("hardware_output_loop_enabled")
    controller_executor_threads = LaunchConfiguration("controller_executor_threads")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    arm_namespace = LaunchConfiguration("arm_namespace")
    disable_after_safe_home = LaunchConfiguration("disable_after_safe_home")
    eef_streaming_diagnostics_enabled = LaunchConfiguration("eef_streaming_diagnostics_enabled")
    eef_streaming_publish_target_tf = LaunchConfiguration("eef_streaming_publish_target_tf")
    eef_streaming_diagnostics_detail = LaunchConfiguration("eef_streaming_diagnostics_detail")
    eef_streaming_target_callback_diagnostics = LaunchConfiguration("eef_streaming_target_callback_diagnostics")
    eef_streaming_internal_target_enabled = LaunchConfiguration("eef_streaming_internal_target_enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "rebotarm_hardware.yaml"]
                ),
            ),
            DeclareLaunchArgument("model", default_value=""),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("joint_state_enabled", default_value="true"),
            DeclareLaunchArgument("hardware_connect_enabled", default_value="true"),
            DeclareLaunchArgument("hardware_output_loop_enabled", default_value="true"),
            DeclareLaunchArgument("controller_executor_threads", default_value="1"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("disable_after_safe_home", default_value="true"),
            DeclareLaunchArgument("eef_streaming_diagnostics_enabled", default_value="true"),
            DeclareLaunchArgument("eef_streaming_publish_target_tf", default_value="true"),
            DeclareLaunchArgument("eef_streaming_diagnostics_detail", default_value="true"),
            DeclareLaunchArgument("eef_streaming_target_callback_diagnostics", default_value="false"),
            DeclareLaunchArgument("eef_streaming_internal_target_enabled", default_value="false"),
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                parameters=[
                    driver_params,
                    {
                        "hardware_config": hardware_config,
                        "model": model,
                        "channel": channel,
                        "joint_state_rate": joint_state_rate,
                        "joint_state_enabled": ParameterValue(
                            joint_state_enabled,
                            value_type=bool,
                        ),
                        "hardware_connect_enabled": ParameterValue(
                            hardware_connect_enabled,
                            value_type=bool,
                        ),
                        "hardware_output_loop_enabled": ParameterValue(
                            hardware_output_loop_enabled,
                            value_type=bool,
                        ),
                        "controller_executor_threads": controller_executor_threads,
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                        "disable_after_safe_home": ParameterValue(
                            disable_after_safe_home,
                            value_type=bool,
                        ),
                        "eef_streaming.diagnostics_enabled": ParameterValue(
                            eef_streaming_diagnostics_enabled,
                            value_type=bool,
                        ),
                        "eef_streaming.publish_target_tf": ParameterValue(
                            eef_streaming_publish_target_tf,
                            value_type=bool,
                        ),
                        "eef_streaming.diagnostics_detail": ParameterValue(
                            eef_streaming_diagnostics_detail,
                            value_type=bool,
                        ),
                        "eef_streaming.target_callback_diagnostics_enabled": ParameterValue(
                            eef_streaming_target_callback_diagnostics,
                            value_type=bool,
                        ),
                        "eef_streaming.internal_target_enabled": ParameterValue(
                            eef_streaming_internal_target_enabled,
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
