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
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    arm_namespace = LaunchConfiguration("arm_namespace")
    disable_after_safe_home = LaunchConfiguration("disable_after_safe_home")
    eef_streaming_publish_target_tf = LaunchConfiguration("eef_streaming_publish_target_tf")
    eef_streaming_diagnostics_detail = LaunchConfiguration("eef_streaming_diagnostics_detail")

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
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("disable_after_safe_home", default_value="true"),
            DeclareLaunchArgument("eef_streaming_publish_target_tf", default_value="true"),
            DeclareLaunchArgument("eef_streaming_diagnostics_detail", default_value="true"),
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
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                        "disable_after_safe_home": ParameterValue(
                            disable_after_safe_home,
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
                    }
                ],
            ),
        ]
    )
