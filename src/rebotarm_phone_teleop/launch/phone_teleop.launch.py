from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    command_enabled = LaunchConfiguration("command_enabled")
    enable_orientation = LaunchConfiguration("enable_orientation")
    position_scale = LaunchConfiguration("position_scale")

    return LaunchDescription(
        [
            DeclareLaunchArgument("command_enabled", default_value="true"),
            DeclareLaunchArgument("enable_orientation", default_value="true"),
            DeclareLaunchArgument("position_scale", default_value="0.8"),
            Node(
                package="rebotarm_phone_teleop",
                executable="phone_eef_teleop",
                name="phone_eef_teleop",
                output="screen",
                parameters=[
                    {
                        "command_enabled": ParameterValue(
                            command_enabled,
                            value_type=bool,
                        ),
                        "enable_orientation": ParameterValue(
                            enable_orientation,
                            value_type=bool,
                        ),
                        "position_scale": ParameterValue(
                            position_scale,
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
