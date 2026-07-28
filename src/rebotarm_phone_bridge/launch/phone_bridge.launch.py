from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bind_host = LaunchConfiguration("bind_host")
    port = LaunchConfiguration("port")
    base_frame_id = LaunchConfiguration("base_frame_id")
    pose_timeout = LaunchConfiguration("pose_timeout")
    pose_filter_time_constant = LaunchConfiguration(
        "pose_filter_time_constant"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("bind_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="9000"),
            DeclareLaunchArgument("base_frame_id", default_value="base_link"),
            DeclareLaunchArgument("pose_timeout", default_value="0.5"),
            DeclareLaunchArgument(
                "pose_filter_time_constant",
                default_value="0.02",
            ),
            Node(
                package="rebotarm_phone_bridge",
                executable="phone_tracking_bridge",
                name="phone_tracking_bridge",
                output="screen",
                parameters=[
                    {
                        "bind_host": bind_host,
                        "port": ParameterValue(port, value_type=int),
                        "base_frame_id": base_frame_id,
                        "pose_timeout": ParameterValue(
                            pose_timeout,
                            value_type=float,
                        ),
                        "pose_filter.time_constant": ParameterValue(
                            pose_filter_time_constant,
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
