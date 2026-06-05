from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("inc_dude"), "config", "inc_dude_params.yaml"]
                ),
                description="Path to the ROS 2 parameter file for evaluation.",
            ),
            Node(
                package="inc_dude",
                executable="evaluation",
                name="evaluation",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
