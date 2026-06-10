"""Launch RGB-D visual odometry for Robot Savo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")

    default_config_file = PathJoinSubstitution(
        [
            FindPackageShare("savo_vo"),
            "config",
            "rgbd_odometry.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="Path to the RGB-D visual odometry config file.",
            ),
            Node(
                package="savo_vo",
                executable="rgbd_odometry_node",
                name="rgbd_odometry_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )