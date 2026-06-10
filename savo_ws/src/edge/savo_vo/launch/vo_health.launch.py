"""Launch visual odometry health monitoring for Robot Savo."""

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
            "vo_health.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="Path to the visual odometry health config file.",
            ),
            Node(
                package="savo_vo",
                executable="vo_health_node",
                name="vo_health_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="savo_vo",
                executable="vo_diagnostics_node",
                name="vo_diagnostics_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )