from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(
        get_package_share_directory("savo_locations")
    )

    default_config = (
        package_share
        / "config"
        / "locations_node.yaml"
    )

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config),
            ),
            Node(
                package="savo_locations",
                executable="savo_locations_node",
                name="savo_locations",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
