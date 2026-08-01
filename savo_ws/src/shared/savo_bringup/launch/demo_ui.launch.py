"""Launch the C++ Robot Savo UI on the edge display."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch the edge UI through its package-owned entry point."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile", default_value="pi"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("savo_ui"), "launch", "ui_bringup.launch.py"]
                    )
                ),
                launch_arguments={"profile": LaunchConfiguration("profile")}.items(),
            ),
        ]
    )
