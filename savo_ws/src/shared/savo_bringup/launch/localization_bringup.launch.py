"""Launch Robot Savo localization with description and sensors."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch localization and its non-motion sensor dependencies."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_vo", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("savo_bringup"),
                            "launch",
                            "core_bringup.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "robot_mode": "diagnostics",
                    "bringup_profile": "bench",
                    "require_locked_geometry": "false",
                    "allow_provisional_geometry": "true",
                    "start_base": "false",
                    "start_control": "false",
                    "start_supervisor": "false",
                    "start_location_lifecycle": "false",
                    "localization_use_vo": LaunchConfiguration("use_vo"),
                }.items(),
            ),
        ]
    )
