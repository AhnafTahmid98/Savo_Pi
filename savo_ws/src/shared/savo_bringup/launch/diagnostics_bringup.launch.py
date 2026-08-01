"""Launch non-motion Robot Savo core diagnostics."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch the core diagnostics profile with motion nodes suppressed."""
    return LaunchDescription(
        [
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
                }.items(),
            )
        ]
    )
