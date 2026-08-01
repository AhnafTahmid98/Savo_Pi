"""Launch supervised manual mapping on the Robot Savo core."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Preserve the legacy manual-mapping entry point."""
    core_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare("savo_bringup"), "launch", "core_bringup.launch.py"]
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_id", default_value="robot_savo_map"),
            DeclareLaunchArgument(
                "map_output_root",
                default_value="/var/lib/robot_savo/maps/sessions",
            ),
            DeclareLaunchArgument("bringup_profile", default_value="lidar_only"),
            DeclareLaunchArgument("require_locked_geometry", default_value="true"),
            DeclareLaunchArgument(
                "allow_provisional_geometry", default_value="false"
            ),
            DeclareLaunchArgument("control_startup_mode", default_value="STOP"),
            DeclareLaunchArgument("localization_use_vo", default_value="false"),
            IncludeLaunchDescription(
                core_launch,
                launch_arguments={
                    "robot_mode": "manual_mapping",
                    "map_id": LaunchConfiguration("map_id"),
                    "map_output_root": LaunchConfiguration("map_output_root"),
                    "bringup_profile": LaunchConfiguration("bringup_profile"),
                    "require_locked_geometry": LaunchConfiguration(
                        "require_locked_geometry"
                    ),
                    "allow_provisional_geometry": LaunchConfiguration(
                        "allow_provisional_geometry"
                    ),
                    "control_startup_mode": LaunchConfiguration(
                        "control_startup_mode"
                    ),
                    "localization_use_vo": LaunchConfiguration(
                        "localization_use_vo"
                    ),
                }.items(),
            ),
        ]
    )
