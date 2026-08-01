"""Launch verified AM-8 production navigation on the Robot Savo core."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Preserve the legacy saved-navigation entry point."""
    core_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare("savo_bringup"), "launch", "core_bringup.launch.py"]
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("bringup_profile", default_value="lidar_only"),
            DeclareLaunchArgument("d435_voxel_validated", default_value="false"),
            DeclareLaunchArgument(
                "production_map_root",
                default_value="/var/lib/robot_savo/maps/production",
            ),
            DeclareLaunchArgument("active_map_contract", default_value=""),
            DeclareLaunchArgument("control_startup_mode", default_value="STOP"),
            IncludeLaunchDescription(
                core_launch,
                launch_arguments={
                    "robot_mode": "saved_map_navigation",
                    "bringup_profile": LaunchConfiguration("bringup_profile"),
                    "d435_voxel_validated": LaunchConfiguration(
                        "d435_voxel_validated"
                    ),
                    "production_map_root": LaunchConfiguration(
                        "production_map_root"
                    ),
                    "active_map_contract": LaunchConfiguration(
                        "active_map_contract"
                    ),
                    "control_startup_mode": LaunchConfiguration(
                        "control_startup_mode"
                    ),
                }.items(),
            ),
        ]
    )
