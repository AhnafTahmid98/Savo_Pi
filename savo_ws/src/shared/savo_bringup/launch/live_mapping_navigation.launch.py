"""Launch the guarded live-mapping navigation stack used by AM-7/AM-8."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch live-map navigation through autonomous mapping ownership."""
    autonomous_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [
                FindPackageShare("savo_bringup"),
                "launch",
                "autonomous_mapping.launch.py",
            ]
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_id", default_value="robot_savo_map"),
            DeclareLaunchArgument(
                "map_output_root",
                default_value="/var/lib/robot_savo/maps/sessions",
            ),
            DeclareLaunchArgument("control_startup_mode", default_value="STOP"),
            IncludeLaunchDescription(
                autonomous_launch,
                launch_arguments={
                    "map_id": LaunchConfiguration("map_id"),
                    "map_output_root": LaunchConfiguration("map_output_root"),
                    "control_startup_mode": LaunchConfiguration(
                        "control_startup_mode"
                    ),
                }.items(),
            ),
        ]
    )
