from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_launch = PathJoinSubstitution(
        [
            FindPackageShare("savo_description"),
            "launch",
            "description.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "require_locked_geometry",
                default_value="true",
                description="Require the production-locked physical geometry profile",
            ),
            DeclareLaunchArgument(
                "allow_provisional_geometry",
                default_value="false",
                description="Controlled bench override for provisional geometry",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(description_launch),
                launch_arguments={
                    "require_locked_geometry": LaunchConfiguration(
                        "require_locked_geometry"
                    ),
                    "allow_provisional_geometry": LaunchConfiguration(
                        "allow_provisional_geometry"
                    ),
                }.items(),
            ),
        ]
    )
