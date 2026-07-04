from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("savo_head")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    package_share,
                    "launch",
                    "head_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "backend": LaunchConfiguration("backend"),
            "use_python_fallback": LaunchConfiguration("use_python_fallback"),
            "enable_scan": "false",
            "enable_tf": "false",
            "enable_status": "false",
            "enable_apriltag_confirm": "false",
            "center_on_start": LaunchConfiguration("center_on_start"),
            "center_on_shutdown": LaunchConfiguration("center_on_shutdown"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="pca9685",
                description="Hardware backend. Use pca9685 on robot, dryrun for PC smoke.",
            ),
            DeclareLaunchArgument(
                "use_python_fallback",
                default_value="false",
                description="Use Python fallback controller instead of C++ production controller.",
            ),
            DeclareLaunchArgument(
                "center_on_start",
                default_value="false",
                description="Center pan-tilt when controller starts.",
            ),
            DeclareLaunchArgument(
                "center_on_shutdown",
                default_value="true",
                description="Center pan-tilt when controller shuts down.",
            ),
            bringup,
        ]
    )
