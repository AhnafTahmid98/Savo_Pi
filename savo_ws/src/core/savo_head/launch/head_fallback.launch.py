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
            "use_python_fallback": "true",
            "enable_scan": LaunchConfiguration("enable_scan"),
            "enable_tf": LaunchConfiguration("enable_tf"),
            "enable_status": LaunchConfiguration("enable_status"),
            "enable_apriltag_confirm": LaunchConfiguration("enable_apriltag_confirm"),
            "center_on_start": LaunchConfiguration("center_on_start"),
            "center_on_shutdown": LaunchConfiguration("center_on_shutdown"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="dryrun",
                description="Fallback backend. Use dryrun for PC, pca9685 for robot hardware.",
            ),
            DeclareLaunchArgument(
                "enable_scan",
                default_value="true",
                description="Start Python fallback scan node.",
            ),
            DeclareLaunchArgument(
                "enable_tf",
                default_value="true",
                description="Start Python fallback TF node.",
            ),
            DeclareLaunchArgument(
                "enable_status",
                default_value="true",
                description="Start Python fallback status node.",
            ),
            DeclareLaunchArgument(
                "enable_apriltag_confirm",
                default_value="true",
                description="Start Python fallback AprilTag confirmation node.",
            ),
            DeclareLaunchArgument(
                "center_on_start",
                default_value="false",
                description="Center pan-tilt when fallback controller starts.",
            ),
            DeclareLaunchArgument(
                "center_on_shutdown",
                default_value="false",
                description="Do not move servos on fallback shutdown by default.",
            ),
            bringup,
        ]
    )
