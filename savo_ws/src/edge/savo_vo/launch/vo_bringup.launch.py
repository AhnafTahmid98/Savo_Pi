"""Launch the complete visual odometry stack for Robot Savo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_realsense = LaunchConfiguration("use_realsense")
    rgbd_config_file = LaunchConfiguration("rgbd_config_file")
    health_config_file = LaunchConfiguration("health_config_file")

    default_rgbd_config_file = PathJoinSubstitution(
        [
            FindPackageShare("savo_vo"),
            "config",
            "rgbd_odometry.yaml",
        ]
    )

    default_health_config_file = PathJoinSubstitution(
        [
            FindPackageShare("savo_vo"),
            "config",
            "vo_health.yaml",
        ]
    )

    realsense_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("savo_realsense"),
            "launch",
            "realsense_vo.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_realsense",
                default_value="false",
                description="Start the RealSense VO camera profile before VO nodes.",
            ),
            DeclareLaunchArgument(
                "rgbd_config_file",
                default_value=default_rgbd_config_file,
                description="Path to the RGB-D visual odometry config file.",
            ),
            DeclareLaunchArgument(
                "health_config_file",
                default_value=default_health_config_file,
                description="Path to the VO health config file.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realsense_launch_file),
                condition=IfCondition(use_realsense),
            ),
            Node(
                package="savo_vo",
                executable="rgbd_odometry_node",
                name="rgbd_odometry_node",
                output="screen",
                parameters=[rgbd_config_file],
            ),
            Node(
                package="savo_vo",
                executable="vo_health_node",
                name="vo_health_node",
                output="screen",
                parameters=[health_config_file],
            ),
            Node(
                package="savo_vo",
                executable="vo_diagnostics_node",
                name="vo_diagnostics_node",
                output="screen",
                parameters=[health_config_file],
            ),
        ]
    )