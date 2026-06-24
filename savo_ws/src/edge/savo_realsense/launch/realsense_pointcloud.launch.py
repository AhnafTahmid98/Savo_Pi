# Copyright 2026 Ahnaf Tahmid

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    camera_config_file = LaunchConfiguration("camera_config_file")

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        parameters=[camera_config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("savo_realsense"),
                "config",
                "realsense_pointcloud_camera.yaml",
            ]),
        ),
        realsense_node,
    ])
