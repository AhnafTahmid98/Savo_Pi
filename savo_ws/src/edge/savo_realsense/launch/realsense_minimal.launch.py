# Copyright 2026 Ahnaf Tahmid
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py",
            ])
        ),
        launch_arguments={
            "config_file": config_file,
        }.items(),
    )

    topic_monitor = Node(
        package="savo_realsense",
        executable="camera_topic_monitor_node",
        name="camera_topic_monitor_node",
        output="screen",
        parameters=[config_file],
    )

    health_node = Node(
        package="savo_realsense",
        executable="camera_health_node",
        name="camera_health_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("savo_realsense"),
                "config",
                "realsense_minimal.yaml",
            ]),
        ),
        realsense_launch,
        topic_monitor,
        health_node,
    ])
