# Copyright 2026 Ahnaf Tahmid

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    camera_config_file = LaunchConfiguration("camera_config_file")
    nodes_config_file = LaunchConfiguration("nodes_config_file")
    use_depth_front_min = LaunchConfiguration("use_depth_front_min")

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        parameters=[camera_config_file],
    )

    topic_monitor = Node(
        package="savo_realsense",
        executable="camera_topic_monitor_node",
        name="camera_topic_monitor_node",
        output="screen",
        parameters=[nodes_config_file],
    )

    health_node = Node(
        package="savo_realsense",
        executable="camera_health_node",
        name="camera_health_node",
        output="screen",
        parameters=[nodes_config_file],
    )

    depth_front_min_node = Node(
        package="savo_realsense",
        executable="depth_front_min_node",
        name="depth_front_min_node",
        output="screen",
        parameters=[nodes_config_file],
        condition=IfCondition(use_depth_front_min),
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
        DeclareLaunchArgument(
            "nodes_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("savo_realsense"),
                "config",
                "realsense_pointcloud_nodes.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "use_depth_front_min",
            default_value="true",
        ),
        realsense_node,
        topic_monitor,
        health_node,
        depth_front_min_node,
    ])
