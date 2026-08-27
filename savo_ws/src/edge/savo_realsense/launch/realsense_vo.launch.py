# Copyright 2026 Ahnaf Tahmid
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    driver_config_file = LaunchConfiguration("driver_config_file")
    monitor_config_file = LaunchConfiguration("monitor_config_file")
    enable_observer_color_relay = LaunchConfiguration(
        "enable_observer_color_relay"
    )

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        parameters=[driver_config_file],
    )

    topic_monitor = Node(
        package="savo_realsense",
        executable="camera_topic_monitor_node",
        name="camera_topic_monitor_node",
        output="screen",
        parameters=[monitor_config_file],
    )

    health_node = Node(
        package="savo_realsense",
        executable="camera_health_node",
        name="camera_health_node",
        output="screen",
        parameters=[monitor_config_file],
    )

    observer_color_relay = Node(
        package="image_transport",
        executable="republish",
        name="d435_observer_color_republisher",
        output="screen",
        condition=IfCondition(enable_observer_color_relay),
        parameters=[{
            "in_transport": "raw",
            "out_transport": "compressed",
        }],
        remappings=[
            ("in", "/camera/camera/color/image_raw"),
            (
                "out/compressed",
                "/savo_observer/d435/color/image_raw/compressed",
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "driver_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("savo_realsense"),
                "config",
                "realsense_vo_driver.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "monitor_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("savo_realsense"),
                "config",
                "realsense_vo_profile.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "enable_observer_color_relay",
            default_value="false",
            description=(
                "Publish an observer-only compressed copy of D435 color"
            ),
        ),
        realsense_node,
        topic_monitor,
        health_node,
        observer_color_relay,
    ])
