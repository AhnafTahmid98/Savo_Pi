# Copyright 2026 Ahnaf Tahmid

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    camera_config_file = LaunchConfiguration("camera_config_file")
    nodes_config_file = LaunchConfiguration("nodes_config_file")
    use_depth_front_min = LaunchConfiguration("use_depth_front_min")
    require_vo_health = LaunchConfiguration("require_vo_health")
    require_obstacle_cloud_health = LaunchConfiguration(
        "require_obstacle_cloud_health"
    )
    enable_observer_color_relay = LaunchConfiguration(
        "enable_observer_color_relay"
    )
    realsense_start_delay_s = LaunchConfiguration(
        "realsense_start_delay_s"
    )
    camera_support_start_delay_s = LaunchConfiguration(
        "camera_support_start_delay_s"
    )
    observer_relay_start_delay_s = LaunchConfiguration(
        "observer_relay_start_delay_s"
    )

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        parameters=[camera_config_file],
    )

    health_node = Node(
        package="savo_realsense",
        executable="camera_health_node",
        name="camera_health_node",
        output="screen",
        parameters=[
            nodes_config_file,
            {
                "require_depth_signal": ParameterValue(
                    use_depth_front_min, value_type=bool
                ),
                "require_vo_health": ParameterValue(
                    require_vo_health, value_type=bool
                ),
                "require_obstacle_cloud_health": ParameterValue(
                    require_obstacle_cloud_health, value_type=bool
                ),
            },
        ],
    )

    depth_front_min_node = Node(
        package="savo_realsense",
        executable="depth_front_min_node",
        name="depth_front_min_node",
        output="screen",
        parameters=[nodes_config_file],
        condition=IfCondition(use_depth_front_min),
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
            (
                "qos_overrides./camera/camera/color/image_raw."
                "subscription.reliability"
            ): "best_effort",
            (
                "qos_overrides./camera/camera/color/image_raw."
                "subscription.durability"
            ): "volatile",
            (
                "qos_overrides./camera/camera/color/image_raw."
                "subscription.history"
            ): "keep_last",
            (
                "qos_overrides./camera/camera/color/image_raw."
                "subscription.depth"
            ): 1,
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
            "camera_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("savo_realsense"),
                "config",
                "realsense_d435_camera.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "nodes_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("savo_realsense"),
                "config",
                "realsense_d435_nodes.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "use_depth_front_min",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "require_vo_health",
            default_value="false",
            description="Require fresh /vo/health in RealSense health",
        ),
        DeclareLaunchArgument(
            "require_obstacle_cloud_health",
            default_value="false",
            description=(
                "Require fresh obstacle-cloud health in RealSense health"
            ),
        ),
        DeclareLaunchArgument(
            "enable_observer_color_relay",
            default_value="true",
            description="Publish the production observer compressed D435 color",
        ),
        DeclareLaunchArgument(
            "realsense_start_delay_s",
            default_value="0.0",
            description="Seconds from launch start before the D435 driver starts",
        ),
        DeclareLaunchArgument(
            "camera_support_start_delay_s",
            default_value="0.0",
            description=(
                "Seconds from launch start before camera health and "
                "depth_front_min start"
            ),
        ),
        DeclareLaunchArgument(
            "observer_relay_start_delay_s",
            default_value="0.0",
            description=(
                "Seconds from launch start before the compressed color relay starts"
            ),
        ),
        TimerAction(
            period=realsense_start_delay_s,
            actions=[realsense_node],
            cancel_on_shutdown=True,
        ),
        TimerAction(
            period=camera_support_start_delay_s,
            actions=[health_node, depth_front_min_node],
            cancel_on_shutdown=True,
        ),
        TimerAction(
            period=observer_relay_start_delay_s,
            actions=[observer_color_relay],
            cancel_on_shutdown=True,
        ),
    ])
