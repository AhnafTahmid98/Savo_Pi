from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")
    use_depth_front_min = LaunchConfiguration("use_depth_front_min")

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

    depth_front_min_node = Node(
        package="savo_realsense",
        executable="depth_front_min_node",
        name="depth_front_min_node",
        output="screen",
        parameters=[config_file],
        condition=_if_true(use_depth_front_min),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("savo_realsense"),
                "config",
                "realsense_d435.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "use_depth_front_min",
            default_value="true",
        ),
        realsense_launch,
        topic_monitor,
        health_node,
        depth_front_min_node,
    ])


def _if_true(value: LaunchConfiguration):
    from launch.conditions import IfCondition

    return IfCondition(value)