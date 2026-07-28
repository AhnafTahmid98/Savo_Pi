# Copyright 2026 Ahnaf Tahmid

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")
    serial_no = ParameterValue(
        LaunchConfiguration("serial_no"),
        value_type=str,
    )

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        parameters=[
            config_file,
            {"serial_no": serial_no},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("savo_realsense"),
                "config",
                "realsense_pointcloud_camera.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "serial_no",
            default_value="801212070967",
            description="Required Intel RealSense D435 serial number.",
        ),
        realsense_node,
    ])
