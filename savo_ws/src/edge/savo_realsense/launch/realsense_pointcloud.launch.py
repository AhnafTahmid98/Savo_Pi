# Copyright 2026 Ahnaf Tahmid

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        parameters=[{
            "camera_name": "camera",
            "camera_namespace": "camera",

            "enable_color": True,
            "enable_depth": True,
            "enable_infra": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_gyro": False,
            "enable_accel": False,

            "depth_module.depth_profile": "848x480x30",
            "rgb_camera.color_profile": "640x480x30",

            "align_depth.enable": True,
            "enable_sync": True,

            "pointcloud__neon_.enable": True,
            "pointcloud__neon_.stream_filter": 2,
            "pointcloud__neon_.stream_index_filter": 0,
            "pointcloud__neon_.allow_no_texture_points": True,
            "pointcloud__neon_.ordered_pc": False,
            "pointcloud__neon_.pointcloud_qos": "SENSOR_DATA",

            "publish_tf": True,
            "tf_publish_rate": 0.0,

            "initial_reset": False,
            "reconnect_timeout": 6.0,
            "wait_for_device_timeout": -1.0,
        }],
    )

    return LaunchDescription([
        realsense_node,
    ])
