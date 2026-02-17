#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — RealSense Bringup Launch (savo_perception)

Starts Intel RealSense D435 via realsense2_camera.
This launch is intentionally minimal and reliable for your current use:
  - We NEED: /camera/camera/depth/image_rect_raw  (for depth_front_min_node.py)
  - Optional: color stream for debugging/UI

Current topics (confirmed):
  /camera/camera/depth/image_rect_raw
  /camera/camera/color/image_raw
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ---- Launch args ----
    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="camera",
        description="RealSense camera namespace/name (default: camera)",
    )

    # Enable/disable streams
    enable_depth_arg = DeclareLaunchArgument(
        "enable_depth",
        default_value="true",
        description="Enable depth stream",
    )
    enable_color_arg = DeclareLaunchArgument(
        "enable_color",
        default_value="true",
        description="Enable color stream (useful for debugging)",
    )

    # Recommended stable defaults for D435
    depth_width_arg = DeclareLaunchArgument("depth_width", default_value="640")
    depth_height_arg = DeclareLaunchArgument("depth_height", default_value="480")
    depth_fps_arg = DeclareLaunchArgument("depth_fps", default_value="30")

    color_width_arg = DeclareLaunchArgument("color_width", default_value="640")
    color_height_arg = DeclareLaunchArgument("color_height", default_value="480")
    color_fps_arg = DeclareLaunchArgument("color_fps", default_value="30")

    # Pointcloud is optional and heavier; keep off by default (LiDAR-first costmaps for now)
    pointcloud_arg = DeclareLaunchArgument(
        "pointcloud",
        default_value="false",
        description="Enable pointcloud publishing (heavier; keep false unless needed)",
    )

    # Align depth to color is not required for front-min ROI; keep false for performance
    align_depth_arg = DeclareLaunchArgument(
        "align_depth",
        default_value="false",
        description="Align depth to color (not required for depth_front_min_node)",
    )

    camera_name = LaunchConfiguration("camera_name")
    enable_depth = LaunchConfiguration("enable_depth")
    enable_color = LaunchConfiguration("enable_color")

    depth_width = LaunchConfiguration("depth_width")
    depth_height = LaunchConfiguration("depth_height")
    depth_fps = LaunchConfiguration("depth_fps")

    color_width = LaunchConfiguration("color_width")
    color_height = LaunchConfiguration("color_height")
    color_fps = LaunchConfiguration("color_fps")

    pointcloud = LaunchConfiguration("pointcloud")
    align_depth = LaunchConfiguration("align_depth")

    # ---- Node: realsense2_camera ----
    # NOTE: package/executable names are standard for ROS2 Jazzy apt installs.
    # If your system uses rs_launch.py elsewhere, this is the direct equivalent.
    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera",
        namespace=camera_name,
        output="screen",
        parameters=[
            {
                # Streams
                "enable_depth": enable_depth,
                "enable_color": enable_color,

                # Depth stream
                "depth_module.profile": [depth_width, depth_height, depth_fps],

                # Color stream
                "rgb_camera.profile": [color_width, color_height, color_fps],

                # Optional features
                "pointcloud.enable": pointcloud,
                "align_depth.enable": align_depth,

                # Keep things stable/quiet
                "initial_reset": False,
                "unite_imu_method": "none",  # D435 IMU not used
                "enable_sync": False,
            }
        ],
    )

    return LaunchDescription(
        [
            camera_name_arg,
            enable_depth_arg,
            enable_color_arg,
            depth_width_arg,
            depth_height_arg,
            depth_fps_arg,
            color_width_arg,
            color_height_arg,
            color_fps_arg,
            pointcloud_arg,
            align_depth_arg,
            realsense_node,
        ]
    )
