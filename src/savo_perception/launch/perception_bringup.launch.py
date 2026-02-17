#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Perception Bringup Launch (savo_perception)

Starts the full near-field safety perception chain:

Nodes:
  - vl53_node              -> /savo_perception/range/left_m, /right_m
  - ultrasonic_node        -> /savo_perception/range/front_ultrasonic_m
  - depth_front_min_node   -> /depth/min_front_m
  - safety_stop_node       -> /safety/stop, /safety/slowdown_factor

Optional:
  - realsense2_camera can be started by separate launch (realsense_bringup.launch.py)
    or enabled here via `start_realsense:=true`.

Config (recommended):
  - config/range_safety.yaml
  - config/depth_front.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ---- Launch args ----
    start_realsense_arg = DeclareLaunchArgument(
        "start_realsense",
        default_value="false",
        description="If true, also start RealSense driver (realsense_bringup.launch.py).",
    )

    range_cfg_arg = DeclareLaunchArgument(
        "range_cfg",
        default_value=PathJoinSubstitution(
            [FindPackageShare("savo_perception"), "config", "range_safety.yaml"]
        ),
        description="Path to range safety config YAML.",
    )

    depth_cfg_arg = DeclareLaunchArgument(
        "depth_cfg",
        default_value=PathJoinSubstitution(
            [FindPackageShare("savo_perception"), "config", "depth_front.yaml"]
        ),
        description="Path to depth front-min config YAML.",
    )

    start_realsense = LaunchConfiguration("start_realsense")
    range_cfg = LaunchConfiguration("range_cfg")
    depth_cfg = LaunchConfiguration("depth_cfg")

    # ---- Optional: include RealSense bringup ----
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("savo_perception"), "launch", "realsense_bringup.launch.py"]
            )
        ),
        launch_arguments={
            # Keep defaults; you can override from CLI if needed
            "camera_name": "camera",
            "enable_depth": "true",
            "enable_color": "true",
            "pointcloud": "false",
            "align_depth": "false",
        }.items(),
    )

    # Wrap in a group so you can toggle it cleanly
    realsense_group = GroupAction(
        actions=[realsense_launch],
        condition=None,  # we'll control via "if" style below using launch_ros Node? Not needed; use Python if.
    )

    # NOTE: LaunchConditions are possible, but simplest is: include always, and let user set start_realsense true/false
    # via an `IfCondition`. We’ll use IfCondition for correctness.
    from launch.conditions import IfCondition  # local import keeps file clean
    realsense_launch_with_if = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("savo_perception"), "launch", "realsense_bringup.launch.py"]
            )
        ),
        condition=IfCondition(start_realsense),
        launch_arguments={
            "camera_name": "camera",
            "enable_depth": "true",
            "enable_color": "true",
            "pointcloud": "false",
            "align_depth": "false",
        }.items(),
    )

    # ---- Perception nodes ----
    vl53_node = Node(
        package="savo_perception",
        executable="vl53_node",
        name="vl53_node",
        output="screen",
        parameters=[
            # Keep node params aligned with range_safety.yaml hints (but node has its own defaults too)
            {
                "rate_hz": 25.0,
                "median_n": 5,
                "stale_timeout_s": 0.30,
                "min_valid_m": 0.02,
                "max_valid_m": 3.00,
                "publish_raw": False,
            }
        ],
    )

    ultrasonic_node = Node(
        package="savo_perception",
        executable="ultrasonic_node",
        name="ultrasonic_node",
        output="screen",
        parameters=[
            {
                "rate_hz": 15.0,
                "stale_timeout_s": 0.30,
                "min_valid_m": 0.02,
                "max_valid_m": 3.00,
            }
        ],
    )

    depth_front_min_node = Node(
        package="savo_perception",
        executable="depth_front_min_node",
        name="depth_front_min_node",
        output="screen",
        parameters=[depth_cfg],  # YAML drives ROI/percentile/scale/topic
    )

    safety_stop_node = Node(
        package="savo_perception",
        executable="safety_stop_node",
        name="safety_stop_node",
        output="screen",
        parameters=[range_cfg],  # YAML drives thresholds, debounce, topics, etc.
    )

    return LaunchDescription(
        [
            start_realsense_arg,
            range_cfg_arg,
            depth_cfg_arg,

            # Optional RealSense driver
            realsense_launch_with_if,

            # Core chain
            vl53_node,
            ultrasonic_node,
            depth_front_min_node,
            safety_stop_node,
        ]
    )
