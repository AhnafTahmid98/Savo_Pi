#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Safety Bringup Launch (savo_perception)

Starts the near-field safety stack:
  - vl53_node              (ToF left/right)
  - ultrasonic_node        (front ultrasonic)
  - depth_front_min_node   (D435 front-min from depth ROI)
  - safety_stop_node       (fusion -> /safety/stop + /safety/slowdown_factor)
  - cmd_vel_safety_gate    (C++ gate: /cmd_vel -> /cmd_vel_safe)

Config files (installed under share/savo_perception/config):
  - range_safety.yaml
  - depth_front.yaml

Topics (locked contract):
  Inputs:
    /depth/min_front_m
    /savo_perception/range/front_ultrasonic_m
    /savo_perception/range/left_m
    /savo_perception/range/right_m
    /cmd_vel
  Outputs:
    /safety/stop
    /safety/slowdown_factor
    /cmd_vel_safe
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("savo_perception")
    range_cfg = os.path.join(pkg_share, "config", "range_safety.yaml")
    depth_cfg = os.path.join(pkg_share, "config", "depth_front.yaml")

    # Launch args (allow overriding config paths)
    range_config_arg = DeclareLaunchArgument(
        "range_config",
        default_value=range_cfg,
        description="Path to range_safety.yaml",
    )
    depth_config_arg = DeclareLaunchArgument(
        "depth_config",
        default_value=depth_cfg,
        description="Path to depth_front.yaml",
    )

    # Optional switches
    enable_depth_arg = DeclareLaunchArgument(
        "enable_depth",
        default_value="true",
        description="Start depth_front_min_node (RealSense D435 front-min)",
    )
    enable_gate_arg = DeclareLaunchArgument(
        "enable_gate",
        default_value="true",
        description="Start cmd_vel_safety_gate (C++ /cmd_vel -> /cmd_vel_safe)",
    )

    range_config = LaunchConfiguration("range_config")
    depth_config = LaunchConfiguration("depth_config")
    enable_depth = LaunchConfiguration("enable_depth")
    enable_gate = LaunchConfiguration("enable_gate")

    # ---------------- Nodes ----------------

    vl53_node = Node(
        package="savo_perception",
        executable="vl53_node",
        name="vl53_node",
        output="screen",
        parameters=[range_config],
    )

    ultrasonic_node = Node(
        package="savo_perception",
        executable="ultrasonic_node",
        name="ultrasonic_node",
        output="screen",
        parameters=[range_config],
    )

    depth_front_min_node = Node(
        package="savo_perception",
        executable="depth_front_min_node",
        name="depth_front_min_node",
        output="screen",
        parameters=[depth_config, range_config],
        condition=None,  # we apply condition below via IfCondition wrapper
    )

    safety_stop_node = Node(
        package="savo_perception",
        executable="safety_stop_node",
        name="safety_stop_node",
        output="screen",
        parameters=[range_config],
    )

    cmd_vel_gate = Node(
        package="savo_perception",
        executable="cmd_vel_safety_gate",
        name="cmd_vel_safety_gate",
        output="screen",
        parameters=[range_config],
        condition=None,  # we apply condition below via IfCondition wrapper
    )

    # Conditions (keep imports local to avoid lint noise if not used)
    from launch.conditions import IfCondition

    depth_front_min_node.condition = IfCondition(enable_depth)
    cmd_vel_gate.condition = IfCondition(enable_gate)

    # NOTE:
    # - This launch assumes your RealSense driver is started elsewhere, e.g.:
    #     ros2 launch realsense2_camera rs_launch.py
    # - If you want, we can add RealSense as an optional include later.

    return LaunchDescription(
        [
            range_config_arg,
            depth_config_arg,
            enable_depth_arg,
            enable_gate_arg,
            vl53_node,
            ultrasonic_node,
            depth_front_min_node,
            safety_stop_node,
            cmd_vel_gate,
        ]
    )
