#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch wheel odom, IMU, EKF, and the optional localization dashboard."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _pkg_share(pkg: str) -> str:
    """Return the installed share directory of a ROS package."""
    return get_package_share_directory(pkg)


def generate_launch_description() -> LaunchDescription:
    """Build the localization launch description."""
    pkg = "savo_localization"
    share = _pkg_share(pkg)

    # launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_wheel_odom = LaunchConfiguration("use_wheel_odom")
    use_wheel_odom_fallback = LaunchConfiguration("use_wheel_odom_fallback")
    use_imu = LaunchConfiguration("use_imu")
    use_ekf = LaunchConfiguration("use_ekf")
    use_dashboard = LaunchConfiguration("use_dashboard")

    # Keep wheel odom TF off when EKF owns odom -> base_link.
    wheel_odom_publish_tf = LaunchConfiguration("wheel_odom_publish_tf")

    # config path overrides
    frames_yaml = LaunchConfiguration("frames_yaml")
    common_yaml = LaunchConfiguration("common_yaml")
    imu_yaml = LaunchConfiguration("imu_yaml")
    encoders_yaml = LaunchConfiguration("encoders_yaml")
    ekf_yaml = LaunchConfiguration("ekf_yaml")

    # default config paths
    default_frames_yaml = os.path.join(share, "config", "frames.yaml")
    default_common_yaml = os.path.join(share, "config", "localization_common.yaml")
    default_imu_yaml = os.path.join(share, "config", "imu.yaml")
    default_encoders_yaml = os.path.join(share, "config", "encoders.yaml")
    default_ekf_yaml = os.path.join(share, "config", "ekf_odom.yaml")

    # nodes
    wheel_odom_node = Node(
        package=pkg,
        executable="wheel_odom_node",
        name="wheel_odom_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            # EKF should be the only odom -> base_link TF publisher.
            {"publish_tf": wheel_odom_publish_tf},
            common_yaml,
            frames_yaml,
            encoders_yaml,
        ],
        condition=IfCondition(use_wheel_odom),
    )

    # Python fallback for boards where the C++ GPIO path is unavailable.
    wheel_odom_fallback_node = Node(
        package=pkg,
        executable="wheel_odom_fallback_node.py",
        name="wheel_odom_fallback_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            # EKF should be the only odom -> base_link TF publisher.
            {"publish_tf": False},
            common_yaml,
            frames_yaml,
            encoders_yaml,
        ],
        condition=IfCondition(use_wheel_odom_fallback),
    )

    imu_node = Node(
        package=pkg,
        executable="imu_node.py",  # matches installed filename
        name="imu_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            common_yaml,
            frames_yaml,
            imu_yaml,
        ],
        condition=IfCondition(use_imu),
    )

    # robot_localization EKF:
    # Fuses wheel odom + IMU and publishes /odometry/filtered + TF (odom -> base_link)
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            ekf_yaml,
        ],
        condition=IfCondition(use_ekf),
    )

    # Optional terminal diagnostics dashboard for localization bringup
    localization_dashboard_node = Node(
        package=pkg,
        executable="localization_dashboard.py",  # matches installed filename
        name="localization_dashboard",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            common_yaml,
            frames_yaml,
        ],
        condition=IfCondition(use_dashboard),
    )

    # -------------------------------------------------------------------------
    # Build LaunchDescription
    # -------------------------------------------------------------------------
    ld = LaunchDescription()

    # -------------------------------------------------------------------------
    # Declare launch arguments (CLI-configurable)
    # -------------------------------------------------------------------------
    ld.add_action(DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock (simulation time). Keep false on the real robot.",
    ))

    ld.add_action(DeclareLaunchArgument(
        "use_wheel_odom",
        default_value="true",
        description="Start wheel_odom_node (C++ rear-encoder odometry).",
    ))

    ld.add_action(DeclareLaunchArgument(
        "use_wheel_odom_fallback",
        default_value="false",
        description="Start Python fallback wheel odom publisher (debug/fallback).",
    ))

    ld.add_action(DeclareLaunchArgument(
        "use_imu",
        default_value="true",
        description="Start imu_node (BNO055 IMU publisher).",
    ))

    ld.add_action(DeclareLaunchArgument(
        "use_ekf",
        default_value="true",
        description="Start robot_localization ekf_node (requires ros-jazzy-robot-localization).",
    ))

    ld.add_action(DeclareLaunchArgument(
        "use_dashboard",
        default_value="false",
        description="Start localization_dashboard terminal diagnostics node.",
    ))

    # TF ownership / behavior
    ld.add_action(DeclareLaunchArgument(
        "wheel_odom_publish_tf",
        default_value="false",
        description="Whether wheel_odom_node publishes odom->base_link TF. Keep false when EKF is enabled.",
    ))

    # YAML file path overrides
    ld.add_action(DeclareLaunchArgument(
        "frames_yaml",
        default_value=default_frames_yaml,
        description="Path to frames.yaml (frame-id contract).",
    ))

    ld.add_action(DeclareLaunchArgument(
        "common_yaml",
        default_value=default_common_yaml,
        description="Path to localization_common.yaml (shared params).",
    ))

    ld.add_action(DeclareLaunchArgument(
        "imu_yaml",
        default_value=default_imu_yaml,
        description="Path to imu.yaml (IMU node parameters).",
    ))

    ld.add_action(DeclareLaunchArgument(
        "encoders_yaml",
        default_value=default_encoders_yaml,
        description="Path to encoders.yaml (encoder / wheel odom parameters).",
    ))

    ld.add_action(DeclareLaunchArgument(
        "ekf_yaml",
        default_value=default_ekf_yaml,
        description="Path to ekf_odom.yaml (robot_localization EKF config).",
    ))

    # -------------------------------------------------------------------------
    # Helpful startup logs (always shown)
    # -------------------------------------------------------------------------
    ld.add_action(LogInfo(msg=[
        "[savo_localization] bringup: ",
        "wheel_odom=", use_wheel_odom,
        " fallback=", use_wheel_odom_fallback,
        " imu=", use_imu,
        " ekf=", use_ekf,
        " dashboard=", use_dashboard,
    ]))
    ld.add_action(LogInfo(msg=["[savo_localization] wheel_odom_publish_tf: ", wheel_odom_publish_tf]))
    ld.add_action(LogInfo(msg=["[savo_localization] frames_yaml: ", frames_yaml]))
    ld.add_action(LogInfo(msg=["[savo_localization] common_yaml: ", common_yaml]))
    ld.add_action(LogInfo(msg=["[savo_localization] imu_yaml: ", imu_yaml]))
    ld.add_action(LogInfo(msg=["[savo_localization] encoders_yaml: ", encoders_yaml]))
    ld.add_action(LogInfo(msg=["[savo_localization] ekf_yaml: ", ekf_yaml]))

    # -------------------------------------------------------------------------
    # Conditional warning-style logs (Jazzy-safe using LogInfo + condition)
    # -------------------------------------------------------------------------

    # Warning if BOTH wheel odom sources are enabled (duplicate /wheel/odom risk)
    ld.add_action(LogInfo(
        condition=IfCondition(PythonExpression([
            '"', use_wheel_odom, '" == "true" and "',
            use_wheel_odom_fallback, '" == "true"'
        ])),
        msg="[savo_localization] WARNING: use_wheel_odom and use_wheel_odom_fallback are both true. "
            "This may create duplicate /wheel/odom publishers. Enable only one.",
    ))

    # Warning if EKF is enabled while wheel_odom TF publishing is also enabled (duplicate TF risk)
    ld.add_action(LogInfo(
        condition=IfCondition(PythonExpression([
            '"', use_ekf, '" == "true" and "',
            wheel_odom_publish_tf, '" == "true"'
        ])),
        msg="[savo_localization] WARNING: EKF is enabled while wheel_odom_publish_tf=true. "
            "This can cause duplicate TF (odom->base_link). Recommended: wheel_odom_publish_tf:=false.",
    ))

    # -------------------------------------------------------------------------
    # Add nodes (after args and logs)
    # -------------------------------------------------------------------------
    ld.add_action(wheel_odom_node)
    ld.add_action(wheel_odom_fallback_node)
    ld.add_action(imu_node)
    ld.add_action(ekf_node)
    ld.add_action(localization_dashboard_node)

    return ld
