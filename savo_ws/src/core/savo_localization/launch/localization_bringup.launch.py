#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch the Robot Savo core localization stack."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("savo_localization")

    imu_config = LaunchConfiguration("imu_config")
    encoders_config = LaunchConfiguration("encoders_config")
    wheel_odom_config = LaunchConfiguration("wheel_odom_config")
    ekf_config = LaunchConfiguration("ekf_config")
    vo_config = LaunchConfiguration("vo_config")
    topics_config = LaunchConfiguration("topics_config")
    frames_config = LaunchConfiguration("frames_config")
    diagnostics_config = LaunchConfiguration("diagnostics_config")

    baseline_profile_config = LaunchConfiguration("baseline_profile_config")
    vo_profile_config = LaunchConfiguration("vo_profile_config")

    use_imu = LaunchConfiguration("use_imu")
    use_wheel_odom = LaunchConfiguration("use_wheel_odom")
    use_ekf = LaunchConfiguration("use_ekf")
    use_vo = LaunchConfiguration("use_vo")
    use_health = LaunchConfiguration("use_health")
    use_dashboard = LaunchConfiguration("use_dashboard")
    use_state_publisher = LaunchConfiguration("use_state_publisher")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "imu_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "imu.yaml"]
                ),
                description="BNO055 IMU configuration.",
            ),
            DeclareLaunchArgument(
                "encoders_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "encoders.yaml"]
                ),
                description="Four-wheel encoder GPIO configuration.",
            ),
            DeclareLaunchArgument(
                "wheel_odom_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "wheel_odom.yaml"]
                ),
                description="Wheel odometry configuration.",
            ),
            DeclareLaunchArgument(
                "ekf_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "ekf_odom.yaml"]
                ),
                description="Base robot_localization EKF configuration.",
            ),
            DeclareLaunchArgument(
                "vo_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "vo_fusion_optional.yaml"]
                ),
                description="Optional EKF overlay for /vo/odom from savo-edge.",
            ),
            DeclareLaunchArgument(
                "topics_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "topics.yaml"]
                ),
                description="Shared localization topic contract.",
            ),
            DeclareLaunchArgument(
                "frames_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "frames.yaml"]
                ),
                description="Shared localization frame contract.",
            ),
            DeclareLaunchArgument(
                "diagnostics_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "diagnostics.yaml"]
                ),
                description="Diagnostic behavior profile.",
            ),
            DeclareLaunchArgument(
                "baseline_profile_config",
                default_value=PathJoinSubstitution(
                    [
                        package_share,
                        "config",
                        "profiles",
                        "robot_savo_4enc_imu_ekf.yaml",
                    ]
                ),
                description="Normal 4-encoder + IMU + EKF profile.",
            ),
            DeclareLaunchArgument(
                "vo_profile_config",
                default_value=PathJoinSubstitution(
                    [
                        package_share,
                        "config",
                        "profiles",
                        "robot_savo_4enc_imu_vo_ekf.yaml",
                    ]
                ),
                description="4-encoder + IMU + VO + EKF profile.",
            ),
            DeclareLaunchArgument(
                "use_imu",
                default_value="true",
                description="Start the C++ IMU node.",
            ),
            DeclareLaunchArgument(
                "use_wheel_odom",
                default_value="true",
                description="Start the C++ four-encoder wheel odometry node.",
            ),
            DeclareLaunchArgument(
                "use_ekf",
                default_value="true",
                description="Start robot_localization EKF.",
            ),
            DeclareLaunchArgument(
                "use_vo",
                default_value="false",
                description="Fuse /vo/odom from savo-edge.",
            ),
            DeclareLaunchArgument(
                "use_health",
                default_value="false",
                description="Start localization_health_node.",
            ),
            DeclareLaunchArgument(
                "use_dashboard",
                default_value="false",
                description="Start localization_dashboard.",
            ),
            DeclareLaunchArgument(
                "use_state_publisher",
                default_value="false",
                description="Start ekf_state_publisher_node.",
            ),

            Node(
                package="savo_localization",
                executable="imu_node",
                name="imu_node",
                output="screen",
                condition=IfCondition(use_imu),
                parameters=[
                    topics_config,
                    frames_config,
                    imu_config,
                    baseline_profile_config,
                ],
            ),

            Node(
                package="savo_localization",
                executable="wheel_odom_node",
                name="wheel_odom_node",
                output="screen",
                condition=IfCondition(use_wheel_odom),
                parameters=[
                    topics_config,
                    frames_config,
                    encoders_config,
                    wheel_odom_config,
                    baseline_profile_config,
                ],
            ),

            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                condition=UnlessCondition(use_vo),
                parameters=[
                    topics_config,
                    frames_config,
                    ekf_config,
                    baseline_profile_config,
                ],
                remappings=[
                    ("odometry/filtered", "/odometry/filtered"),
                ],
            ),

            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                condition=IfCondition(use_vo),
                parameters=[
                    topics_config,
                    frames_config,
                    ekf_config,
                    vo_config,
                    vo_profile_config,
                ],
                remappings=[
                    ("odometry/filtered", "/odometry/filtered"),
                ],
            ),

            Node(
                package="savo_localization",
                executable="localization_health_node.py",
                name="localization_health_node",
                output="screen",
                condition=IfCondition(use_health),
                parameters=[
                    topics_config,
                    frames_config,
                    diagnostics_config,
                    baseline_profile_config,
                ],
            ),

            Node(
                package="savo_localization",
                executable="ekf_state_publisher_node.py",
                name="ekf_state_publisher_node",
                output="screen",
                condition=IfCondition(use_state_publisher),
                parameters=[
                    topics_config,
                    frames_config,
                    diagnostics_config,
                    baseline_profile_config,
                ],
            ),

            Node(
                package="savo_localization",
                executable="localization_dashboard.py",
                name="localization_dashboard",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(use_dashboard),
                parameters=[
                    topics_config,
                    frames_config,
                    diagnostics_config,
                    baseline_profile_config,
                ],
            ),
        ]
    )