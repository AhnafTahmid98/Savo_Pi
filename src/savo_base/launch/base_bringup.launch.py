#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — Base Bringup Launch (savo_base)
--------------------------------------------
Professional one-command bringup for Robot Savo base stack.

Starts:
- base_driver_node            (hardware execution / dryrun capable)
- base_watchdog_node          (command freshness watchdog, optional)
- base_state_publisher_node   (state summary aggregator, optional)
- base_heartbeat_node         (liveness pulse/state, optional)
- base_diag_runner_node       (diagnostic command runner, optional)

Design goals
------------
- Clean layered YAML loading (shared configs + profile override)
- Easy profile switching:
    * dryrun_sim_motoroff
    * bench_test
    * real_robot_v1
- Optional supervisor/telemetry nodes for bringup and debugging
- Consistent topic wiring with locked Robot Savo perception/control pipeline

Recommended usage
-----------------
# Dry software-only test (no motor output)
ros2 launch savo_base base_bringup.launch.py profile:=dryrun_sim_motoroff

# Bench test (robot lifted / controlled test)
ros2 launch savo_base base_bringup.launch.py profile:=bench_test

# Real robot driving
ros2 launch savo_base base_bringup.launch.py profile:=real_robot_v1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("savo_base")

    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------
    profile = LaunchConfiguration("profile")
    use_watchdog = LaunchConfiguration("use_watchdog")
    use_state_publisher = LaunchConfiguration("use_state_publisher")
    use_heartbeat = LaunchConfiguration("use_heartbeat")
    use_diag_runner = LaunchConfiguration("use_diag_runner")
    output = LaunchConfiguration("output")
    log_level = LaunchConfiguration("log_level")

    # Optional custom profile path (if provided, it overrides profile selection)
    profile_path = LaunchConfiguration("profile_path")

    # -------------------------------------------------------------------------
    # Config file paths (installed under share/savo_base/config)
    # -------------------------------------------------------------------------
    topics_yaml = PathJoinSubstitution([pkg_share, "config", "topics.yaml"])
    safety_yaml = PathJoinSubstitution([pkg_share, "config", "safety_timeouts.yaml"])
    board_yaml = PathJoinSubstitution([pkg_share, "config", "motor_board_freenove.yaml"])
    mecanum_yaml = PathJoinSubstitution([pkg_share, "config", "mecanum_kinematics.yaml"])

    selected_profile_yaml = PathJoinSubstitution(
        [pkg_share, "config", "profiles", PythonExpression([profile, " + '.yaml'"])]
    )

    # If profile_path is empty string -> use selected_profile_yaml
    # else -> use profile_path
    effective_profile_yaml = PythonExpression(
        ["'", profile_path, "' if '", profile_path, "' != '' else '", selected_profile_yaml, "'"]
    )

    # -------------------------------------------------------------------------
    # Common base_driver_node parameter layering (profile last = strongest override)
    # NOTE:
    #   This is the intended professional layering model for your package.
    # -------------------------------------------------------------------------
    base_driver_params = [
        topics_yaml,
        safety_yaml,
        board_yaml,
        mecanum_yaml,
        effective_profile_yaml,
    ]

    # -------------------------------------------------------------------------
    # Nodes
    # -------------------------------------------------------------------------
    base_driver_node = Node(
        package="savo_base",
        executable="base_driver_node",
        name="base_driver_node",
        output=output,
        parameters=base_driver_params,
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Watchdog supervisor (does not directly drive motors)
    base_watchdog_node = Node(
        package="savo_base",
        executable="base_watchdog_node",
        name="base_watchdog_node",
        output=output,
        condition=IfCondition(use_watchdog),
        parameters=[
            {
                "cmd_topic": "/cmd_vel_safe",
                "timeout_s": 0.30,
                "warn_ratio": 0.70,
                "loop_hz": 20.0,
                "publish_hz": 5.0,
                "watchdog_state_topic": "/savo_base/watchdog_state",
                "watchdog_trip_topic": "/savo_base/watchdog_trip",
                "publish_trip_topic": True,
                "publish_stop_request": False,
                "stop_request_topic": "/savo_base/watchdog_stop_request",
                "pretty_json": False,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Aggregated state summary publisher
    base_state_publisher_node = Node(
        package="savo_base",
        executable="base_state_publisher_node",
        name="base_state_publisher_node",
        output=output,
        condition=IfCondition(use_state_publisher),
        parameters=[
            {
                "cmd_topic": "/cmd_vel_safe",
                "safety_stop_topic": "/safety/stop",
                "slowdown_topic": "/safety/slowdown_factor",
                "watchdog_topic": "/savo_base/watchdog_state",
                "base_state_topic": "/savo_base/base_state",
                "motor_board_status_topic": "/savo_base/motor_board_status",
                "subscribe_cmd": True,
                "subscribe_safety_stop": True,
                "subscribe_slowdown": True,
                "subscribe_watchdog": True,
                "subscribe_base_state": True,
                "subscribe_motor_board_status": False,
                "summary_topic": "/savo_base/state_summary",
                "heartbeat_topic": "/savo_base/state_heartbeat",
                "publish_hz": 5.0,
                "heartbeat_hz": 1.0,
                "cmd_stale_s": 0.30,
                "safety_stale_s": 0.50,
                "watchdog_stale_s": 1.50,
                "base_state_stale_s": 1.50,
                "motor_board_status_stale_s": 1.50,
                "pretty_json": False,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Simple liveness heartbeat node
    base_heartbeat_node = Node(
        package="savo_base",
        executable="base_heartbeat_node",
        name="base_heartbeat_node",
        output=output,
        condition=IfCondition(use_heartbeat),
        parameters=[
            {
                "heartbeat_topic": "/savo_base/heartbeat",
                "heartbeat_state_topic": "/savo_base/heartbeat_state",
                "heartbeat_hz": 2.0,
                "state_publish_hz": 1.0,
                "pulse_mode": "toggle",
                "monitor_cmd": True,
                "monitor_watchdog_trip": True,
                "monitor_watchdog_state": True,
                "monitor_base_state": False,
                "cmd_topic": "/cmd_vel_safe",
                "watchdog_trip_topic": "/savo_base/watchdog_trip",
                "watchdog_state_topic": "/savo_base/watchdog_state",
                "base_state_topic": "/savo_base/base_state",
                "cmd_stale_s": 0.30,
                "watchdog_trip_stale_s": 1.00,
                "watchdog_state_stale_s": 1.50,
                "base_state_stale_s": 1.50,
                "pretty_json": False,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Diagnostics runner (optional; useful for remote-triggered checks)
    base_diag_runner_node = Node(
        package="savo_base",
        executable="base_diag_runner_node",
        name="base_diag_runner_node",
        output=output,
        condition=IfCondition(use_diag_runner),
        parameters=[
            {
                "run_request_topic": "/savo_base/diag/run_request",
                "cancel_request_topic": "/savo_base/diag/cancel_request",
                "state_topic": "/savo_base/diag/state",
                "event_topic": "/savo_base/diag/event",
                "busy_topic": "/savo_base/diag/busy",
                "state_publish_hz": 2.0,
                "default_timeout_s": 20.0,
                "max_timeout_s": 120.0,
                "terminate_grace_s": 2.0,
                "allow_parallel_runs": False,
                "shell_mode": False,
                "capture_output": True,
                "max_output_chars": 4000,
                "pretty_json": False,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # -------------------------------------------------------------------------
    # Launch description
    # -------------------------------------------------------------------------
    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument(
                "profile",
                default_value="dryrun_sim_motoroff",
                description=(
                    "Base profile name under config/profiles/ without .yaml "
                    "(e.g. dryrun_sim_motoroff | bench_test | real_robot_v1)"
                ),
            ),
            DeclareLaunchArgument(
                "profile_path",
                default_value="",
                description=(
                    "Optional absolute path to a profile YAML. If non-empty, "
                    "overrides the 'profile' selection."
                ),
            ),
            DeclareLaunchArgument(
                "use_watchdog",
                default_value="true",
                description="Start base_watchdog_node (command freshness supervisor).",
            ),
            DeclareLaunchArgument(
                "use_state_publisher",
                default_value="true",
                description="Start base_state_publisher_node (aggregated state summary).",
            ),
            DeclareLaunchArgument(
                "use_heartbeat",
                default_value="true",
                description="Start base_heartbeat_node (liveness pulse/state).",
            ),
            DeclareLaunchArgument(
                "use_diag_runner",
                default_value="false",
                description="Start base_diag_runner_node (diagnostic command runner).",
            ),
            DeclareLaunchArgument(
                "output",
                default_value="screen",
                description="Node output mode (screen|log).",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level (debug|info|warn|error|fatal).",
            ),
            # Nodes
            base_driver_node,
            base_watchdog_node,
            base_state_publisher_node,
            base_heartbeat_node,
            base_diag_runner_node,
        ]
    )