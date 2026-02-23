#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — Base Bringup Launch (savo_base)
--------------------------------------------
Professional one-command bringup for Robot Savo base stack.

Starts:
- base_driver_node.py            (hardware execution / dryrun capable)
- base_watchdog_node.py          (command freshness watchdog, optional)
- base_state_publisher_node.py   (state summary aggregator, optional)
- base_heartbeat_node.py         (liveness pulse/state, optional)
- base_diag_runner_node.py       (diagnostic command runner, optional)

Design goals
------------
- Clean layered YAML loading (shared configs + profile override)
- Easy profile switching using YAML filenames:
    * dryrun_sim_motoroff.yaml
    * bench_test.yaml
    * real_robot_v1.yaml
- Optional supervisor/telemetry nodes for bringup and debugging
- Consistent topic wiring with locked Robot Savo perception/control pipeline

Recommended usage
-----------------
# Dry software-only test (no motor output)
ros2 launch savo_base base_bringup.launch.py profile:=dryrun_sim_motoroff.yaml

# Bench test (robot lifted / controlled test)
ros2 launch savo_base base_bringup.launch.py profile:=bench_test.yaml

# Real robot driving
ros2 launch savo_base base_bringup.launch.py profile:=real_robot_v1.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    diagnostics_yaml = PathJoinSubstitution([pkg_share, "config", "diagnostics.yaml"])

    # Profile handling (professional + robust):
    # - profile is a YAML filename (e.g. "dryrun_sim_motoroff.yaml")
    # - profile_path can override it if user provides a custom absolute path
    selected_profile_yaml = PathJoinSubstitution([pkg_share, "config", "profiles", profile])

    # -------------------------------------------------------------------------
    # Common parameter layering
    # NOTE:
    # - base_driver gets layered YAML files
    # - profile override should be last
    # -------------------------------------------------------------------------
    base_driver_params_default = [
        topics_yaml,
        safety_yaml,
        board_yaml,
        mecanum_yaml,
        diagnostics_yaml,
        selected_profile_yaml,
    ]

    base_driver_params_custom_profile = [
        topics_yaml,
        safety_yaml,
        board_yaml,
        mecanum_yaml,
        diagnostics_yaml,
        profile_path,  # custom absolute path override
    ]

    # -------------------------------------------------------------------------
    # Nodes
    # IMPORTANT:
    # In your current hybrid package setup, Python executables are installed
    # as files in lib/savo_base and must be referenced with ".py".
    # -------------------------------------------------------------------------
    base_driver_node_default = Node(
        package="savo_base",
        executable="base_driver_node.py",
        name="base_driver_node",
        output=output,
        condition=IfCondition(
            # run this node when profile_path == ""
            LaunchConfiguration("use_profile_name")
        ),
        parameters=base_driver_params_default,
        arguments=["--ros-args", "--log-level", log_level],
    )

    base_driver_node_custom = Node(
        package="savo_base",
        executable="base_driver_node.py",
        name="base_driver_node",
        output=output,
        condition=IfCondition(
            # run this node when profile_path != ""
            LaunchConfiguration("use_profile_path")
        ),
        parameters=base_driver_params_custom_profile,
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Watchdog supervisor (does not directly drive motors)
    base_watchdog_node = Node(
        package="savo_base",
        executable="base_watchdog_node.py",
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
        executable="base_state_publisher_node.py",
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
        executable="base_heartbeat_node.py",
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
        executable="base_diag_runner_node.py",
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
                default_value="dryrun_sim_motoroff.yaml",
                description=(
                    "Base profile YAML filename under config/profiles/ "
                    "(e.g. dryrun_sim_motoroff.yaml | bench_test.yaml | real_robot_v1.yaml)"
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

            # Internal booleans for robust profile_path switching without PythonExpression
            DeclareLaunchArgument(
                "use_profile_name",
                default_value="true",
                description="Internal: use selected profile under config/profiles/",
            ),
            DeclareLaunchArgument(
                "use_profile_path",
                default_value="false",
                description="Internal: use custom profile_path override",
            ),

            DeclareLaunchArgument(
                "use_watchdog",
                default_value="true",
                description="Start base_watchdog_node.py (command freshness supervisor).",
            ),
            DeclareLaunchArgument(
                "use_state_publisher",
                default_value="true",
                description="Start base_state_publisher_node.py (aggregated state summary).",
            ),
            DeclareLaunchArgument(
                "use_heartbeat",
                default_value="true",
                description="Start base_heartbeat_node.py (liveness pulse/state).",
            ),
            DeclareLaunchArgument(
                "use_diag_runner",
                default_value="false",
                description="Start base_diag_runner_node.py (diagnostic command runner).",
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

            # Informational logs
            LogInfo(msg="[savo_base] Starting base_bringup.launch.py"),
            LogInfo(msg=["[savo_base] profile: ", profile]),
            LogInfo(msg=["[savo_base] profile_path: ", profile_path]),
            LogInfo(msg=["[savo_base] use_watchdog: ", use_watchdog]),
            LogInfo(msg=["[savo_base] use_state_publisher: ", use_state_publisher]),
            LogInfo(msg=["[savo_base] use_heartbeat: ", use_heartbeat]),
            LogInfo(msg=["[savo_base] use_diag_runner: ", use_diag_runner]),

            # Nodes
            # Default profile-based driver node
            base_driver_node_default,

            # Optional custom-profile-path driver node (enable manually with:
            #   use_profile_name:=false use_profile_path:=true profile_path:=/abs/path/file.yaml)
            base_driver_node_custom,

            base_watchdog_node,
            base_state_publisher_node,
            base_heartbeat_node,
            base_diag_runner_node,
        ]
    )