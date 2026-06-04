#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Full base stack bringup with profile selection.

  ros2 launch savo_base base_bringup.launch.py profile:=dryrun_sim_motoroff.yaml
  ros2 launch savo_base base_bringup.launch.py profile:=bench_test.yaml
  ros2 launch savo_base base_bringup.launch.py profile:=real_robot_v1.yaml
  ros2 launch savo_base base_bringup.launch.py profile_path:=/path/to/custom.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


# profile_path vs profile resolved at launch time — OpaqueFunction avoids boolean flag gymnastics
def _build_base_driver_node(context, *args, **kwargs):
    profile = LaunchConfiguration("profile").perform(context)
    profile_path = LaunchConfiguration("profile_path").perform(context)
    output = LaunchConfiguration("output").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)

    pkg_share = get_package_share_directory("savo_base")

    if profile_path.strip():
        selected_profile = profile_path.strip()
        profile_source = "custom profile_path"
    else:
        selected_profile = os.path.join(pkg_share, "config", "profiles", profile)
        profile_source = f"package profile ({profile})"

    params = [
        os.path.join(pkg_share, "config", "topics.yaml"),
        os.path.join(pkg_share, "config", "safety_timeouts.yaml"),
        os.path.join(pkg_share, "config", "motor_board_freenove.yaml"),
        os.path.join(pkg_share, "config", "mecanum_kinematics.yaml"),
        os.path.join(pkg_share, "config", "diagnostics.yaml"),
        selected_profile,
    ]

    return [
        LogInfo(msg=f"[savo_base] base_driver_node using {profile_source}: {selected_profile}"),
        Node(
            package="savo_base",
            executable="base_driver_node.py",
            name="base_driver_node",
            output=output,
            parameters=params,
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    use_watchdog = LaunchConfiguration("use_watchdog")
    use_state_publisher = LaunchConfiguration("use_state_publisher")
    use_heartbeat = LaunchConfiguration("use_heartbeat")
    use_diag_runner = LaunchConfiguration("use_diag_runner")
    output = LaunchConfiguration("output")
    log_level = LaunchConfiguration("log_level")

    # helper nodes
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

    return LaunchDescription([
        # profile arguments
        DeclareLaunchArgument(
            "profile",
            default_value="dryrun_sim_motoroff.yaml",
            description=(
                "Profile YAML filename under config/profiles/ "
                "(e.g. dryrun_sim_motoroff.yaml | bench_test.yaml | real_robot_v1.yaml). "
                "Used only when profile_path is empty."
            ),
        ),
        DeclareLaunchArgument(
            "profile_path",
            default_value="",
            description=(
                "Optional absolute path to a profile YAML file. "
                "If non-empty, overrides profile."
            ),
        ),

        # helper node toggles
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

        # output / logging
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

        # informational logs
        LogInfo(msg="[savo_base] Starting base_bringup.launch.py"),
        LogInfo(msg=["[savo_base] use_watchdog: ", use_watchdog]),
        LogInfo(msg=["[savo_base] use_state_publisher: ", use_state_publisher]),
        LogInfo(msg=["[savo_base] use_heartbeat: ", use_heartbeat]),
        LogInfo(msg=["[savo_base] use_diag_runner: ", use_diag_runner]),

        # base_driver_node (profile resolved at launch time)
        OpaqueFunction(function=_build_base_driver_node),

        # helper nodes
        base_watchdog_node,
        base_state_publisher_node,
        base_heartbeat_node,
        base_diag_runner_node,
    ])
