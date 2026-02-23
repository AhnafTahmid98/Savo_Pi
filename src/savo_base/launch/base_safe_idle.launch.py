#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — Base Safe Idle Launch (savo_base)
----------------------------------------------
Professional "safe idle" bringup for the Robot Savo base stack.

Purpose
-------
Start the base stack in a non-commanding state so you can safely verify:
- parameter layering
- hardware backend initialization (or dryrun)
- watchdog/state/heartbeat helper nodes
- /safety/stop and /safety/slowdown_factor subscriptions
- base state publishing

This launch does NOT publish motion commands. It is intended for:
- first power-on checks
- pre-drive verification
- bench or real robot idle monitoring

Profiles
--------
Loads layered configs and applies a profile last:
- dryrun_sim_motoroff.yaml
- bench_test.yaml
- real_robot_v1.yaml

Recommended usage
-----------------
# Safest possible idle (software-only motor-off)
ros2 launch savo_base base_safe_idle.launch.py profile:=dryrun_sim_motoroff.yaml

# Bench idle (robot lifted)
ros2 launch savo_base base_safe_idle.launch.py profile:=bench_test.yaml

# Real robot idle monitoring (motors initialized, no commands sent)
ros2 launch savo_base base_safe_idle.launch.py profile:=real_robot_v1.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("savo_base")

    # -------------------------------------------------------------------------
    # Launch args
    # -------------------------------------------------------------------------
    profile = LaunchConfiguration("profile")
    output = LaunchConfiguration("output")
    log_level = LaunchConfiguration("log_level")

    # Helper nodes (safe idle usually wants visibility ON)
    use_watchdog = LaunchConfiguration("use_watchdog")
    use_state_publisher = LaunchConfiguration("use_state_publisher")
    use_heartbeat = LaunchConfiguration("use_heartbeat")
    use_diag_runner = LaunchConfiguration("use_diag_runner")

    # Optional custom absolute profile path
    profile_path = LaunchConfiguration("profile_path")

    # Reuse the professional base bringup (single source of truth)
    base_bringup_launch = PathJoinSubstitution([pkg_share, "launch", "base_bringup.launch.py"])

    include_base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_bringup_launch),
        launch_arguments={
            # If profile_path is non-empty, base_bringup will use it (as designed)
            "profile": profile,
            "profile_path": profile_path,
            "use_watchdog": use_watchdog,
            "use_state_publisher": use_state_publisher,
            "use_heartbeat": use_heartbeat,
            "use_diag_runner": use_diag_runner,
            "output": output,
            "log_level": log_level,
        }.items(),
    )

    return LaunchDescription([
        # ---------------------------------------------------------------------
        # Profile selection
        # ---------------------------------------------------------------------
        DeclareLaunchArgument(
            "profile",
            default_value="bench_test",
            description=(
                "Profile name under config/profiles/ without .yaml "
                "(dryrun_sim_motoroff | bench_test | real_robot_v1). "
                "Default is bench_test for a conservative idle bringup."
            ),
        ),
        DeclareLaunchArgument(
            "profile_path",
            default_value="",
            description=(
                "Optional absolute profile YAML path. If non-empty, overrides 'profile'."
            ),
        ),

        # ---------------------------------------------------------------------
        # Helper nodes (safe idle visibility defaults)
        # ---------------------------------------------------------------------
        DeclareLaunchArgument(
            "use_watchdog",
            default_value="true",
            description="Start base_watchdog_node (recommended for safe idle).",
        ),
        DeclareLaunchArgument(
            "use_state_publisher",
            default_value="true",
            description="Start base_state_publisher_node (recommended for monitoring).",
        ),
        DeclareLaunchArgument(
            "use_heartbeat",
            default_value="true",
            description="Start base_heartbeat_node (recommended for liveness checks).",
        ),
        DeclareLaunchArgument(
            "use_diag_runner",
            default_value="false",
            description="Start base_diag_runner_node (optional).",
        ),

        # ---------------------------------------------------------------------
        # Logging/output
        # ---------------------------------------------------------------------
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

        # ---------------------------------------------------------------------
        # Info logs
        # ---------------------------------------------------------------------
        LogInfo(msg="[savo_base] Starting base_safe_idle.launch.py"),
        LogInfo(msg=[
            "[savo_base] Safe-idle mode: no motion test CLI is started. ",
            "This launch only brings up the base stack and monitoring nodes."
        ]),
        LogInfo(msg=[
            "[savo_base] Selected profile (or profile_path override): ", profile
        ]),
        LogInfo(msg=(
            "[savo_base] For real robot first power-on, keep robot lifted / clear area "
            "and verify /safety/stop, /safety/slowdown_factor, and /savo_base/base_state."
        )),

        # ---------------------------------------------------------------------
        # Reused base stack bringup
        # ---------------------------------------------------------------------
        include_base_bringup,
    ])