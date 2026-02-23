#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — Base Teleop Test Launch (savo_base)
------------------------------------------------
Professional teleop test bringup for Robot Savo `savo_base`.

Purpose
-------
Starts the base stack (driver + monitoring helper nodes) and then launches
`teleop_letters_cli.py` for manual keyboard testing.

This launch is designed for:
- dryrun teleop validation (safe software-only)
- bench teleop tests (robot lifted)
- controlled real-robot manual tests

Safety design
-------------
- Defaults to DRYRUN motor-off profile (`dryrun_sim_motoroff`)
- Teleop amplitudes default to conservative values
- Optional safety-stop and slowdown publishers can be enabled in teleop
- Teleop starts after a short delay so base stack initializes first

Examples
--------
# Safe default (dryrun motor-off)
ros2 launch savo_base base_teleop_test.launch.py

# Bench teleop (robot lifted)
ros2 launch savo_base base_teleop_test.launch.py profile:=bench_test

# Real robot teleop (clear area, operator ready)
ros2 launch savo_base base_teleop_test.launch.py \
  profile:=real_robot_v1 teleop_enable_safety_stop_pub:=true teleop_enable_slowdown_pub:=true

# Start teleop in pulse mode
ros2 launch savo_base base_teleop_test.launch.py teleop_pulse_mode:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _as_bool(text: str) -> bool:
    return str(text).strip().lower() in ("1", "true", "yes", "on")


def _build_teleop_process(context, *args, **kwargs):
    """Build ExecuteProcess for teleop_letters_cli.py using actual CLI flags from script."""
    cmd_topic = LaunchConfiguration("teleop_cmd_topic").perform(context)
    safety_stop_topic = LaunchConfiguration("teleop_safety_stop_topic").perform(context)
    slowdown_topic = LaunchConfiguration("teleop_slowdown_topic").perform(context)

    vx = LaunchConfiguration("teleop_vx").perform(context)
    vy = LaunchConfiguration("teleop_vy").perform(context)
    wz = LaunchConfiguration("teleop_wz").perform(context)
    publish_hz = LaunchConfiguration("teleop_publish_hz").perform(context)

    pulse_mode = _as_bool(LaunchConfiguration("teleop_pulse_mode").perform(context))
    enable_safety_stop_pub = _as_bool(LaunchConfiguration("teleop_enable_safety_stop_pub").perform(context))
    enable_slowdown_pub = _as_bool(LaunchConfiguration("teleop_enable_slowdown_pub").perform(context))

    speed_scale_step = LaunchConfiguration("teleop_speed_scale_step").perform(context)
    slowdown_step = LaunchConfiguration("teleop_slowdown_step").perform(context)
    slowdown = LaunchConfiguration("teleop_slowdown").perform(context)

    cmd = [
        "ros2", "run", "savo_base", "teleop_letters_cli.py",
        "--cmd-topic", str(cmd_topic),
        "--safety-stop-topic", str(safety_stop_topic),
        "--slowdown-topic", str(slowdown_topic),
        "--vx", str(vx),
        "--vy", str(vy),
        "--wz", str(wz),
        "--publish-hz", str(publish_hz),
        "--speed-scale-step", str(speed_scale_step),
        "--slowdown-step", str(slowdown_step),
        "--slowdown", str(slowdown),
    ]

    if pulse_mode:
        cmd.append("--pulse-mode")
    if enable_safety_stop_pub:
        cmd.append("--enable-safety-stop-pub")
    if enable_slowdown_pub:
        cmd.append("--enable-slowdown-pub")

    return [
        LogInfo(msg="[savo_base] Starting teleop_letters_cli (manual keyboard control)"),
        LogInfo(msg=(
            "[savo_base] Teleop keys: w/s/a/d (move), q/e (rotate), x (stop), . (quit), h (help)."
        )),
        ExecuteProcess(
            cmd=cmd,
            output="screen",
            shell=False,
            emulate_tty=True,
        ),
    ]


def _build_dump_params_process(context, *args, **kwargs):
    node_name = LaunchConfiguration("dump_node").perform(context)
    out_path = LaunchConfiguration("dump_out").perform(context)
    dump_format = LaunchConfiguration("dump_format").perform(context)
    ros2_yaml = _as_bool(LaunchConfiguration("dump_ros2_yaml").perform(context))

    cmd = [
        "ros2", "run", "savo_base", "dump_effective_params.py",
        "--node", str(node_name),
        "--format", str(dump_format),
    ]
    if ros2_yaml:
        cmd.append("--ros2-yaml")
    if str(out_path).strip():
        cmd += ["--out", str(out_path)]

    return [
        LogInfo(msg=f"[savo_base] Dumping effective params from {node_name}"),
        ExecuteProcess(cmd=cmd, output="screen", shell=False, emulate_tty=True),
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("savo_base")

    # Reuse base_safe_idle (which reuses base_bringup) so teleop launch stays clean
    base_safe_idle_launch = PathJoinSubstitution([pkg_share, "launch", "base_safe_idle.launch.py"])

    # -------------------------------------------------------------------------
    # Base stack args
    # -------------------------------------------------------------------------
    profile = LaunchConfiguration("profile")
    profile_path = LaunchConfiguration("profile_path")

    use_watchdog = LaunchConfiguration("use_watchdog")
    use_state_publisher = LaunchConfiguration("use_state_publisher")
    use_heartbeat = LaunchConfiguration("use_heartbeat")
    use_diag_runner = LaunchConfiguration("use_diag_runner")

    output = LaunchConfiguration("output")
    log_level = LaunchConfiguration("log_level")

    # Optional helper actions
    run_dump_params = LaunchConfiguration("run_dump_params")
    run_teleop = LaunchConfiguration("run_teleop")
    dump_delay_s = LaunchConfiguration("dump_delay_s")
    teleop_delay_s = LaunchConfiguration("teleop_delay_s")

    include_base_safe_idle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_safe_idle_launch),
        launch_arguments={
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
        # Base stack profile / helper nodes
        # ---------------------------------------------------------------------
        DeclareLaunchArgument(
            "profile",
            default_value="dryrun_sim_motoroff",
            description=(
                "Profile name under config/profiles/ without .yaml "
                "(dryrun_sim_motoroff | bench_test | real_robot_v1). "
                "Default is dryrun for safest teleop testing."
            ),
        ),
        DeclareLaunchArgument(
            "profile_path",
            default_value="",
            description="Optional absolute profile YAML path (overrides profile if non-empty).",
        ),

        DeclareLaunchArgument("use_watchdog", default_value="true"),
        DeclareLaunchArgument("use_state_publisher", default_value="true"),
        DeclareLaunchArgument("use_heartbeat", default_value="true"),
        DeclareLaunchArgument("use_diag_runner", default_value="false"),

        DeclareLaunchArgument("output", default_value="screen"),
        DeclareLaunchArgument("log_level", default_value="info"),

        # ---------------------------------------------------------------------
        # Teleop controls
        # ---------------------------------------------------------------------
        DeclareLaunchArgument(
            "run_teleop",
            default_value="true",
            description="Start teleop_letters_cli after base stack startup.",
        ),
        DeclareLaunchArgument(
            "teleop_delay_s",
            default_value="2.0",
            description="Delay before starting teleop CLI (seconds).",
        ),

        # Teleop topics (match locked stack defaults)
        DeclareLaunchArgument("teleop_cmd_topic", default_value="/cmd_vel_safe"),
        DeclareLaunchArgument("teleop_safety_stop_topic", default_value="/safety/stop"),
        DeclareLaunchArgument("teleop_slowdown_topic", default_value="/safety/slowdown_factor"),

        # Teleop amplitudes / cadence (conservative defaults)
        DeclareLaunchArgument("teleop_vx", default_value="0.12"),
        DeclareLaunchArgument("teleop_vy", default_value="0.12"),
        DeclareLaunchArgument("teleop_wz", default_value="0.20"),
        DeclareLaunchArgument("teleop_publish_hz", default_value="20.0"),

        # Teleop mode / helper publisher options
        DeclareLaunchArgument("teleop_pulse_mode", default_value="false"),
        DeclareLaunchArgument("teleop_enable_safety_stop_pub", default_value="false"),
        DeclareLaunchArgument("teleop_enable_slowdown_pub", default_value="false"),
        DeclareLaunchArgument("teleop_speed_scale_step", default_value="0.10"),
        DeclareLaunchArgument("teleop_slowdown_step", default_value="0.10"),
        DeclareLaunchArgument("teleop_slowdown", default_value="1.0"),

        # ---------------------------------------------------------------------
        # Optional params dump (useful during bringup)
        # ---------------------------------------------------------------------
        DeclareLaunchArgument("run_dump_params", default_value="false"),
        DeclareLaunchArgument("dump_delay_s", default_value="1.5"),
        DeclareLaunchArgument("dump_node", default_value="/base_driver_node"),
        DeclareLaunchArgument("dump_format", default_value="yaml"),
        DeclareLaunchArgument("dump_ros2_yaml", default_value="true"),
        DeclareLaunchArgument("dump_out", default_value=""),

        # ---------------------------------------------------------------------
        # Informational logs
        # ---------------------------------------------------------------------
        LogInfo(msg="[savo_base] Starting base_teleop_test.launch.py"),
        LogInfo(msg=(
            "[savo_base] Professional teleop workflow: base safe-idle bringup first, "
            "then delayed teleop_letters_cli start."
        )),
        LogInfo(msg=(
            "[savo_base] SAFETY: For bench/real tests, clear the area and keep stop/power cut ready. "
            "Use profile=dryrun_sim_motoroff for software-only validation."
        )),

        # ---------------------------------------------------------------------
        # Base stack bringup (safe idle style)
        # ---------------------------------------------------------------------
        include_base_safe_idle,

        # ---------------------------------------------------------------------
        # Optional post-start actions
        # ---------------------------------------------------------------------
        TimerAction(
            period=dump_delay_s,
            actions=[
                OpaqueFunction(
                    function=_build_dump_params_process,
                    condition=IfCondition(run_dump_params),
                )
            ],
        ),

        TimerAction(
            period=teleop_delay_s,
            actions=[
                OpaqueFunction(
                    function=_build_teleop_process,
                    condition=IfCondition(run_teleop),
                )
            ],
        ),
    ])