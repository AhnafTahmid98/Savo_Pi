#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — Dryrun Test Launch (savo_base)
-------------------------------------------
Professional one-command DRYRUN validation for the `savo_base` stack.

What this launch does
---------------------
1) Starts `base_bringup.launch.py` with profile `dryrun_sim_motoroff.yaml`
   (software-only, motor-off safety profile)
2) Optionally dumps effective runtime params from /base_driver_node
3) Optionally runs a smoke command (zero / pulse / cmd)
4) Optionally starts poke-test CLI (short motion pokes in dryrun)
5) Optionally starts interactive letters teleop (dryrun)

Design goals
------------
- Safe by default (no real hardware movement)
- Verifies launch wiring + topic contracts + watchdog behavior
- Works with your installed scripts from CMakeLists.txt
- Reuses `base_bringup.launch.py` for consistency

Examples
--------
# Default dryrun stack bringup only
ros2 launch savo_base base_dryrun_test.launch.py

# Also dump effective params after startup
ros2 launch savo_base base_dryrun_test.launch.py run_dump_params:=true

# Run a short watchdog pulse test after startup
ros2 launch svo_base base_dryrun_test.launch.py run_smoke:=true smoke_mode:=pulse

# Run poke test in dryrun
ros2 launch savo_base base_dryrun_test.launch.py run_poke:=true

# Start keyboard teleop (dryrun)
ros2 launch savo_base base_dryrun_test.launch.py run_teleop:=true
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


def _build_smoke_process(context, *args, **kwargs):
    smoke_mode = LaunchConfiguration("smoke_mode").perform(context)
    cmd_topic = LaunchConfiguration("smoke_cmd_topic").perform(context)
    stop_topic = LaunchConfiguration("smoke_stop_topic").perform(context)
    slowdown_topic = LaunchConfiguration("smoke_slowdown_topic").perform(context)

    cmd = [
        "ros2", "run", "savo_base", "base_smoke_test_cli.py",
    ]

    # Keep to documented subcommands from your script header
    if smoke_mode == "zero":
        cmd += ["zero"]
    elif smoke_mode == "pulse":
        cmd += ["pulse", "--vx", "0.10", "--on", "0.4", "--off", "1.0"]
    elif smoke_mode == "cmd":
        cmd += ["cmd", "--vx", "0.10", "--duration", "1.0"]
    else:
        # safe fallback
        cmd += ["zero"]

    # Common topic overrides
    cmd += [
        "--cmd-topic", str(cmd_topic),
        "--safety-stop-topic", str(stop_topic),
        "--slowdown-topic", str(slowdown_topic),
    ]

    return [
        LogInfo(msg=f"[savo_base] Running base_smoke_test_cli ({smoke_mode})"),
        ExecuteProcess(cmd=cmd, output="screen", shell=False, emulate_tty=True),
    ]


def _build_poke_process(context, *args, **kwargs):
    cmd_topic = LaunchConfiguration("poke_cmd_topic").perform(context)
    stop_topic = LaunchConfiguration("poke_stop_topic").perform(context)
    slowdown_topic = LaunchConfiguration("poke_slowdown_topic").perform(context)

    cmd = [
        "ros2", "run", "savo_base", "poke_test_cli.py",
        "--cmd-topic", str(cmd_topic),
        "--safety-stop-topic", str(stop_topic),
        "--slowdown-topic", str(slowdown_topic),
        "--vx", "0.08",
        "--vy", "0.08",
        "--wz", "0.15",
        "--pulse", "0.25",
    ]

    return [
        LogInfo(msg="[savo_base] Running poke_test_cli (dryrun-safe amplitudes)"),
        ExecuteProcess(cmd=cmd, output="screen", shell=False, emulate_tty=True),
    ]


def _build_teleop_process(context, *args, **kwargs):
    cmd_topic = LaunchConfiguration("teleop_cmd_topic").perform(context)
    stop_topic = LaunchConfiguration("teleop_stop_topic").perform(context)
    slowdown_topic = LaunchConfiguration("teleop_slowdown_topic").perform(context)

    cmd = [
        "ros2", "run", "savo_base", "teleop_letters_cli.py",
        "--cmd-topic", str(cmd_topic),
        "--safety-stop-topic", str(stop_topic),
        "--slowdown-topic", str(slowdown_topic),
        "--vx", "0.10",
        "--vy", "0.10",
        "--wz", "0.15",
    ]

    return [
        LogInfo(msg="[savo_base] Starting teleop_letters_cli (DRYRUN)"),
        ExecuteProcess(cmd=cmd, output="screen", shell=False, emulate_tty=True),
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("savo_base")
    base_bringup_launch = PathJoinSubstitution([pkg_share, "launch", "base_bringup.launch.py"])

    # Base bringup args passthrough
    use_watchdog = LaunchConfiguration("use_watchdog")
    use_state_publisher = LaunchConfiguration("use_state_publisher")
    use_heartbeat = LaunchConfiguration("use_heartbeat")
    use_diag_runner = LaunchConfiguration("use_diag_runner")
    log_level = LaunchConfiguration("log_level")
    output = LaunchConfiguration("output")

    # Optional actions
    run_dump_params = LaunchConfiguration("run_dump_params")
    run_smoke = LaunchConfiguration("run_smoke")
    run_poke = LaunchConfiguration("run_poke")
    run_teleop = LaunchConfiguration("run_teleop")

    # Startup delays (seconds)
    dump_delay_s = LaunchConfiguration("dump_delay_s")
    smoke_delay_s = LaunchConfiguration("smoke_delay_s")
    poke_delay_s = LaunchConfiguration("poke_delay_s")
    teleop_delay_s = LaunchConfiguration("teleop_delay_s")

    include_base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_bringup_launch),
        launch_arguments={
            # Force DRYRUN profile for this launch (IMPORTANT: full filename)
            "profile": "dryrun_sim_motoroff.yaml",
            "use_watchdog": use_watchdog,
            "use_state_publisher": use_state_publisher,
            "use_heartbeat": use_heartbeat,
            "use_diag_runner": use_diag_runner,
            "log_level": log_level,
            "output": output,
        }.items(),
    )

    return LaunchDescription([
        # ---------------------------------------------------------------------
        # Bringup controls (forwarded to base_bringup)
        # ---------------------------------------------------------------------
        DeclareLaunchArgument("use_watchdog", default_value="true"),
        DeclareLaunchArgument("use_state_publisher", default_value="true"),
        DeclareLaunchArgument("use_heartbeat", default_value="true"),
        DeclareLaunchArgument("use_diag_runner", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("output", default_value="screen"),

        # ---------------------------------------------------------------------
        # Optional helper actions
        # ---------------------------------------------------------------------
        DeclareLaunchArgument(
            "run_dump_params",
            default_value="true",
            description="Dump effective params from /base_driver_node after startup.",
        ),
        DeclareLaunchArgument(
            "run_smoke",
            default_value="false",
            description="Run base_smoke_test_cli after startup.",
        ),
        DeclareLaunchArgument(
            "run_poke",
            default_value="false",
            description="Run poke_test_cli after startup (dryrun-safe amplitudes).",
        ),
        DeclareLaunchArgument(
            "run_teleop",
            default_value="false",
            description="Start teleop_letters_cli after startup (interactive).",
        ),

        # Delays
        DeclareLaunchArgument("dump_delay_s", default_value="2.0"),
        DeclareLaunchArgument("smoke_delay_s", default_value="2.5"),
        DeclareLaunchArgument("poke_delay_s", default_value="3.0"),
        DeclareLaunchArgument("teleop_delay_s", default_value="3.0"),

        # ---------------------------------------------------------------------
        # Dump params options
        # ---------------------------------------------------------------------
        DeclareLaunchArgument("dump_node", default_value="/base_driver_node"),
        DeclareLaunchArgument("dump_format", default_value="yaml"),
        DeclareLaunchArgument("dump_ros2_yaml", default_value="true"),
        DeclareLaunchArgument("dump_out", default_value=""),

        # ---------------------------------------------------------------------
        # Smoke test options
        # ---------------------------------------------------------------------
        DeclareLaunchArgument(
            "smoke_mode",
            default_value="zero",
            description="Smoke mode: zero | pulse | cmd",
        ),
        DeclareLaunchArgument("smoke_cmd_topic", default_value="/cmd_vel_safe"),
        DeclareLaunchArgument("smoke_stop_topic", default_value="/safety/stop"),
        DeclareLaunchArgument("smoke_slowdown_topic", default_value="/safety/slowdown_factor"),

        # ---------------------------------------------------------------------
        # Poke CLI topic options
        # ---------------------------------------------------------------------
        DeclareLaunchArgument("poke_cmd_topic", default_value="/cmd_vel_safe"),
        DeclareLaunchArgument("poke_stop_topic", default_value="/safety/stop"),
        DeclareLaunchArgument("poke_slowdown_topic", default_value="/safety/slowdown_factor"),

        # ---------------------------------------------------------------------
        # Teleop CLI topic options
        # ---------------------------------------------------------------------
        DeclareLaunchArgument("teleop_cmd_topic", default_value="/cmd_vel_safe"),
        DeclareLaunchArgument("teleop_stop_topic", default_value="/safety/stop"),
        DeclareLaunchArgument("teleop_slowdown_topic", default_value="/safety/slowdown_factor"),

        # ---------------------------------------------------------------------
        # Main bringup
        # ---------------------------------------------------------------------
        LogInfo(msg="[savo_base] Starting DRYRUN test launch (profile=dryrun_sim_motoroff.yaml)"),
        include_base_bringup,

        # ---------------------------------------------------------------------
        # Optional post-start actions (delayed)
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
            period=smoke_delay_s,
            actions=[
                OpaqueFunction(
                    function=_build_smoke_process,
                    condition=IfCondition(run_smoke),
                )
            ],
        ),
        TimerAction(
            period=poke_delay_s,
            actions=[
                OpaqueFunction(
                    function=_build_poke_process,
                    condition=IfCondition(run_poke),
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

        LogInfo(
            msg=(
                "[savo_base] DRYRUN safety note: this launch expects "
                "config/profiles/dryrun_sim_motoroff.yaml to be motor-off (max_duty=0)."
            )
        ),
    ])