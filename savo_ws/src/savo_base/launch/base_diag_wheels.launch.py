#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — Wheel & Safety Diagnostics Launch (savo_base)
----------------------------------------------------------
Professional diagnostics launcher for wheel-direction validation and
emergency-stop behavior checks.

Runs package-local diagnostics (CLI tools) in a controlled workflow:

1) wheel_direction_check.py (optional)
   - one-wheel-at-a-time direction verification (FL/RL/FR/RR)
   - direct PCA9685 access (non-ROS hardware diagnostic)

2) emergency_stop_check.py (optional)
   - observes /safety/stop and verifies /cmd_vel_safe zeroing
   - can optionally publish a test stop pulse

Safe defaults
-------------
- Wheel direction check: OFF (because it can move wheels)
- Emergency stop check: ON (passive observe mode)
- Wheel diagnostic dry-run: ON if wheel check is enabled

Examples
--------
# Passive safety check only (default)
ros2 launch savo_base base_diag_wheels.launch.py

# Wheel direction dry-run (no hardware writes)
ros2 launch savo_base base_diag_wheels.launch.py run_wheel_check:=true wheel_dry_run:=true

# Real wheel direction test (ROBOT LIFTED!)
ros2 launch savo_base base_diag_wheels.launch.py \
  run_wheel_check:=true wheel_dry_run:=false wheel_yes:=false wheel_verbose:=true

# Emergency-stop active pulse test
ros2 launch savo_base base_diag_wheels.launch.py estop_pulse_stop:=true estop_pulse_duration:=1.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _as_bool(text: str) -> bool:
    return str(text).strip().lower() in ("1", "true", "yes", "on")


def _build_wheel_direction_process(context, *args, **kwargs):
    """Build ExecuteProcess for wheel_direction_check.py using actual CLI flags."""
    i2c_bus = LaunchConfiguration("wheel_i2c_bus").perform(context)
    addr = LaunchConfiguration("wheel_addr").perform(context)
    pwm_freq = LaunchConfiguration("wheel_pwm_freq").perform(context)
    quench_ms = LaunchConfiguration("wheel_quench_ms").perform(context)

    duty = LaunchConfiguration("wheel_duty").perform(context)
    step_time = LaunchConfiguration("wheel_step_time").perform(context)
    pause_time = LaunchConfiguration("wheel_pause_time").perform(context)
    wheels = LaunchConfiguration("wheel_wheels").perform(context)

    interactive_confirm = _as_bool(LaunchConfiguration("wheel_interactive_confirm").perform(context))
    dry_run = _as_bool(LaunchConfiguration("wheel_dry_run").perform(context))
    verbose = _as_bool(LaunchConfiguration("wheel_verbose").perform(context))
    yes = _as_bool(LaunchConfiguration("wheel_yes").perform(context))

    invert_fl = _as_bool(LaunchConfiguration("wheel_invert_fl").perform(context))
    invert_rl = _as_bool(LaunchConfiguration("wheel_invert_rl").perform(context))
    invert_fr = _as_bool(LaunchConfiguration("wheel_invert_fr").perform(context))
    invert_rr = _as_bool(LaunchConfiguration("wheel_invert_rr").perform(context))

    cmd = [
        "python3",
        "-m",
        "savo_base.diagnostics.wheel_direction_check",
        "--i2c-bus", str(i2c_bus),
        "--addr", str(addr),
        "--pwm-freq", str(pwm_freq),
        "--quench-ms", str(quench_ms),
        "--duty", str(duty),
        "--step-time", str(step_time),
        "--pause-time", str(pause_time),
        "--wheels", str(wheels),
    ]

    if interactive_confirm:
        cmd.append("--interactive-confirm")
    if invert_fl:
        cmd.append("--invert-fl")
    if invert_rl:
        cmd.append("--invert-rl")
    if invert_fr:
        cmd.append("--invert-fr")
    if invert_rr:
        cmd.append("--invert-rr")
    if dry_run:
        cmd.append("--dry-run")
    if verbose:
        cmd.append("--verbose")
    if yes:
        cmd.append("--yes")

    mode = "DRYRUN" if dry_run else "REAL-HW"
    return [
        LogInfo(
            msg=(
                f"[savo_base] Running wheel_direction_check ({mode}) "
                f"wheels={wheels} duty={duty}"
            )
        ),
        ExecuteProcess(
            cmd=cmd,
            output="screen",
            shell=False,
            emulate_tty=True,
        ),
    ]


def _build_estop_check_process(context, *args, **kwargs):
    """Build ExecuteProcess for emergency_stop_check.py using actual CLI flags."""
    stop_topic = LaunchConfiguration("estop_stop_topic").perform(context)
    cmd_topic = LaunchConfiguration("estop_cmd_topic").perform(context)

    observe_timeout = LaunchConfiguration("estop_observe_timeout").perform(context)
    wait_stop_event_timeout = LaunchConfiguration("estop_wait_stop_event_timeout").perform(context)
    reaction_timeout = LaunchConfiguration("estop_reaction_timeout").perform(context)
    spin_hz = LaunchConfiguration("estop_spin_hz").perform(context)

    epsilon_lin = LaunchConfiguration("estop_epsilon_lin").perform(context)
    epsilon_ang = LaunchConfiguration("estop_epsilon_ang").perform(context)

    pulse_stop = _as_bool(LaunchConfiguration("estop_pulse_stop").perform(context))
    pulse_duration = LaunchConfiguration("estop_pulse_duration").perform(context)

    cmd = [
        "python3",
        "-m",
        "savo_base.diagnostics.emergency_stop_check",
        "--stop-topic", str(stop_topic),
        "--cmd-topic", str(cmd_topic),
        "--observe-timeout", str(observe_timeout),
        "--wait-stop-event-timeout", str(wait_stop_event_timeout),
        "--reaction-timeout", str(reaction_timeout),
        "--spin-hz", str(spin_hz),
        "--epsilon-lin", str(epsilon_lin),
        "--epsilon-ang", str(epsilon_ang),
        "--pulse-duration", str(pulse_duration),
    ]

    if pulse_stop:
        cmd.append("--pulse-stop")

    mode = "ACTIVE-PULSE" if pulse_stop else "PASSIVE-OBSERVE"
    return [
        LogInfo(msg=f"[savo_base] Running emergency_stop_check ({mode})"),
        ExecuteProcess(
            cmd=cmd,
            output="screen",
            shell=False,
            emulate_tty=True,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    run_wheel_check = LaunchConfiguration("run_wheel_check")
    run_estop_check = LaunchConfiguration("run_estop_check")

    return LaunchDescription([
        # ---------------------------------------------------------------------
        # Wheel direction diagnostic (non-ROS hardware check)
        # ---------------------------------------------------------------------
        DeclareLaunchArgument(
            "run_wheel_check",
            default_value="false",
            description="Run wheel_direction_check diagnostic (can move wheels).",
        ),
        DeclareLaunchArgument("wheel_i2c_bus", default_value="1", description="PCA9685 I2C bus (default: 1)."),
        DeclareLaunchArgument("wheel_addr", default_value="0x40", description="PCA9685 address (default: 0x40)."),
        DeclareLaunchArgument("wheel_pwm_freq", default_value="50.0", description="PCA9685 PWM frequency Hz."),
        DeclareLaunchArgument("wheel_quench_ms", default_value="18", description="Direction-flip quench delay ms."),

        DeclareLaunchArgument("wheel_duty", default_value="1500", description="Signed duty magnitude (0..4095)."),
        DeclareLaunchArgument("wheel_step_time", default_value="0.8", description="Duration of each spin step (s)."),
        DeclareLaunchArgument("wheel_pause_time", default_value="0.4", description="Pause after each step (s)."),
        DeclareLaunchArgument("wheel_wheels", default_value="FL,RL,FR,RR", description="Comma list of wheels to test."),

        DeclareLaunchArgument(
            "wheel_interactive_confirm",
            default_value="false",
            description="Ask manual y/n confirmation after each wheel.",
        ),
        DeclareLaunchArgument("wheel_invert_fl", default_value="false", description="Temporary invert FL in diagnostic."),
        DeclareLaunchArgument("wheel_invert_rl", default_value="false", description="Temporary invert RL in diagnostic."),
        DeclareLaunchArgument("wheel_invert_fr", default_value="false", description="Temporary invert FR in diagnostic."),
        DeclareLaunchArgument("wheel_invert_rr", default_value="false", description="Temporary invert RR in diagnostic."),

        DeclareLaunchArgument(
            "wheel_dry_run",
            default_value="true",
            description="Dry-run mode for wheel diagnostic (safe default).",
        ),
        DeclareLaunchArgument("wheel_verbose", default_value="true", description="Verbose output for wheel diagnostic."),
        DeclareLaunchArgument(
            "wheel_yes",
            default_value="false",
            description="Skip wheel diagnostic safety confirmation prompt.",
        ),

        # ---------------------------------------------------------------------
        # Emergency stop diagnostic (ROS topic behavior check)
        # ---------------------------------------------------------------------
        DeclareLaunchArgument(
            "run_estop_check",
            default_value="true",
            description="Run emergency_stop_check diagnostic.",
        ),
        DeclareLaunchArgument("estop_stop_topic", default_value="/safety/stop", description="Emergency stop Bool topic."),
        DeclareLaunchArgument("estop_cmd_topic", default_value="/cmd_vel_safe", description="Twist topic to verify zeroing."),

        DeclareLaunchArgument("estop_observe_timeout", default_value="3.0", description="Initial topic presence timeout (s)."),
        DeclareLaunchArgument("estop_wait_stop_event_timeout", default_value="8.0", description="Wait for STOP=TRUE timeout (s)."),
        DeclareLaunchArgument("estop_reaction_timeout", default_value="1.0", description="Allowed cmd zeroing reaction time (s)."),
        DeclareLaunchArgument("estop_spin_hz", default_value="50.0", description="Internal polling/spin helper rate (Hz)."),

        DeclareLaunchArgument("estop_epsilon_lin", default_value="0.01", description="Linear zero threshold (m/s)."),
        DeclareLaunchArgument("estop_epsilon_ang", default_value="0.02", description="Angular zero threshold (rad/s)."),

        DeclareLaunchArgument(
            "estop_pulse_stop",
            default_value="false",
            description="Publish a test STOP pulse (active test).",
        ),
        DeclareLaunchArgument(
            "estop_pulse_duration",
            default_value="1.0",
            description="Duration of test STOP=TRUE pulse (s).",
        ),

        # ---------------------------------------------------------------------
        # Execution
        # ---------------------------------------------------------------------
        LogInfo(
            msg=(
                "[savo_base] Wheel/Safety diagnostics launch starting. "
                "Defaults: wheel check OFF, estop check ON (passive)."
            )
        ),

        OpaqueFunction(
            function=_build_wheel_direction_process,
            condition=IfCondition(run_wheel_check),
        ),

        OpaqueFunction(
            function=_build_estop_check_process,
            condition=IfCondition(run_estop_check),
        ),

        LogInfo(
            msg=(
                "[savo_base] SAFETY: If enabling real wheel diagnostic, lift robot wheels "
                "off ground and keep power cut / emergency stop ready."
            )
        ),
    ])