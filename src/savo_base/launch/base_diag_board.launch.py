#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — Base Board Diagnostics Launch (savo_base)
------------------------------------------------------
Professional board-level diagnostics launcher for Robot Savo `savo_base`.

Purpose
-------
Runs package-local diagnostics for base-board bringup in a controlled way:

1) I2C board/device visibility check
   - validates expected devices on I2C bus(es)
   - confirms PCA9685 / ADS7830 / UPS HAT visibility (and optional others)

2) PWM sweep check (optional)
   - validates motor board write path through `savo_base.drivers`
   - supports DRYRUN or real hardware
   - supports conservative, lift-the-robot bench testing

Design notes
------------
- Uses `python3 -m savo_base.diagnostics.<tool>` so it works with the installed
  Python package layout (`ament_python_install_package(savo_base)`).
- Keeps diagnostics outside the main base driver bringup path.
- Safe defaults:
  * I2C check enabled
  * PWM sweep disabled by default
  * PWM dryrun enabled by default if you turn it on

Examples
--------
# I2C check only (default)
ros2 launch savo_base base_diag_board.launch.py

# I2C strict check on bus 1 + also check bus 0
ros2 launch savo_base base_diag_board.launch.py i2c_strict:=true include_bus0:=true

# PWM dryrun smoke test (no motor movement)
ros2 launch savo_base base_diag_board.launch.py run_pwm_sweep:=true pwm_dryrun:=true pwm_verbose:=true

# Real hardware PWM single-wheel sweep (robot lifted!)
ros2 launch savo_base base_diag_board.launch.py \
  run_pwm_sweep:=true pwm_dryrun:=false pwm_backend:=freenove \
  pwm_pattern:=single pwm_max_duty:=700 pwm_step_duty:=350 pwm_no_confirm:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _as_bool(text: str) -> bool:
    return str(text).strip().lower() in ("1", "true", "yes", "on")


def _build_i2c_check_process(context, *args, **kwargs):
    """Build ExecuteProcess for board_i2c_check.py with optional flags."""
    bus = LaunchConfiguration("i2c_bus").perform(context)
    include_bus0 = _as_bool(LaunchConfiguration("include_bus0").perform(context))
    strict = _as_bool(LaunchConfiguration("i2c_strict").perform(context))
    verbose = _as_bool(LaunchConfiguration("i2c_verbose").perform(context))

    cmd = [
        "python3",
        "-m",
        "savo_base.diagnostics.board_i2c_check",
        "--bus",
        str(bus),
    ]

    if include_bus0:
        cmd.append("--include-bus0")
    if strict:
        cmd.append("--strict")
    if verbose:
        cmd.append("--verbose")

    return [
        LogInfo(msg=f"[savo_base] Running board_i2c_check on bus {bus}"),
        ExecuteProcess(
            cmd=cmd,
            output="screen",
            shell=False,
            emulate_tty=True,
        ),
    ]


def _build_pwm_sweep_process(context, *args, **kwargs):
    """Build ExecuteProcess for pwm_sweep_check.py (dryrun or real hardware)."""
    pwm_backend = LaunchConfiguration("pwm_backend").perform(context)
    pwm_board_name = LaunchConfiguration("pwm_board_name").perform(context)
    pwm_dryrun = _as_bool(LaunchConfiguration("pwm_dryrun").perform(context))
    pwm_i2c_bus = LaunchConfiguration("pwm_i2c_bus").perform(context)
    pwm_addr = LaunchConfiguration("pwm_addr").perform(context)
    pwm_freq_hz = LaunchConfiguration("pwm_freq_hz").perform(context)
    pwm_quench_ms = LaunchConfiguration("pwm_quench_ms").perform(context)
    pwm_pattern = LaunchConfiguration("pwm_pattern").perform(context)
    pwm_max_duty = LaunchConfiguration("pwm_max_duty").perform(context)
    pwm_step_duty = LaunchConfiguration("pwm_step_duty").perform(context)
    pwm_hold_s = LaunchConfiguration("pwm_hold_s").perform(context)
    pwm_settle_s = LaunchConfiguration("pwm_settle_s").perform(context)
    pwm_repeat = LaunchConfiguration("pwm_repeat").perform(context)

    pwm_no_confirm = _as_bool(LaunchConfiguration("pwm_no_confirm").perform(context))
    pwm_verbose = _as_bool(LaunchConfiguration("pwm_verbose").perform(context))
    pwm_no_zero_between_steps = _as_bool(
        LaunchConfiguration("pwm_no_zero_between_steps").perform(context)
    )

    cmd = [
        "python3",
        "-m",
        "savo_base.diagnostics.pwm_sweep_check",
        "--backend",
        str(pwm_backend),
        "--board-name",
        str(pwm_board_name),
        "--i2c-bus",
        str(pwm_i2c_bus),
        "--addr",
        str(pwm_addr),
        "--pwm-freq-hz",
        str(pwm_freq_hz),
        "--quench-ms",
        str(pwm_quench_ms),
        "--pattern",
        str(pwm_pattern),
        "--max-duty",
        str(pwm_max_duty),
        "--step-duty",
        str(pwm_step_duty),
        "--hold-s",
        str(pwm_hold_s),
        "--settle-s",
        str(pwm_settle_s),
        "--repeat",
        str(pwm_repeat),
    ]

    if pwm_dryrun:
        cmd.append("--dryrun")
    if pwm_no_confirm:
        cmd.append("--no-confirm")
    if pwm_verbose:
        cmd.append("--verbose")
    if pwm_no_zero_between_steps:
        cmd.append("--no-zero-between-steps")

    mode_label = "DRYRUN" if pwm_dryrun else "REAL-HW"
    return [
        LogInfo(
            msg=(
                f"[savo_base] Running pwm_sweep_check ({mode_label}) "
                f"pattern={pwm_pattern}, max_duty={pwm_max_duty}, step={pwm_step_duty}"
            )
        ),
        ExecuteProcess(
            cmd=cmd,
            output="screen",
            shell=False,
            emulate_tty=True,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    # -------------------------------------------------------------------------
    # Launch args: I2C check
    # -------------------------------------------------------------------------
    run_i2c_check = LaunchConfiguration("run_i2c_check")
    run_pwm_sweep = LaunchConfiguration("run_pwm_sweep")

    return LaunchDescription(
        [
            # =================================================================
            # I2C board/device check arguments
            # =================================================================
            DeclareLaunchArgument(
                "run_i2c_check",
                default_value="true",
                description="Run board_i2c_check diagnostic.",
            ),
            DeclareLaunchArgument(
                "i2c_bus",
                default_value="1",
                description="Primary I2C bus for board_i2c_check (Robot Savo default: 1).",
            ),
            DeclareLaunchArgument(
                "include_bus0",
                default_value="false",
                description="Also check I2C-0 (useful for FL ToF visibility check).",
            ),
            DeclareLaunchArgument(
                "i2c_strict",
                default_value="false",
                description="Fail board_i2c_check if required devices are missing.",
            ),
            DeclareLaunchArgument(
                "i2c_verbose",
                default_value="true",
                description="Verbose output for board_i2c_check.",
            ),

            # =================================================================
            # PWM sweep diagnostic arguments (optional)
            # =================================================================
            DeclareLaunchArgument(
                "run_pwm_sweep",
                default_value="false",
                description="Run pwm_sweep_check diagnostic after I2C check.",
            ),
            DeclareLaunchArgument(
                "pwm_backend",
                default_value="auto",
                description="PWM sweep backend: auto | freenove | dryrun",
            ),
            DeclareLaunchArgument(
                "pwm_board_name",
                default_value="robot_savo_freenove_mecanum",
                description="Board profile/factory name label used by pwm_sweep_check.",
            ),
            DeclareLaunchArgument(
                "pwm_dryrun",
                default_value="true",
                description="Force dry-run backend for pwm_sweep_check (safe default).",
            ),
            DeclareLaunchArgument(
                "pwm_i2c_bus",
                default_value="1",
                description="PCA9685 I2C bus for pwm_sweep_check (Robot Savo default: 1).",
            ),
            DeclareLaunchArgument(
                "pwm_addr",
                default_value="0x40",
                description="PCA9685 address (Robot Savo default: 0x40).",
            ),
            DeclareLaunchArgument(
                "pwm_freq_hz",
                default_value="50.0",
                description="PCA9685 PWM frequency Hz (Robot Savo default: 50.0).",
            ),
            DeclareLaunchArgument(
                "pwm_quench_ms",
                default_value="18",
                description="Direction-flip quench delay ms (Robot Savo default: 18).",
            ),
            DeclareLaunchArgument(
                "pwm_pattern",
                default_value="single",
                description="PWM sweep pattern: single | all | rotate | full",
            ),
            DeclareLaunchArgument(
                "pwm_max_duty",
                default_value="700",
                description="Max abs duty for sweep (start low on real hardware).",
            ),
            DeclareLaunchArgument(
                "pwm_step_duty",
                default_value="350",
                description="Duty step size for sweep.",
            ),
            DeclareLaunchArgument(
                "pwm_hold_s",
                default_value="0.50",
                description="Hold time per step (seconds).",
            ),
            DeclareLaunchArgument(
                "pwm_settle_s",
                default_value="0.30",
                description="Settle time between steps (seconds).",
            ),
            DeclareLaunchArgument(
                "pwm_repeat",
                default_value="1",
                description="Repeat count for selected PWM sweep pattern.",
            ),
            DeclareLaunchArgument(
                "pwm_no_confirm",
                default_value="false",
                description="Skip safety confirmation prompt in pwm_sweep_check.",
            ),
            DeclareLaunchArgument(
                "pwm_verbose",
                default_value="true",
                description="Verbose output for pwm_sweep_check.",
            ),
            DeclareLaunchArgument(
                "pwm_no_zero_between_steps",
                default_value="false",
                description="Do not zero between PWM sweep steps.",
            ),

            # =================================================================
            # Diagnostics execution
            # =================================================================
            LogInfo(
                msg=(
                    "[savo_base] Base board diagnostics launch starting. "
                    "Defaults: I2C check ON, PWM sweep OFF."
                )
            ),

            OpaqueFunction(
                function=_build_i2c_check_process,
                condition=IfCondition(run_i2c_check),
            ),

            OpaqueFunction(
                function=_build_pwm_sweep_process,
                condition=IfCondition(run_pwm_sweep),
            ),

            LogInfo(
                msg=(
                    "[savo_base] NOTE: For real PWM sweep, lift robot wheels off ground "
                    "and keep emergency stop/power cut ready."
                )
            ),
        ]
    )