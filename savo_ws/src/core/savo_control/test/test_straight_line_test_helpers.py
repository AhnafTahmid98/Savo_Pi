#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for pure straight-line PID test helpers."""

from __future__ import annotations

from math import isclose, pi

from savo_control.models import TwistCommand
from savo_control.nodes.straight_line_test_helpers import (
    STRAIGHT_LINE_SOURCE,
    Pose2D,
    StraightLineConfig,
    StraightLineRun,
    StraightLineState,
    StraightLineStep,
    make_lateral_pid,
    make_straight_line_command,
    make_yaw_pid,
    pose_delta_in_start_frame,
    start_run,
    status_text,
    step_straight_line,
    stop_command,
    yaw_error_to_start,
)


def assert_close(value: float, expected: float) -> None:
    assert isclose(value, expected, abs_tol=1e-9)


def test_source_is_canonical():
    assert STRAIGHT_LINE_SOURCE == "straight_line_pid_test"


def test_state_values_are_canonical():
    assert [state.value for state in StraightLineState] == [
        "IDLE",
        "RUNNING",
        "GOAL_REACHED",
        "BLOCKED",
        "STALE_ODOM",
        "DISABLED",
        "TIMEOUT",
    ]


def test_pose_sanitized():
    pose = Pose2D(
        x=float("nan"),
        y=float("inf"),
        yaw=3.0 * pi,
    ).sanitized()

    assert pose.x == 0.0
    assert pose.y == 0.0
    assert_close(pose.yaw, pi)


def test_config_defaults_are_safe():
    cfg = StraightLineConfig()

    assert cfg.target_distance_m == 0.80
    assert cfg.goal_tolerance_m == 0.04
    assert cfg.forward_vx_m_s == 0.10
    assert cfg.max_vx_m_s == 0.14
    assert cfg.max_vy_m_s == 0.08
    assert cfg.max_wz_rad_s == 0.35
    assert cfg.lateral_kp == 0.80
    assert cfg.yaw_kp == 1.20
    assert cfg.min_dt_s == 1.0e-4
    assert cfg.max_duration_s == 12.0


def test_config_sanitized_clamps_values():
    cfg = StraightLineConfig(
        target_distance_m=-1.0,
        goal_tolerance_m=-1.0,
        forward_vx_m_s=2.0,
        max_vx_m_s=0.20,
        max_vy_m_s=-0.10,
        max_wz_rad_s=-0.50,
        lateral_kp=float("nan"),
        yaw_kp=float("inf"),
        min_dt_s=-1.0,
        max_duration_s=-1.0,
    ).sanitized()

    assert cfg.target_distance_m == 0.0
    assert cfg.goal_tolerance_m == 0.0
    assert cfg.forward_vx_m_s == 0.20
    assert cfg.max_vx_m_s == 0.20
    assert cfg.max_vy_m_s == 0.10
    assert cfg.max_wz_rad_s == 0.50
    assert cfg.lateral_kp == 0.0
    assert cfg.yaw_kp == 0.0
    assert cfg.min_dt_s == 1.0e-6
    assert cfg.max_duration_s == 0.1


def test_stop_command():
    cmd = stop_command(stamp_sec=2.0)

    assert isinstance(cmd, TwistCommand)
    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.wz == 0.0
    assert cmd.source == STRAIGHT_LINE_SOURCE
    assert cmd.stamp_sec == 2.0


def test_start_run():
    cfg = StraightLineConfig(target_distance_m=1.0)
    pose = Pose2D(0.0, 0.0, 0.0)

    run = start_run(pose=pose, config=cfg, now_s=5.0)

    assert isinstance(run, StraightLineRun)
    assert run.start_pose == pose
    assert run.target_distance_m == 1.0
    assert run.start_s == 5.0
    assert run.elapsed_s(now_s=5.5) == 0.5


def test_run_elapsed_does_not_go_negative():
    run = start_run(
        pose=Pose2D(),
        config=StraightLineConfig(),
        now_s=5.0,
    )

    assert run.elapsed_s(now_s=4.0) == 0.0


def test_pose_delta_in_start_frame_zero_yaw():
    forward, lateral = pose_delta_in_start_frame(
        start=Pose2D(0.0, 0.0, 0.0),
        current=Pose2D(1.0, 0.25, 0.0),
    )

    assert_close(forward, 1.0)
    assert_close(lateral, 0.25)


def test_pose_delta_in_start_frame_rotated_start():
    forward, lateral = pose_delta_in_start_frame(
        start=Pose2D(0.0, 0.0, pi / 2.0),
        current=Pose2D(0.0, 1.0, pi / 2.0),
    )

    assert_close(forward, 1.0)
    assert_close(lateral, 0.0)


def test_yaw_error_to_start():
    assert_close(yaw_error_to_start(start_yaw=0.0, current_yaw=0.10), -0.10)
    assert_close(yaw_error_to_start(start_yaw=0.10, current_yaw=0.0), 0.10)


def test_make_straight_line_command_tracking_centerline():
    cfg = StraightLineConfig(
        forward_vx_m_s=0.10,
        max_vx_m_s=0.14,
        max_vy_m_s=0.08,
        max_wz_rad_s=0.35,
        lateral_kp=0.80,
        yaw_kp=1.20,
    ).sanitized()

    cmd = make_straight_line_command(
        forward_vx=0.10,
        lateral_error_m=0.0,
        yaw_error_rad=0.0,
        config=cfg,
        stamp_sec=1.0,
    )

    assert cmd.vx == 0.10
    assert cmd.vy == 0.0
    assert cmd.wz == 0.0
    assert cmd.source == STRAIGHT_LINE_SOURCE
    assert cmd.stamp_sec == 1.0


def test_make_straight_line_command_lateral_and_yaw_correction():
    cfg = StraightLineConfig(
        max_vx_m_s=0.14,
        max_vy_m_s=0.08,
        max_wz_rad_s=0.35,
        lateral_kp=0.80,
        yaw_kp=1.20,
    ).sanitized()

    cmd = make_straight_line_command(
        forward_vx=0.20,
        lateral_error_m=0.20,
        yaw_error_rad=-0.20,
        config=cfg,
    )

    assert cmd.vx == 0.14
    assert cmd.vy == -0.08
    assert_close(cmd.wz, -0.24)


def test_step_disabled():
    step = step_straight_line(
        run=None,
        pose=Pose2D(),
        config=StraightLineConfig(),
        now_s=1.0,
        enabled=False,
    )

    assert step.state == StraightLineState.DISABLED
    assert step.finished is True
    assert step.run is None
    assert step.command.vx == 0.0
    assert step.reason == "disabled"


def test_step_safety_stop():
    step = step_straight_line(
        run=None,
        pose=Pose2D(),
        config=StraightLineConfig(),
        now_s=1.0,
        safety_stop=True,
    )

    assert step.state == StraightLineState.BLOCKED
    assert step.finished is True
    assert step.run is None
    assert step.command.vx == 0.0
    assert step.reason == "safety_stop"


def test_step_stale_odom():
    run = start_run(
        pose=Pose2D(),
        config=StraightLineConfig(),
        now_s=1.0,
    )

    step = step_straight_line(
        run=run,
        pose=Pose2D(),
        config=StraightLineConfig(),
        now_s=1.5,
        odom_fresh=False,
    )

    assert step.state == StraightLineState.STALE_ODOM
    assert step.finished is False
    assert step.run == run
    assert step.command.vx == 0.0
    assert step.reason == "odom_stale"


def test_step_stale_odom_when_pose_missing():
    step = step_straight_line(
        run=None,
        pose=None,
        config=StraightLineConfig(),
        now_s=1.0,
        odom_fresh=True,
    )

    assert step.state == StraightLineState.STALE_ODOM
    assert step.command.vx == 0.0
    assert step.reason == "odom_stale"


def test_step_starts_run_when_missing():
    cfg = StraightLineConfig(target_distance_m=1.0).sanitized()

    step = step_straight_line(
        run=None,
        pose=Pose2D(0.0, 0.0, 0.0),
        config=cfg,
        now_s=1.0,
    )

    assert step.state == StraightLineState.RUNNING
    assert step.run is not None
    assert step.run.start_s == 1.0
    assert step.command.vx > 0.0
    assert step.remaining_m == 1.0


def test_step_running():
    cfg = StraightLineConfig(target_distance_m=1.0).sanitized()
    run = start_run(pose=Pose2D(0.0, 0.0, 0.0), config=cfg, now_s=1.0)

    step = step_straight_line(
        run=run,
        pose=Pose2D(0.5, 0.1, 0.05),
        config=cfg,
        now_s=2.0,
    )

    assert isinstance(step, StraightLineStep)
    assert step.state == StraightLineState.RUNNING
    assert step.finished is False
    assert step.run == run
    assert step.command.vx > 0.0
    assert step.command.vy < 0.0
    assert step.command.wz < 0.0
    assert_close(step.forward_progress_m, 0.5)
    assert_close(step.lateral_error_m, 0.1)
    assert step.remaining_m > 0.0
    assert step.reason == "tracking"


def test_step_goal_reached():
    cfg = StraightLineConfig(
        target_distance_m=1.0,
        goal_tolerance_m=0.04,
    ).sanitized()
    run = start_run(pose=Pose2D(0.0, 0.0, 0.0), config=cfg, now_s=1.0)

    step = step_straight_line(
        run=run,
        pose=Pose2D(0.97, 0.0, 0.0),
        config=cfg,
        now_s=2.0,
    )

    assert step.state == StraightLineState.GOAL_REACHED
    assert step.finished is True
    assert step.run is None
    assert step.command.vx == 0.0
    assert_close(step.forward_progress_m, 0.97)
    assert_close(step.remaining_m, 0.03)
    assert step.reason == "goal_reached"


def test_step_timeout():
    cfg = StraightLineConfig(
        target_distance_m=10.0,
        max_duration_s=2.0,
    ).sanitized()
    run = start_run(pose=Pose2D(0.0, 0.0, 0.0), config=cfg, now_s=1.0)

    step = step_straight_line(
        run=run,
        pose=Pose2D(0.0, 0.0, 0.0),
        config=cfg,
        now_s=4.0,
    )

    assert step.state == StraightLineState.TIMEOUT
    assert step.finished is True
    assert step.run is None
    assert step.command.vx == 0.0
    assert step.reason == "timeout"


def test_status_text():
    text = status_text(
        state=StraightLineState.RUNNING,
        enabled=True,
        reason="tracking",
        forward_progress_m=0.5,
        lateral_error_m=0.1,
        yaw_error_rad=-0.05,
        remaining_m=0.5,
        command=TwistCommand(vx=0.1, vy=-0.08, wz=-0.06),
    )

    assert "state=RUNNING" in text
    assert "enabled=true" in text
    assert "reason=tracking" in text
    assert "progress_m=0.500" in text
    assert "lateral_error_m=0.100" in text
    assert "yaw_error_rad=-0.050" in text
    assert "remaining_m=0.500" in text
    assert "vx=0.100" in text
    assert "vy=-0.080" in text
    assert "wz=-0.060" in text


def test_make_lateral_pid():
    pid = make_lateral_pid(StraightLineConfig(lateral_kp=0.8, max_vy_m_s=0.08))

    assert pid.config().kp == 0.8
    assert pid.config().output_min == -0.08
    assert pid.config().output_max == 0.08


def test_make_yaw_pid():
    pid = make_yaw_pid(StraightLineConfig(yaw_kp=1.2, max_wz_rad_s=0.35))

    assert pid.config().kp == 1.2
    assert pid.config().output_min == -0.35
    assert pid.config().output_max == 0.35
