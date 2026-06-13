#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for odometry sanity checks used by Robot Savo localization."""

from __future__ import annotations

import math

import pytest

from savo_localization.constants import (
    FRAME_BASE_LINK,
    FRAME_ODOM,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_WARN,
)
from savo_localization.diagnostics.odom_sanity_check import (
    OdomJumpCheckResult,
    OdomSanityCheckResult,
    check_odom_jump,
    check_odom_sample_window,
    check_pose_twist,
    check_wheel_odom_sample,
    check_wheel_odom_state,
    odom_pose_summary,
    odom_sample_summary,
    odom_twist_summary,
)
from savo_localization.math.odom_math import Pose2D, Twist2D
from savo_localization.models.wheel_odom_state import (
    WheelOdomState,
    make_wheel_odom_sample,
)


def make_sample(
    *,
    stamp_s: float = 1.0,
    dt_s: float = 0.1,
    x_m: float = 0.0,
    y_m: float = 0.0,
    yaw_rad: float = 0.0,
    vx_mps: float = 0.0,
    vy_mps: float = 0.0,
    omega_rad_s: float = 0.0,
    odom_frame_id: str = FRAME_ODOM,
    base_frame_id: str = FRAME_BASE_LINK,
):
    return make_wheel_odom_sample(
        stamp_s=stamp_s,
        dt_s=dt_s,
        pose=Pose2D(x_m=x_m, y_m=y_m, yaw_rad=yaw_rad),
        twist=Twist2D(
            vx_mps=vx_mps,
            vy_mps=vy_mps,
            omega_rad_s=omega_rad_s,
        ),
        odom_frame_id=odom_frame_id,
        base_frame_id=base_frame_id,
        active_wheel_count=4,
        encoder_sample_count=1,
    )


def test_odom_sanity_check_result_to_dict() -> None:
    result = OdomSanityCheckResult(
        status=STATUS_OK,
        ok=True,
        message="odometry sanity check healthy",
        reasons=[],
        linear_speed_mps=0.2,
        angular_speed_rad_s=0.0,
    )

    data = result.to_dict()

    assert data["status"] == STATUS_OK
    assert data["ok"] is True
    assert data["message"] == "odometry sanity check healthy"
    assert data["reasons"] == []
    assert data["linear_speed_mps"] == pytest.approx(0.2)
    assert data["angular_speed_rad_s"] == pytest.approx(0.0)


def test_odom_jump_check_result_to_dict() -> None:
    result = OdomJumpCheckResult(
        status=STATUS_OK,
        ok=True,
        message="odometry jump check healthy",
        reasons=[],
        distance_jump_m=0.1,
        yaw_jump_rad=0.2,
        dt_s=0.1,
    )

    data = result.to_dict()

    assert data["status"] == STATUS_OK
    assert data["ok"] is True
    assert data["message"] == "odometry jump check healthy"
    assert data["distance_jump_m"] == pytest.approx(0.1)
    assert data["yaw_jump_rad"] == pytest.approx(0.2)
    assert data["dt_s"] == pytest.approx(0.1)


def test_check_pose_twist_healthy_stationary() -> None:
    result = check_pose_twist(
        pose=Pose2D(),
        twist=Twist2D(),
        dt_s=0.1,
        odom_frame_id=FRAME_ODOM,
        base_frame_id=FRAME_BASE_LINK,
    )

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.reasons == []
    assert result.pose_finite is True
    assert result.twist_finite is True
    assert result.frame_ok is True
    assert result.dt_ok is True


def test_check_pose_twist_healthy_mecanum_strafe() -> None:
    result = check_pose_twist(
        pose=Pose2D(x_m=0.0, y_m=0.1, yaw_rad=0.0),
        twist=Twist2D(vx_mps=0.0, vy_mps=0.2, omega_rad_s=0.0),
        dt_s=0.1,
        odom_frame_id=FRAME_ODOM,
        base_frame_id=FRAME_BASE_LINK,
    )

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.linear_speed_mps == pytest.approx(0.2)
    assert result.angular_speed_rad_s == pytest.approx(0.0)


def test_check_pose_twist_warns_on_high_linear_speed() -> None:
    result = check_pose_twist(
        pose=Pose2D(),
        twist=Twist2D(vx_mps=2.0, vy_mps=0.0, omega_rad_s=0.0),
        dt_s=0.1,
        odom_frame_id=FRAME_ODOM,
        base_frame_id=FRAME_BASE_LINK,
        max_linear_mps=1.5,
    )

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert result.linear_speed_mps == pytest.approx(2.0)
    assert any("linear speed" in reason.lower() for reason in result.reasons)


def test_check_pose_twist_warns_on_high_angular_speed() -> None:
    result = check_pose_twist(
        pose=Pose2D(),
        twist=Twist2D(vx_mps=0.0, vy_mps=0.0, omega_rad_s=5.0),
        dt_s=0.1,
        odom_frame_id=FRAME_ODOM,
        base_frame_id=FRAME_BASE_LINK,
        max_angular_rad_s=4.0,
    )

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert result.angular_speed_rad_s == pytest.approx(5.0)
    assert any("angular speed" in reason.lower() for reason in result.reasons)


def test_check_pose_twist_errors_on_non_finite_pose() -> None:
    result = check_pose_twist(
        pose=Pose2D(x_m=math.nan, y_m=0.0, yaw_rad=0.0),
        twist=Twist2D(),
        dt_s=0.1,
        odom_frame_id=FRAME_ODOM,
        base_frame_id=FRAME_BASE_LINK,
    )

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert result.pose_finite is False
    assert any("pose" in reason.lower() for reason in result.reasons)


def test_check_pose_twist_errors_on_non_finite_twist() -> None:
    result = check_pose_twist(
        pose=Pose2D(),
        twist=Twist2D(vx_mps=0.0, vy_mps=math.inf, omega_rad_s=0.0),
        dt_s=0.1,
        odom_frame_id=FRAME_ODOM,
        base_frame_id=FRAME_BASE_LINK,
    )

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert result.twist_finite is False
    assert any("twist" in reason.lower() for reason in result.reasons)


def test_check_pose_twist_errors_on_bad_dt() -> None:
    result = check_pose_twist(
        pose=Pose2D(),
        twist=Twist2D(),
        dt_s=-0.1,
        odom_frame_id=FRAME_ODOM,
        base_frame_id=FRAME_BASE_LINK,
    )

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert result.dt_ok is False
    assert any("dt" in reason.lower() for reason in result.reasons)


def test_check_pose_twist_warns_on_bad_frames_when_required() -> None:
    result = check_pose_twist(
        pose=Pose2D(),
        twist=Twist2D(),
        dt_s=0.1,
        odom_frame_id="odom",
        base_frame_id="odom",
        require_frame_ids=True,
    )

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert result.frame_ok is False
    assert any("frame" in reason.lower() for reason in result.reasons)


def test_check_pose_twist_ignores_bad_frames_when_not_required() -> None:
    result = check_pose_twist(
        pose=Pose2D(),
        twist=Twist2D(),
        dt_s=0.1,
        odom_frame_id="odom",
        base_frame_id="odom",
        require_frame_ids=False,
    )

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.frame_ok is True


def test_check_pose_twist_rejects_bad_limits() -> None:
    with pytest.raises(ValueError):
        check_pose_twist(
            pose=Pose2D(),
            twist=Twist2D(),
            dt_s=0.1,
            odom_frame_id=FRAME_ODOM,
            base_frame_id=FRAME_BASE_LINK,
            max_linear_mps=0.0,
        )

    with pytest.raises(ValueError):
        check_pose_twist(
            pose=Pose2D(),
            twist=Twist2D(),
            dt_s=0.1,
            odom_frame_id=FRAME_ODOM,
            base_frame_id=FRAME_BASE_LINK,
            max_angular_rad_s=0.0,
        )


def test_check_wheel_odom_sample_healthy() -> None:
    sample = make_sample(vx_mps=0.2)

    result = check_wheel_odom_sample(sample)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.linear_speed_mps == pytest.approx(0.2)


def test_check_wheel_odom_sample_warns_on_speed_limit() -> None:
    sample = make_sample(vx_mps=2.0)

    result = check_wheel_odom_sample(
        sample,
        max_linear_mps=1.5,
    )

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert result.reasons


def test_check_wheel_odom_state_without_sample_is_error() -> None:
    state = WheelOdomState()

    result = check_wheel_odom_state(state)

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert any("no sample" in reason.lower() for reason in result.reasons)


def test_check_wheel_odom_state_with_sample_is_healthy() -> None:
    state = WheelOdomState()
    sample = make_sample(stamp_s=10.0, vx_mps=0.2)

    state.mark_sample(sample, now_s=10.1)

    result = check_wheel_odom_state(state)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.linear_speed_mps == pytest.approx(0.2)


def test_check_odom_jump_healthy_small_motion() -> None:
    result = check_odom_jump(
        previous_pose=Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0),
        current_pose=Pose2D(x_m=0.05, y_m=0.0, yaw_rad=0.05),
        dt_s=0.1,
        max_distance_jump_m=0.5,
        max_yaw_jump_rad=1.0,
    )

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.distance_jump_m == pytest.approx(0.05)
    assert result.yaw_jump_rad == pytest.approx(0.05)


def test_check_odom_jump_warns_on_large_distance_jump() -> None:
    result = check_odom_jump(
        previous_pose=Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0),
        current_pose=Pose2D(x_m=1.0, y_m=0.0, yaw_rad=0.0),
        dt_s=0.1,
        max_distance_jump_m=0.5,
        max_yaw_jump_rad=1.0,
    )

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert result.distance_jump_m == pytest.approx(1.0)
    assert any("distance jump" in reason.lower() for reason in result.reasons)


def test_check_odom_jump_warns_on_large_yaw_jump() -> None:
    result = check_odom_jump(
        previous_pose=Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0),
        current_pose=Pose2D(x_m=0.0, y_m=0.0, yaw_rad=1.5),
        dt_s=0.1,
        max_distance_jump_m=0.5,
        max_yaw_jump_rad=1.0,
    )

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert result.yaw_jump_rad == pytest.approx(1.5)
    assert any("yaw jump" in reason.lower() for reason in result.reasons)


def test_check_odom_jump_uses_shortest_yaw_delta() -> None:
    result = check_odom_jump(
        previous_pose=Pose2D(x_m=0.0, y_m=0.0, yaw_rad=math.radians(170.0)),
        current_pose=Pose2D(x_m=0.0, y_m=0.0, yaw_rad=math.radians(-170.0)),
        dt_s=0.1,
        max_distance_jump_m=0.5,
        max_yaw_jump_rad=1.0,
    )

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.yaw_jump_rad == pytest.approx(math.radians(20.0))


def test_check_odom_jump_errors_on_non_finite_pose() -> None:
    result = check_odom_jump(
        previous_pose=Pose2D(x_m=math.nan, y_m=0.0, yaw_rad=0.0),
        current_pose=Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0),
        dt_s=0.1,
    )

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert any("non-finite" in reason for reason in result.reasons)


def test_check_odom_jump_errors_on_bad_dt() -> None:
    result = check_odom_jump(
        previous_pose=Pose2D(),
        current_pose=Pose2D(),
        dt_s=-0.1,
    )

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert any("invalid dt" in reason for reason in result.reasons)


def test_check_odom_jump_rejects_bad_limits() -> None:
    with pytest.raises(ValueError):
        check_odom_jump(
            previous_pose=Pose2D(),
            current_pose=Pose2D(),
            dt_s=0.1,
            max_distance_jump_m=-0.1,
        )

    with pytest.raises(ValueError):
        check_odom_jump(
            previous_pose=Pose2D(),
            current_pose=Pose2D(),
            dt_s=0.1,
            max_yaw_jump_rad=-0.1,
        )


def test_check_odom_sample_window_empty_is_error() -> None:
    result = check_odom_sample_window([])

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert any("empty" in reason.lower() for reason in result.reasons)


def test_check_odom_sample_window_healthy_samples() -> None:
    samples = [
        make_sample(stamp_s=1.0, x_m=0.00, vx_mps=0.2),
        make_sample(stamp_s=1.1, x_m=0.02, vx_mps=0.2),
        make_sample(stamp_s=1.2, x_m=0.04, vx_mps=0.2),
    ]

    result = check_odom_sample_window(samples)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.linear_speed_mps == pytest.approx(0.2)


def test_check_odom_sample_window_warns_on_jump() -> None:
    samples = [
        make_sample(stamp_s=1.0, x_m=0.0, vx_mps=0.2),
        make_sample(stamp_s=1.1, x_m=1.0, vx_mps=0.2),
    ]

    result = check_odom_sample_window(
        samples,
        max_distance_jump_m=0.5,
    )

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert any("jump" in reason.lower() for reason in result.reasons)


def test_check_odom_sample_window_errors_on_non_finite_sample() -> None:
    samples = [
        make_sample(stamp_s=1.0, x_m=0.0),
        make_sample(stamp_s=1.1, x_m=math.nan),
    ]

    result = check_odom_sample_window(samples)

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert result.reasons


def test_odom_pose_summary() -> None:
    summary = odom_pose_summary(
        Pose2D(x_m=1.0, y_m=-2.0, yaw_rad=0.5)
    )

    assert summary["x_m"] == pytest.approx(1.0)
    assert summary["y_m"] == pytest.approx(-2.0)
    assert summary["yaw_rad"] == pytest.approx(0.5)
    assert summary["finite"] is True


def test_odom_pose_summary_marks_non_finite() -> None:
    summary = odom_pose_summary(
        Pose2D(x_m=math.nan, y_m=0.0, yaw_rad=0.0)
    )

    assert summary["finite"] is False


def test_odom_twist_summary() -> None:
    summary = odom_twist_summary(
        Twist2D(vx_mps=0.3, vy_mps=0.4, omega_rad_s=-1.2)
    )

    assert summary["vx_mps"] == pytest.approx(0.3)
    assert summary["vy_mps"] == pytest.approx(0.4)
    assert summary["omega_rad_s"] == pytest.approx(-1.2)
    assert summary["linear_speed_mps"] == pytest.approx(0.5)
    assert summary["angular_speed_rad_s"] == pytest.approx(1.2)
    assert summary["finite"] is True


def test_odom_sample_summary() -> None:
    sample = make_sample(
        stamp_s=2.0,
        dt_s=0.1,
        x_m=1.0,
        y_m=2.0,
        yaw_rad=0.3,
        vx_mps=0.2,
        vy_mps=0.1,
        omega_rad_s=0.4,
    )

    summary = odom_sample_summary(sample)

    assert summary["stamp_s"] == pytest.approx(2.0)
    assert summary["dt_s"] == pytest.approx(0.1)
    assert summary["odom_frame_id"] == FRAME_ODOM
    assert summary["base_frame_id"] == FRAME_BASE_LINK
    assert summary["pose"]["x_m"] == pytest.approx(1.0)
    assert summary["twist"]["vx_mps"] == pytest.approx(0.2)
    assert summary["active_wheel_count"] == 4
    assert summary["encoder_sample_count"] == 1
    assert summary["moving"] is True
    assert summary["finite"] is True


def test_robot_savo_forward_odom_is_sane() -> None:
    sample = make_sample(
        stamp_s=1.0,
        dt_s=0.1,
        x_m=0.02,
        y_m=0.0,
        yaw_rad=0.0,
        vx_mps=0.2,
        vy_mps=0.0,
        omega_rad_s=0.0,
    )

    result = check_wheel_odom_sample(sample)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.linear_speed_mps == pytest.approx(0.2)
    assert result.angular_speed_rad_s == pytest.approx(0.0)


def test_robot_savo_strafe_odom_is_sane() -> None:
    sample = make_sample(
        stamp_s=1.0,
        dt_s=0.1,
        x_m=0.0,
        y_m=0.02,
        yaw_rad=0.0,
        vx_mps=0.0,
        vy_mps=0.2,
        omega_rad_s=0.0,
    )

    result = check_wheel_odom_sample(sample)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.linear_speed_mps == pytest.approx(0.2)
    assert result.angular_speed_rad_s == pytest.approx(0.0)


def test_robot_savo_rotation_odom_is_sane() -> None:
    sample = make_sample(
        stamp_s=1.0,
        dt_s=0.1,
        x_m=0.0,
        y_m=0.0,
        yaw_rad=0.1,
        vx_mps=0.0,
        vy_mps=0.0,
        omega_rad_s=1.0,
    )

    result = check_wheel_odom_sample(sample)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.linear_speed_mps == pytest.approx(0.0)
    assert result.angular_speed_rad_s == pytest.approx(1.0)