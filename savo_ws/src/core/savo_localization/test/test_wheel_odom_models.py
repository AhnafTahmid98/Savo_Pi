#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for wheel odometry models used by Robot Savo localization."""

from __future__ import annotations

import math

import pytest

from savo_localization.constants import (
    FRAME_BASE_LINK,
    FRAME_ODOM,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
)
from savo_localization.math.odom_math import Pose2D, Twist2D
from savo_localization.models.wheel_odom_state import (
    WheelOdomHealthState,
    WheelOdomSample,
    WheelOdomState,
    make_wheel_odom_sample,
)


def test_wheel_odom_sample_defaults() -> None:
    sample = WheelOdomSample()

    assert sample.stamp_s == pytest.approx(0.0)
    assert sample.dt_s == pytest.approx(0.0)
    assert sample.odom_frame_id == FRAME_ODOM
    assert sample.base_frame_id == FRAME_BASE_LINK

    assert sample.pose.x_m == pytest.approx(0.0)
    assert sample.pose.y_m == pytest.approx(0.0)
    assert sample.pose.yaw_rad == pytest.approx(0.0)

    assert sample.twist.vx_mps == pytest.approx(0.0)
    assert sample.twist.vy_mps == pytest.approx(0.0)
    assert sample.twist.omega_rad_s == pytest.approx(0.0)

    assert sample.active_wheel_count == 0
    assert sample.encoder_sample_count == 0
    assert sample.moving is False
    assert sample.finite is True


def test_make_wheel_odom_sample_stationary() -> None:
    sample = make_wheel_odom_sample(
        stamp_s=10.0,
        dt_s=0.1,
        pose=Pose2D(x_m=1.0, y_m=2.0, yaw_rad=0.5),
        twist=Twist2D(vx_mps=0.0, vy_mps=0.0, omega_rad_s=0.0),
        odom_frame_id=FRAME_ODOM,
        base_frame_id=FRAME_BASE_LINK,
        active_wheel_count=0,
        encoder_sample_count=5,
    )

    assert sample.stamp_s == pytest.approx(10.0)
    assert sample.dt_s == pytest.approx(0.1)
    assert sample.pose.x_m == pytest.approx(1.0)
    assert sample.pose.y_m == pytest.approx(2.0)
    assert sample.pose.yaw_rad == pytest.approx(0.5)
    assert sample.twist.vx_mps == pytest.approx(0.0)
    assert sample.twist.vy_mps == pytest.approx(0.0)
    assert sample.twist.omega_rad_s == pytest.approx(0.0)
    assert sample.active_wheel_count == 0
    assert sample.encoder_sample_count == 5
    assert sample.moving is False
    assert sample.finite is True


def test_make_wheel_odom_sample_forward_motion() -> None:
    sample = make_wheel_odom_sample(
        stamp_s=10.0,
        dt_s=0.1,
        pose=Pose2D(x_m=0.25, y_m=0.0, yaw_rad=0.0),
        twist=Twist2D(vx_mps=0.20, vy_mps=0.0, omega_rad_s=0.0),
        active_wheel_count=4,
        encoder_sample_count=10,
    )

    assert sample.moving is True
    assert sample.linear_speed_mps == pytest.approx(0.20)
    assert sample.angular_speed_rad_s == pytest.approx(0.0)
    assert sample.active_wheel_count == 4
    assert sample.encoder_sample_count == 10


def test_make_wheel_odom_sample_strafe_motion() -> None:
    sample = make_wheel_odom_sample(
        stamp_s=10.0,
        dt_s=0.1,
        pose=Pose2D(x_m=0.0, y_m=0.25, yaw_rad=0.0),
        twist=Twist2D(vx_mps=0.0, vy_mps=0.20, omega_rad_s=0.0),
        active_wheel_count=4,
        encoder_sample_count=10,
    )

    assert sample.moving is True
    assert sample.linear_speed_mps == pytest.approx(0.20)
    assert sample.angular_speed_rad_s == pytest.approx(0.0)


def test_make_wheel_odom_sample_rotation_motion() -> None:
    sample = make_wheel_odom_sample(
        stamp_s=10.0,
        dt_s=0.1,
        pose=Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.3),
        twist=Twist2D(vx_mps=0.0, vy_mps=0.0, omega_rad_s=1.2),
        active_wheel_count=4,
        encoder_sample_count=10,
    )

    assert sample.moving is True
    assert sample.linear_speed_mps == pytest.approx(0.0)
    assert sample.angular_speed_rad_s == pytest.approx(1.2)


def test_wheel_odom_sample_rejects_bad_dt() -> None:
    with pytest.raises(ValueError):
        make_wheel_odom_sample(
            stamp_s=10.0,
            dt_s=-0.1,
            pose=Pose2D(),
            twist=Twist2D(),
        )


def test_wheel_odom_sample_rejects_bad_frames() -> None:
    with pytest.raises(ValueError):
        make_wheel_odom_sample(
            stamp_s=10.0,
            dt_s=0.1,
            pose=Pose2D(),
            twist=Twist2D(),
            odom_frame_id="base_link",
            base_frame_id="base_link",
        )


def test_wheel_odom_sample_non_finite_pose() -> None:
    sample = WheelOdomSample(
        stamp_s=1.0,
        dt_s=0.1,
        pose=Pose2D(x_m=math.nan, y_m=0.0, yaw_rad=0.0),
        twist=Twist2D(),
    )

    assert sample.finite is False


def test_wheel_odom_sample_non_finite_twist() -> None:
    sample = WheelOdomSample(
        stamp_s=1.0,
        dt_s=0.1,
        pose=Pose2D(),
        twist=Twist2D(vx_mps=0.0, vy_mps=math.inf, omega_rad_s=0.0),
    )

    assert sample.finite is False


def test_wheel_odom_sample_to_dict() -> None:
    sample = make_wheel_odom_sample(
        stamp_s=2.5,
        dt_s=0.1,
        pose=Pose2D(x_m=1.0, y_m=-0.5, yaw_rad=0.25),
        twist=Twist2D(vx_mps=0.1, vy_mps=0.2, omega_rad_s=0.3),
        odom_frame_id=FRAME_ODOM,
        base_frame_id=FRAME_BASE_LINK,
        active_wheel_count=4,
        encoder_sample_count=12,
    )

    data = sample.to_dict()

    assert data["stamp_s"] == pytest.approx(2.5)
    assert data["dt_s"] == pytest.approx(0.1)
    assert data["odom_frame_id"] == FRAME_ODOM
    assert data["base_frame_id"] == FRAME_BASE_LINK

    assert data["pose"]["x_m"] == pytest.approx(1.0)
    assert data["pose"]["y_m"] == pytest.approx(-0.5)
    assert data["pose"]["yaw_rad"] == pytest.approx(0.25)

    assert data["twist"]["vx_mps"] == pytest.approx(0.1)
    assert data["twist"]["vy_mps"] == pytest.approx(0.2)
    assert data["twist"]["omega_rad_s"] == pytest.approx(0.3)

    assert data["linear_speed_mps"] == pytest.approx(math.hypot(0.1, 0.2))
    assert data["angular_speed_rad_s"] == pytest.approx(0.3)
    assert data["active_wheel_count"] == 4
    assert data["encoder_sample_count"] == 12
    assert data["moving"] is True
    assert data["finite"] is True


def test_wheel_odom_health_state_defaults() -> None:
    health = WheelOdomHealthState()

    assert health.status == STATUS_UNKNOWN
    assert health.ready is False
    assert health.hardware_ok is False
    assert health.data_ok is False
    assert health.rate_ok is False
    assert health.frame_ok is False
    assert health.finite_ok is False


def test_wheel_odom_health_state_ready_when_ok() -> None:
    health = WheelOdomHealthState(
        status=STATUS_OK,
        message="wheel odometry healthy",
        hardware_ok=True,
        data_ok=True,
        rate_ok=True,
        frame_ok=True,
        finite_ok=True,
        sample_count=10,
        active_wheel_count=4,
    )

    assert health.ready is True

    data = health.to_dict()
    assert data["ready"] is True
    assert data["status"] == STATUS_OK
    assert data["sample_count"] == 10
    assert data["active_wheel_count"] == 4


def test_wheel_odom_health_state_to_dict() -> None:
    health = WheelOdomHealthState(
        status=STATUS_OK,
        message="wheel odometry healthy",
        reasons=[],
        hardware_ok=True,
        data_ok=True,
        rate_ok=True,
        frame_ok=True,
        finite_ok=True,
        sample_count=20,
        active_wheel_count=4,
        rate_hz=30.0,
        last_age_s=0.05,
    )

    data = health.to_dict()

    assert data["status"] == STATUS_OK
    assert data["message"] == "wheel odometry healthy"
    assert data["reasons"] == []
    assert data["ready"] is True
    assert data["sample_count"] == 20
    assert data["active_wheel_count"] == 4
    assert data["rate_hz"] == pytest.approx(30.0)
    assert data["last_age_s"] == pytest.approx(0.05)


def test_wheel_odom_state_defaults() -> None:
    state = WheelOdomState()

    assert state.ready is False
    assert state.sample_count == 0
    assert state.last_sample is None
    assert state.last_sample_age_s is None
    assert state.health.status == STATUS_UNKNOWN
    assert state.total_distance_m == pytest.approx(0.0)
    assert state.total_rotation_rad == pytest.approx(0.0)


def test_wheel_odom_state_mark_sample() -> None:
    state = WheelOdomState()

    sample = make_wheel_odom_sample(
        stamp_s=10.0,
        dt_s=0.1,
        pose=Pose2D(x_m=0.02, y_m=0.0, yaw_rad=0.0),
        twist=Twist2D(vx_mps=0.2, vy_mps=0.0, omega_rad_s=0.0),
        active_wheel_count=4,
        encoder_sample_count=1,
    )

    state.mark_sample(sample, now_s=10.2)

    assert state.sample_count == 1
    assert state.last_sample is sample
    assert state.last_sample_age_s == pytest.approx(0.2)
    assert state.ready is True
    assert state.health.sample_count == 1
    assert state.health.active_wheel_count == 4


def test_wheel_odom_state_accumulates_distance() -> None:
    state = WheelOdomState()

    first = make_wheel_odom_sample(
        stamp_s=1.0,
        dt_s=0.5,
        pose=Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0),
        twist=Twist2D(vx_mps=0.2, vy_mps=0.0, omega_rad_s=0.0),
        active_wheel_count=4,
        encoder_sample_count=1,
    )

    second = make_wheel_odom_sample(
        stamp_s=1.5,
        dt_s=0.5,
        pose=Pose2D(x_m=0.1, y_m=0.0, yaw_rad=0.0),
        twist=Twist2D(vx_mps=0.2, vy_mps=0.0, omega_rad_s=0.0),
        active_wheel_count=4,
        encoder_sample_count=2,
    )

    state.mark_sample(first, now_s=1.0)
    state.mark_sample(second, now_s=1.5)

    assert state.sample_count == 2
    assert state.total_distance_m == pytest.approx(0.1)
    assert state.total_rotation_rad == pytest.approx(0.0)


def test_wheel_odom_state_accumulates_rotation() -> None:
    state = WheelOdomState()

    first = make_wheel_odom_sample(
        stamp_s=1.0,
        dt_s=0.5,
        pose=Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0),
        twist=Twist2D(vx_mps=0.0, vy_mps=0.0, omega_rad_s=1.0),
        active_wheel_count=4,
        encoder_sample_count=1,
    )

    second = make_wheel_odom_sample(
        stamp_s=1.5,
        dt_s=0.5,
        pose=Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.5),
        twist=Twist2D(vx_mps=0.0, vy_mps=0.0, omega_rad_s=1.0),
        active_wheel_count=4,
        encoder_sample_count=2,
    )

    state.mark_sample(first, now_s=1.0)
    state.mark_sample(second, now_s=1.5)

    assert state.sample_count == 2
    assert state.total_distance_m == pytest.approx(0.0)
    assert state.total_rotation_rad == pytest.approx(0.5)


def test_wheel_odom_state_mark_stale() -> None:
    state = WheelOdomState()

    state.mark_stale(age_s=1.25)

    assert state.ready is False
    assert state.last_sample_age_s == pytest.approx(1.25)
    assert state.health.status == STATUS_STALE


def test_wheel_odom_state_to_dict_contains_core_fields() -> None:
    state = WheelOdomState()

    sample = make_wheel_odom_sample(
        stamp_s=10.0,
        dt_s=0.1,
        pose=Pose2D(x_m=0.02, y_m=0.0, yaw_rad=0.0),
        twist=Twist2D(vx_mps=0.2, vy_mps=0.0, omega_rad_s=0.0),
        active_wheel_count=4,
        encoder_sample_count=1,
    )

    state.mark_sample(sample, now_s=10.0)
    data = state.to_dict()

    assert data["ready"] is True
    assert data["sample_count"] == 1
    assert data["last_sample_age_s"] == pytest.approx(0.0)
    assert data["total_distance_m"] == pytest.approx(0.0)
    assert data["total_rotation_rad"] == pytest.approx(0.0)
    assert "health" in data
    assert "last_sample" in data


def test_robot_savo_forward_wheel_odom_sample_is_valid() -> None:
    sample = make_wheel_odom_sample(
        stamp_s=1.0,
        dt_s=0.1,
        pose=Pose2D(x_m=0.02, y_m=0.0, yaw_rad=0.0),
        twist=Twist2D(vx_mps=0.2, vy_mps=0.0, omega_rad_s=0.0),
        active_wheel_count=4,
        encoder_sample_count=1,
    )

    assert sample.odom_frame_id == FRAME_ODOM
    assert sample.base_frame_id == FRAME_BASE_LINK
    assert sample.finite is True
    assert sample.moving is True
    assert sample.linear_speed_mps == pytest.approx(0.2)
    assert sample.angular_speed_rad_s == pytest.approx(0.0)


def test_robot_savo_side_motion_wheel_odom_sample_is_valid() -> None:
    sample = make_wheel_odom_sample(
        stamp_s=1.0,
        dt_s=0.1,
        pose=Pose2D(x_m=0.0, y_m=0.02, yaw_rad=0.0),
        twist=Twist2D(vx_mps=0.0, vy_mps=0.2, omega_rad_s=0.0),
        active_wheel_count=4,
        encoder_sample_count=1,
    )

    assert sample.finite is True
    assert sample.moving is True
    assert sample.linear_speed_mps == pytest.approx(0.2)
    assert sample.angular_speed_rad_s == pytest.approx(0.0)


def test_robot_savo_rotation_wheel_odom_sample_is_valid() -> None:
    sample = make_wheel_odom_sample(
        stamp_s=1.0,
        dt_s=0.1,
        pose=Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.1),
        twist=Twist2D(vx_mps=0.0, vy_mps=0.0, omega_rad_s=1.0),
        active_wheel_count=4,
        encoder_sample_count=1,
    )

    assert sample.finite is True
    assert sample.moving is True
    assert sample.linear_speed_mps == pytest.approx(0.0)
    assert sample.angular_speed_rad_s == pytest.approx(1.0)