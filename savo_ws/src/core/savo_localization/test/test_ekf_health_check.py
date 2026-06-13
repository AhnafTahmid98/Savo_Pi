#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for EKF health checks used by Robot Savo localization."""

from __future__ import annotations

import pytest

from savo_localization.constants import (
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_WARN,
)
from savo_localization.diagnostics.ekf_health_check import (
    EkfHealthCheckResult,
    check_ekf_health,
    check_ekf_input,
    check_ekf_output,
    check_ekf_rate_health,
    check_ekf_tf,
    ekf_health_summary,
    summarize_ekf_inputs,
)
from savo_localization.models.ekf_health import (
    EkfHealth,
    EkfInputHealth,
    EkfOutputHealth,
    EkfTfHealth,
)


def make_healthy_input(
    *,
    name: str = "wheel_odom",
    topic: str = "/wheel/odom",
    rate_hz: float = 30.0,
) -> EkfInputHealth:
    return EkfInputHealth(
        name=name,
        topic=topic,
        enabled=True,
        seen=True,
        fresh=True,
        last_age_s=0.05,
        message_count=30,
        rate_hz=rate_hz,
        status=STATUS_OK,
        message="input healthy",
        reasons=[],
    )


def make_stale_input(
    *,
    name: str = "wheel_odom",
    topic: str = "/wheel/odom",
) -> EkfInputHealth:
    return EkfInputHealth(
        name=name,
        topic=topic,
        enabled=True,
        seen=True,
        fresh=False,
        last_age_s=1.5,
        message_count=30,
        rate_hz=30.0,
        status=STATUS_STALE,
        message="input stale",
        reasons=["input stale"],
    )


def make_missing_input(
    *,
    name: str = "imu",
    topic: str = "/imu/data",
) -> EkfInputHealth:
    return EkfInputHealth(
        name=name,
        topic=topic,
        enabled=True,
        seen=False,
        fresh=False,
        last_age_s=None,
        message_count=0,
        rate_hz=0.0,
        status=STATUS_ERROR,
        message="input missing",
        reasons=["no messages received"],
    )


def make_healthy_output() -> EkfOutputHealth:
    return EkfOutputHealth(
        topic="/odometry/filtered",
        seen=True,
        fresh=True,
        last_age_s=0.05,
        message_count=30,
        rate_hz=30.0,
        frame_ok=True,
        finite_ok=True,
        covariance_ok=True,
        status=STATUS_OK,
        message="EKF output healthy",
        reasons=[],
    )


def make_stale_output() -> EkfOutputHealth:
    return EkfOutputHealth(
        topic="/odometry/filtered",
        seen=True,
        fresh=False,
        last_age_s=1.5,
        message_count=30,
        rate_hz=30.0,
        frame_ok=True,
        finite_ok=True,
        covariance_ok=True,
        status=STATUS_STALE,
        message="EKF output stale",
        reasons=["output stale"],
    )


def make_missing_output() -> EkfOutputHealth:
    return EkfOutputHealth(
        topic="/odometry/filtered",
        seen=False,
        fresh=False,
        last_age_s=None,
        message_count=0,
        rate_hz=0.0,
        frame_ok=False,
        finite_ok=False,
        covariance_ok=False,
        status=STATUS_ERROR,
        message="EKF output missing",
        reasons=["no messages received"],
    )


def make_healthy_tf() -> EkfTfHealth:
    return EkfTfHealth(
        parent_frame="odom",
        child_frame="base_link",
        available=True,
        fresh=True,
        last_age_s=0.05,
        status=STATUS_OK,
        message="EKF TF healthy",
        reasons=[],
    )


def make_stale_tf() -> EkfTfHealth:
    return EkfTfHealth(
        parent_frame="odom",
        child_frame="base_link",
        available=True,
        fresh=False,
        last_age_s=1.5,
        status=STATUS_STALE,
        message="EKF TF stale",
        reasons=["TF stale"],
    )


def make_missing_tf() -> EkfTfHealth:
    return EkfTfHealth(
        parent_frame="odom",
        child_frame="base_link",
        available=False,
        fresh=False,
        last_age_s=None,
        status=STATUS_STALE,
        message="EKF TF missing",
        reasons=["transform not available"],
    )


def test_ekf_health_check_result_to_dict() -> None:
    result = EkfHealthCheckResult(
        status=STATUS_OK,
        ok=True,
        usable=True,
        message="EKF healthy",
        reasons=[],
        active_input_count=2,
        healthy_input_count=2,
        output_ok=True,
        tf_ok=True,
    )

    data = result.to_dict()

    assert data["status"] == STATUS_OK
    assert data["ok"] is True
    assert data["usable"] is True
    assert data["message"] == "EKF healthy"
    assert data["reasons"] == []
    assert data["active_input_count"] == 2
    assert data["healthy_input_count"] == 2
    assert data["output_ok"] is True
    assert data["tf_ok"] is True


def test_check_ekf_input_healthy() -> None:
    result = check_ekf_input(make_healthy_input())

    assert result.ok is True
    assert result.usable is True
    assert result.status == STATUS_OK
    assert result.active_input_count == 1
    assert result.healthy_input_count == 1
    assert result.reasons == []


def test_check_ekf_input_disabled_is_ok() -> None:
    sensor = EkfInputHealth(
        name="vo_odom",
        topic="/vo/odom",
        enabled=False,
        seen=False,
        fresh=False,
        last_age_s=None,
        message_count=0,
        rate_hz=0.0,
        status=STATUS_OK,
        message="input disabled",
        reasons=[],
    )

    result = check_ekf_input(sensor)

    assert result.ok is True
    assert result.usable is True
    assert result.status == STATUS_OK
    assert result.active_input_count == 0
    assert result.healthy_input_count == 0
    assert result.reasons == []


def test_check_ekf_input_missing_is_error() -> None:
    result = check_ekf_input(make_missing_input())

    assert result.ok is False
    assert result.usable is False
    assert result.status == STATUS_ERROR
    assert result.active_input_count == 1
    assert result.healthy_input_count == 0
    assert any("no messages" in reason.lower() for reason in result.reasons)


def test_check_ekf_input_stale_is_stale() -> None:
    result = check_ekf_input(
        make_stale_input(),
        stale_timeout_s=0.5,
    )

    assert result.ok is False
    assert result.usable is False
    assert result.status == STATUS_STALE
    assert result.active_input_count == 1
    assert result.healthy_input_count == 0
    assert any("stale" in reason.lower() for reason in result.reasons)


def test_check_ekf_input_rejects_bad_timeout() -> None:
    sensor = make_healthy_input()

    with pytest.raises(ValueError):
        check_ekf_input(sensor, stale_timeout_s=0.0)


def test_check_ekf_output_healthy() -> None:
    result = check_ekf_output(make_healthy_output())

    assert result.ok is True
    assert result.usable is True
    assert result.status == STATUS_OK
    assert result.output_ok is True
    assert result.reasons == []


def test_check_ekf_output_missing_is_error() -> None:
    result = check_ekf_output(make_missing_output())

    assert result.ok is False
    assert result.usable is False
    assert result.status == STATUS_ERROR
    assert result.output_ok is False
    assert any("no messages" in reason.lower() for reason in result.reasons)


def test_check_ekf_output_stale_is_stale() -> None:
    result = check_ekf_output(
        make_stale_output(),
        stale_timeout_s=0.5,
    )

    assert result.ok is False
    assert result.usable is False
    assert result.status == STATUS_STALE
    assert result.output_ok is False
    assert any("stale" in reason.lower() for reason in result.reasons)


def test_check_ekf_output_bad_frames_is_error() -> None:
    output = make_healthy_output()
    output.frame_ok = False

    result = check_ekf_output(output)

    assert result.ok is False
    assert result.status == STATUS_STALE
    assert any("frame" in reason.lower() for reason in result.reasons)


def test_check_ekf_output_bad_finite_values_is_error() -> None:
    output = make_healthy_output()
    output.finite_ok = False

    result = check_ekf_output(output)

    assert result.ok is False
    assert result.status == STATUS_STALE
    assert any("finite" in reason.lower() for reason in result.reasons)


def test_check_ekf_output_bad_covariance_is_error() -> None:
    output = make_healthy_output()
    output.covariance_ok = False

    result = check_ekf_output(output)

    assert result.ok is False
    assert result.status == STATUS_STALE
    assert any("covariance" in reason.lower() for reason in result.reasons)


def test_check_ekf_tf_healthy() -> None:
    result = check_ekf_tf(make_healthy_tf())

    assert result.ok is True
    assert result.usable is True
    assert result.status == STATUS_OK
    assert result.tf_ok is True
    assert result.reasons == []


def test_check_ekf_tf_missing_is_stale() -> None:
    result = check_ekf_tf(make_missing_tf())

    assert result.ok is False
    assert result.usable is False
    assert result.status == STATUS_STALE
    assert result.tf_ok is False
    assert any("transform not available" in reason.lower() for reason in result.reasons)


def test_check_ekf_tf_stale_is_stale() -> None:
    result = check_ekf_tf(
        make_stale_tf(),
        stale_timeout_s=0.5,
    )

    assert result.ok is False
    assert result.usable is False
    assert result.status == STATUS_STALE
    assert result.tf_ok is False
    assert any("stale" in reason.lower() for reason in result.reasons)


def test_check_ekf_health_healthy_baseline() -> None:
    health = EkfHealth(
        inputs=[
            make_healthy_input(name="wheel_odom", topic="/wheel/odom"),
            make_healthy_input(name="imu", topic="/imu/data", rate_hz=25.0),
        ],
        output=make_healthy_output(),
        tf=make_healthy_tf(),
    )

    result = check_ekf_health(health)

    assert result.ok is True
    assert result.usable is True
    assert result.status == STATUS_OK
    assert result.active_input_count == 2
    assert result.healthy_input_count == 2
    assert result.output_ok is True
    assert result.tf_ok is True
    assert result.reasons == []


def test_check_ekf_health_missing_inputs_is_error() -> None:
    health = EkfHealth(
        inputs=[],
        output=make_healthy_output(),
        tf=make_healthy_tf(),
    )

    result = check_ekf_health(health)

    assert result.ok is False
    assert result.usable is False
    assert result.status == STATUS_ERROR
    assert result.active_input_count == 0
    assert result.healthy_input_count == 0
    assert any("no ekf inputs" in reason.lower() for reason in result.reasons)


def test_check_ekf_health_not_enough_healthy_inputs_is_error() -> None:
    health = EkfHealth(
        inputs=[
            make_healthy_input(name="wheel_odom", topic="/wheel/odom"),
            make_stale_input(name="imu", topic="/imu/data"),
        ],
        output=make_healthy_output(),
        tf=make_healthy_tf(),
    )

    result = check_ekf_health(
        health,
        min_healthy_inputs=2,
    )

    assert result.ok is False
    assert result.usable is False
    assert result.status == STATUS_ERROR
    assert result.active_input_count == 2
    assert result.healthy_input_count == 1
    assert any("not enough" in reason.lower() for reason in result.reasons)


def test_check_ekf_health_usable_with_one_input_when_allowed() -> None:
    health = EkfHealth(
        inputs=[
            make_healthy_input(name="wheel_odom", topic="/wheel/odom"),
            make_missing_input(name="imu", topic="/imu/data"),
        ],
        output=make_healthy_output(),
        tf=make_healthy_tf(),
    )

    result = check_ekf_health(
        health,
        min_healthy_inputs=1,
    )

    assert result.ok is True
    assert result.usable is True
    assert result.status == STATUS_WARN
    assert result.active_input_count == 2
    assert result.healthy_input_count == 1
    assert result.reasons


def test_check_ekf_health_missing_output_is_error_when_required() -> None:
    health = EkfHealth(
        inputs=[
            make_healthy_input(name="wheel_odom", topic="/wheel/odom"),
            make_healthy_input(name="imu", topic="/imu/data"),
        ],
        output=make_missing_output(),
        tf=make_healthy_tf(),
    )

    result = check_ekf_health(
        health,
        require_output=True,
    )

    assert result.ok is False
    assert result.usable is False
    assert result.status == STATUS_ERROR
    assert result.output_ok is False
    assert any("output" in reason.lower() for reason in result.reasons)


def test_check_ekf_health_missing_output_can_be_ignored() -> None:
    health = EkfHealth(
        inputs=[
            make_healthy_input(name="wheel_odom", topic="/wheel/odom"),
            make_healthy_input(name="imu", topic="/imu/data"),
        ],
        output=make_missing_output(),
        tf=make_healthy_tf(),
    )

    result = check_ekf_health(
        health,
        require_output=False,
    )

    assert result.ok is True
    assert result.usable is True
    assert result.status in (STATUS_OK, STATUS_WARN)


def test_check_ekf_health_missing_tf_is_error_when_required() -> None:
    health = EkfHealth(
        inputs=[
            make_healthy_input(name="wheel_odom", topic="/wheel/odom"),
            make_healthy_input(name="imu", topic="/imu/data"),
        ],
        output=make_healthy_output(),
        tf=make_missing_tf(),
    )

    result = check_ekf_health(
        health,
        require_tf=True,
    )

    assert result.ok is False
    assert result.usable is False
    assert result.status == STATUS_ERROR
    assert result.tf_ok is False
    assert any("tf" in reason.lower() for reason in result.reasons)


def test_check_ekf_health_missing_tf_can_be_ignored() -> None:
    health = EkfHealth(
        inputs=[
            make_healthy_input(name="wheel_odom", topic="/wheel/odom"),
            make_healthy_input(name="imu", topic="/imu/data"),
        ],
        output=make_healthy_output(),
        tf=make_missing_tf(),
    )

    result = check_ekf_health(
        health,
        require_tf=False,
    )

    assert result.ok is True
    assert result.usable is True
    assert result.status in (STATUS_OK, STATUS_WARN)


def test_check_ekf_health_rejects_bad_min_healthy_inputs() -> None:
    health = EkfHealth(
        inputs=[make_healthy_input()],
        output=make_healthy_output(),
        tf=make_healthy_tf(),
    )

    with pytest.raises(ValueError):
        check_ekf_health(health, min_healthy_inputs=0)


def test_check_ekf_rate_health_ok() -> None:
    result = check_ekf_rate_health(
        measured_hz=30.0,
        expected_hz=30.0,
        tolerance_ratio=0.50,
    )

    assert result.ok is True
    assert result.usable is True
    assert result.status == STATUS_OK
    assert result.reasons == []


def test_check_ekf_rate_health_warns_when_low() -> None:
    result = check_ekf_rate_health(
        measured_hz=10.0,
        expected_hz=30.0,
        tolerance_ratio=0.50,
    )

    assert result.ok is True
    assert result.usable is True
    assert result.status == STATUS_WARN
    assert result.reasons


def test_check_ekf_rate_health_errors_when_zero() -> None:
    result = check_ekf_rate_health(
        measured_hz=0.0,
        expected_hz=30.0,
    )

    assert result.ok is False
    assert result.usable is False
    assert result.status == STATUS_ERROR
    assert result.reasons


def test_check_ekf_rate_health_rejects_bad_expected_rate() -> None:
    with pytest.raises(ValueError):
        check_ekf_rate_health(measured_hz=30.0, expected_hz=0.0)


def test_check_ekf_rate_health_rejects_bad_tolerance() -> None:
    with pytest.raises(ValueError):
        check_ekf_rate_health(
            measured_hz=30.0,
            expected_hz=30.0,
            tolerance_ratio=-0.1,
        )

    with pytest.raises(ValueError):
        check_ekf_rate_health(
            measured_hz=30.0,
            expected_hz=30.0,
            tolerance_ratio=1.0,
        )


def test_summarize_ekf_inputs() -> None:
    inputs = [
        make_healthy_input(name="wheel_odom", topic="/wheel/odom"),
        make_healthy_input(name="imu", topic="/imu/data", rate_hz=25.0),
        EkfInputHealth(
            name="vo_odom",
            topic="/vo/odom",
            enabled=False,
            seen=False,
            fresh=False,
            last_age_s=None,
            message_count=0,
            rate_hz=0.0,
            status=STATUS_OK,
            message="disabled",
            reasons=[],
        ),
    ]

    summary = summarize_ekf_inputs(inputs)

    assert summary["active_input_count"] == 2
    assert summary["healthy_input_count"] == 2
    assert summary["inputs"]["wheel_odom"]["topic"] == "/wheel/odom"
    assert summary["inputs"]["imu"]["topic"] == "/imu/data"
    assert summary["inputs"]["vo_odom"]["enabled"] is False


def test_ekf_health_summary() -> None:
    health = EkfHealth(
        inputs=[
            make_healthy_input(name="wheel_odom", topic="/wheel/odom"),
            make_healthy_input(name="imu", topic="/imu/data", rate_hz=25.0),
        ],
        output=make_healthy_output(),
        tf=make_healthy_tf(),
    )

    summary = ekf_health_summary(health)

    assert summary["status"] in (STATUS_OK, STATUS_WARN)
    assert "message" in summary
    assert "ready" in summary
    assert "usable" in summary
    assert "inputs" in summary
    assert "output" in summary
    assert "tf" in summary


def test_robot_savo_baseline_ekf_health_is_ok() -> None:
    health = EkfHealth(
        inputs=[
            make_healthy_input(
                name="wheel_odom",
                topic="/wheel/odom",
                rate_hz=30.0,
            ),
            make_healthy_input(
                name="imu",
                topic="/imu/data",
                rate_hz=25.0,
            ),
        ],
        output=make_healthy_output(),
        tf=make_healthy_tf(),
    )

    result = check_ekf_health(
        health,
        require_tf=True,
        require_output=True,
        min_healthy_inputs=2,
    )

    assert result.ok is True
    assert result.usable is True
    assert result.status == STATUS_OK
    assert result.active_input_count == 2
    assert result.healthy_input_count == 2
    assert result.output_ok is True
    assert result.tf_ok is True


def test_robot_savo_vo_input_can_be_optional() -> None:
    health = EkfHealth(
        inputs=[
            make_healthy_input(
                name="wheel_odom",
                topic="/wheel/odom",
                rate_hz=30.0,
            ),
            make_healthy_input(
                name="imu",
                topic="/imu/data",
                rate_hz=25.0,
            ),
            EkfInputHealth(
                name="vo_odom",
                topic="/vo/odom",
                enabled=False,
                seen=False,
                fresh=False,
                last_age_s=None,
                message_count=0,
                rate_hz=0.0,
                status=STATUS_OK,
                message="VO disabled",
                reasons=[],
            ),
        ],
        output=make_healthy_output(),
        tf=make_healthy_tf(),
    )

    result = check_ekf_health(
        health,
        require_tf=True,
        require_output=True,
        min_healthy_inputs=2,
    )

    assert result.ok is True
    assert result.usable is True
    assert result.status == STATUS_OK
    assert result.active_input_count == 2
    assert result.healthy_input_count == 2