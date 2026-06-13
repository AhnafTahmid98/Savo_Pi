#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for IMU health checks used by Robot Savo localization."""

from __future__ import annotations

import pytest

from savo_localization.constants import (
    GRAVITY_MPS2,
    IMU_HEALTH_GRADE_A,
    IMU_HEALTH_GRADE_B,
    IMU_HEALTH_GRADE_C,
    IMU_HEALTH_GRADE_F,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_WARN,
)
from savo_localization.diagnostics.imu_health_check import (
    ImuHealthCheckResult,
    ImuStats,
    check_imu_sample,
    check_imu_samples,
    compute_imu_stats,
    grade_from_reasons,
    status_from_grade,
)
from savo_localization.models.imu_state import (
    ImuCalibration,
    ImuVector3,
    make_imu_sample,
)


def make_stationary_sample(
    *,
    stamp_s: float = 1.0,
    accel_z: float = GRAVITY_MPS2,
    gyro_z: float = 0.0,
    temp_c: float = 35.0,
    sys_status: int = 5,
    sys_error: int = 0,
    calibration: ImuCalibration | None = None,
):
    return make_imu_sample(
        stamp_s=stamp_s,
        accel_mps2=ImuVector3(x=0.0, y=0.0, z=accel_z),
        gyro_dps=ImuVector3(x=0.0, y=0.0, z=gyro_z),
        calibration=calibration or ImuCalibration(system=3, gyro=3, accel=3, mag=3),
        temp_c=temp_c,
        sys_status=sys_status,
        sys_error=sys_error,
    )


def test_imu_stats_defaults() -> None:
    stats = ImuStats()

    assert stats.sample_count == 0
    assert stats.accel_norm_mean_mps2 == pytest.approx(0.0)
    assert stats.accel_norm_std_mps2 == pytest.approx(0.0)
    assert stats.gyro_norm_mean_dps == pytest.approx(0.0)
    assert stats.gyro_norm_std_dps == pytest.approx(0.0)
    assert stats.temperature_mean_c is None


def test_imu_stats_to_dict() -> None:
    stats = ImuStats(
        sample_count=3,
        accel_norm_mean_mps2=GRAVITY_MPS2,
        accel_norm_std_mps2=0.05,
        gyro_norm_mean_dps=0.1,
        gyro_norm_std_dps=0.02,
        temperature_mean_c=35.0,
    )

    data = stats.to_dict()

    assert data["sample_count"] == 3
    assert data["accel_norm_mean_mps2"] == pytest.approx(GRAVITY_MPS2)
    assert data["accel_norm_std_mps2"] == pytest.approx(0.05)
    assert data["gyro_norm_mean_dps"] == pytest.approx(0.1)
    assert data["gyro_norm_std_dps"] == pytest.approx(0.02)
    assert data["temperature_mean_c"] == pytest.approx(35.0)


def test_compute_imu_stats_empty_samples() -> None:
    stats = compute_imu_stats([])

    assert stats.sample_count == 0
    assert stats.accel_norm_mean_mps2 == pytest.approx(0.0)
    assert stats.gyro_norm_mean_dps == pytest.approx(0.0)


def test_compute_imu_stats_single_sample() -> None:
    sample = make_stationary_sample(temp_c=34.0)

    stats = compute_imu_stats([sample])

    assert stats.sample_count == 1
    assert stats.accel_norm_mean_mps2 == pytest.approx(GRAVITY_MPS2)
    assert stats.accel_norm_std_mps2 == pytest.approx(0.0)
    assert stats.gyro_norm_mean_dps == pytest.approx(0.0)
    assert stats.gyro_norm_std_dps == pytest.approx(0.0)
    assert stats.temperature_mean_c == pytest.approx(34.0)


def test_compute_imu_stats_multiple_samples() -> None:
    samples = [
        make_stationary_sample(stamp_s=1.0, accel_z=9.70, gyro_z=0.0, temp_c=34.0),
        make_stationary_sample(stamp_s=2.0, accel_z=9.80, gyro_z=0.2, temp_c=35.0),
        make_stationary_sample(stamp_s=3.0, accel_z=9.90, gyro_z=0.4, temp_c=36.0),
    ]

    stats = compute_imu_stats(samples)

    assert stats.sample_count == 3
    assert stats.accel_norm_mean_mps2 == pytest.approx(9.80)
    assert stats.gyro_norm_mean_dps == pytest.approx(0.2)
    assert stats.temperature_mean_c == pytest.approx(35.0)


def test_grade_from_reasons_no_reasons() -> None:
    assert grade_from_reasons([]) == IMU_HEALTH_GRADE_A


def test_grade_from_reasons_warning_count() -> None:
    assert grade_from_reasons(["calibration not fully ready"]) in (
        IMU_HEALTH_GRADE_B,
        IMU_HEALTH_GRADE_C,
    )


def test_grade_from_reasons_many_reasons_is_failure_grade() -> None:
    grade = grade_from_reasons(
        [
            "chip check failed",
            "system error",
            "acceleration outside range",
            "gyro noise high",
        ]
    )

    assert grade == IMU_HEALTH_GRADE_F


def test_status_from_grade_ok() -> None:
    assert status_from_grade(IMU_HEALTH_GRADE_A) == STATUS_OK


def test_status_from_grade_warn() -> None:
    assert status_from_grade(IMU_HEALTH_GRADE_B) == STATUS_WARN
    assert status_from_grade(IMU_HEALTH_GRADE_C) == STATUS_WARN


def test_status_from_grade_error() -> None:
    assert status_from_grade(IMU_HEALTH_GRADE_F) == STATUS_ERROR


def test_imu_health_check_result_to_dict() -> None:
    result = ImuHealthCheckResult(
        status=STATUS_OK,
        ok=True,
        grade=IMU_HEALTH_GRADE_A,
        message="IMU healthy",
        reasons=[],
        stats=ImuStats(sample_count=1),
    )

    data = result.to_dict()

    assert data["status"] == STATUS_OK
    assert data["ok"] is True
    assert data["grade"] == IMU_HEALTH_GRADE_A
    assert data["message"] == "IMU healthy"
    assert data["reasons"] == []
    assert data["stats"]["sample_count"] == 1


def test_check_imu_sample_healthy_stationary_sample() -> None:
    sample = make_stationary_sample()

    result = check_imu_sample(sample)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.grade == IMU_HEALTH_GRADE_A
    assert result.stats.sample_count == 1
    assert result.reasons == []


def test_check_imu_sample_warns_when_calibration_not_motion_ready() -> None:
    sample = make_stationary_sample(
        calibration=ImuCalibration(system=0, gyro=1, accel=1, mag=0),
    )

    result = check_imu_sample(sample)

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert result.grade in (IMU_HEALTH_GRADE_B, IMU_HEALTH_GRADE_C)
    assert result.reasons


def test_check_imu_sample_errors_on_system_error() -> None:
    sample = make_stationary_sample(sys_error=1)

    result = check_imu_sample(sample)

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert result.grade == IMU_HEALTH_GRADE_F
    assert any("system error" in reason.lower() for reason in result.reasons)


def test_check_imu_sample_warns_on_bad_gravity_norm() -> None:
    sample = make_stationary_sample(accel_z=3.0)

    result = check_imu_sample(sample)

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert any("accel" in reason.lower() or "gravity" in reason.lower() for reason in result.reasons)


def test_check_imu_sample_warns_on_high_temperature() -> None:
    sample = make_stationary_sample(temp_c=90.0)

    result = check_imu_sample(sample)

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert any("temperature" in reason.lower() for reason in result.reasons)


def test_check_imu_samples_empty_list_is_error() -> None:
    result = check_imu_samples([])

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert result.grade == IMU_HEALTH_GRADE_F
    assert result.reasons


def test_check_imu_samples_healthy_window() -> None:
    samples = [
        make_stationary_sample(stamp_s=1.0, accel_z=9.78, gyro_z=0.0),
        make_stationary_sample(stamp_s=2.0, accel_z=9.81, gyro_z=0.1),
        make_stationary_sample(stamp_s=3.0, accel_z=9.83, gyro_z=-0.1),
    ]

    result = check_imu_samples(samples)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.grade == IMU_HEALTH_GRADE_A
    assert result.stats.sample_count == 3


def test_check_imu_samples_warns_on_noisy_acceleration_window() -> None:
    samples = [
        make_stationary_sample(stamp_s=1.0, accel_z=8.0),
        make_stationary_sample(stamp_s=2.0, accel_z=9.8),
        make_stationary_sample(stamp_s=3.0, accel_z=12.0),
    ]

    result = check_imu_samples(samples)

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert result.reasons


def test_check_imu_samples_warns_on_noisy_gyro_window() -> None:
    samples = [
        make_stationary_sample(stamp_s=1.0, gyro_z=0.0),
        make_stationary_sample(stamp_s=2.0, gyro_z=15.0),
        make_stationary_sample(stamp_s=3.0, gyro_z=-15.0),
    ]

    result = check_imu_samples(samples)

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert result.reasons


def test_robot_savo_stationary_imu_window_is_healthy() -> None:
    samples = [
        make_stationary_sample(stamp_s=float(index), accel_z=GRAVITY_MPS2, gyro_z=0.0)
        for index in range(1, 6)
    ]

    result = check_imu_samples(samples)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.grade == IMU_HEALTH_GRADE_A
    assert result.stats.sample_count == 5


def test_robot_savo_motion_ready_but_mag_not_ready_is_usable() -> None:
    samples = [
        make_stationary_sample(
            stamp_s=1.0,
            calibration=ImuCalibration(system=0, gyro=3, accel=3, mag=0),
        ),
        make_stationary_sample(
            stamp_s=2.0,
            calibration=ImuCalibration(system=0, gyro=3, accel=3, mag=0),
        ),
    ]

    result = check_imu_samples(samples)

    assert result.ok is True
    assert result.status in (STATUS_OK, STATUS_WARN)
    assert result.grade in (IMU_HEALTH_GRADE_A, IMU_HEALTH_GRADE_B, IMU_HEALTH_GRADE_C)
