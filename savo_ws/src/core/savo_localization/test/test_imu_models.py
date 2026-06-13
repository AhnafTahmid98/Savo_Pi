#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for IMU data models used by Robot Savo localization."""

from __future__ import annotations

import math

import pytest

from savo_localization.constants import (
    GRAVITY_MPS2,
    IMU_HEALTH_GRADE_A,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
)
from savo_localization.models.imu_state import (
    ImuCalibration,
    ImuEuler,
    ImuHealthState,
    ImuRawSample,
    ImuState,
    ImuVector3,
    imu_is_moving,
    make_imu_calibration,
    make_imu_sample,
    make_imu_vector3,
)


def test_imu_vector3_defaults() -> None:
    vector = ImuVector3()

    assert vector.x == pytest.approx(0.0)
    assert vector.y == pytest.approx(0.0)
    assert vector.z == pytest.approx(0.0)


def test_make_imu_vector3() -> None:
    vector = make_imu_vector3(1.0, -2.0, 3.5)

    assert vector.x == pytest.approx(1.0)
    assert vector.y == pytest.approx(-2.0)
    assert vector.z == pytest.approx(3.5)


def test_imu_vector3_norm() -> None:
    vector = ImuVector3(x=3.0, y=4.0, z=12.0)

    assert vector.norm == pytest.approx(13.0)


def test_imu_vector3_to_dict() -> None:
    vector = ImuVector3(x=1.0, y=2.0, z=3.0)

    assert vector.to_dict() == {
        "x": 1.0,
        "y": 2.0,
        "z": 3.0,
        "norm": pytest.approx(math.sqrt(14.0)),
    }


def test_imu_euler_defaults() -> None:
    euler = ImuEuler()

    assert euler.yaw_deg == pytest.approx(0.0)
    assert euler.roll_deg == pytest.approx(0.0)
    assert euler.pitch_deg == pytest.approx(0.0)
    assert euler.available is False


def test_imu_euler_to_dict() -> None:
    euler = ImuEuler(
        yaw_deg=90.0,
        roll_deg=1.5,
        pitch_deg=-2.0,
        available=True,
    )

    data = euler.to_dict()

    assert data["yaw_deg"] == pytest.approx(90.0)
    assert data["roll_deg"] == pytest.approx(1.5)
    assert data["pitch_deg"] == pytest.approx(-2.0)
    assert data["available"] is True


def test_imu_calibration_defaults() -> None:
    calibration = ImuCalibration()

    assert calibration.system == 0
    assert calibration.gyro == 0
    assert calibration.accel == 0
    assert calibration.mag == 0
    assert calibration.motion_ready is False
    assert calibration.fully_calibrated is False


def test_make_imu_calibration() -> None:
    calibration = make_imu_calibration(
        system=1,
        gyro=2,
        accel=3,
        mag=0,
    )

    assert calibration.system == 1
    assert calibration.gyro == 2
    assert calibration.accel == 3
    assert calibration.mag == 0


def test_imu_calibration_motion_ready() -> None:
    calibration = ImuCalibration(system=0, gyro=2, accel=2, mag=0)

    assert calibration.motion_ready is True
    assert calibration.fully_calibrated is False


def test_imu_calibration_fully_calibrated() -> None:
    calibration = ImuCalibration(system=3, gyro=3, accel=3, mag=3)

    assert calibration.motion_ready is True
    assert calibration.fully_calibrated is True


@pytest.mark.parametrize(
    ("system", "gyro", "accel", "mag"),
    [
        (-1, 0, 0, 0),
        (0, -1, 0, 0),
        (0, 0, -1, 0),
        (0, 0, 0, -1),
        (4, 0, 0, 0),
        (0, 4, 0, 0),
        (0, 0, 4, 0),
        (0, 0, 0, 4),
    ],
)
def test_imu_calibration_rejects_invalid_levels(
    system: int,
    gyro: int,
    accel: int,
    mag: int,
) -> None:
    calibration = ImuCalibration(
        system=system,
        gyro=gyro,
        accel=accel,
        mag=mag,
    )

    with pytest.raises(ValueError):
        calibration.validate()


def test_imu_calibration_to_dict() -> None:
    calibration = ImuCalibration(system=1, gyro=2, accel=3, mag=0)

    assert calibration.to_dict() == {
        "system": 1,
        "gyro": 2,
        "accel": 3,
        "mag": 0,
        "motion_ready": True,
        "fully_calibrated": False,
    }


def test_imu_calibration_to_tuple_named() -> None:
    calibration = ImuCalibration(system=1, gyro=2, accel=3, mag=0)

    assert calibration.to_tuple_named() == {
        "system": 1,
        "gyro": 2,
        "accel": 3,
        "mag": 0,
    }


def test_imu_health_state_defaults() -> None:
    health = ImuHealthState()

    assert health.status == STATUS_UNKNOWN
    assert health.grade is not None
    assert health.hardware_ok is False
    assert health.data_ok is False
    assert health.ready is False


def test_imu_health_state_ready_when_ok() -> None:
    health = ImuHealthState(
        status=STATUS_OK,
        grade=IMU_HEALTH_GRADE_A,
        message="IMU healthy",
        hardware_ok=True,
        data_ok=True,
        temperature_ok=True,
    )

    assert health.ready is True
    assert health.to_dict()["ready"] is True


def test_imu_raw_sample_defaults() -> None:
    sample = ImuRawSample()

    assert sample.accel_mps2.x == pytest.approx(0.0)
    assert sample.gyro_dps.z == pytest.approx(0.0)
    assert sample.gravity_norm_mps2 == pytest.approx(0.0)
    assert sample.sys_status == 0
    assert sample.sys_error == 0
    assert sample.moving is False


def test_make_imu_sample_stationary() -> None:
    sample = make_imu_sample(
        stamp_s=10.0,
        accel_mps2=ImuVector3(x=0.0, y=0.0, z=GRAVITY_MPS2),
        gyro_dps=ImuVector3(x=0.0, y=0.0, z=0.0),
        calibration=ImuCalibration(system=3, gyro=3, accel=3, mag=3),
        temp_c=35.0,
        sys_status=5,
        sys_error=0,
    )

    assert sample.stamp_s == pytest.approx(10.0)
    assert sample.gravity_norm_mps2 == pytest.approx(GRAVITY_MPS2)
    assert sample.temp_c == pytest.approx(35.0)
    assert sample.sys_status == 5
    assert sample.sys_error == 0
    assert sample.calibration.fully_calibrated is True
    assert sample.moving is False


def test_make_imu_sample_detects_motion_from_gyro() -> None:
    sample = make_imu_sample(
        stamp_s=10.0,
        accel_mps2=ImuVector3(x=0.0, y=0.0, z=GRAVITY_MPS2),
        gyro_dps=ImuVector3(x=0.0, y=0.0, z=20.0),
        calibration=ImuCalibration(system=3, gyro=3, accel=3, mag=3),
    )

    assert sample.moving is True


def test_imu_is_moving_false_for_stationary_values() -> None:
    assert not imu_is_moving(
        gyro_dps=ImuVector3(x=0.0, y=0.0, z=0.2),
        accel_mps2=ImuVector3(x=0.0, y=0.0, z=GRAVITY_MPS2),
    )


def test_imu_is_moving_true_for_large_gyro() -> None:
    assert imu_is_moving(
        gyro_dps=ImuVector3(x=0.0, y=0.0, z=15.0),
        accel_mps2=ImuVector3(x=0.0, y=0.0, z=GRAVITY_MPS2),
    )


def test_imu_is_moving_true_for_accel_change() -> None:
    assert imu_is_moving(
        gyro_dps=ImuVector3(x=0.0, y=0.0, z=0.0),
        accel_mps2=ImuVector3(x=2.0, y=0.0, z=GRAVITY_MPS2),
    )


def test_imu_raw_sample_to_dict() -> None:
    sample = make_imu_sample(
        stamp_s=1.5,
        accel_mps2=ImuVector3(x=0.1, y=0.2, z=9.8),
        gyro_dps=ImuVector3(x=0.3, y=0.4, z=0.5),
        mag_ut=ImuVector3(x=1.0, y=2.0, z=3.0),
        euler_deg=ImuEuler(yaw_deg=90.0, roll_deg=0.0, pitch_deg=0.0, available=True),
        calibration=ImuCalibration(system=1, gyro=2, accel=3, mag=0),
        temp_c=32.0,
        sys_status=5,
        sys_error=0,
    )

    data = sample.to_dict()

    assert data["stamp_s"] == pytest.approx(1.5)
    assert data["accel_mps2"]["x"] == pytest.approx(0.1)
    assert data["gyro_dps"]["z"] == pytest.approx(0.5)
    assert data["mag_ut"]["y"] == pytest.approx(2.0)
    assert data["euler_deg"]["yaw_deg"] == pytest.approx(90.0)
    assert data["calibration"]["gyro"] == 2
    assert data["temp_c"] == pytest.approx(32.0)
    assert data["sys_status"] == 5
    assert data["sys_error"] == 0


def test_imu_state_defaults() -> None:
    state = ImuState()

    assert state.ready is False
    assert state.sample_count == 0
    assert state.last_sample is None
    assert state.health.status == STATUS_UNKNOWN


def test_imu_state_mark_sample() -> None:
    state = ImuState()

    sample = make_imu_sample(
        stamp_s=10.0,
        accel_mps2=ImuVector3(x=0.0, y=0.0, z=GRAVITY_MPS2),
        gyro_dps=ImuVector3(x=0.0, y=0.0, z=0.0),
        calibration=ImuCalibration(system=3, gyro=3, accel=3, mag=3),
        temp_c=35.0,
        sys_status=5,
        sys_error=0,
    )

    state.mark_sample(sample, now_s=10.1)

    assert state.sample_count == 1
    assert state.last_sample is sample
    assert state.last_sample_age_s == pytest.approx(0.1)
    assert state.chip_ok is True
    assert state.sys_status == 5
    assert state.sys_error == 0
    assert state.calibration.fully_calibrated is True


def test_imu_state_mark_stale() -> None:
    state = ImuState()

    state.mark_stale(age_s=1.25)

    assert state.ready is False
    assert state.health.status == STATUS_STALE
    assert state.last_sample_age_s == pytest.approx(1.25)


def test_imu_state_to_dict_contains_core_fields() -> None:
    state = ImuState()

    sample = make_imu_sample(
        stamp_s=10.0,
        accel_mps2=ImuVector3(x=0.0, y=0.0, z=GRAVITY_MPS2),
        gyro_dps=ImuVector3(x=0.0, y=0.0, z=0.0),
        calibration=ImuCalibration(system=3, gyro=3, accel=3, mag=3),
        temp_c=35.0,
        sys_status=5,
        sys_error=0,
    )

    state.mark_sample(sample, now_s=10.0)
    data = state.to_dict()

    assert data["sample_count"] == 1
    assert data["chip_ok"] is True
    assert data["sys_status"] == 5
    assert data["sys_error"] == 0
    assert data["calibration"]["fully_calibrated"] is True
    assert "health" in data
    assert "last_sample" in data


def test_robot_savo_stationary_imu_sample_is_reasonable() -> None:
    sample = make_imu_sample(
        stamp_s=1.0,
        accel_mps2=ImuVector3(x=0.0, y=0.0, z=GRAVITY_MPS2),
        gyro_dps=ImuVector3(x=0.0, y=0.0, z=0.0),
        calibration=ImuCalibration(system=3, gyro=3, accel=3, mag=3),
        temp_c=35.0,
        sys_status=5,
        sys_error=0,
    )

    assert sample.gravity_norm_mps2 == pytest.approx(GRAVITY_MPS2)
    assert sample.moving is False
    assert sample.vibe_score == pytest.approx(0.0)
    assert sample.calibration.motion_ready is True