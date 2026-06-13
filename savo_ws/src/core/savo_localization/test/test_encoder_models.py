#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for encoder data models used by Robot Savo localization."""

from __future__ import annotations

import pytest

from savo_localization.constants import (
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
    WHEEL_FL,
    WHEEL_FR,
    WHEEL_ORDER,
    WHEEL_RL,
    WHEEL_RR,
)
from savo_localization.models.encoder_state import (
    EncoderHealthState,
    EncoderSample,
    EncoderState,
    EncoderWheelSnapshot,
    WheelEncoderState,
    make_encoder_sample,
    make_encoder_sample_from_counts,
    make_wheel_snapshot,
)


def test_encoder_wheel_snapshot_defaults() -> None:
    wheel = EncoderWheelSnapshot(name=WHEEL_FL)

    assert wheel.name == WHEEL_FL
    assert wheel.count == 0
    assert wheel.delta_count == 0
    assert wheel.counts_per_second == pytest.approx(0.0)
    assert wheel.speed_mps == pytest.approx(0.0)
    assert wheel.direction == 0
    assert wheel.direction_symbol == "0"
    assert wheel.illegal_transitions == 0
    assert wheel.active is False


def test_encoder_wheel_snapshot_forward_motion() -> None:
    wheel = EncoderWheelSnapshot(
        name=WHEEL_FL,
        count=120,
        delta_count=8,
        counts_per_second=16.0,
        speed_mps=0.04,
        illegal_transitions=0,
    )

    assert wheel.direction == 1
    assert wheel.direction_symbol == "+"
    assert wheel.active is True


def test_encoder_wheel_snapshot_reverse_motion() -> None:
    wheel = EncoderWheelSnapshot(
        name=WHEEL_FR,
        count=-90,
        delta_count=-6,
        counts_per_second=-12.0,
        speed_mps=-0.03,
        illegal_transitions=1,
    )

    assert wheel.direction == -1
    assert wheel.direction_symbol == "-"
    assert wheel.active is True
    assert wheel.illegal_transitions == 1


def test_encoder_wheel_snapshot_to_dict() -> None:
    wheel = EncoderWheelSnapshot(
        name=WHEEL_RL,
        count=50,
        delta_count=5,
        counts_per_second=10.0,
        speed_mps=0.025,
        illegal_transitions=2,
    )

    data = wheel.to_dict()

    assert data["name"] == WHEEL_RL
    assert data["count"] == 50
    assert data["delta_count"] == 5
    assert data["counts_per_second"] == pytest.approx(10.0)
    assert data["speed_mps"] == pytest.approx(0.025)
    assert data["direction"] == 1
    assert data["direction_symbol"] == "+"
    assert data["illegal_transitions"] == 2
    assert data["active"] is True


def test_make_wheel_snapshot() -> None:
    wheel = make_wheel_snapshot(
        name=WHEEL_RR,
        count=80,
        delta_count=10,
        counts_per_second=20.0,
        speed_mps=0.05,
        illegal_transitions=0,
    )

    assert wheel.name == WHEEL_RR
    assert wheel.count == 80
    assert wheel.delta_count == 10
    assert wheel.counts_per_second == pytest.approx(20.0)
    assert wheel.speed_mps == pytest.approx(0.05)
    assert wheel.direction == 1
    assert wheel.active is True


def test_wheel_encoder_state_defaults() -> None:
    state = WheelEncoderState(name=WHEEL_FL)

    assert state.name == WHEEL_FL
    assert state.count == 0
    assert state.previous_count == 0
    assert state.delta_count == 0
    assert state.counts_per_second == pytest.approx(0.0)
    assert state.speed_mps == pytest.approx(0.0)
    assert state.direction == 0
    assert state.active is False
    assert state.illegal_transitions == 0


def test_wheel_encoder_state_mark_count_forward() -> None:
    state = WheelEncoderState(name=WHEEL_FL)

    state.mark_count(
        count=12,
        dt_s=0.5,
        speed_mps=0.06,
        illegal_transitions=0,
    )

    assert state.previous_count == 0
    assert state.count == 12
    assert state.delta_count == 12
    assert state.counts_per_second == pytest.approx(24.0)
    assert state.speed_mps == pytest.approx(0.06)
    assert state.direction == 1
    assert state.direction_symbol == "+"
    assert state.active is True


def test_wheel_encoder_state_mark_count_reverse() -> None:
    state = WheelEncoderState(name=WHEEL_FR, count=20)

    state.mark_count(
        count=10,
        dt_s=0.5,
        speed_mps=-0.05,
        illegal_transitions=1,
    )

    assert state.previous_count == 20
    assert state.count == 10
    assert state.delta_count == -10
    assert state.counts_per_second == pytest.approx(-20.0)
    assert state.speed_mps == pytest.approx(-0.05)
    assert state.direction == -1
    assert state.direction_symbol == "-"
    assert state.active is True
    assert state.illegal_transitions == 1


def test_wheel_encoder_state_snapshot() -> None:
    state = WheelEncoderState(name=WHEEL_RL)
    state.mark_count(
        count=7,
        dt_s=0.25,
        speed_mps=0.02,
        illegal_transitions=0,
    )

    snapshot = state.snapshot()

    assert snapshot.name == WHEEL_RL
    assert snapshot.count == 7
    assert snapshot.delta_count == 7
    assert snapshot.counts_per_second == pytest.approx(28.0)
    assert snapshot.speed_mps == pytest.approx(0.02)
    assert snapshot.active is True


def test_wheel_encoder_state_rejects_bad_dt() -> None:
    state = WheelEncoderState(name=WHEEL_FL)

    with pytest.raises(ValueError):
        state.mark_count(
            count=1,
            dt_s=0.0,
            speed_mps=0.0,
            illegal_transitions=0,
        )


def test_encoder_sample_defaults() -> None:
    sample = EncoderSample()

    assert sample.stamp_s == pytest.approx(0.0)
    assert sample.dt_s == pytest.approx(0.0)
    assert sample.vx_mps == pytest.approx(0.0)
    assert sample.vy_mps == pytest.approx(0.0)
    assert sample.omega_rad_s == pytest.approx(0.0)
    assert sample.active_wheel_count == 0
    assert sample.all_wheels_active is False
    assert sample.total_illegal_transitions == 0


def test_encoder_sample_wheel_order() -> None:
    sample = make_encoder_sample(
        stamp_s=1.0,
        dt_s=0.5,
        wheels=[
            make_wheel_snapshot(name=WHEEL_FL, count=10, delta_count=1),
            make_wheel_snapshot(name=WHEEL_FR, count=11, delta_count=1),
            make_wheel_snapshot(name=WHEEL_RL, count=12, delta_count=1),
            make_wheel_snapshot(name=WHEEL_RR, count=13, delta_count=1),
        ],
        vx_mps=0.1,
        vy_mps=0.0,
        omega_rad_s=0.0,
    )

    assert [wheel.name for wheel in sample.wheels] == list(WHEEL_ORDER)
    assert set(sample.wheel_map.keys()) == set(WHEEL_ORDER)


def test_encoder_sample_active_wheel_count() -> None:
    sample = make_encoder_sample(
        stamp_s=1.0,
        dt_s=0.5,
        wheels=[
            make_wheel_snapshot(name=WHEEL_FL, count=10, delta_count=1),
            make_wheel_snapshot(name=WHEEL_FR, count=10, delta_count=0),
            make_wheel_snapshot(name=WHEEL_RL, count=10, delta_count=-1),
            make_wheel_snapshot(name=WHEEL_RR, count=10, delta_count=0),
        ],
    )

    assert sample.active_wheel_count == 2
    assert sample.all_wheels_active is False


def test_encoder_sample_all_wheels_active() -> None:
    sample = make_encoder_sample(
        stamp_s=1.0,
        dt_s=0.5,
        wheels=[
            make_wheel_snapshot(name=WHEEL_FL, count=10, delta_count=1),
            make_wheel_snapshot(name=WHEEL_FR, count=10, delta_count=1),
            make_wheel_snapshot(name=WHEEL_RL, count=10, delta_count=1),
            make_wheel_snapshot(name=WHEEL_RR, count=10, delta_count=1),
        ],
    )

    assert sample.active_wheel_count == 4
    assert sample.all_wheels_active is True


def test_encoder_sample_total_illegal_transitions() -> None:
    sample = make_encoder_sample(
        stamp_s=1.0,
        dt_s=0.5,
        wheels=[
            make_wheel_snapshot(name=WHEEL_FL, illegal_transitions=1),
            make_wheel_snapshot(name=WHEEL_FR, illegal_transitions=2),
            make_wheel_snapshot(name=WHEEL_RL, illegal_transitions=3),
            make_wheel_snapshot(name=WHEEL_RR, illegal_transitions=4),
        ],
    )

    assert sample.total_illegal_transitions == 10


def test_encoder_sample_linear_speed_magnitude() -> None:
    sample = make_encoder_sample(
        stamp_s=1.0,
        dt_s=0.5,
        wheels=[
            make_wheel_snapshot(name=WHEEL_FL),
            make_wheel_snapshot(name=WHEEL_FR),
            make_wheel_snapshot(name=WHEEL_RL),
            make_wheel_snapshot(name=WHEEL_RR),
        ],
        vx_mps=0.3,
        vy_mps=0.4,
        omega_rad_s=0.0,
    )

    assert sample.linear_speed_mps == pytest.approx(0.5)


def test_encoder_sample_to_dict() -> None:
    sample = make_encoder_sample(
        stamp_s=2.0,
        dt_s=0.1,
        wheels=[
            make_wheel_snapshot(name=WHEEL_FL, count=1, delta_count=1),
            make_wheel_snapshot(name=WHEEL_FR, count=2, delta_count=1),
            make_wheel_snapshot(name=WHEEL_RL, count=3, delta_count=1),
            make_wheel_snapshot(name=WHEEL_RR, count=4, delta_count=1),
        ],
        vx_mps=0.1,
        vy_mps=0.2,
        omega_rad_s=0.3,
    )

    data = sample.to_dict()

    assert data["stamp_s"] == pytest.approx(2.0)
    assert data["dt_s"] == pytest.approx(0.1)
    assert data["vx_mps"] == pytest.approx(0.1)
    assert data["vy_mps"] == pytest.approx(0.2)
    assert data["omega_rad_s"] == pytest.approx(0.3)
    assert data["active_wheel_count"] == 4
    assert data["all_wheels_active"] is True
    assert data["wheels"][WHEEL_FL]["count"] == 1
    assert data["wheels"][WHEEL_RR]["count"] == 4


def test_make_encoder_sample_from_counts_stationary() -> None:
    sample = make_encoder_sample_from_counts(
        stamp_s=1.0,
        dt_s=0.5,
        current_counts={
            WHEEL_FL: 10,
            WHEEL_FR: 20,
            WHEEL_RL: 30,
            WHEEL_RR: 40,
        },
        previous_counts={
            WHEEL_FL: 10,
            WHEEL_FR: 20,
            WHEEL_RL: 30,
            WHEEL_RR: 40,
        },
        wheel_speeds_mps={
            WHEEL_FL: 0.0,
            WHEEL_FR: 0.0,
            WHEEL_RL: 0.0,
            WHEEL_RR: 0.0,
        },
    )

    assert sample.active_wheel_count == 0
    assert sample.all_wheels_active is False
    assert sample.total_illegal_transitions == 0

    for wheel in sample.wheels:
        assert wheel.delta_count == 0
        assert wheel.direction == 0
        assert wheel.active is False


def test_make_encoder_sample_from_counts_forward() -> None:
    sample = make_encoder_sample_from_counts(
        stamp_s=1.0,
        dt_s=0.5,
        current_counts={
            WHEEL_FL: 20,
            WHEEL_FR: 30,
            WHEEL_RL: 40,
            WHEEL_RR: 50,
        },
        previous_counts={
            WHEEL_FL: 10,
            WHEEL_FR: 20,
            WHEEL_RL: 30,
            WHEEL_RR: 40,
        },
        wheel_speeds_mps={
            WHEEL_FL: 0.05,
            WHEEL_FR: 0.05,
            WHEEL_RL: 0.05,
            WHEEL_RR: 0.05,
        },
        vx_mps=0.05,
        vy_mps=0.0,
        omega_rad_s=0.0,
    )

    assert sample.active_wheel_count == 4
    assert sample.all_wheels_active is True
    assert sample.vx_mps == pytest.approx(0.05)

    for wheel in sample.wheels:
        assert wheel.delta_count == 10
        assert wheel.direction == 1
        assert wheel.direction_symbol == "+"
        assert wheel.active is True


def test_encoder_health_state_defaults() -> None:
    health = EncoderHealthState()

    assert health.status == STATUS_UNKNOWN
    assert health.ready is False
    assert health.hardware_ok is False
    assert health.data_ok is False
    assert health.all_wheels_seen is False


def test_encoder_health_state_ready_when_ok() -> None:
    health = EncoderHealthState(
        status=STATUS_OK,
        message="encoders healthy",
        hardware_ok=True,
        data_ok=True,
        all_wheels_seen=True,
        illegal_transition_ok=True,
        rate_ok=True,
        active_wheel_count=4,
        total_illegal_transitions=0,
    )

    assert health.ready is True
    assert health.to_dict()["ready"] is True


def test_encoder_health_state_to_dict() -> None:
    health = EncoderHealthState(
        status=STATUS_OK,
        message="encoders healthy",
        reasons=[],
        hardware_ok=True,
        data_ok=True,
        all_wheels_seen=True,
        illegal_transition_ok=True,
        rate_ok=True,
        active_wheel_count=4,
        total_illegal_transitions=0,
        min_counts_per_second=10.0,
        max_counts_per_second=20.0,
    )

    data = health.to_dict()

    assert data["status"] == STATUS_OK
    assert data["message"] == "encoders healthy"
    assert data["ready"] is True
    assert data["active_wheel_count"] == 4
    assert data["total_illegal_transitions"] == 0
    assert data["min_counts_per_second"] == pytest.approx(10.0)
    assert data["max_counts_per_second"] == pytest.approx(20.0)


def test_encoder_state_defaults() -> None:
    state = EncoderState()

    assert state.ready is False
    assert state.sample_count == 0
    assert state.last_sample is None
    assert state.last_sample_age_s is None
    assert state.health.status == STATUS_UNKNOWN


def test_encoder_state_mark_sample() -> None:
    state = EncoderState()

    sample = make_encoder_sample(
        stamp_s=10.0,
        dt_s=0.5,
        wheels=[
            make_wheel_snapshot(name=WHEEL_FL, count=10, delta_count=1),
            make_wheel_snapshot(name=WHEEL_FR, count=10, delta_count=1),
            make_wheel_snapshot(name=WHEEL_RL, count=10, delta_count=1),
            make_wheel_snapshot(name=WHEEL_RR, count=10, delta_count=1),
        ],
        vx_mps=0.1,
        vy_mps=0.0,
        omega_rad_s=0.0,
    )

    state.mark_sample(sample, now_s=10.2)

    assert state.sample_count == 1
    assert state.last_sample is sample
    assert state.last_sample_age_s == pytest.approx(0.2)
    assert state.ready is True
    assert state.health.active_wheel_count == 4
    assert state.health.total_illegal_transitions == 0


def test_encoder_state_mark_stale() -> None:
    state = EncoderState()

    state.mark_stale(age_s=1.5)

    assert state.ready is False
    assert state.last_sample_age_s == pytest.approx(1.5)
    assert state.health.status == STATUS_STALE


def test_encoder_state_to_dict_contains_core_fields() -> None:
    state = EncoderState()

    sample = make_encoder_sample(
        stamp_s=10.0,
        dt_s=0.5,
        wheels=[
            make_wheel_snapshot(name=WHEEL_FL, count=10, delta_count=1),
            make_wheel_snapshot(name=WHEEL_FR, count=10, delta_count=1),
            make_wheel_snapshot(name=WHEEL_RL, count=10, delta_count=1),
            make_wheel_snapshot(name=WHEEL_RR, count=10, delta_count=1),
        ],
        vx_mps=0.1,
        vy_mps=0.0,
        omega_rad_s=0.0,
    )

    state.mark_sample(sample, now_s=10.0)
    data = state.to_dict()

    assert data["sample_count"] == 1
    assert data["ready"] is True
    assert data["last_sample_age_s"] == pytest.approx(0.0)
    assert "health" in data
    assert "last_sample" in data


def test_robot_savo_forward_encoder_pattern() -> None:
    sample = make_encoder_sample_from_counts(
        stamp_s=1.0,
        dt_s=0.5,
        current_counts={
            WHEEL_FL: 110,
            WHEEL_FR: 110,
            WHEEL_RL: 110,
            WHEEL_RR: 110,
        },
        previous_counts={
            WHEEL_FL: 100,
            WHEEL_FR: 100,
            WHEEL_RL: 100,
            WHEEL_RR: 100,
        },
        wheel_speeds_mps={
            WHEEL_FL: 0.05,
            WHEEL_FR: 0.05,
            WHEEL_RL: 0.05,
            WHEEL_RR: 0.05,
        },
        vx_mps=0.05,
        vy_mps=0.0,
        omega_rad_s=0.0,
    )

    directions = {wheel.name: wheel.direction for wheel in sample.wheels}

    assert directions == {
        WHEEL_FL: 1,
        WHEEL_FR: 1,
        WHEEL_RL: 1,
        WHEEL_RR: 1,
    }


def test_robot_savo_rotate_ccw_encoder_pattern() -> None:
    sample = make_encoder_sample_from_counts(
        stamp_s=1.0,
        dt_s=0.5,
        current_counts={
            WHEEL_FL: 90,
            WHEEL_FR: 110,
            WHEEL_RL: 90,
            WHEEL_RR: 110,
        },
        previous_counts={
            WHEEL_FL: 100,
            WHEEL_FR: 100,
            WHEEL_RL: 100,
            WHEEL_RR: 100,
        },
        wheel_speeds_mps={
            WHEEL_FL: -0.05,
            WHEEL_FR: 0.05,
            WHEEL_RL: -0.05,
            WHEEL_RR: 0.05,
        },
        vx_mps=0.0,
        vy_mps=0.0,
        omega_rad_s=0.5,
    )

    directions = {wheel.name: wheel.direction for wheel in sample.wheels}

    assert directions == {
        WHEEL_FL: -1,
        WHEEL_FR: 1,
        WHEEL_RL: -1,
        WHEEL_RR: 1,
    }
