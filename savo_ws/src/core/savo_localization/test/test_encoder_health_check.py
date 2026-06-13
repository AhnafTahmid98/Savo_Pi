#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for encoder health checks used by Robot Savo localization."""

from __future__ import annotations

import pytest

from savo_localization.constants import (
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_WARN,
    WHEEL_FL,
    WHEEL_FR,
    WHEEL_RL,
    WHEEL_RR,
)
from savo_localization.diagnostics.encoder_health_check import (
    EncoderHealthCheckResult,
    WheelHealthResult,
    check_encoder_sample,
    check_encoder_samples,
    check_encoder_state,
    check_wheel_snapshot,
    encoder_activity_summary,
)
from savo_localization.models.encoder_state import (
    EncoderState,
    make_encoder_sample,
    make_encoder_sample_from_counts,
    make_wheel_snapshot,
)


def make_forward_sample(
    *,
    stamp_s: float = 1.0,
    dt_s: float = 0.5,
    delta_count: int = 10,
    illegal_transitions: int = 0,
):
    previous = {
        WHEEL_FL: 100,
        WHEEL_FR: 100,
        WHEEL_RL: 100,
        WHEEL_RR: 100,
    }

    current = {
        WHEEL_FL: 100 + delta_count,
        WHEEL_FR: 100 + delta_count,
        WHEEL_RL: 100 + delta_count,
        WHEEL_RR: 100 + delta_count,
    }

    illegal = {
        WHEEL_FL: illegal_transitions,
        WHEEL_FR: illegal_transitions,
        WHEEL_RL: illegal_transitions,
        WHEEL_RR: illegal_transitions,
    }

    speeds = {
        WHEEL_FL: 0.05,
        WHEEL_FR: 0.05,
        WHEEL_RL: 0.05,
        WHEEL_RR: 0.05,
    }

    return make_encoder_sample_from_counts(
        stamp_s=stamp_s,
        dt_s=dt_s,
        current_counts=current,
        previous_counts=previous,
        wheel_speeds_mps=speeds,
        illegal_transitions=illegal,
        vx_mps=0.05,
        vy_mps=0.0,
        omega_rad_s=0.0,
    )


def make_stationary_sample(
    *,
    stamp_s: float = 1.0,
    dt_s: float = 0.5,
):
    return make_encoder_sample_from_counts(
        stamp_s=stamp_s,
        dt_s=dt_s,
        current_counts={
            WHEEL_FL: 100,
            WHEEL_FR: 100,
            WHEEL_RL: 100,
            WHEEL_RR: 100,
        },
        previous_counts={
            WHEEL_FL: 100,
            WHEEL_FR: 100,
            WHEEL_RL: 100,
            WHEEL_RR: 100,
        },
        wheel_speeds_mps={
            WHEEL_FL: 0.0,
            WHEEL_FR: 0.0,
            WHEEL_RL: 0.0,
            WHEEL_RR: 0.0,
        },
        vx_mps=0.0,
        vy_mps=0.0,
        omega_rad_s=0.0,
    )


def test_wheel_health_result_to_dict() -> None:
    result = WheelHealthResult(
        name=WHEEL_FL,
        status=STATUS_OK,
        ok=True,
        message="wheel healthy",
        reasons=[],
        count=120,
        delta_count=10,
        counts_per_second=20.0,
        speed_mps=0.05,
        illegal_transitions=0,
        active=True,
    )

    data = result.to_dict()

    assert data["name"] == WHEEL_FL
    assert data["status"] == STATUS_OK
    assert data["ok"] is True
    assert data["message"] == "wheel healthy"
    assert data["reasons"] == []
    assert data["count"] == 120
    assert data["delta_count"] == 10
    assert data["counts_per_second"] == pytest.approx(20.0)
    assert data["speed_mps"] == pytest.approx(0.05)
    assert data["illegal_transitions"] == 0
    assert data["active"] is True


def test_encoder_health_check_result_to_dict() -> None:
    result = EncoderHealthCheckResult(
        status=STATUS_OK,
        ok=True,
        message="encoders healthy",
        reasons=[],
        wheel_results={
            WHEEL_FL: WheelHealthResult(
                name=WHEEL_FL,
                status=STATUS_OK,
                ok=True,
                message="wheel healthy",
                active=True,
            )
        },
    )

    data = result.to_dict()

    assert data["status"] == STATUS_OK
    assert data["ok"] is True
    assert data["message"] == "encoders healthy"
    assert data["reasons"] == []
    assert data["active_wheel_count"] == 1
    assert data["checked_wheel_count"] == 1
    assert data["wheel_results"][WHEEL_FL]["ok"] is True


def test_check_wheel_snapshot_active_wheel_is_healthy() -> None:
    wheel = make_wheel_snapshot(
        name=WHEEL_FL,
        count=120,
        delta_count=10,
        counts_per_second=20.0,
        speed_mps=0.05,
        illegal_transitions=0,
    )

    result = check_wheel_snapshot(wheel)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.name == WHEEL_FL
    assert result.active is True
    assert result.illegal_transitions == 0
    assert result.reasons == []


def test_check_wheel_snapshot_stationary_wheel_is_usable() -> None:
    wheel = make_wheel_snapshot(
        name=WHEEL_FR,
        count=120,
        delta_count=0,
        counts_per_second=0.0,
        speed_mps=0.0,
        illegal_transitions=0,
    )

    result = check_wheel_snapshot(wheel)

    assert result.ok is True
    assert result.status in (STATUS_OK, STATUS_WARN)
    assert result.name == WHEEL_FR
    assert result.active is False


def test_check_wheel_snapshot_warns_on_illegal_transitions() -> None:
    wheel = make_wheel_snapshot(
        name=WHEEL_RL,
        count=120,
        delta_count=10,
        counts_per_second=20.0,
        speed_mps=0.05,
        illegal_transitions=50,
    )

    result = check_wheel_snapshot(
        wheel,
        max_illegal_transitions=20,
    )

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert result.reasons
    assert any("illegal" in reason.lower() for reason in result.reasons)


def test_check_encoder_sample_forward_motion_healthy() -> None:
    sample = make_forward_sample()

    result = check_encoder_sample(sample)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.active_wheel_count == 4
    assert result.checked_wheel_count == 4
    assert result.reasons == []


def test_check_encoder_sample_stationary_usable() -> None:
    sample = make_stationary_sample()

    result = check_encoder_sample(sample)

    assert result.ok is True
    assert result.status in (STATUS_OK, STATUS_WARN)
    assert result.checked_wheel_count == 4
    assert result.active_wheel_count == 0


def test_check_encoder_sample_requires_active_wheels_when_requested() -> None:
    sample = make_stationary_sample()

    result = check_encoder_sample(
        sample,
        min_active_wheels=4,
    )

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert result.active_wheel_count == 0
    assert any("active" in reason.lower() or "wheel" in reason.lower() for reason in result.reasons)


def test_check_encoder_sample_warns_on_illegal_transitions() -> None:
    sample = make_forward_sample(illegal_transitions=25)

    result = check_encoder_sample(
        sample,
        max_illegal_transitions=20,
    )

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert result.total_illegal_transitions >= 100
    assert result.reasons


def test_check_encoder_sample_missing_wheel_is_error() -> None:
    sample = make_encoder_sample(
        stamp_s=1.0,
        dt_s=0.5,
        wheels=[
            make_wheel_snapshot(name=WHEEL_FL, count=10, delta_count=1),
            make_wheel_snapshot(name=WHEEL_FR, count=10, delta_count=1),
            make_wheel_snapshot(name=WHEEL_RL, count=10, delta_count=1),
        ],
        vx_mps=0.1,
        vy_mps=0.0,
        omega_rad_s=0.0,
    )

    result = check_encoder_sample(sample)

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert result.checked_wheel_count == 3
    assert result.reasons


def test_check_encoder_samples_empty_list_is_error() -> None:
    result = check_encoder_samples([])

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert result.reasons


def test_check_encoder_samples_healthy_window() -> None:
    samples = [
        make_forward_sample(stamp_s=1.0),
        make_forward_sample(stamp_s=1.5),
        make_forward_sample(stamp_s=2.0),
    ]

    result = check_encoder_samples(samples)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.checked_wheel_count == 4
    assert result.active_wheel_count == 4


def test_check_encoder_samples_uses_latest_sample() -> None:
    first = make_stationary_sample(stamp_s=1.0)
    second = make_forward_sample(stamp_s=2.0)

    result = check_encoder_samples([first, second])

    assert result.ok is True
    assert result.active_wheel_count == 4


def test_check_encoder_samples_reports_latest_failure() -> None:
    first = make_forward_sample(stamp_s=1.0)
    second = make_forward_sample(stamp_s=2.0, illegal_transitions=30)

    result = check_encoder_samples(
        [first, second],
        max_illegal_transitions=20,
    )

    assert result.ok is True
    assert result.status == STATUS_WARN
    assert result.reasons


def test_check_encoder_state_without_sample_is_error() -> None:
    state = EncoderState()

    result = check_encoder_state(state)

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert result.reasons


def test_check_encoder_state_with_sample_is_healthy() -> None:
    state = EncoderState()
    sample = make_forward_sample(stamp_s=10.0)

    state.mark_sample(sample, now_s=10.1)

    result = check_encoder_state(state)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.active_wheel_count == 4


def test_check_encoder_state_stale_is_stale() -> None:
    state = EncoderState()
    sample = make_forward_sample(stamp_s=10.0)

    state.mark_sample(sample, now_s=10.0)
    state.mark_stale(age_s=1.5)

    result = check_encoder_state(
        state,
        stale_timeout_s=0.5,
    )

    assert result.ok is False
    assert result.status == STATUS_STALE
    assert result.reasons


def test_encoder_activity_summary_forward_sample() -> None:
    sample = make_forward_sample()

    summary = encoder_activity_summary(sample)

    assert summary["active_wheel_count"] == 4
    assert summary["all_wheels_active"] is True
    assert summary["total_illegal_transitions"] == 0
    assert summary["wheel_directions"] == {
        WHEEL_FL: 1,
        WHEEL_FR: 1,
        WHEEL_RL: 1,
        WHEEL_RR: 1,
    }


def test_encoder_activity_summary_stationary_sample() -> None:
    sample = make_stationary_sample()

    summary = encoder_activity_summary(sample)

    assert summary["active_wheel_count"] == 0
    assert summary["all_wheels_active"] is False
    assert summary["total_illegal_transitions"] == 0
    assert summary["wheel_directions"] == {
        WHEEL_FL: 0,
        WHEEL_FR: 0,
        WHEEL_RL: 0,
        WHEEL_RR: 0,
    }


def test_robot_savo_forward_encoder_health_is_ok() -> None:
    sample = make_forward_sample(delta_count=12)

    result = check_encoder_sample(
        sample,
        min_active_wheels=4,
        max_illegal_transitions=20,
    )

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.active_wheel_count == 4

    for wheel_result in result.wheel_results.values():
        assert wheel_result.ok is True
        assert wheel_result.active is True


def test_robot_savo_one_dead_wheel_is_error_when_motion_expected() -> None:
    sample = make_encoder_sample_from_counts(
        stamp_s=1.0,
        dt_s=0.5,
        current_counts={
            WHEEL_FL: 110,
            WHEEL_FR: 110,
            WHEEL_RL: 110,
            WHEEL_RR: 100,
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
            WHEEL_RR: 0.0,
        },
        vx_mps=0.05,
        vy_mps=0.0,
        omega_rad_s=0.0,
    )

    result = check_encoder_sample(
        sample,
        min_active_wheels=4,
    )

    assert result.ok is False
    assert result.status == STATUS_ERROR
    assert result.active_wheel_count == 3
    assert WHEEL_RR in result.wheel_results
    assert result.reasons