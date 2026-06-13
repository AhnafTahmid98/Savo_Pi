#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Encoder health checks for Robot Savo four-wheel odometry. No ROS imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Iterable

from savo_localization.constants import (
    DEFAULT_MAX_ILLEGAL_TRANSITIONS,
    DEFAULT_MIN_VALID_WHEEL_COUNT,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_WARN,
    WHEEL_ORDER,
)
from savo_localization.models.encoder_state import (
    EncoderHealthState,
    EncoderSample,
    EncoderState,
    EncoderWheelSnapshot,
)


@dataclass(frozen=True)
class WheelHealthResult:
    name: str
    ok: bool
    active: bool
    count: int
    delta_count: int
    counts_per_second: float
    speed_mps: float
    illegal_transitions: int
    reasons: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class EncoderHealthCheckResult:
    status: str
    ok: bool
    message: str
    reasons: list[str] = field(default_factory=list)

    active_wheel_count: int = 0
    total_illegal_transitions: int = 0
    wheel_results: dict[str, WheelHealthResult] = field(default_factory=dict)

    def to_dict(self) -> dict[str, object]:
        return {
            "status": self.status,
            "ok": bool(self.ok),
            "message": self.message,
            "reasons": list(self.reasons),
            "active_wheel_count": int(self.active_wheel_count),
            "total_illegal_transitions": int(self.total_illegal_transitions),
            "wheel_results": {
                name: result.to_dict()
                for name, result in self.wheel_results.items()
            },
        }

    def to_health_state(self) -> EncoderHealthState:
        health = EncoderHealthState(
            status=self.status,
            message=self.message,
            reasons=list(self.reasons),
            hardware_ok=self.status in (STATUS_OK, STATUS_WARN),
            data_ok=self.status in (STATUS_OK, STATUS_WARN),
            all_wheels_seen=len(self.wheel_results) == len(WHEEL_ORDER),
            illegal_transition_ok=self.total_illegal_transitions
            <= DEFAULT_MAX_ILLEGAL_TRANSITIONS,
            rate_ok=True,
            active_wheel_count=self.active_wheel_count,
            total_illegal_transitions=self.total_illegal_transitions,
        )

        rates = [
            abs(result.counts_per_second)
            for result in self.wheel_results.values()
        ]

        if rates:
            health.min_counts_per_second = min(rates)
            health.max_counts_per_second = max(rates)

        return health


def check_encoder_sample(
    sample: EncoderSample,
    *,
    expect_motion: bool = False,
    min_active_wheels: int = DEFAULT_MIN_VALID_WHEEL_COUNT,
    max_illegal_transitions: int = DEFAULT_MAX_ILLEGAL_TRANSITIONS,
) -> EncoderHealthCheckResult:
    wheel_results = {
        wheel.name: check_wheel_snapshot(
            wheel,
            max_illegal_transitions=max_illegal_transitions,
        )
        for wheel in sample.wheels
    }

    reasons: list[str] = []

    missing_wheels = [
        wheel_name for wheel_name in WHEEL_ORDER if wheel_name not in wheel_results
    ]
    if missing_wheels:
        reasons.append(f"missing wheel encoder states: {', '.join(missing_wheels)}")

    total_illegal = sum(
        result.illegal_transitions for result in wheel_results.values()
    )
    if total_illegal > max_illegal_transitions:
        reasons.append(
            "too many illegal encoder transitions: "
            f"{total_illegal} > {max_illegal_transitions}"
        )

    active_count = sum(1 for result in wheel_results.values() if result.active)
    if expect_motion and active_count < min_active_wheels:
        reasons.append(
            "not enough active wheels during expected motion: "
            f"{active_count}/{min_active_wheels}"
        )

    for result in wheel_results.values():
        reasons.extend(
            f"{result.name}: {reason}" for reason in result.reasons
        )

    if not wheel_results:
        return EncoderHealthCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="no encoder wheel states available",
            reasons=["sample contains no wheel snapshots"],
            active_wheel_count=0,
            total_illegal_transitions=0,
            wheel_results={},
        )

    if total_illegal > max_illegal_transitions:
        return EncoderHealthCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="encoder health check failed",
            reasons=reasons,
            active_wheel_count=active_count,
            total_illegal_transitions=total_illegal,
            wheel_results=wheel_results,
        )

    if reasons:
        return EncoderHealthCheckResult(
            status=STATUS_WARN,
            ok=True,
            message="encoders usable with health notes",
            reasons=reasons,
            active_wheel_count=active_count,
            total_illegal_transitions=total_illegal,
            wheel_results=wheel_results,
        )

    return EncoderHealthCheckResult(
        status=STATUS_OK,
        ok=True,
        message="encoders healthy",
        reasons=[],
        active_wheel_count=active_count,
        total_illegal_transitions=total_illegal,
        wheel_results=wheel_results,
    )


def check_encoder_state(
    state: EncoderState,
    *,
    expect_motion: bool = False,
    min_active_wheels: int = DEFAULT_MIN_VALID_WHEEL_COUNT,
    max_illegal_transitions: int = DEFAULT_MAX_ILLEGAL_TRANSITIONS,
) -> EncoderHealthCheckResult:
    if state.last_sample is None:
        return EncoderHealthCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="no encoder sample available",
            reasons=["EncoderState.last_sample is None"],
        )

    return check_encoder_sample(
        state.last_sample,
        expect_motion=expect_motion,
        min_active_wheels=min_active_wheels,
        max_illegal_transitions=max_illegal_transitions,
    )


def check_wheel_snapshot(
    wheel: EncoderWheelSnapshot,
    *,
    max_illegal_transitions: int = DEFAULT_MAX_ILLEGAL_TRANSITIONS,
) -> WheelHealthResult:
    reasons: list[str] = []

    if wheel.name not in WHEEL_ORDER:
        reasons.append(f"unsupported wheel name: {wheel.name}")

    if wheel.illegal_transitions > max_illegal_transitions:
        reasons.append(
            "illegal transitions above limit: "
            f"{wheel.illegal_transitions} > {max_illegal_transitions}"
        )

    return WheelHealthResult(
        name=wheel.name,
        ok=not reasons,
        active=bool(wheel.active),
        count=int(wheel.count),
        delta_count=int(wheel.delta_count),
        counts_per_second=float(wheel.counts_per_second),
        speed_mps=float(wheel.speed_mps),
        illegal_transitions=int(wheel.illegal_transitions),
        reasons=reasons,
    )


def check_encoder_samples(
    samples: Iterable[EncoderSample],
    *,
    expect_motion: bool = False,
    min_active_wheels: int = DEFAULT_MIN_VALID_WHEEL_COUNT,
    max_illegal_transitions: int = DEFAULT_MAX_ILLEGAL_TRANSITIONS,
) -> EncoderHealthCheckResult:
    sample_list = list(samples)

    if not sample_list:
        return EncoderHealthCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="no encoder samples available",
            reasons=["sample list is empty"],
        )

    combined_reasons: list[str] = []
    total_illegal = 0
    min_active = len(WHEEL_ORDER)
    latest_result: EncoderHealthCheckResult | None = None

    for sample in sample_list:
        result = check_encoder_sample(
            sample,
            expect_motion=expect_motion,
            min_active_wheels=min_active_wheels,
            max_illegal_transitions=max_illegal_transitions,
        )
        latest_result = result

        combined_reasons.extend(result.reasons)
        total_illegal = max(total_illegal, result.total_illegal_transitions)
        min_active = min(min_active, result.active_wheel_count)

    if latest_result is None:
        return EncoderHealthCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="no encoder samples evaluated",
            reasons=["internal evaluation produced no result"],
        )

    unique_reasons = _unique_preserve_order(combined_reasons)

    if total_illegal > max_illegal_transitions:
        return EncoderHealthCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="encoder sample window failed",
            reasons=unique_reasons,
            active_wheel_count=latest_result.active_wheel_count,
            total_illegal_transitions=total_illegal,
            wheel_results=latest_result.wheel_results,
        )

    if unique_reasons:
        return EncoderHealthCheckResult(
            status=STATUS_WARN,
            ok=True,
            message="encoder sample window usable with notes",
            reasons=unique_reasons,
            active_wheel_count=latest_result.active_wheel_count,
            total_illegal_transitions=total_illegal,
            wheel_results=latest_result.wheel_results,
        )

    return EncoderHealthCheckResult(
        status=STATUS_OK,
        ok=True,
        message="encoder sample window healthy",
        reasons=[],
        active_wheel_count=latest_result.active_wheel_count,
        total_illegal_transitions=total_illegal,
        wheel_results=latest_result.wheel_results,
    )


def encoder_activity_summary(sample: EncoderSample) -> dict[str, object]:
    return {
        "active_wheel_count": sample.active_wheel_count,
        "all_wheels_active": sample.all_wheels_active,
        "total_illegal_transitions": sample.total_illegal_transitions,
        "linear_speed_mps": sample.linear_speed_mps,
        "vx_mps": sample.vx_mps,
        "vy_mps": sample.vy_mps,
        "omega_rad_s": sample.omega_rad_s,
        "wheels": {
            wheel.name: {
                "active": wheel.active,
                "count": wheel.count,
                "delta_count": wheel.delta_count,
                "counts_per_second": wheel.counts_per_second,
                "speed_mps": wheel.speed_mps,
                "direction": wheel.direction,
                "direction_symbol": wheel.direction_symbol,
                "illegal_transitions": wheel.illegal_transitions,
            }
            for wheel in sample.wheels
        },
    }


def _unique_preserve_order(values: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []

    for value in values:
        if value in seen:
            continue

        seen.add(value)
        result.append(value)

    return result