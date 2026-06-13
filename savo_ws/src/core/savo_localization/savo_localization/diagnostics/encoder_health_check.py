#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Encoder health checks for Robot Savo four-wheel odometry. No ROS imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Iterable

from savo_localization.constants import (
    DEFAULT_MAX_ILLEGAL_TRANSITIONS,
    DEFAULT_STALE_TIMEOUT_S,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
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
    status: str = STATUS_OK
    ok: bool = True
    message: str = "wheel healthy"
    reasons: list[str] = field(default_factory=list)

    count: int = 0
    delta_count: int = 0
    counts_per_second: float = 0.0
    speed_mps: float = 0.0
    illegal_transitions: int = 0
    active: bool = False

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

    @property
    def checked_wheel_count(self) -> int:
        return len(self.wheel_results)

    @property
    def computed_active_wheel_count(self) -> int:
        return sum(1 for result in self.wheel_results.values() if result.active)

    def to_dict(self) -> dict[str, object]:
        active_count = self.active_wheel_count

        if active_count == 0 and self.computed_active_wheel_count > 0:
            active_count = self.computed_active_wheel_count

        return {
            "status": self.status,
            "ok": bool(self.ok),
            "message": self.message,
            "reasons": list(self.reasons),
            "checked_wheel_count": self.checked_wheel_count,
            "active_wheel_count": int(active_count),
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
            illegal_transition_ok=(
                self.total_illegal_transitions <= DEFAULT_MAX_ILLEGAL_TRANSITIONS
            ),
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


def check_wheel_snapshot(
    wheel: EncoderWheelSnapshot,
    *,
    max_illegal_transitions: int = DEFAULT_MAX_ILLEGAL_TRANSITIONS,
) -> WheelHealthResult:
    reasons: list[str] = []
    status = STATUS_OK
    message = "wheel healthy"

    if wheel.name not in WHEEL_ORDER:
        reasons.append(f"unsupported wheel name: {wheel.name}")
        status = STATUS_ERROR
        message = "wheel not healthy"

    if wheel.illegal_transitions > max_illegal_transitions:
        reasons.append(
            "illegal transitions above limit: "
            f"{wheel.illegal_transitions} > {max_illegal_transitions}"
        )

        if status != STATUS_ERROR:
            status = STATUS_WARN
            message = "wheel usable with health notes"

    return WheelHealthResult(
        name=wheel.name,
        status=status,
        ok=status in (STATUS_OK, STATUS_WARN),
        message=message,
        reasons=_unique_preserve_order(reasons),
        count=int(wheel.count),
        delta_count=int(wheel.delta_count),
        counts_per_second=float(wheel.counts_per_second),
        speed_mps=float(wheel.speed_mps),
        illegal_transitions=int(wheel.illegal_transitions),
        active=bool(wheel.active),
    )


def check_encoder_sample(
    sample: EncoderSample,
    *,
    min_active_wheels: int | None = None,
    max_illegal_transitions: int = DEFAULT_MAX_ILLEGAL_TRANSITIONS,
    expect_motion: bool | None = None,
) -> EncoderHealthCheckResult:
    wheel_results = {
        wheel.name: check_wheel_snapshot(
            wheel,
            max_illegal_transitions=max_illegal_transitions,
        )
        for wheel in sample.wheels
    }

    if not wheel_results:
        return EncoderHealthCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="no encoder wheel states available",
            reasons=["sample contains no wheel snapshots"],
        )

    reasons: list[str] = []

    missing_wheels = [
        wheel_name
        for wheel_name in WHEEL_ORDER
        if wheel_name not in wheel_results
    ]

    if missing_wheels:
        reasons.append(f"missing wheel encoder states: {', '.join(missing_wheels)}")

    active_count = sum(1 for result in wheel_results.values() if result.active)
    total_illegal = sum(
        result.illegal_transitions
        for result in wheel_results.values()
    )

    for result in wheel_results.values():
        reasons.extend(
            f"{result.name}: {reason}"
            for reason in result.reasons
        )

    hard_error = bool(missing_wheels)

    if min_active_wheels is not None and active_count < min_active_wheels:
        hard_error = True
        reasons.append(
            "not enough active wheels during expected motion: "
            f"{active_count}/{min_active_wheels}"
        )

    if expect_motion is True and active_count == 0:
        hard_error = True
        reasons.append("no active wheels during expected motion")

    if hard_error:
        return EncoderHealthCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="encoder health check failed",
            reasons=_unique_preserve_order(reasons),
            active_wheel_count=active_count,
            total_illegal_transitions=total_illegal,
            wheel_results=wheel_results,
        )

    if reasons:
        return EncoderHealthCheckResult(
            status=STATUS_WARN,
            ok=True,
            message="encoders usable with health notes",
            reasons=_unique_preserve_order(reasons),
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


def check_encoder_samples(
    samples: Iterable[EncoderSample],
    *,
    min_active_wheels: int | None = None,
    max_illegal_transitions: int = DEFAULT_MAX_ILLEGAL_TRANSITIONS,
    expect_motion: bool | None = None,
) -> EncoderHealthCheckResult:
    sample_list = list(samples)

    if not sample_list:
        return EncoderHealthCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="no encoder samples available",
            reasons=["sample list is empty"],
        )

    latest_result = check_encoder_sample(
        sample_list[-1],
        min_active_wheels=min_active_wheels,
        max_illegal_transitions=max_illegal_transitions,
        expect_motion=expect_motion,
    )

    window_reasons: list[str] = []
    max_total_illegal = latest_result.total_illegal_transitions

    for sample in sample_list:
        result = check_encoder_sample(
            sample,
            min_active_wheels=min_active_wheels,
            max_illegal_transitions=max_illegal_transitions,
            expect_motion=expect_motion,
        )
        window_reasons.extend(result.reasons)
        max_total_illegal = max(max_total_illegal, result.total_illegal_transitions)

    if not latest_result.ok:
        return EncoderHealthCheckResult(
            status=latest_result.status,
            ok=False,
            message=latest_result.message,
            reasons=_unique_preserve_order(latest_result.reasons),
            active_wheel_count=latest_result.active_wheel_count,
            total_illegal_transitions=max_total_illegal,
            wheel_results=latest_result.wheel_results,
        )

    unique_reasons = _unique_preserve_order(window_reasons)

    if unique_reasons:
        return EncoderHealthCheckResult(
            status=STATUS_WARN,
            ok=True,
            message="encoder sample window usable with notes",
            reasons=unique_reasons,
            active_wheel_count=latest_result.active_wheel_count,
            total_illegal_transitions=max_total_illegal,
            wheel_results=latest_result.wheel_results,
        )

    return EncoderHealthCheckResult(
        status=STATUS_OK,
        ok=True,
        message="encoder sample window healthy",
        reasons=[],
        active_wheel_count=latest_result.active_wheel_count,
        total_illegal_transitions=max_total_illegal,
        wheel_results=latest_result.wheel_results,
    )


def check_encoder_state(
    state: EncoderState,
    *,
    min_active_wheels: int | None = None,
    max_illegal_transitions: int = DEFAULT_MAX_ILLEGAL_TRANSITIONS,
    stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S,
    expect_motion: bool | None = None,
) -> EncoderHealthCheckResult:
    if state.last_sample is None:
        return EncoderHealthCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="no encoder sample available",
            reasons=["no sample available"],
        )

    if state.last_sample_age_s is not None and state.last_sample_age_s > stale_timeout_s:
        return EncoderHealthCheckResult(
            status=STATUS_STALE,
            ok=False,
            message="encoder sample stale",
            reasons=[
                f"sample stale: age_s={state.last_sample_age_s:.3f} "
                f"> timeout_s={stale_timeout_s:.3f}"
            ],
        )

    return check_encoder_sample(
        state.last_sample,
        min_active_wheels=min_active_wheels,
        max_illegal_transitions=max_illegal_transitions,
        expect_motion=expect_motion,
    )


def encoder_activity_summary(sample: EncoderSample) -> dict[str, object]:
    wheel_directions = {
        wheel.name: int(wheel.direction or 0)
        for wheel in sample.wheels
    }

    wheel_active = {
        wheel.name: bool(wheel.active)
        for wheel in sample.wheels
    }

    return {
        "active_wheel_count": sample.active_wheel_count,
        "all_wheels_active": sample.all_wheels_active,
        "total_illegal_transitions": sample.total_illegal_transitions,
        "linear_speed_mps": sample.linear_speed_mps,
        "vx_mps": sample.vx_mps,
        "vy_mps": sample.vy_mps,
        "omega_rad_s": sample.omega_rad_s,
        "wheel_directions": wheel_directions,
        "wheel_active": wheel_active,
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


__all__ = [
    "WheelHealthResult",
    "EncoderHealthCheckResult",
    "check_wheel_snapshot",
    "check_encoder_sample",
    "check_encoder_samples",
    "check_encoder_state",
    "encoder_activity_summary",
]
