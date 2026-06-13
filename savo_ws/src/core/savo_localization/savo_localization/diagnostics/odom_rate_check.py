#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Odometry rate checks for Robot Savo localization. No ROS imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Iterable

from savo_localization.constants import (
    DEFAULT_EKF_RATE_HZ,
    DEFAULT_WHEEL_ODOM_RATE_HZ,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_WARN,
)


@dataclass(frozen=True)
class RateCheckResult:
    name: str
    status: str
    ok: bool
    message: str

    measured_hz: float
    expected_hz: float
    min_allowed_hz: float
    max_allowed_hz: float | None = None

    sample_count: int = 0
    last_age_s: float | None = None
    reasons: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def check_rate(
    *,
    name: str,
    measured_hz: float,
    expected_hz: float,
    tolerance_ratio: float = 0.50,
    max_ratio: float | None = 2.00,
    sample_count: int = 0,
    last_age_s: float | None = None,
    stale_timeout_s: float | None = None,
) -> RateCheckResult:
    if expected_hz <= 0.0:
        raise ValueError(f"expected_hz must be > 0.0, got {expected_hz}")

    if not 0.0 <= tolerance_ratio < 1.0:
        raise ValueError(
            f"tolerance_ratio must be in range [0.0, 1.0), got {tolerance_ratio}"
        )

    if max_ratio is not None and max_ratio <= 1.0:
        raise ValueError(f"max_ratio must be > 1.0 when set, got {max_ratio}")

    measured = max(0.0, float(measured_hz))
    expected = float(expected_hz)

    min_allowed = expected * (1.0 - float(tolerance_ratio))
    max_allowed = expected * float(max_ratio) if max_ratio is not None else None

    reasons: list[str] = []

    if stale_timeout_s is not None:
        if stale_timeout_s <= 0.0:
            raise ValueError(f"stale_timeout_s must be > 0.0, got {stale_timeout_s}")

        if last_age_s is None:
            return RateCheckResult(
                name=name,
                status=STATUS_STALE,
                ok=False,
                message=f"{name} has no timestamp",
                measured_hz=measured,
                expected_hz=expected,
                min_allowed_hz=min_allowed,
                max_allowed_hz=max_allowed,
                sample_count=int(sample_count),
                last_age_s=last_age_s,
                reasons=["last_age_s is not available"],
            )

        if float(last_age_s) > float(stale_timeout_s):
            return RateCheckResult(
                name=name,
                status=STATUS_STALE,
                ok=False,
                message=f"{name} data stale",
                measured_hz=measured,
                expected_hz=expected,
                min_allowed_hz=min_allowed,
                max_allowed_hz=max_allowed,
                sample_count=int(sample_count),
                last_age_s=float(last_age_s),
                reasons=[f"age_s={float(last_age_s):.3f} > timeout_s={stale_timeout_s:.3f}"],
            )

    if sample_count <= 0 and measured <= 0.0:
        return RateCheckResult(
            name=name,
            status=STATUS_ERROR,
            ok=False,
            message=f"{name} rate not measured",
            measured_hz=measured,
            expected_hz=expected,
            min_allowed_hz=min_allowed,
            max_allowed_hz=max_allowed,
            sample_count=int(sample_count),
            last_age_s=last_age_s,
            reasons=["no samples received"],
        )

    if measured < min_allowed:
        reasons.append(
            f"rate too low: {measured:.3f} Hz < {min_allowed:.3f} Hz"
        )

    if max_allowed is not None and measured > max_allowed:
        reasons.append(
            f"rate too high: {measured:.3f} Hz > {max_allowed:.3f} Hz"
        )

    if reasons:
        return RateCheckResult(
            name=name,
            status=STATUS_WARN,
            ok=True,
            message=f"{name} rate usable with notes",
            measured_hz=measured,
            expected_hz=expected,
            min_allowed_hz=min_allowed,
            max_allowed_hz=max_allowed,
            sample_count=int(sample_count),
            last_age_s=last_age_s,
            reasons=reasons,
        )

    return RateCheckResult(
        name=name,
        status=STATUS_OK,
        ok=True,
        message=f"{name} rate healthy",
        measured_hz=measured,
        expected_hz=expected,
        min_allowed_hz=min_allowed,
        max_allowed_hz=max_allowed,
        sample_count=int(sample_count),
        last_age_s=last_age_s,
        reasons=[],
    )


def check_wheel_odom_rate(
    *,
    measured_hz: float,
    expected_hz: float = DEFAULT_WHEEL_ODOM_RATE_HZ,
    sample_count: int = 0,
    last_age_s: float | None = None,
    stale_timeout_s: float | None = None,
) -> RateCheckResult:
    return check_rate(
        name="wheel_odom",
        measured_hz=measured_hz,
        expected_hz=expected_hz,
        tolerance_ratio=0.50,
        max_ratio=2.00,
        sample_count=sample_count,
        last_age_s=last_age_s,
        stale_timeout_s=stale_timeout_s,
    )


def check_ekf_rate(
    *,
    measured_hz: float,
    expected_hz: float = DEFAULT_EKF_RATE_HZ,
    sample_count: int = 0,
    last_age_s: float | None = None,
    stale_timeout_s: float | None = None,
) -> RateCheckResult:
    return check_rate(
        name="ekf",
        measured_hz=measured_hz,
        expected_hz=expected_hz,
        tolerance_ratio=0.50,
        max_ratio=2.00,
        sample_count=sample_count,
        last_age_s=last_age_s,
        stale_timeout_s=stale_timeout_s,
    )


def check_odom_rate_window(
    *,
    name: str,
    stamps_s: Iterable[float],
    expected_hz: float,
    tolerance_ratio: float = 0.50,
    max_ratio: float | None = 2.00,
    last_age_s: float | None = None,
    stale_timeout_s: float | None = None,
) -> RateCheckResult:
    stamps = [float(stamp) for stamp in stamps_s]

    if len(stamps) < 2:
        return check_rate(
            name=name,
            measured_hz=0.0,
            expected_hz=expected_hz,
            tolerance_ratio=tolerance_ratio,
            max_ratio=max_ratio,
            sample_count=len(stamps),
            last_age_s=last_age_s,
            stale_timeout_s=stale_timeout_s,
        )

    measured_hz = estimate_rate_from_stamps(stamps)

    return check_rate(
        name=name,
        measured_hz=measured_hz,
        expected_hz=expected_hz,
        tolerance_ratio=tolerance_ratio,
        max_ratio=max_ratio,
        sample_count=len(stamps),
        last_age_s=last_age_s,
        stale_timeout_s=stale_timeout_s,
    )


def estimate_rate_from_stamps(stamps_s: Iterable[float]) -> float:
    stamps = [float(stamp) for stamp in stamps_s]

    if len(stamps) < 2:
        return 0.0

    stamps.sort()

    periods: list[float] = []
    for previous, current in zip(stamps[:-1], stamps[1:]):
        period = current - previous
        if period > 0.0:
            periods.append(period)

    if not periods:
        return 0.0

    mean_period = sum(periods) / float(len(periods))

    if mean_period <= 0.0:
        return 0.0

    return 1.0 / mean_period


def combine_rate_results(
    *,
    name: str,
    results: Iterable[RateCheckResult],
) -> RateCheckResult:
    result_list = list(results)

    if not result_list:
        return RateCheckResult(
            name=name,
            status=STATUS_ERROR,
            ok=False,
            message=f"{name} has no rate results",
            measured_hz=0.0,
            expected_hz=0.0,
            min_allowed_hz=0.0,
            reasons=["result list is empty"],
        )

    reasons: list[str] = []
    statuses = [result.status for result in result_list]

    for result in result_list:
        reasons.extend(f"{result.name}: {reason}" for reason in result.reasons)

    if STATUS_ERROR in statuses:
        status = STATUS_ERROR
        ok = False
        message = f"{name} rate check failed"
    elif STATUS_STALE in statuses:
        status = STATUS_STALE
        ok = False
        message = f"{name} rate check stale"
    elif STATUS_WARN in statuses:
        status = STATUS_WARN
        ok = True
        message = f"{name} rate check usable with notes"
    else:
        status = STATUS_OK
        ok = True
        message = f"{name} rate check healthy"

    first = result_list[0]

    return RateCheckResult(
        name=name,
        status=status,
        ok=ok,
        message=message,
        measured_hz=first.measured_hz,
        expected_hz=first.expected_hz,
        min_allowed_hz=first.min_allowed_hz,
        max_allowed_hz=first.max_allowed_hz,
        sample_count=sum(result.sample_count for result in result_list),
        last_age_s=max(
            (
                float(result.last_age_s)
                for result in result_list
                if result.last_age_s is not None
            ),
            default=None,
        ),
        reasons=_unique_preserve_order(reasons),
    )


def rate_ratio(measured_hz: float, expected_hz: float) -> float:
    if expected_hz <= 0.0:
        raise ValueError(f"expected_hz must be > 0.0, got {expected_hz}")

    return max(0.0, float(measured_hz)) / float(expected_hz)


def rate_percent(measured_hz: float, expected_hz: float) -> float:
    return rate_ratio(measured_hz, expected_hz) * 100.0


def _unique_preserve_order(values: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []

    for value in values:
        if value in seen:
            continue

        seen.add(value)
        result.append(value)

    return result