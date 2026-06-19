#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""LiDAR scan readiness check. No ROS runtime required."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from savo_mapping.constants import (
    DEFAULT_MIN_SCAN_RATE_HZ,
    DEFAULT_SCAN_STALE_TIMEOUT_S,
    FRAME_LASER,
    TOPIC_SCAN,
)
from savo_mapping.utils.diagnostics import (
    DiagnosticItem,
    make_error,
    make_ok,
    make_stale,
    make_warn,
)


# =============================================================================
# Scan readiness result
# =============================================================================
@dataclass(frozen=True)
class ScanReadyResult:
    ok: bool
    topic: str = TOPIC_SCAN
    frame_id: str = FRAME_LASER
    stale: bool = True

    msg_count: int = 0
    rate_hz: float = 0.0
    age_s: Optional[float] = None

    range_count: int = 0
    finite_count: int = 0
    min_observed_m: Optional[float] = None
    max_observed_m: Optional[float] = None

    message: str = "Waiting for LiDAR scan."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "topic": self.topic,
            "frame_id": self.frame_id,
            "stale": self.stale,
            "msg_count": self.msg_count,
            "rate_hz": self.rate_hz,
            "age_s": self.age_s,
            "range_count": self.range_count,
            "finite_count": self.finite_count,
            "min_observed_m": self.min_observed_m,
            "max_observed_m": self.max_observed_m,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Check logic
# =============================================================================
def evaluate_scan_ready(
    msg_count: int,
    rate_hz: float,
    age_s: Optional[float],
    range_count: int,
    finite_count: int,
    frame_id: str = FRAME_LASER,
    topic: str = TOPIC_SCAN,
    min_rate_hz: float = DEFAULT_MIN_SCAN_RATE_HZ,
    stale_timeout_s: float = DEFAULT_SCAN_STALE_TIMEOUT_S,
    min_finite_ranges: int = 5,
    min_observed_m: Optional[float] = None,
    max_observed_m: Optional[float] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> ScanReadyResult:
    count = max(0, int(msg_count))
    rate = max(0.0, float(rate_hz))
    ranges = max(0, int(range_count))
    finite = max(0, int(finite_count))

    stale = True if age_s is None else float(age_s) > max(0.0, stale_timeout_s)

    failures: list[str] = []

    if count <= 0:
        failures.append("no_messages")

    if stale:
        failures.append("stale")

    if rate < min_rate_hz:
        failures.append("rate_low")

    if ranges <= 0:
        failures.append("empty_scan")

    if finite < min_finite_ranges:
        failures.append("not_enough_valid_ranges")

    ok = not failures

    if ok:
        message = "LiDAR scan ready."
    else:
        message = f"LiDAR scan not ready: {', '.join(failures)}."

    return ScanReadyResult(
        ok=ok,
        topic=str(topic),
        frame_id=str(frame_id),
        stale=stale,
        msg_count=count,
        rate_hz=rate,
        age_s=age_s,
        range_count=ranges,
        finite_count=finite,
        min_observed_m=min_observed_m,
        max_observed_m=max_observed_m,
        message=message,
        extra={
            "failures": failures,
            "min_rate_hz": min_rate_hz,
            "stale_timeout_s": stale_timeout_s,
            "min_finite_ranges": min_finite_ranges,
            **dict(extra or {}),
        },
    )


def evaluate_scan_summary(
    summary: Dict[str, Any],
    msg_count: int,
    rate_hz: float,
    age_s: Optional[float],
    topic: str = TOPIC_SCAN,
    min_rate_hz: float = DEFAULT_MIN_SCAN_RATE_HZ,
    stale_timeout_s: float = DEFAULT_SCAN_STALE_TIMEOUT_S,
) -> ScanReadyResult:
    return evaluate_scan_ready(
        msg_count=msg_count,
        rate_hz=rate_hz,
        age_s=age_s,
        range_count=int(summary.get("range_count", 0)),
        finite_count=int(summary.get("finite_count", 0)),
        frame_id=str(summary.get("frame_id", FRAME_LASER)),
        topic=topic,
        min_rate_hz=min_rate_hz,
        stale_timeout_s=stale_timeout_s,
        min_observed_m=summary.get("min_observed_m"),
        max_observed_m=summary.get("max_observed_m"),
        extra={
            "range_min_m": summary.get("range_min_m"),
            "range_max_m": summary.get("range_max_m"),
        },
    )


def evaluate_scan_msg(
    msg: Any,
    msg_count: int,
    rate_hz: float,
    age_s: Optional[float],
    topic: str = TOPIC_SCAN,
    min_rate_hz: float = DEFAULT_MIN_SCAN_RATE_HZ,
    stale_timeout_s: float = DEFAULT_SCAN_STALE_TIMEOUT_S,
) -> ScanReadyResult:
    from savo_mapping.ros.adapters import scan_summary

    return evaluate_scan_summary(
        summary=scan_summary(msg),
        msg_count=msg_count,
        rate_hz=rate_hz,
        age_s=age_s,
        topic=topic,
        min_rate_hz=min_rate_hz,
        stale_timeout_s=stale_timeout_s,
    )


def scan_result_to_diagnostic(
    result: ScanReadyResult,
    required: bool = True,
) -> DiagnosticItem:
    values = result.to_dict()

    if result.ok:
        return make_ok(
            "scan",
            result.message,
            required=required,
            values=values,
        )

    if result.stale:
        return make_stale(
            "scan",
            result.message,
            required=required,
            values=values,
        )

    if result.msg_count <= 0:
        return make_error(
            "scan",
            result.message,
            required=required,
            values=values,
        )

    return make_warn(
        "scan",
        result.message,
        required=required,
        values=values,
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    result = evaluate_scan_ready(
        msg_count=20,
        rate_hz=3.8,
        age_s=0.10,
        range_count=720,
        finite_count=640,
        min_observed_m=0.42,
        max_observed_m=5.8,
    )

    print(result.to_json(indent=2))
    print(scan_result_to_diagnostic(result).to_dict())


if __name__ == "__main__":
    main()


__all__ = [
    "ScanReadyResult",
    "evaluate_scan_ready",
    "evaluate_scan_summary",
    "evaluate_scan_msg",
    "scan_result_to_diagnostic",
    "main",
]