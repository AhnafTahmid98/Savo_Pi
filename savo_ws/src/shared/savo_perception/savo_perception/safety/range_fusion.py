#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Range fusion policy for perception safety decisions."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Optional, Sequence

from savo_perception.constants import (
    FAIL_SAFE_ON_STALE_DEFAULT,
    FRONT_SLOW_M_DEFAULT,
    FRONT_STOP_M_DEFAULT,
    SENSOR_STALE_TIMEOUT_S_DEFAULT,
    SIDE_SLOW_M_DEFAULT,
    SIDE_STOP_M_DEFAULT,
    SLOWDOWN_DEFAULT,
    SLOWDOWN_MIN_DEFAULT,
)
from savo_perception.models.range_sample import RangeSnapshot
from savo_perception.models.safety_state import SafetyDecision, clamp_slowdown_factor


@dataclass(frozen=True)
class RangeFusionConfig:
    front_stop_m: float = FRONT_STOP_M_DEFAULT
    front_slow_m: float = FRONT_SLOW_M_DEFAULT
    side_stop_m: float = SIDE_STOP_M_DEFAULT
    side_slow_m: float = SIDE_SLOW_M_DEFAULT
    ultrasonic_stop_m: float = 0.10
    stale_timeout_s: float = SENSOR_STALE_TIMEOUT_S_DEFAULT
    fail_safe_on_stale: bool = FAIL_SAFE_ON_STALE_DEFAULT
    required_sensors: tuple[str, ...] = field(
        default_factory=lambda: ("tof_left", "tof_right")
    )

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class RangeFusionResult:
    decision: SafetyDecision
    front_distance_m: Optional[float]
    side_distance_m: Optional[float]
    front_sources: Dict[str, float]
    side_sources: Dict[str, float]
    stale_sensors: list[str]
    invalid_sensors: list[str]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "decision": self.decision.to_dict(),
            "front_distance_m": self.front_distance_m,
            "side_distance_m": self.side_distance_m,
            "front_sources": dict(self.front_sources),
            "side_sources": dict(self.side_sources),
            "stale_sensors": list(self.stale_sensors),
            "invalid_sensors": list(self.invalid_sensors),
        }


def slowdown_from_distance(
    distance_m: Optional[float],
    *,
    stop_m: float,
    slow_m: float,
) -> float:
    if distance_m is None:
        return SLOWDOWN_DEFAULT

    d = float(distance_m)
    stop = float(stop_m)
    slow = float(slow_m)

    if d <= stop:
        return 0.0

    if d >= slow:
        return SLOWDOWN_DEFAULT

    span = max(0.001, slow - stop)
    ratio = (d - stop) / span
    return clamp_slowdown_factor(max(SLOWDOWN_MIN_DEFAULT, ratio))


def min_optional(a: Optional[float], b: Optional[float]) -> Optional[float]:
    values = [v for v in (a, b) if v is not None]
    return min(values) if values else None


def required_stale_sensors(
    stale_sensors: Sequence[str],
    required_sensors: Sequence[str],
) -> list[str]:
    stale = set(stale_sensors)
    return [name for name in required_sensors if name in stale]


def fuse_range_snapshot(
    snapshot: RangeSnapshot,
    config: RangeFusionConfig | None = None,
) -> RangeFusionResult:
    cfg = config or RangeFusionConfig()

    front_sources = snapshot.front_candidates(cfg.stale_timeout_s)
    side_sources = snapshot.side_candidates(cfg.stale_timeout_s)

    front_distance_m = snapshot.min_front_m(cfg.stale_timeout_s)
    side_distance_m = snapshot.min_side_m(cfg.stale_timeout_s)
    ultrasonic_front_distance_m = (
        float(snapshot.ultrasonic_front.distance_m)
        if snapshot.ultrasonic_front.usable(cfg.stale_timeout_s)
        else None
    )

    stale = snapshot.stale_sensors(cfg.stale_timeout_s)
    invalid = snapshot.invalid_sensors()
    required_stale = required_stale_sensors(stale, cfg.required_sensors)

    if cfg.fail_safe_on_stale and required_stale:
        decision = SafetyDecision.stop(
            reason="required_sensor_stale",
            front_distance_m=front_distance_m,
            side_distance_m=side_distance_m,
            stale_sensors=required_stale,
            invalid_sensors=invalid,
        )
        return RangeFusionResult(
            decision=decision,
            front_distance_m=front_distance_m,
            side_distance_m=side_distance_m,
            front_sources=front_sources,
            side_sources=side_sources,
            stale_sensors=stale,
            invalid_sensors=invalid,
        )

    if (
        ultrasonic_front_distance_m is not None
        and math.isfinite(ultrasonic_front_distance_m)
        and ultrasonic_front_distance_m <= cfg.ultrasonic_stop_m
    ):
        decision = SafetyDecision.stop(
            reason="ultrasonic_close_stop",
            front_distance_m=front_distance_m,
            side_distance_m=side_distance_m,
            stale_sensors=stale,
            invalid_sensors=invalid,
        )
    elif side_distance_m is not None and side_distance_m <= cfg.side_stop_m:
        decision = SafetyDecision.stop(
            reason="side_stop_zone",
            front_distance_m=front_distance_m,
            side_distance_m=side_distance_m,
            stale_sensors=stale,
            invalid_sensors=invalid,
        )
    else:
        front_slowdown = slowdown_from_distance(
            front_distance_m,
            stop_m=cfg.front_stop_m,
            slow_m=cfg.front_slow_m,
        )
        side_slowdown = slowdown_from_distance(
            side_distance_m,
            stop_m=cfg.side_stop_m,
            slow_m=cfg.side_slow_m,
        )
        slowdown = min(front_slowdown, side_slowdown)

        if slowdown < SLOWDOWN_DEFAULT:
            reason = "front_slow_zone" if front_slowdown <= side_slowdown else "side_slow_zone"
            decision = SafetyDecision.slow(
                slowdown_factor=slowdown,
                reason=reason,
                front_distance_m=front_distance_m,
                side_distance_m=side_distance_m,
            )
        else:
            decision = SafetyDecision.clear(
                reason="clear",
            )

    return RangeFusionResult(
        decision=decision,
        front_distance_m=front_distance_m,
        side_distance_m=side_distance_m,
        front_sources=front_sources,
        side_sources=side_sources,
        stale_sensors=stale,
        invalid_sensors=invalid,
    )


__all__ = [
    "RangeFusionConfig",
    "RangeFusionResult",
    "slowdown_from_distance",
    "min_optional",
    "required_stale_sensors",
    "fuse_range_snapshot",
]
