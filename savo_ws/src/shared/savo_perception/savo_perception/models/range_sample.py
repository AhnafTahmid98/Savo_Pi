#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Range sample models for ToF, ultrasonic, and depth inputs."""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class RangeSample:
    sensor_name: str
    distance_m: Optional[float]
    stamp_mono_s: float
    frame_id: str = ""
    valid: bool = True
    source: str = ""
    error: str = ""

    @classmethod
    def now(
        cls,
        *,
        sensor_name: str,
        distance_m: Optional[float],
        frame_id: str = "",
        source: str = "",
        error: str = "",
    ) -> "RangeSample":
        valid = is_valid_distance(distance_m)
        return cls(
            sensor_name=sensor_name,
            distance_m=float(distance_m) if valid else None,
            stamp_mono_s=time.monotonic(),
            frame_id=frame_id,
            valid=valid and not bool(error),
            source=source,
            error=error,
        )

    @classmethod
    def invalid(
        cls,
        *,
        sensor_name: str,
        error: str,
        frame_id: str = "",
        source: str = "",
    ) -> "RangeSample":
        return cls(
            sensor_name=sensor_name,
            distance_m=None,
            stamp_mono_s=time.monotonic(),
            frame_id=frame_id,
            valid=False,
            source=source,
            error=error,
        )

    def age_s(self, now_mono_s: Optional[float] = None) -> float:
        now_s = time.monotonic() if now_mono_s is None else float(now_mono_s)
        return max(0.0, now_s - float(self.stamp_mono_s))

    def is_stale(self, stale_timeout_s: float, now_mono_s: Optional[float] = None) -> bool:
        return self.age_s(now_mono_s) > float(stale_timeout_s)

    def usable(self, stale_timeout_s: Optional[float] = None) -> bool:
        if not self.valid:
            return False
        if self.distance_m is None:
            return False
        if not is_valid_distance(self.distance_m):
            return False
        if stale_timeout_s is not None and self.is_stale(stale_timeout_s):
            return False
        return True

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class RangePair:
    left: RangeSample
    right: RangeSample

    def min_distance_m(self, stale_timeout_s: Optional[float] = None) -> Optional[float]:
        values = []

        if self.left.usable(stale_timeout_s):
            values.append(float(self.left.distance_m))

        if self.right.usable(stale_timeout_s):
            values.append(float(self.right.distance_m))

        return min(values) if values else None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "left": self.left.to_dict(),
            "right": self.right.to_dict(),
            "min_distance_m": self.min_distance_m(),
        }


@dataclass(frozen=True)
class RangeSnapshot:
    depth_front: RangeSample
    tof_left: RangeSample
    tof_right: RangeSample
    ultrasonic_front: RangeSample

    def front_candidates(self, stale_timeout_s: Optional[float] = None) -> Dict[str, float]:
        out: Dict[str, float] = {}

        for sample in (self.depth_front, self.ultrasonic_front):
            if sample.usable(stale_timeout_s):
                out[sample.sensor_name] = float(sample.distance_m)

        return out

    def side_candidates(self, stale_timeout_s: Optional[float] = None) -> Dict[str, float]:
        out: Dict[str, float] = {}

        for sample in (self.tof_left, self.tof_right):
            if sample.usable(stale_timeout_s):
                out[sample.sensor_name] = float(sample.distance_m)

        return out

    def min_front_m(self, stale_timeout_s: Optional[float] = None) -> Optional[float]:
        values = list(self.front_candidates(stale_timeout_s).values())
        return min(values) if values else None

    def min_side_m(self, stale_timeout_s: Optional[float] = None) -> Optional[float]:
        values = list(self.side_candidates(stale_timeout_s).values())
        return min(values) if values else None

    def all_samples(self) -> tuple[RangeSample, RangeSample, RangeSample, RangeSample]:
        return (
            self.depth_front,
            self.tof_left,
            self.tof_right,
            self.ultrasonic_front,
        )

    def stale_sensors(self, stale_timeout_s: float) -> list[str]:
        return [
            sample.sensor_name
            for sample in self.all_samples()
            if sample.is_stale(stale_timeout_s)
        ]

    def invalid_sensors(self) -> list[str]:
        return [
            sample.sensor_name
            for sample in self.all_samples()
            if not sample.valid
        ]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "depth_front": self.depth_front.to_dict(),
            "tof_left": self.tof_left.to_dict(),
            "tof_right": self.tof_right.to_dict(),
            "ultrasonic_front": self.ultrasonic_front.to_dict(),
            "min_front_m": self.min_front_m(),
            "min_side_m": self.min_side_m(),
        }


def is_valid_distance(
    distance_m: Optional[float],
    *,
    min_m: float = 0.0,
    max_m: float = 10.0,
) -> bool:
    if distance_m is None:
        return False

    try:
        value = float(distance_m)
    except Exception:
        return False

    if not math.isfinite(value):
        return False

    return float(min_m) <= value <= float(max_m)


def sanitize_distance(
    distance_m: Optional[float],
    *,
    min_m: float = 0.0,
    max_m: float = 10.0,
) -> Optional[float]:
    if not is_valid_distance(distance_m, min_m=min_m, max_m=max_m):
        return None
    return float(distance_m)


def make_invalid_snapshot(reason: str = "no_data") -> RangeSnapshot:
    return RangeSnapshot(
        depth_front=RangeSample.invalid(sensor_name="depth_front", error=reason),
        tof_left=RangeSample.invalid(sensor_name="tof_left", error=reason),
        tof_right=RangeSample.invalid(sensor_name="tof_right", error=reason),
        ultrasonic_front=RangeSample.invalid(sensor_name="ultrasonic_front", error=reason),
    )


__all__ = [
    "RangeSample",
    "RangePair",
    "RangeSnapshot",
    "is_valid_distance",
    "sanitize_distance",
    "make_invalid_snapshot",
]