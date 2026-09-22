#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Sensor health models for perception range inputs."""

from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence

from savo_perception.constants import (
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
)
from savo_perception.models.range_sample import RangeSample


@dataclass(frozen=True)
class SensorHealth:
    sensor_name: str
    ok: bool
    status: str
    last_distance_m: Optional[float] = None
    age_s: Optional[float] = None
    stale: bool = False
    valid: bool = True
    error: str = ""
    source: str = ""
    stamp_mono_s: float = field(default_factory=time.monotonic)

    @classmethod
    def from_sample(
        cls,
        sample: RangeSample,
        *,
        stale_timeout_s: float,
        now_mono_s: Optional[float] = None,
    ) -> "SensorHealth":
        age = sample.age_s(now_mono_s)
        stale = age > float(stale_timeout_s)
        valid = (
            sample.valid
            and sample.distance_m is not None
            and math.isfinite(float(sample.distance_m))
            and float(sample.distance_m) > 0.0
        )

        if stale:
            status = STATUS_STALE
            ok = False
        elif not valid:
            status = STATUS_ERROR
            ok = False
        else:
            status = STATUS_OK
            ok = True

        return cls(
            sensor_name=sample.sensor_name,
            ok=ok,
            status=status,
            last_distance_m=sample.distance_m if valid else None,
            age_s=age,
            stale=stale,
            valid=valid,
            error=sample.error,
            source=sample.source,
        )

    @classmethod
    def missing(cls, sensor_name: str, *, reason: str = "missing") -> "SensorHealth":
        return cls(
            sensor_name=sensor_name,
            ok=False,
            status=STATUS_ERROR,
            last_distance_m=None,
            age_s=None,
            stale=False,
            valid=False,
            error=reason,
        )

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class RangeHealthSummary:
    sensors: List[SensorHealth]
    stamp_mono_s: float = field(default_factory=time.monotonic)

    @property
    def ok(self) -> bool:
        return all(sensor.ok for sensor in self.sensors)

    @property
    def stale_sensors(self) -> List[str]:
        return [sensor.sensor_name for sensor in self.sensors if sensor.stale]

    @property
    def error_sensors(self) -> List[str]:
        return [
            sensor.sensor_name
            for sensor in self.sensors
            if not sensor.ok and not sensor.stale
        ]

    @property
    def status(self) -> str:
        if self.ok:
            return STATUS_OK
        if self.stale_sensors:
            return STATUS_STALE
        return STATUS_ERROR

    def get(self, sensor_name: str) -> Optional[SensorHealth]:
        for sensor in self.sensors:
            if sensor.sensor_name == sensor_name:
                return sensor
        return None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "ok": self.ok,
            "status": self.status,
            "stale_sensors": self.stale_sensors,
            "error_sensors": self.error_sensors,
            "stamp_mono_s": self.stamp_mono_s,
            "sensors": [sensor.to_dict() for sensor in self.sensors],
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))


def summarize_samples(
    samples: List[RangeSample],
    *,
    stale_timeout_s: float,
    now_mono_s: Optional[float] = None,
) -> RangeHealthSummary:
    return RangeHealthSummary(
        sensors=[
            SensorHealth.from_sample(
                sample,
                stale_timeout_s=stale_timeout_s,
                now_mono_s=now_mono_s,
            )
            for sample in samples
        ]
    )


def evaluate_required_range_health(
    health_by_name: Mapping[str, SensorHealth],
    *,
    required_sensors: Sequence[str],
    below_minimum_rate_sensors: Sequence[str] = (),
) -> Dict[str, Any]:
    """Evaluate aggregate health using only explicitly required sensors."""
    below_minimum = set(below_minimum_rate_sensors)
    stale_required: List[str] = []
    error_required: List[str] = []

    for sensor_name in required_sensors:
        health = health_by_name.get(sensor_name)
        if health is None:
            error_required.append(sensor_name)
        elif health.stale:
            stale_required.append(sensor_name)
        elif not health.ok or sensor_name in below_minimum:
            error_required.append(sensor_name)

    if error_required:
        status = STATUS_ERROR
    elif stale_required:
        status = STATUS_STALE
    else:
        status = STATUS_OK

    return {
        "ok": not stale_required and not error_required,
        "status": status,
        "stale_required_sensors": stale_required,
        "error_required_sensors": error_required,
    }


__all__ = [
    "SensorHealth",
    "RangeHealthSummary",
    "evaluate_required_range_health",
    "summarize_samples",
]
