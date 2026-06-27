#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""HC-SR04 ultrasonic driver for Python fallback and diagnostics."""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass
from typing import Optional

from savo_perception.constants import (
    ULTRASONIC_ECHO_PIN_DEFAULT,
    ULTRASONIC_MAX_DISTANCE_M_DEFAULT,
    ULTRASONIC_TRIG_PIN_DEFAULT,
)
from savo_perception.models import RangeSample


def import_gpiozero():
    try:
        from gpiozero import Device, DistanceSensor  # type: ignore
        from gpiozero.pins.lgpio import LGPIOFactory  # type: ignore

        return Device, DistanceSensor, LGPIOFactory
    except Exception as exc:
        raise ImportError(
            "gpiozero/lgpio not found. Install: sudo apt install -y python3-gpiozero python3-lgpio"
        ) from exc


@dataclass
class UltrasonicConfig:
    trig_pin: int = ULTRASONIC_TRIG_PIN_DEFAULT
    echo_pin: int = ULTRASONIC_ECHO_PIN_DEFAULT
    max_distance_m: float = ULTRASONIC_MAX_DISTANCE_M_DEFAULT
    valid_min_m: float = 0.02
    valid_max_m: float = ULTRASONIC_MAX_DISTANCE_M_DEFAULT
    queue_len: int = 1
    pin_factory: str = "lgpio"

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class UltrasonicReading:
    sensor_name: str
    distance_m: Optional[float]
    stamp_mono_s: float
    trig_pin: int
    echo_pin: int
    error: str = ""

    def to_sample(self) -> RangeSample:
        if self.distance_m is None:
            return RangeSample.invalid(
                sensor_name=self.sensor_name,
                error=self.error or "invalid_distance",
                source="ultrasonic",
            )

        return RangeSample(
            sensor_name=self.sensor_name,
            distance_m=self.distance_m,
            stamp_mono_s=self.stamp_mono_s,
            valid=True,
            source="ultrasonic",
            error="",
        )


class UltrasonicDriver:
    def __init__(self, config: Optional[UltrasonicConfig] = None) -> None:
        self.config = config or UltrasonicConfig()
        self._sensor = None
        self._started = False

    @property
    def started(self) -> bool:
        return self._started

    def start(self) -> None:
        if self._started:
            return

        if self.config.pin_factory != "lgpio":
            raise ValueError("Only lgpio pin_factory is supported in production fallback driver")

        Device, DistanceSensor, LGPIOFactory = import_gpiozero()
        Device.pin_factory = LGPIOFactory()

        self._sensor = DistanceSensor(
            echo=int(self.config.echo_pin),
            trigger=int(self.config.trig_pin),
            max_distance=float(self.config.max_distance_m),
            queue_len=int(self.config.queue_len),
        )

        self._started = True

    def stop(self) -> None:
        if self._sensor is not None:
            try:
                self._sensor.close()
            except Exception:
                pass

        self._sensor = None
        self._started = False

    def close(self) -> None:
        self.stop()

    def __enter__(self) -> "UltrasonicDriver":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    def read_once(self) -> UltrasonicReading:
        self._require_started()

        stamp = time.monotonic()
        distance_m = self._read_distance_m()
        error = "" if distance_m is not None else "invalid_or_no_echo"

        return UltrasonicReading(
            sensor_name="ultrasonic_front",
            distance_m=distance_m,
            stamp_mono_s=stamp,
            trig_pin=int(self.config.trig_pin),
            echo_pin=int(self.config.echo_pin),
            error=error,
        )

    def read_sample(self) -> RangeSample:
        return self.read_once().to_sample()

    def _read_distance_m(self) -> Optional[float]:
        if self._sensor is None:
            return None

        try:
            distance_m = float(self._sensor.distance)
        except Exception:
            return None

        if not math.isfinite(distance_m):
            return None

        if distance_m < self.config.valid_min_m or distance_m > self.config.valid_max_m:
            return None

        return distance_m

    def _require_started(self) -> None:
        if not self._started:
            raise RuntimeError("UltrasonicDriver is not started")


__all__ = [
    "UltrasonicConfig",
    "UltrasonicReading",
    "UltrasonicDriver",
    "import_gpiozero",
]
