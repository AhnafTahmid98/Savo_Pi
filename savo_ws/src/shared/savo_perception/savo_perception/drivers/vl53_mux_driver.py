#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""VL53L1X dual range driver through TCA9548A mux."""

from __future__ import annotations

import statistics
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from typing import Deque, Optional

from savo_perception.constants import (
    I2C_BUS_DEFAULT,
    TCA9548A_ADDR_DEFAULT,
    VL53_LEFT_CHANNEL_DEFAULT,
    VL53_MEDIAN_WINDOW_DEFAULT,
    VL53_RIGHT_CHANNEL_DEFAULT,
    VL53L1X_ADDR_DEFAULT,
)
from savo_perception.models import RangeSample


def import_smbus():
    try:
        from smbus2 import SMBus  # type: ignore

        return SMBus
    except Exception as exc:
        raise ImportError("smbus2 not found. Install: sudo apt install -y python3-smbus2") from exc


def import_vl53_driver():
    try:
        from VL53L1X import VL53L1X  # type: ignore

        return VL53L1X
    except Exception:
        try:
            from vl53l1x import VL53L1X  # type: ignore

            return VL53L1X
        except Exception as exc:
            raise ImportError(
                "VL53L1X Python driver not found. Expected module: VL53L1X or vl53l1x."
            ) from exc


@dataclass
class Vl53MuxConfig:
    bus: int = I2C_BUS_DEFAULT
    tca_addr: int = TCA9548A_ADDR_DEFAULT
    vl53_addr: int = VL53L1X_ADDR_DEFAULT
    left_channel: int = VL53_LEFT_CHANNEL_DEFAULT
    right_channel: int = VL53_RIGHT_CHANNEL_DEFAULT
    median_window: int = VL53_MEDIAN_WINDOW_DEFAULT
    settle_s: float = 0.002
    init_settle_s: float = 0.05
    valid_min_m: float = 0.02
    valid_max_m: float = 4.00

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class Vl53Reading:
    sensor_name: str
    raw_m: Optional[float]
    filtered_m: Optional[float]
    stamp_mono_s: float
    channel: int
    error: str = ""

    def to_sample(self) -> RangeSample:
        if self.filtered_m is None:
            return RangeSample.invalid(
                sensor_name=self.sensor_name,
                error=self.error or "invalid_distance",
                source="vl53_mux",
            )

        return RangeSample(
            sensor_name=self.sensor_name,
            distance_m=self.filtered_m,
            stamp_mono_s=self.stamp_mono_s,
            valid=True,
            source="vl53_mux",
            error="",
        )


@dataclass
class _SensorRuntime:
    name: str
    channel: int
    sensor: object
    history: Deque[float] = field(default_factory=deque)
    latest: Optional[Vl53Reading] = None


class TCA9548A:
    def __init__(self, bus: int = I2C_BUS_DEFAULT, address: int = TCA9548A_ADDR_DEFAULT) -> None:
        SMBus = import_smbus()
        self.bus_num = int(bus)
        self.address = int(address)
        self.bus = SMBus(self.bus_num)

    def select(self, channel: int) -> None:
        ch = int(channel)
        if not 0 <= ch <= 7:
            raise ValueError("TCA9548A channel must be 0..7")
        self.bus.write_byte(self.address, 1 << ch)

    def disable_all(self) -> None:
        self.bus.write_byte(self.address, 0x00)

    def close(self) -> None:
        try:
            self.disable_all()
        finally:
            close = getattr(self.bus, "close", None)
            if callable(close):
                close()


class Vl53MuxDriver:
    def __init__(self, config: Optional[Vl53MuxConfig] = None) -> None:
        self.config = config or Vl53MuxConfig()
        if self.config.median_window < 1 or self.config.median_window % 2 == 0:
            raise ValueError("median_window must be odd and >= 1")

        self._tca: Optional[TCA9548A] = None
        self._left: Optional[_SensorRuntime] = None
        self._right: Optional[_SensorRuntime] = None
        self._started = False

    @property
    def started(self) -> bool:
        return self._started

    def start(self) -> None:
        if self._started:
            return

        self._tca = TCA9548A(bus=self.config.bus, address=self.config.tca_addr)
        self._right = self._make_sensor("tof_right", self.config.right_channel)
        self._left = self._make_sensor("tof_left", self.config.left_channel)
        self._started = True

    def stop(self) -> None:
        for runtime in (self._right, self._left):
            if runtime is not None:
                self._stop_sensor(runtime)

        if self._tca is not None:
            self._tca.close()

        self._tca = None
        self._left = None
        self._right = None
        self._started = False

    def close(self) -> None:
        self.stop()

    def __enter__(self) -> "Vl53MuxDriver":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    def read_once(self) -> tuple[Vl53Reading, Vl53Reading]:
        self._require_started()

        right = self._update_sensor(self._right)
        left = self._update_sensor(self._left)

        return left, right

    def read_samples(self) -> tuple[RangeSample, RangeSample]:
        left, right = self.read_once()
        return left.to_sample(), right.to_sample()

    def _make_sensor(self, name: str, channel: int) -> _SensorRuntime:
        assert self._tca is not None

        VL53L1X = import_vl53_driver()

        self._tca.select(channel)
        time.sleep(self.config.init_settle_s)

        sensor = VL53L1X(i2c_bus=self.config.bus, i2c_address=self.config.vl53_addr)

        if hasattr(sensor, "open"):
            sensor.open()

        try:
            sensor.set_distance_mode("short")
        except Exception:
            try:
                sensor.distance_mode = "short"
            except Exception:
                pass

        try:
            if hasattr(sensor, "set_timing"):
                sensor.set_timing(50, 70)
        except Exception:
            pass

        if hasattr(sensor, "start_ranging"):
            sensor.start_ranging()
        elif hasattr(sensor, "start"):
            sensor.start()

        return _SensorRuntime(
            name=name,
            channel=int(channel),
            sensor=sensor,
            history=deque(maxlen=self.config.median_window),
        )

    def _update_sensor(self, runtime: Optional[_SensorRuntime]) -> Vl53Reading:
        self._require_started()

        if runtime is None:
            raise RuntimeError("VL53 runtime not initialized")

        assert self._tca is not None
        self._tca.select(runtime.channel)
        time.sleep(self.config.settle_s)

        stamp = time.monotonic()
        raw_m = self._read_distance_m(runtime.sensor)
        error = ""

        if raw_m is None:
            error = "invalid_or_no_reading"
        else:
            runtime.history.append(raw_m)

        filtered_m = self._median_or_none(runtime.history)

        reading = Vl53Reading(
            sensor_name=runtime.name,
            raw_m=raw_m,
            filtered_m=filtered_m,
            stamp_mono_s=stamp,
            channel=runtime.channel,
            error=error,
        )
        runtime.latest = reading
        return reading

    def _stop_sensor(self, runtime: _SensorRuntime) -> None:
        try:
            if self._tca is not None:
                self._tca.select(runtime.channel)
                time.sleep(self.config.settle_s)

            if hasattr(runtime.sensor, "stop_ranging"):
                runtime.sensor.stop_ranging()
            elif hasattr(runtime.sensor, "stop"):
                runtime.sensor.stop()
        except Exception:
            pass

    def _read_distance_m(self, sensor: object) -> Optional[float]:
        try:
            if hasattr(sensor, "get_distance"):
                distance_mm = sensor.get_distance()
            else:
                distance_mm = getattr(sensor, "distance", None)

            if distance_mm is None:
                return None

            distance_mm = int(round(float(distance_mm)))

            if distance_mm <= 0:
                return None

            distance_m = distance_mm / 1000.0
            if distance_m < self.config.valid_min_m or distance_m > self.config.valid_max_m:
                return None

            return distance_m
        except Exception:
            return None

    @staticmethod
    def _median_or_none(values: Deque[float]) -> Optional[float]:
        if not values:
            return None
        return float(statistics.median(values))

    def _require_started(self) -> None:
        if not self._started:
            raise RuntimeError("Vl53MuxDriver is not started")


__all__ = [
    "Vl53MuxConfig",
    "Vl53Reading",
    "TCA9548A",
    "Vl53MuxDriver",
    "import_smbus",
    "import_vl53_driver",
]
