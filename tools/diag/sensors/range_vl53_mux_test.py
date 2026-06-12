#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Dual VL53L1X range diagnostic through TCA9548A using smbus2."""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

from smbus2 import SMBus


TCA_ADDR = 0x70
VL53_ADDR = 0x29

# Current detected Robot Savo mux mapping.
RIGHT_CH = 2
LEFT_CH = 3


@dataclass
class SensorSample:
    raw_m: Optional[float]
    filt_m: Optional[float]
    t_mono: float


@dataclass
class SensorRuntime:
    name: str
    channel: int
    sensor: object
    hist: deque[float]
    latest: SensorSample


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


class TCA9548A:
    def __init__(self, bus: int = 1, address: int = TCA_ADDR):
        self.bus_num = bus
        self.address = address
        self.bus = SMBus(bus)

    def select(self, channel: int) -> None:
        if not 0 <= channel <= 7:
            raise ValueError("TCA9548A channel must be 0..7")
        self.bus.write_byte(self.address, 1 << channel)

    def disable_all(self) -> None:
        self.bus.write_byte(self.address, 0x00)


def fmt_m(value: Optional[float]) -> str:
    return f"{value:6.3f}" if value is not None else "  --- "


def read_distance_m(sensor) -> Optional[float]:
    try:
        if hasattr(sensor, "get_distance"):
            distance_mm = sensor.get_distance()
        else:
            distance_mm = getattr(sensor, "distance", None)

        if distance_mm is None:
            return None

        if isinstance(distance_mm, float):
            distance_mm = int(round(distance_mm))

        distance_mm = int(distance_mm)

        if distance_mm <= 0 or distance_mm >= 4000:
            return None

        return distance_mm / 1000.0

    except Exception:
        return None


def median_or_none(values: deque[float]) -> Optional[float]:
    if not values:
        return None
    return float(statistics.median(values))


def make_sensor(tca: TCA9548A, name: str, channel: int, median_n: int) -> SensorRuntime:
    VL53L1X = import_vl53_driver()

    tca.select(channel)
    time.sleep(0.05)

    sensor = VL53L1X(i2c_bus=tca.bus_num, i2c_address=VL53_ADDR)

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

    now = time.perf_counter()

    return SensorRuntime(
        name=name,
        channel=channel,
        sensor=sensor,
        hist=deque(maxlen=median_n),
        latest=SensorSample(None, None, now),
    )


def update_sensor(tca: TCA9548A, runtime: SensorRuntime) -> None:
    tca.select(runtime.channel)
    time.sleep(0.002)

    now = time.perf_counter()
    raw_m = read_distance_m(runtime.sensor)

    if raw_m is not None:
        runtime.hist.append(raw_m)

    filt_m = median_or_none(runtime.hist)
    runtime.latest = SensorSample(raw_m, filt_m, now)


def stop_sensor(tca: TCA9548A, runtime: SensorRuntime) -> None:
    try:
        tca.select(runtime.channel)
        time.sleep(0.002)

        if hasattr(runtime.sensor, "stop_ranging"):
            runtime.sensor.stop_ranging()
        elif hasattr(runtime.sensor, "stop"):
            runtime.sensor.stop()
    except Exception:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description="Robot Savo dual VL53L1X TCA mux diagnostic")
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--tca-addr", type=lambda s: int(s, 0), default=TCA_ADDR)
    parser.add_argument("--right-ch", type=int, default=RIGHT_CH)
    parser.add_argument("--left-ch", type=int, default=LEFT_CH)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--median", type=int, default=5)
    parser.add_argument("--stale", type=float, default=0.50)
    parser.add_argument("--threshold", type=float, default=0.20)
    parser.add_argument("--duration", type=float, default=0.0, help="0 = run until Ctrl+C")
    args = parser.parse_args()

    if args.median < 1 or args.median % 2 == 0:
        raise ValueError("--median must be odd and >= 1")

    tca = TCA9548A(bus=args.bus, address=args.tca_addr)

    print(f"[TCA9548A] bus=i2c-{args.bus} address=0x{args.tca_addr:02X}")
    print(f"[VL53L1X] address=0x{VL53_ADDR:02X}")
    print(f"[Mapping] RIGHT=channel {args.right_ch}  LEFT=channel {args.left_ch}")

    right = make_sensor(tca, "RIGHT", args.right_ch, args.median)
    left = make_sensor(tca, "LEFT", args.left_ch, args.median)

    period = 1.0 / max(args.rate, 0.1)
    t0 = time.perf_counter()

    print("\n time_s | R_raw  R_filt | L_raw  L_filt | Alert")
    print("-" * 62)

    try:
        while True:
            loop_t = time.perf_counter()

            update_sensor(tca, right)
            update_sensor(tca, left)

            now = time.perf_counter()
            elapsed = now - t0

            alerts = []

            if right.latest.filt_m is not None and right.latest.filt_m < args.threshold:
                alerts.append(f"RIGHT {right.latest.filt_m:.3f}m")

            if left.latest.filt_m is not None and left.latest.filt_m < args.threshold:
                alerts.append(f"LEFT {left.latest.filt_m:.3f}m")

            if (now - right.latest.t_mono) > args.stale:
                alerts.append("RIGHT STALE")

            if (now - left.latest.t_mono) > args.stale:
                alerts.append("LEFT STALE")

            alert = ", ".join(alerts) if alerts else "-"

            print(
                f"{elapsed:6.2f} | "
                f"{fmt_m(right.latest.raw_m)} {fmt_m(right.latest.filt_m)} | "
                f"{fmt_m(left.latest.raw_m)} {fmt_m(left.latest.filt_m)} | "
                f"{alert}"
            )

            if args.duration > 0 and elapsed >= args.duration:
                break

            sleep_s = period - (time.perf_counter() - loop_t)
            if sleep_s > 0:
                time.sleep(sleep_s)

    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)

    finally:
        stop_sensor(tca, right)
        stop_sensor(tca, left)
        tca.disable_all()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())