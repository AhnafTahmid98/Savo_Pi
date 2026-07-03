#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""HC-SR04 ultrasonic diagnostic for Robot Savo."""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
from statistics import mean, pstdev
from typing import Optional

from savo_perception.constants import (
    ULTRASONIC_ECHO_PIN_DEFAULT,
    ULTRASONIC_MAX_DISTANCE_M_DEFAULT,
    ULTRASONIC_TRIG_PIN_DEFAULT,
)
from savo_perception.drivers import UltrasonicConfig, UltrasonicDriver, UltrasonicReading


def fmt_m(value: Optional[float]) -> str:
    return f"{value:6.3f}" if value is not None else "  --- "


def fmt_cm(value: Optional[float]) -> str:
    return f"{value * 100.0:6.1f}" if value is not None else "  --- "


def alert_text(reading: UltrasonicReading, *, threshold_m: float) -> str:
    if reading.error:
        return reading.error

    if reading.distance_m is not None and reading.distance_m <= threshold_m:
        return f"NEAR {reading.distance_m:.3f}m"

    return "-"


def open_csv(path: Optional[str]):
    if not path:
        return None, None

    csv_path = Path(path).expanduser()
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    f = csv_path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(f)
    writer.writerow(
        [
            "time_s",
            "distance_m",
            "distance_cm",
            "trig_pin",
            "echo_pin",
            "alert",
        ]
    )
    return f, writer


def write_csv_row(
    writer,
    *,
    elapsed_s: float,
    reading: UltrasonicReading,
    alert: str,
) -> None:
    if writer is None:
        return

    distance_m = reading.distance_m
    writer.writerow(
        [
            f"{elapsed_s:.3f}",
            "" if distance_m is None else f"{distance_m:.6f}",
            "" if distance_m is None else f"{distance_m * 100.0:.3f}",
            reading.trig_pin,
            reading.echo_pin,
            alert,
        ]
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Robot Savo HC-SR04 ultrasonic diagnostic"
    )

    parser.add_argument("--trig", type=int, default=ULTRASONIC_TRIG_PIN_DEFAULT)
    parser.add_argument("--echo", type=int, default=ULTRASONIC_ECHO_PIN_DEFAULT)
    parser.add_argument("--max-distance", type=float, default=ULTRASONIC_MAX_DISTANCE_M_DEFAULT)
    parser.add_argument("--valid-min", type=float, default=0.02)
    parser.add_argument("--valid-max", type=float, default=ULTRASONIC_MAX_DISTANCE_M_DEFAULT)
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--samples", type=int, default=40, help="0 = run until Ctrl+C")
    parser.add_argument("--threshold", type=float, default=0.35)
    parser.add_argument("--csv", type=str, default=None)

    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    cfg = UltrasonicConfig(
        trig_pin=args.trig,
        echo_pin=args.echo,
        max_distance_m=args.max_distance,
        valid_min_m=args.valid_min,
        valid_max_m=args.valid_max,
        queue_len=1,
        pin_factory="lgpio",
    )

    driver = UltrasonicDriver(cfg)
    fcsv, writer = open_csv(args.csv)

    print(
        f"[Ultrasonic] TRIG={cfg.trig_pin} ECHO={cfg.echo_pin} "
        f"max={cfg.max_distance_m:.2f}m threshold={args.threshold:.2f}m"
    )
    print(f"[Valid] {cfg.valid_min_m:.2f}m .. {cfg.valid_max_m:.2f}m")
    print(f"[Timing] rate={args.rate:.2f}Hz samples={args.samples}")
    if args.csv:
        print(f"[CSV] {Path(args.csv).expanduser()}")

    period_s = 1.0 / max(float(args.rate), 0.1)
    t0 = time.monotonic()
    valid_values: list[float] = []

    print("\n time_s | distance_m | distance_cm | Alert")
    print("-" * 52)

    try:
        driver.start()

        count = 0
        while True:
            loop_start = time.monotonic()

            reading = driver.read_once()
            elapsed = time.monotonic() - t0
            alert = alert_text(reading, threshold_m=args.threshold)

            if reading.distance_m is not None:
                valid_values.append(reading.distance_m)

            print(
                f"{elapsed:6.2f} | "
                f"{fmt_m(reading.distance_m)}     | "
                f"{fmt_cm(reading.distance_m)} cm | "
                f"{alert}"
            )

            write_csv_row(
                writer,
                elapsed_s=elapsed,
                reading=reading,
                alert=alert,
            )

            count += 1
            if args.samples > 0 and count >= args.samples:
                break

            sleep_s = period_s - (time.monotonic() - loop_start)
            if sleep_s > 0.0:
                time.sleep(sleep_s)

    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)
    finally:
        driver.close()
        if fcsv is not None:
            fcsv.close()

    if valid_values:
        avg = mean(valid_values)
        std = pstdev(valid_values) if len(valid_values) > 1 else 0.0
        print(
            f"\nSummary: N={len(valid_values)} "
            f"mean={avg * 100.0:.1f}cm std={std * 100.0:.1f}cm"
        )
    else:
        print("\nSummary: No valid ultrasonic readings.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
