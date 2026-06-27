#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Sample Robot Savo HC-SR04 ultrasonic sensor from CLI."""

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
from savo_perception.drivers import UltrasonicConfig, UltrasonicDriver


def fmt_m(value: Optional[float]) -> str:
    return f"{value:6.3f}" if value is not None else "  --- "


def fmt_cm(value: Optional[float]) -> str:
    return f"{value * 100.0:6.1f}" if value is not None else "  --- "


def open_csv(path: Optional[str]):
    if not path:
        return None, None

    csv_path = Path(path).expanduser()
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    f = csv_path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(f)
    writer.writerow(["time_s", "distance_m", "distance_cm", "valid", "error"])
    return f, writer


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Sample Robot Savo HC-SR04 ultrasonic sensor"
    )

    parser.add_argument("--trig", type=int, default=ULTRASONIC_TRIG_PIN_DEFAULT)
    parser.add_argument("--echo", type=int, default=ULTRASONIC_ECHO_PIN_DEFAULT)
    parser.add_argument("--max-distance", type=float, default=ULTRASONIC_MAX_DISTANCE_M_DEFAULT)
    parser.add_argument("--valid-min", type=float, default=0.02)
    parser.add_argument("--valid-max", type=float, default=ULTRASONIC_MAX_DISTANCE_M_DEFAULT)

    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--samples", type=int, default=20, help="0 = run until Ctrl+C")
    parser.add_argument("--csv", type=str, default=None)

    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

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
        f"[UltrasonicSample] TRIG={cfg.trig_pin} ECHO={cfg.echo_pin} "
        f"max={cfg.max_distance_m:.2f}m"
    )
    print(f"[Valid] {cfg.valid_min_m:.2f}m .. {cfg.valid_max_m:.2f}m")
    print(f"[Timing] rate={args.rate:.2f}Hz samples={args.samples}")
    if args.csv:
        print(f"[CSV] {Path(args.csv).expanduser()}")

    period_s = 1.0 / max(float(args.rate), 0.1)
    t0 = time.monotonic()
    values: list[float] = []

    print()
    print(" time_s | distance_m | distance_cm | state")
    print("-" * 52)

    try:
        driver.start()

        count = 0
        while True:
            loop_start = time.monotonic()

            reading = driver.read_once()
            elapsed = time.monotonic() - t0
            distance = reading.distance_m
            valid = distance is not None

            if valid:
                values.append(float(distance))

            state = "OK" if valid else reading.error or "INVALID"

            print(
                f"{elapsed:6.2f} | "
                f"{fmt_m(distance)}     | "
                f"{fmt_cm(distance)} cm | "
                f"{state}"
            )

            if writer is not None:
                writer.writerow(
                    [
                        f"{elapsed:.3f}",
                        "" if distance is None else f"{distance:.6f}",
                        "" if distance is None else f"{distance * 100.0:.3f}",
                        valid,
                        reading.error,
                    ]
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

    if values:
        avg = mean(values)
        std = pstdev(values) if len(values) > 1 else 0.0
        print(
            f"\nSummary: N={len(values)} "
            f"mean={avg * 100.0:.1f}cm std={std * 100.0:.1f}cm"
        )
    else:
        print("\nSummary: No valid ultrasonic readings.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())