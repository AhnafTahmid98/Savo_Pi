#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Dual VL53L1X mux diagnostic for Robot Savo."""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
from typing import Optional

from savo_perception.constants import (
    I2C_BUS_DEFAULT,
    TCA9548A_ADDR_DEFAULT,
    VL53_LEFT_CHANNEL_DEFAULT,
    VL53_MEDIAN_WINDOW_DEFAULT,
    VL53_RIGHT_CHANNEL_DEFAULT,
    VL53L1X_ADDR_DEFAULT,
)
from savo_perception.drivers import Vl53MuxConfig, Vl53MuxDriver, Vl53Reading


def fmt_m(value: Optional[float]) -> str:
    return f"{value:6.3f}" if value is not None else "  --- "


def alert_text(
    left: Vl53Reading,
    right: Vl53Reading,
    *,
    threshold_m: float,
) -> str:
    alerts: list[str] = []

    if right.filtered_m is not None and right.filtered_m < threshold_m:
        alerts.append(f"RIGHT {right.filtered_m:.3f}m")

    if left.filtered_m is not None and left.filtered_m < threshold_m:
        alerts.append(f"LEFT {left.filtered_m:.3f}m")

    if right.error:
        alerts.append(f"RIGHT {right.error}")

    if left.error:
        alerts.append(f"LEFT {left.error}")

    return ", ".join(alerts) if alerts else "-"


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
            "right_raw_m",
            "right_filtered_m",
            "left_raw_m",
            "left_filtered_m",
            "right_channel",
            "left_channel",
            "alert",
        ]
    )
    return f, writer


def write_csv_row(
    writer,
    *,
    elapsed_s: float,
    left: Vl53Reading,
    right: Vl53Reading,
    alert: str,
) -> None:
    if writer is None:
        return

    writer.writerow(
        [
            f"{elapsed_s:.3f}",
            "" if right.raw_m is None else f"{right.raw_m:.6f}",
            "" if right.filtered_m is None else f"{right.filtered_m:.6f}",
            "" if left.raw_m is None else f"{left.raw_m:.6f}",
            "" if left.filtered_m is None else f"{left.filtered_m:.6f}",
            right.channel,
            left.channel,
            alert,
        ]
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Robot Savo dual VL53L1X TCA9548A mux diagnostic"
    )

    parser.add_argument("--bus", type=int, default=I2C_BUS_DEFAULT)
    parser.add_argument("--tca-addr", type=lambda s: int(s, 0), default=TCA9548A_ADDR_DEFAULT)
    parser.add_argument("--vl53-addr", type=lambda s: int(s, 0), default=VL53L1X_ADDR_DEFAULT)

    parser.add_argument("--right-ch", type=int, default=VL53_RIGHT_CHANNEL_DEFAULT)
    parser.add_argument("--left-ch", type=int, default=VL53_LEFT_CHANNEL_DEFAULT)

    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--median", type=int, default=VL53_MEDIAN_WINDOW_DEFAULT)
    parser.add_argument("--threshold", type=float, default=0.035)
    parser.add_argument("--duration", type=float, default=0.0, help="0 = run until Ctrl+C")

    parser.add_argument("--valid-min", type=float, default=0.02)
    parser.add_argument("--valid-max", type=float, default=4.00)
    parser.add_argument("--csv", type=str, default=None)

    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.median < 1 or args.median % 2 == 0:
        parser.error("--median must be odd and >= 1")

    cfg = Vl53MuxConfig(
        bus=args.bus,
        tca_addr=args.tca_addr,
        vl53_addr=args.vl53_addr,
        right_channel=args.right_ch,
        left_channel=args.left_ch,
        median_window=args.median,
        valid_min_m=args.valid_min,
        valid_max_m=args.valid_max,
    )

    driver = Vl53MuxDriver(cfg)
    fcsv, writer = open_csv(args.csv)

    print(f"[TCA9548A] bus=i2c-{cfg.bus} address=0x{cfg.tca_addr:02X}")
    print(f"[VL53L1X] address=0x{cfg.vl53_addr:02X}")
    print(f"[Mapping] RIGHT=channel {cfg.right_channel}  LEFT=channel {cfg.left_channel}")
    print(f"[Timing] rate={args.rate:.2f}Hz  median={cfg.median_window}")
    print(f"[Valid] {cfg.valid_min_m:.2f}m .. {cfg.valid_max_m:.2f}m")
    if args.csv:
        print(f"[CSV] {Path(args.csv).expanduser()}")

    period_s = 1.0 / max(float(args.rate), 0.1)
    t0 = time.monotonic()

    print("\n time_s | R_raw  R_filt | L_raw  L_filt | Alert")
    print("-" * 62)

    try:
        driver.start()

        while True:
            loop_start = time.monotonic()

            left, right = driver.read_once()
            elapsed = time.monotonic() - t0
            alert = alert_text(left, right, threshold_m=args.threshold)

            print(
                f"{elapsed:6.2f} | "
                f"{fmt_m(right.raw_m)} {fmt_m(right.filtered_m)} | "
                f"{fmt_m(left.raw_m)} {fmt_m(left.filtered_m)} | "
                f"{alert}"
            )

            write_csv_row(
                writer,
                elapsed_s=elapsed,
                left=left,
                right=right,
                alert=alert,
            )

            if args.duration > 0.0 and elapsed >= args.duration:
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

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
