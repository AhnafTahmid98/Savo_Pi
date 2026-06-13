#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run a quick non-ROS LiDAR smoke test."""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

from savo_lidar.constants import (
    BACKEND_DRYRUN,
    BACKEND_REAL,
    DEFAULT_BAUDRATE,
    DEFAULT_MAX_RANGE_M,
    DEFAULT_MIN_RANGE_M,
    DEFAULT_SCAN_RATE_HZ,
    DEFAULT_SERIAL_PORT,
)
from savo_lidar.diagnostics import (
    check_lidar_port,
    check_motor_spin,
    check_range_quality,
    compact_status_line,
    format_json_report,
    format_key_value_report,
)
from savo_lidar.drivers import create_lidar_driver
from savo_lidar.models import LidarDriverConfig
from savo_lidar.utils.timing import RateTracker


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="lidar_smoke_test_cli.py",
        description="Run a quick Robot Savo LiDAR smoke test.",
    )
    parser.add_argument(
        "--backend",
        default=BACKEND_DRYRUN,
        choices=(BACKEND_DRYRUN, BACKEND_REAL),
        help="Use dryrun for PC testing or real for RPLIDAR A1 hardware.",
    )
    parser.add_argument(
        "--serial-port",
        default=DEFAULT_SERIAL_PORT,
        help="Preferred RPLIDAR serial port when backend=real.",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help="RPLIDAR serial baudrate.",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=5,
        help="Number of scans to read.",
    )
    parser.add_argument(
        "--min-range",
        type=float,
        default=DEFAULT_MIN_RANGE_M,
        help="Minimum valid range in metres.",
    )
    parser.add_argument(
        "--max-range",
        type=float,
        default=DEFAULT_MAX_RANGE_M,
        help="Maximum valid range in metres.",
    )
    parser.add_argument(
        "--expected-rate",
        type=float,
        default=DEFAULT_SCAN_RATE_HZ,
        help="Expected scan rate in Hz.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print JSON output.",
    )
    parser.add_argument(
        "--details",
        action="store_true",
        help="Print detailed text output.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    try:
        _validate_args(args)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    if args.backend == BACKEND_REAL:
        port_check = check_lidar_port(args.serial_port)

        if not port_check.ok:
            result = {
                "ok": False,
                "stage": "port_check",
                "backend": args.backend,
                "serial_port": args.serial_port,
                "port_check": port_check.to_dict(),
            }
            _print_result(result, json_enabled=args.json, details=args.details)
            return 1

    config = LidarDriverConfig(
        backend=args.backend,
        serial_port=args.serial_port,
        baudrate=args.baudrate,
        min_range_m=args.min_range,
        max_range_m=args.max_range,
        expected_scan_rate_hz=args.expected_rate,
    )
    config.validate()

    driver = create_lidar_driver(config)
    rate = RateTracker()

    last_quality = None
    scan_count = 0

    try:
        driver.start()

        for _ in range(args.samples):
            scan = driver.read_scan()
            scan_count += 1

            measured_rate_hz = rate.tick(time.monotonic())

            last_quality = check_range_quality(
                list(scan.ranges),
                min_range_m=args.min_range,
                max_range_m=args.max_range,
            )

        motor = check_motor_spin(
            driver_running=driver.running,
            scan_count=scan_count,
            scan_rate_hz=rate.rate_hz,
            min_scan_rate_hz=max(0.0, args.expected_rate * 0.5),
        )

        ok = bool(last_quality and last_quality.ok and motor.ok)

        result = {
            "ok": ok,
            "backend": args.backend,
            "serial_port": args.serial_port if args.backend == BACKEND_REAL else None,
            "baudrate": args.baudrate if args.backend == BACKEND_REAL else None,
            "samples": args.samples,
            "scan_count": scan_count,
            "scan_rate_hz": rate.rate_hz,
            "range_quality": last_quality.to_dict() if last_quality else None,
            "motor_spin": motor.to_dict(),
        }

        _print_result(result, json_enabled=args.json, details=args.details)
        return 0 if ok else 1

    except Exception as exc:
        result = {
            "ok": False,
            "backend": args.backend,
            "serial_port": args.serial_port if args.backend == BACKEND_REAL else None,
            "error": str(exc),
        }
        _print_result(result, json_enabled=args.json, details=args.details)
        return 1

    finally:
        try:
            driver.stop()
        except Exception:
            pass


def _validate_args(args: argparse.Namespace) -> None:
    if args.samples <= 0:
        raise ValueError("samples must be > 0")

    if args.baudrate <= 0:
        raise ValueError("baudrate must be > 0")

    if args.min_range <= 0.0:
        raise ValueError("min-range must be > 0.0")

    if args.max_range <= args.min_range:
        raise ValueError("max-range must be greater than min-range")

    if args.expected_rate <= 0.0:
        raise ValueError("expected-rate must be > 0.0")


def _print_result(
    result: dict[str, Any],
    *,
    json_enabled: bool,
    details: bool,
) -> None:
    if json_enabled:
        print(format_json_report(result))
        return

    if details:
        print(format_key_value_report("LiDAR smoke test", result))
        return

    print(
        compact_status_line(
            name="lidar_smoke_test",
            ok=bool(result.get("ok", False)),
            message=str(result.get("error") or result.get("stage") or "completed"),
            backend=result.get("backend"),
            serial_port=result.get("serial_port"),
            scan_count=result.get("scan_count"),
            scan_rate_hz=result.get("scan_rate_hz"),
        )
    )


if __name__ == "__main__":
    sys.exit(main())
