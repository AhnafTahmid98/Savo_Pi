#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Find and validate the RPLIDAR serial port."""

from __future__ import annotations

import argparse
import sys

from savo_lidar.constants import DEFAULT_SERIAL_PORT
from savo_lidar.diagnostics import (
    check_lidar_port,
    compact_status_line,
    format_json_report,
    format_key_value_report,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="find_lidar_port_cli.py",
        description="Find and validate the Robot Savo RPLIDAR serial port.",
    )
    parser.add_argument(
        "--serial-port",
        default=DEFAULT_SERIAL_PORT,
        help="Preferred RPLIDAR serial port.",
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
    result = check_lidar_port(args.serial_port)

    if args.json:
        print(format_json_report(result.to_dict()))
    elif args.details:
        print(format_key_value_report("LiDAR serial port check", result.to_dict()))
    else:
        print(
            compact_status_line(
                name="lidar_port",
                ok=result.ok,
                message=result.message,
                preferred_port=result.preferred_port,
                selected_port=result.selected_port,
                available_ports=result.available_ports,
                user_in_dialout=result.user_in_dialout,
            )
        )

    return 0 if result.ok else 1


if __name__ == "__main__":
    sys.exit(main())
