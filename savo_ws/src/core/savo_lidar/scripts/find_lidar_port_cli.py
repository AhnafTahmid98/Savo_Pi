#!/usr/bin/env python3
"""Check which serial port should be used for the RPLIDAR A1."""

from __future__ import annotations

import argparse
import json
import sys

from savo_lidar.diagnostics import (
    check_lidar_port,
    compact_status_line,
    format_json_report,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Find and validate the Robot Savo RPLIDAR serial port.",
    )
    parser.add_argument(
        "--serial-port",
        default="/dev/ttyUSB0",
        help="Preferred RPLIDAR serial port.",
    )
    parser.add_argument(
        "--json",
        default="false",
        help="Print JSON output. Use true/false.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    result = check_lidar_port(args.serial_port)

    json_enabled = str(args.json).strip().lower() in ("1", "true", "yes", "on")

    if json_enabled:
        print(format_json_report(result.to_dict()))
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