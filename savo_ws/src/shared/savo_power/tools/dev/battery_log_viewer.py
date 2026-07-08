#!/usr/bin/env python3
"""View saved Savo power battery logs.

This is an offline developer tool. It reads JSON/JSONL/CSV files that were
already captured from power topics or diagnostics.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from statistics import mean


VOLTAGE_KEYS = (
    "voltage_v",
    "battery_voltage_v",
    "bus_voltage_v",
)

PERCENT_KEYS = (
    "capacity_pct",
    "soc_pct",
    "battery_pct",
)

SOURCE_KEYS = (
    "source",
    "name",
    "battery_source",
)


def _as_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _first_value(row, keys):
    for key in keys:
        if key in row and row[key] not in ("", None):
            return row[key]

    return None


def _read_json_file(path):
    data = json.loads(path.read_text())

    if isinstance(data, list):
        return [item for item in data if isinstance(item, dict)]

    if isinstance(data, dict):
        if isinstance(data.get("records"), list):
            return [item for item in data["records"] if isinstance(item, dict)]
        return [data]

    return []


def _read_jsonl_file(path):
    rows = []

    for line_number, line in enumerate(path.read_text().splitlines(), start=1):
        line = line.strip()

        if not line:
            continue

        try:
            item = json.loads(line)
        except json.JSONDecodeError as error:
            raise ValueError(f"{path}:{line_number}: invalid JSON line") from error

        if isinstance(item, dict):
            rows.append(item)

    return rows


def _read_csv_file(path):
    with path.open(newline="") as handle:
        return [dict(row) for row in csv.DictReader(handle)]


def read_log(path):
    path = Path(path)

    if not path.is_file():
        raise FileNotFoundError(path)

    suffix = path.suffix.lower()

    if suffix == ".jsonl":
        return _read_jsonl_file(path)

    if suffix == ".json":
        return _read_json_file(path)

    if suffix == ".csv":
        return _read_csv_file(path)

    raise ValueError(f"unsupported log format: {suffix}")


def summarize_rows(rows):
    voltages = []
    percentages = []
    sources = {}

    for row in rows:
        source = str(_first_value(row, SOURCE_KEYS) or "unknown")
        sources[source] = sources.get(source, 0) + 1

        voltage = _as_float(_first_value(row, VOLTAGE_KEYS))
        percent = _as_float(_first_value(row, PERCENT_KEYS))

        if voltage is not None:
            voltages.append(voltage)

        if percent is not None:
            percentages.append(percent)

    return {
        "samples": len(rows),
        "sources": sources,
        "voltage_min": min(voltages) if voltages else None,
        "voltage_avg": mean(voltages) if voltages else None,
        "voltage_max": max(voltages) if voltages else None,
        "percent_min": min(percentages) if percentages else None,
        "percent_avg": mean(percentages) if percentages else None,
        "percent_max": max(percentages) if percentages else None,
    }


def format_value(value, digits=2):
    if value is None:
        return "n/a"

    return f"{value:.{digits}f}"


def format_summary(summary):
    lines = [
        f"samples: {summary['samples']}",
        "sources:",
    ]

    for source, count in sorted(summary["sources"].items()):
        lines.append(f"  {source}: {count}")

    lines.extend(
        [
            "voltage:",
            f"  min: {format_value(summary['voltage_min'])} V",
            f"  avg: {format_value(summary['voltage_avg'])} V",
            f"  max: {format_value(summary['voltage_max'])} V",
            "percent:",
            f"  min: {format_value(summary['percent_min'])} %",
            f"  avg: {format_value(summary['percent_avg'])} %",
            f"  max: {format_value(summary['percent_max'])} %",
        ]
    )

    return "\n".join(lines)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Summarize saved Savo power battery logs.",
    )
    parser.add_argument(
        "log_file",
        help="Path to a JSON, JSONL, or CSV battery log.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print summary as JSON.",
    )

    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    rows = read_log(args.log_file)
    summary = summarize_rows(rows)

    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print(format_summary(summary))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
