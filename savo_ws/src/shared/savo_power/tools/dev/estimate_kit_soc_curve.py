#!/usr/bin/env python3
"""Estimate a simple voltage-to-SoC curve for the Savo base battery.

This is an offline developer tool. It does not read hardware. Use it with
manual voltage samples or saved CSV data while tuning base battery thresholds.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


DEFAULT_EMPTY_VOLTAGE_V = 6.40
DEFAULT_FULL_VOLTAGE_V = 8.40


def clamp(value, low, high):
    return max(low, min(high, value))


def estimate_soc_pct(
    voltage_v,
    *,
    empty_voltage_v=DEFAULT_EMPTY_VOLTAGE_V,
    full_voltage_v=DEFAULT_FULL_VOLTAGE_V,
):
    span = full_voltage_v - empty_voltage_v

    if span <= 0:
        raise ValueError("full voltage must be greater than empty voltage")

    ratio = (float(voltage_v) - empty_voltage_v) / span

    return clamp(ratio * 100.0, 0.0, 100.0)


def estimate_voltage_for_soc(
    soc_pct,
    *,
    empty_voltage_v=DEFAULT_EMPTY_VOLTAGE_V,
    full_voltage_v=DEFAULT_FULL_VOLTAGE_V,
):
    soc = clamp(float(soc_pct), 0.0, 100.0)

    return empty_voltage_v + ((full_voltage_v - empty_voltage_v) * (soc / 100.0))


def load_voltage_samples_csv(path):
    rows = []

    with Path(path).open(newline="") as handle:
        reader = csv.DictReader(handle)

        for row in reader:
            voltage_text = (
                row.get("voltage_v")
                or row.get("battery_voltage_v")
                or row.get("voltage")
            )

            if voltage_text in ("", None):
                continue

            rows.append(float(voltage_text))

    return rows


def build_curve(
    *,
    empty_voltage_v=DEFAULT_EMPTY_VOLTAGE_V,
    full_voltage_v=DEFAULT_FULL_VOLTAGE_V,
    step_pct=10.0,
):
    if step_pct <= 0:
        raise ValueError("step percent must be positive")

    points = []
    soc = 0.0

    while soc <= 100.000001:
        points.append(
            {
                "soc_pct": round(soc, 2),
                "voltage_v": round(
                    estimate_voltage_for_soc(
                        soc,
                        empty_voltage_v=empty_voltage_v,
                        full_voltage_v=full_voltage_v,
                    ),
                    3,
                ),
            }
        )
        soc += step_pct

    if points[-1]["soc_pct"] != 100.0:
        points.append(
            {
                "soc_pct": 100.0,
                "voltage_v": round(full_voltage_v, 3),
            }
        )

    return points


def summarize_samples(
    samples,
    *,
    empty_voltage_v=DEFAULT_EMPTY_VOLTAGE_V,
    full_voltage_v=DEFAULT_FULL_VOLTAGE_V,
):
    estimates = [
        estimate_soc_pct(
            voltage,
            empty_voltage_v=empty_voltage_v,
            full_voltage_v=full_voltage_v,
        )
        for voltage in samples
    ]

    if not samples:
        return {
            "samples": 0,
            "voltage_min": None,
            "voltage_max": None,
            "soc_min": None,
            "soc_max": None,
        }

    return {
        "samples": len(samples),
        "voltage_min": min(samples),
        "voltage_max": max(samples),
        "soc_min": min(estimates),
        "soc_max": max(estimates),
    }


def format_curve(points):
    lines = [
        "SoC % | Voltage V",
        "------+----------",
    ]

    for point in points:
        lines.append(f"{point['soc_pct']:>5.1f} | {point['voltage_v']:>8.3f}")

    return "\n".join(lines)


def format_summary(summary):
    if summary["samples"] == 0:
        return "samples: 0"

    return "\n".join(
        [
            f"samples: {summary['samples']}",
            f"voltage min: {summary['voltage_min']:.3f} V",
            f"voltage max: {summary['voltage_max']:.3f} V",
            f"soc min: {summary['soc_min']:.1f} %",
            f"soc max: {summary['soc_max']:.1f} %",
        ]
    )


def build_parser():
    parser = argparse.ArgumentParser(
        description="Estimate Savo base battery voltage-to-SoC values.",
    )
    parser.add_argument(
        "--empty-voltage",
        type=float,
        default=DEFAULT_EMPTY_VOLTAGE_V,
        help="Voltage treated as 0 percent SoC.",
    )
    parser.add_argument(
        "--full-voltage",
        type=float,
        default=DEFAULT_FULL_VOLTAGE_V,
        help="Voltage treated as 100 percent SoC.",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=10.0,
        help="Curve step in percent.",
    )
    parser.add_argument(
        "--voltage",
        type=float,
        action="append",
        default=[],
        help="Voltage sample to estimate. Can be passed multiple times.",
    )
    parser.add_argument(
        "--csv",
        dest="csv_path",
        help="Optional CSV file with voltage_v or battery_voltage_v column.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print JSON output.",
    )

    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    samples = list(args.voltage)

    if args.csv_path:
        samples.extend(load_voltage_samples_csv(args.csv_path))

    curve = build_curve(
        empty_voltage_v=args.empty_voltage,
        full_voltage_v=args.full_voltage,
        step_pct=args.step,
    )
    summary = summarize_samples(
        samples,
        empty_voltage_v=args.empty_voltage,
        full_voltage_v=args.full_voltage,
    )

    if args.json:
        print(
            json.dumps(
                {
                    "curve": curve,
                    "sample_summary": summary,
                },
                indent=2,
                sort_keys=True,
            )
        )
    else:
        print(format_curve(curve))

        if samples:
            print()
            print(format_summary(summary))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
