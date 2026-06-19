#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Diagnostic report formatting helpers. No ROS imports."""

from __future__ import annotations

import json
from typing import Any, Mapping, Optional

from savo_mapping.utils.diagnostics import DiagnosticReport


# =============================================================================
# Basic formatting
# =============================================================================
def bool_icon(value: bool) -> str:
    return "OK" if value else "FAIL"


def format_bool(value: bool) -> str:
    return "true" if value else "false"


def format_value(value: Any) -> str:
    if value is None:
        return "none"

    if isinstance(value, bool):
        return format_bool(value)

    if isinstance(value, float):
        return f"{value:.3f}"

    return str(value)


# =============================================================================
# Report formatting
# =============================================================================
def format_report_summary(report: DiagnosticReport) -> str:
    return (
        f"{report.name}: "
        f"ok={format_bool(report.ok)} "
        f"degraded={format_bool(report.degraded)} "
        f"failed_required={len(report.failed_required)} "
        f"warnings={len(report.warnings)}"
    )


def format_report_table(report: DiagnosticReport) -> str:
    lines = [
        format_report_summary(report),
        "",
        "name | level | required | enabled | message",
        "--- | --- | --- | --- | ---",
    ]

    for name in sorted(report.items.keys()):
        item = report.items[name]
        lines.append(
            " | ".join(
                (
                    name,
                    item.level,
                    format_bool(item.required),
                    format_bool(item.enabled),
                    item.message,
                )
            )
        )

    return "\n".join(lines)


def format_report_compact(report: DiagnosticReport) -> str:
    lines = [format_report_summary(report)]

    for name in sorted(report.items.keys()):
        item = report.items[name]
        lines.append(
            f"- {name}: {item.level} "
            f"required={format_bool(item.required)} "
            f"enabled={format_bool(item.enabled)} "
            f"{item.message}"
        )

    return "\n".join(lines)


def format_report_json(
    report: DiagnosticReport,
    *,
    indent: Optional[int] = 2,
) -> str:
    return report.to_json(indent=indent)


def format_mapping_status_dict(status: Mapping[str, Any]) -> str:
    lines = ["Robot Savo mapping status:"]

    for key in sorted(status.keys()):
        value = status[key]

        if isinstance(value, Mapping):
            lines.append(f"- {key}:")
            for sub_key in sorted(value.keys()):
                lines.append(f"  - {sub_key}: {format_value(value[sub_key])}")
        else:
            lines.append(f"- {key}: {format_value(value)}")

    return "\n".join(lines)


def format_key_value_block(
    title: str,
    values: Mapping[str, Any],
) -> str:
    lines = [str(title)]

    for key in sorted(values.keys()):
        lines.append(f"- {key}: {format_value(values[key])}")

    return "\n".join(lines)


# =============================================================================
# Selection helpers
# =============================================================================
def format_report(
    report: DiagnosticReport,
    style: str = "compact",
) -> str:
    value = str(style).strip().lower().replace("-", "_")

    if value == "compact":
        return format_report_compact(report)

    if value == "table":
        return format_report_table(report)

    if value == "json":
        return format_report_json(report)

    raise ValueError(f"Unsupported report style: {style}")


def parse_json_report(text: str) -> dict:
    value = str(text).strip()

    if not value:
        return {}

    loaded = json.loads(value)

    if not isinstance(loaded, dict):
        raise ValueError("Report JSON must decode to a dictionary.")

    return loaded


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    from savo_mapping.utils.diagnostics import (
        build_diagnostic_report,
        make_disabled,
        make_ok,
        make_stale,
    )

    report = build_diagnostic_report(
        "savo_mapping_demo",
        (
            make_ok("scan", "LiDAR scan OK.", required=True),
            make_ok("odom", "Filtered odometry OK.", required=True),
            make_stale("pointcloud", "Pointcloud stale.", required=False),
            make_disabled("apriltag", "AprilTag disabled."),
        ),
    )

    print(format_report(report, "compact"))
    print()
    print(format_report(report, "table"))


if __name__ == "__main__":
    main()


__all__ = [
    "bool_icon",
    "format_bool",
    "format_value",
    "format_report_summary",
    "format_report_table",
    "format_report_compact",
    "format_report_json",
    "format_mapping_status_dict",
    "format_key_value_block",
    "format_report",
    "parse_json_report",
    "main",
]