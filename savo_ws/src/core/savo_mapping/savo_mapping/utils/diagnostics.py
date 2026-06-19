#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Diagnostic helpers for Robot Savo mapping. No ROS imports."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Iterable, Optional


# =============================================================================
# Diagnostic levels
# =============================================================================
class DiagnosticLevel(str, Enum):
    OK = "ok"
    WARN = "warn"
    ERROR = "error"
    STALE = "stale"
    DISABLED = "disabled"

    @classmethod
    def values(cls) -> tuple[str, ...]:
        return tuple(level.value for level in cls)

    @classmethod
    def from_string(cls, value: str) -> "DiagnosticLevel":
        normalized = normalize_diagnostic_level(value)

        for level in cls:
            if level.value == normalized:
                return level

        raise ValueError(f"Unsupported diagnostic level: {value!r}")


def normalize_diagnostic_level(value: str) -> str:
    level = str(value).strip().lower().replace("-", "_").replace(" ", "_")

    if not level:
        raise ValueError("Diagnostic level cannot be empty.")

    return level


def require_valid_diagnostic_level(value: str) -> str:
    return DiagnosticLevel.from_string(value).value


# =============================================================================
# Diagnostic item
# =============================================================================
@dataclass(frozen=True)
class DiagnosticItem:
    name: str
    level: str = DiagnosticLevel.OK.value
    message: str = ""
    enabled: bool = True
    required: bool = False
    timestamp_s: float = field(default_factory=time.time)
    values: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        require_valid_diagnostic_level(self.level)

    @property
    def ok(self) -> bool:
        if not self.enabled:
            return True

        return self.level == DiagnosticLevel.OK.value

    @property
    def failed(self) -> bool:
        return self.enabled and self.level in (
            DiagnosticLevel.ERROR.value,
            DiagnosticLevel.STALE.value,
        )

    @property
    def warning(self) -> bool:
        return self.enabled and self.level == DiagnosticLevel.WARN.value

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "level": self.level,
            "ok": self.ok,
            "failed": self.failed,
            "warning": self.warning,
            "enabled": self.enabled,
            "required": self.required,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "values": dict(self.values),
        }


# =============================================================================
# Diagnostic report
# =============================================================================
@dataclass(frozen=True)
class DiagnosticReport:
    name: str = "savo_mapping"
    items: Dict[str, DiagnosticItem] = field(default_factory=dict)
    timestamp_s: float = field(default_factory=time.time)

    @property
    def ok(self) -> bool:
        return all(item.ok for item in self.items.values() if item.required)

    @property
    def degraded(self) -> bool:
        return any(
            item.enabled and item.warning
            for item in self.items.values()
        ) or any(
            item.enabled and item.failed
            for item in self.items.values()
            if not item.required
        )

    @property
    def failed_required(self) -> tuple[str, ...]:
        return tuple(
            name
            for name, item in self.items.items()
            if item.required and item.failed
        )

    @property
    def warnings(self) -> tuple[str, ...]:
        return tuple(
            name
            for name, item in self.items.items()
            if item.warning
        )

    def with_item(self, item: DiagnosticItem) -> "DiagnosticReport":
        updated = dict(self.items)
        updated[item.name] = item

        return DiagnosticReport(
            name=self.name,
            items=updated,
            timestamp_s=time.time(),
        )

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "ok": self.ok,
            "degraded": self.degraded,
            "failed_required": list(self.failed_required),
            "warnings": list(self.warnings),
            "timestamp_s": self.timestamp_s,
            "items": {
                name: item.to_dict()
                for name, item in self.items.items()
            },
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Builders
# =============================================================================
def make_diagnostic_item(
    name: str,
    level: str = DiagnosticLevel.OK.value,
    message: str = "",
    enabled: bool = True,
    required: bool = False,
    values: Optional[Dict[str, Any]] = None,
) -> DiagnosticItem:
    return DiagnosticItem(
        name=str(name).strip(),
        level=require_valid_diagnostic_level(level),
        message=str(message),
        enabled=bool(enabled),
        required=bool(required),
        values=dict(values or {}),
    )


def make_ok(
    name: str,
    message: str = "OK",
    required: bool = False,
    values: Optional[Dict[str, Any]] = None,
) -> DiagnosticItem:
    return make_diagnostic_item(
        name=name,
        level=DiagnosticLevel.OK.value,
        message=message,
        required=required,
        values=values,
    )


def make_warn(
    name: str,
    message: str,
    required: bool = False,
    values: Optional[Dict[str, Any]] = None,
) -> DiagnosticItem:
    return make_diagnostic_item(
        name=name,
        level=DiagnosticLevel.WARN.value,
        message=message,
        required=required,
        values=values,
    )


def make_error(
    name: str,
    message: str,
    required: bool = False,
    values: Optional[Dict[str, Any]] = None,
) -> DiagnosticItem:
    return make_diagnostic_item(
        name=name,
        level=DiagnosticLevel.ERROR.value,
        message=message,
        required=required,
        values=values,
    )


def make_stale(
    name: str,
    message: str,
    required: bool = False,
    values: Optional[Dict[str, Any]] = None,
) -> DiagnosticItem:
    return make_diagnostic_item(
        name=name,
        level=DiagnosticLevel.STALE.value,
        message=message,
        required=required,
        values=values,
    )


def make_disabled(
    name: str,
    message: str = "Disabled",
    required: bool = False,
    values: Optional[Dict[str, Any]] = None,
) -> DiagnosticItem:
    return make_diagnostic_item(
        name=name,
        level=DiagnosticLevel.DISABLED.value,
        message=message,
        enabled=False,
        required=required,
        values=values,
    )


def build_diagnostic_report(
    name: str,
    items: Iterable[DiagnosticItem],
) -> DiagnosticReport:
    report = DiagnosticReport(name=str(name).strip() or "savo_mapping")

    for item in items:
        report = report.with_item(item)

    return report


def merge_reports(
    name: str,
    reports: Iterable[DiagnosticReport],
) -> DiagnosticReport:
    merged = DiagnosticReport(name=str(name).strip() or "savo_mapping")

    for report in reports:
        for item in report.items.values():
            merged = merged.with_item(item)

    return merged


# =============================================================================
# Formatting
# =============================================================================
def format_diagnostic_report(report: DiagnosticReport) -> str:
    lines = [
        f"{report.name}: ok={report.ok} degraded={report.degraded}",
    ]

    for name in sorted(report.items.keys()):
        item = report.items[name]
        lines.append(
            f"- {name}: {item.level} | required={item.required} | {item.message}"
        )

    return "\n".join(lines)


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    report = build_diagnostic_report(
        "savo_mapping_demo",
        (
            make_ok("scan", "LiDAR scan OK.", required=True),
            make_ok("odom", "Filtered odometry OK.", required=True),
            make_stale("pointcloud", "Pointcloud stale.", required=False),
            make_disabled("apriltag", "AprilTag bridge not enabled."),
        ),
    )

    print(format_diagnostic_report(report))
    print(report.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "DiagnosticLevel",
    "DiagnosticItem",
    "DiagnosticReport",
    "normalize_diagnostic_level",
    "require_valid_diagnostic_level",
    "make_diagnostic_item",
    "make_ok",
    "make_warn",
    "make_error",
    "make_stale",
    "make_disabled",
    "build_diagnostic_report",
    "merge_reports",
    "format_diagnostic_report",
    "main",
]