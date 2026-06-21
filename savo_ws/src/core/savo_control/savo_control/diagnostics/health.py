# -*- coding: utf-8 -*-

"""Health helpers for savo_control diagnostics and dashboards."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .report_formatter import (
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_WARN,
    DiagnosticItem,
    summarize_items,
)


@dataclass(frozen=True)
class HealthCheck:
    name: str
    ok: bool
    stale: bool = False
    warning: bool = False
    value: object = ""
    note: str = ""

    def status(self) -> str:
        if self.stale:
            return STATUS_STALE
        if not self.ok:
            return STATUS_ERROR
        if self.warning:
            return STATUS_WARN
        return STATUS_OK

    def to_diagnostic_item(self) -> DiagnosticItem:
        return DiagnosticItem(
            name=self.name,
            status=self.status(),
            value=self.value,
            note=self.note,
        )

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "ok": self.ok,
            "stale": self.stale,
            "warning": self.warning,
            "status": self.status(),
            "value": self.value,
            "note": self.note,
        }


@dataclass(frozen=True)
class HealthReport:
    name: str
    checks: tuple[HealthCheck, ...]

    @property
    def overall(self) -> str:
        return summarize_items(check.to_diagnostic_item() for check in self.checks)

    def ok(self) -> bool:
        return self.overall == STATUS_OK

    def has_errors(self) -> bool:
        return self.overall == STATUS_ERROR

    def has_stale(self) -> bool:
        return any(check.status() == STATUS_STALE for check in self.checks)

    def to_items(self) -> list[DiagnosticItem]:
        return [check.to_diagnostic_item() for check in self.checks]

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "overall": self.overall,
            "ok": self.ok(),
            "checks": [check.to_dict() for check in self.checks],
        }

    def status_text(self) -> str:
        lines = [f"{self.name}: {self.overall}"]
        lines.extend(item.to_line() for item in self.to_items())
        return "\n".join(lines)


def make_health_report(name: str, checks: Iterable[HealthCheck]) -> HealthReport:
    return HealthReport(
        name=name,
        checks=tuple(checks),
    )


def freshness_check(
    name: str,
    *,
    age_s: float,
    timeout_s: float,
    value: object = "",
) -> HealthCheck:
    stale = age_s > timeout_s

    return HealthCheck(
        name=name,
        ok=not stale,
        stale=stale,
        value=value if value != "" else f"{age_s:.3f}s",
        note=f"timeout={timeout_s:.3f}s",
    )


def bool_health_check(
    name: str,
    *,
    ok: bool,
    value: object = "",
    note: str = "",
) -> HealthCheck:
    return HealthCheck(
        name=name,
        ok=ok,
        value=value,
        note=note,
    )


def safety_health_check(
    *,
    safety_stop: bool,
    slowdown_factor: float = 1.0,
) -> HealthCheck:
    if safety_stop:
        return HealthCheck(
            name="safety_stop",
            ok=False,
            value=True,
            note="safety stop active",
        )

    warning = slowdown_factor < 1.0

    return HealthCheck(
        name="safety_stop",
        ok=True,
        warning=warning,
        value=False,
        note=f"slowdown={slowdown_factor:.2f}",
    )


def command_health_check(
    *,
    command_fresh: bool,
    selected_source: str,
) -> HealthCheck:
    return HealthCheck(
        name="command",
        ok=command_fresh,
        stale=not command_fresh,
        value=selected_source,
        note="selected command source",
    )


def odom_health_check(*, odom_fresh: bool, age_s: float | None = None) -> HealthCheck:
    return HealthCheck(
        name="odometry",
        ok=odom_fresh,
        stale=not odom_fresh,
        value="" if age_s is None else f"{age_s:.3f}s",
        note="filtered odom freshness",
    )


def status_name_for_checks(checks: Iterable[HealthCheck]) -> str:
    report = make_health_report("status", checks)
    overall = report.overall

    if overall in {
        STATUS_OK,
        STATUS_WARN,
        STATUS_ERROR,
        STATUS_STALE,
    }:
        return overall

    return STATUS_UNKNOWN


__all__ = [
    "HealthCheck",
    "HealthReport",
    "bool_health_check",
    "command_health_check",
    "freshness_check",
    "make_health_report",
    "odom_health_check",
    "safety_health_check",
    "status_name_for_checks",
]
