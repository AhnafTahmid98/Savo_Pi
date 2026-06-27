#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Report formatting helpers for perception diagnostics."""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Iterable, List, Optional

from savo_perception.constants import PACKAGE_NAME, STATUS_ERROR, STATUS_OK, STATUS_STALE


@dataclass(frozen=True)
class ReportRow:
    name: str
    status: str
    message: str
    data: Dict[str, Any] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.status == STATUS_OK

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class DiagnosticReport:
    title: str
    rows: List[ReportRow] = field(default_factory=list)
    package: str = PACKAGE_NAME
    stamp_mono_s: float = field(default_factory=time.monotonic)

    def add(
        self,
        *,
        name: str,
        status: str,
        message: str,
        data: Optional[Dict[str, Any]] = None,
    ) -> None:
        self.rows.append(
            ReportRow(
                name=name,
                status=status,
                message=message,
                data=dict(data or {}),
            )
        )

    def add_ok(
        self,
        name: str,
        message: str,
        data: Optional[Dict[str, Any]] = None,
    ) -> None:
        self.add(name=name, status=STATUS_OK, message=message, data=data)

    def add_stale(
        self,
        name: str,
        message: str,
        data: Optional[Dict[str, Any]] = None,
    ) -> None:
        self.add(name=name, status=STATUS_STALE, message=message, data=data)

    def add_error(
        self,
        name: str,
        message: str,
        data: Optional[Dict[str, Any]] = None,
    ) -> None:
        self.add(name=name, status=STATUS_ERROR, message=message, data=data)

    @property
    def ok(self) -> bool:
        return all(row.ok for row in self.rows)

    @property
    def status(self) -> str:
        if self.ok:
            return STATUS_OK
        if any(row.status == STATUS_ERROR for row in self.rows):
            return STATUS_ERROR
        return STATUS_STALE

    def to_dict(self) -> Dict[str, Any]:
        return {
            "package": self.package,
            "title": self.title,
            "ok": self.ok,
            "status": self.status,
            "stamp_mono_s": self.stamp_mono_s,
            "rows": [row.to_dict() for row in self.rows],
        }

    def to_json(self, *, pretty: bool = False) -> str:
        if pretty:
            return json.dumps(self.to_dict(), indent=2, sort_keys=True)
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))

    def to_text(self) -> str:
        return format_report_text(self)


def status_mark(status: str) -> str:
    if status == STATUS_OK:
        return "PASS"
    if status == STATUS_STALE:
        return "STALE"
    return "FAIL"


def format_row_text(row: ReportRow, *, name_width: int = 24) -> str:
    mark = status_mark(row.status)
    return f"{mark:5s} | {row.name:<{name_width}s} | {row.message}"


def format_report_text(report: DiagnosticReport) -> str:
    name_width = max([24] + [len(row.name) for row in report.rows])

    lines = [
        f"[{report.package}] {report.title}",
        "-" * (name_width + 32),
    ]

    for row in report.rows:
        lines.append(format_row_text(row, name_width=name_width))

    lines.extend(
        [
            "-" * (name_width + 32),
            f"Result: {status_mark(report.status)} ({report.status})",
        ]
    )

    return "\n".join(lines)


def report_from_rows(title: str, rows: Iterable[ReportRow]) -> DiagnosticReport:
    report = DiagnosticReport(title=title)
    for row in rows:
        report.rows.append(row)
    return report


def print_report(report: DiagnosticReport, *, json_output: bool = False, pretty_json: bool = True) -> None:
    if json_output:
        print(report.to_json(pretty=pretty_json))
    else:
        print(report.to_text())


__all__ = [
    "ReportRow",
    "DiagnosticReport",
    "status_mark",
    "format_row_text",
    "format_report_text",
    "report_from_rows",
    "print_report",
]