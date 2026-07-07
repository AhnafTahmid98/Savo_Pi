"""Shared diagnostic report formatting helpers for Robot Savo power tools."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, is_dataclass
from enum import Enum
from typing import Any, Iterable, Mapping


class ReportStatus(str, Enum):
    """Normalized diagnostic report status."""

    OK = "OK"
    WARN = "WARN"
    ERROR = "ERROR"
    MISSING = "MISSING"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class ReportLine:
    """One formatted diagnostic line."""

    name: str
    status: ReportStatus = ReportStatus.UNKNOWN
    detail: str = ""

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "status": self.status.value,
            "detail": self.detail,
        }

    def format_text(self) -> str:
        suffix = f" — {self.detail}" if self.detail else ""
        return f"[{self.status.value}] {self.name}{suffix}"


@dataclass(frozen=True)
class DiagnosticReport:
    """A complete diagnostic report."""

    title: str
    ok: bool
    lines: tuple[ReportLine, ...] = ()
    metadata: tuple[tuple[str, object], ...] = ()

    @property
    def status(self) -> ReportStatus:
        return ReportStatus.OK if self.ok else ReportStatus.ERROR

    @property
    def line_count(self) -> int:
        return len(self.lines)

    @property
    def error_count(self) -> int:
        return sum(1 for line in self.lines if line.status == ReportStatus.ERROR)

    @property
    def missing_count(self) -> int:
        return sum(1 for line in self.lines if line.status == ReportStatus.MISSING)

    @property
    def warning_count(self) -> int:
        return sum(1 for line in self.lines if line.status == ReportStatus.WARN)

    def to_dict(self) -> dict[str, object]:
        return {
            "title": self.title,
            "ok": self.ok,
            "status": self.status.value,
            "line_count": self.line_count,
            "warning_count": self.warning_count,
            "error_count": self.error_count,
            "missing_count": self.missing_count,
            "metadata": {
                key: to_jsonable(value)
                for key, value in self.metadata
            },
            "lines": [line.to_dict() for line in self.lines],
        }

    def format_text(self) -> str:
        lines = [self.title]

        if self.metadata:
            lines.append(format_key_values(self.metadata))

        for line in self.lines:
            lines.append(line.format_text())

        if self.ok:
            lines.append("Result: PASS")
        else:
            lines.append(
                "Result: FAIL "
                f"warnings={self.warning_count} "
                f"errors={self.error_count} "
                f"missing={self.missing_count}"
            )

        return "\n".join(lines)

    def format_json(self, *, indent: int = 2, sort_keys: bool = True) -> str:
        return json.dumps(
            self.to_dict(),
            indent=indent,
            sort_keys=sort_keys,
        )


def normalize_report_status(value: str | ReportStatus | None) -> ReportStatus:
    """Normalize a status value."""

    if isinstance(value, ReportStatus):
        return value

    if value is None:
        return ReportStatus.UNKNOWN

    normalized = str(value).strip().upper()

    for status in ReportStatus:
        if normalized == status.value:
            return status

    return ReportStatus.UNKNOWN


def report_status_from_ok(
    ok: bool | None,
    *,
    missing: bool = False,
    warning: bool = False,
) -> ReportStatus:
    """Convert booleans into a report status."""

    if missing:
        return ReportStatus.MISSING

    if warning:
        return ReportStatus.WARN

    if ok is True:
        return ReportStatus.OK

    if ok is False:
        return ReportStatus.ERROR

    return ReportStatus.UNKNOWN


def safe_text(value: object | None) -> str:
    """Return stable text for optional values."""

    if value is None:
        return "unknown"

    return str(value)


def format_float(
    value: float | int | None,
    *,
    precision: int = 2,
    suffix: str = "",
) -> str:
    """Format a numeric value safely."""

    if value is None:
        return "unknown"

    return f"{float(value):.{int(precision)}f}{suffix}"


def format_percentage(value: float | int | None) -> str:
    """Format a percentage value safely."""

    return format_float(value, precision=1, suffix="%")


def format_voltage(value: float | int | None) -> str:
    """Format a voltage value safely."""

    return format_float(value, precision=2, suffix=" V")


def format_key_values(
    values: Mapping[str, object] | Iterable[tuple[str, object]],
) -> str:
    """Format key/value data as compact one-line text."""

    if isinstance(values, Mapping):
        items = values.items()
    else:
        items = values

    return " ".join(
        f"{key}={safe_text(value)}"
        for key, value in items
    )


def make_report_line(
    name: str,
    *,
    ok: bool | None = None,
    status: str | ReportStatus | None = None,
    detail: str = "",
    missing: bool = False,
    warning: bool = False,
) -> ReportLine:
    """Create a report line from either explicit status or booleans."""

    if status is None:
        normalized = report_status_from_ok(
            ok,
            missing=missing,
            warning=warning,
        )
    else:
        normalized = normalize_report_status(status)

    return ReportLine(
        name=name,
        status=normalized,
        detail=detail,
    )


def make_report(
    title: str,
    lines: Iterable[ReportLine],
    *,
    metadata: Mapping[str, object] | Iterable[tuple[str, object]] = (),
    ok: bool | None = None,
) -> DiagnosticReport:
    """Create a diagnostic report from lines."""

    line_tuple = tuple(lines)

    if isinstance(metadata, Mapping):
        metadata_tuple = tuple(metadata.items())
    else:
        metadata_tuple = tuple(metadata)

    if ok is None:
        ok_value = all(line.status == ReportStatus.OK for line in line_tuple)
    else:
        ok_value = bool(ok)

    return DiagnosticReport(
        title=title,
        ok=ok_value,
        lines=line_tuple,
        metadata=metadata_tuple,
    )


def to_jsonable(value: object) -> object:
    """Convert dataclasses/enums/containers into JSON-safe objects."""

    if isinstance(value, Enum):
        return value.value

    if is_dataclass(value) and not isinstance(value, type):
        return to_jsonable(asdict(value))

    if isinstance(value, Mapping):
        return {
            str(key): to_jsonable(item)
            for key, item in value.items()
        }

    if isinstance(value, tuple | list):
        return [
            to_jsonable(item)
            for item in value
        ]

    if isinstance(value, set):
        return sorted(to_jsonable(item) for item in value)

    return value


def format_json(
    value: object,
    *,
    indent: int = 2,
    sort_keys: bool = True,
) -> str:
    """Format any JSON-safe or dataclass-like object as JSON."""

    return json.dumps(
        to_jsonable(value),
        indent=indent,
        sort_keys=sort_keys,
    )


def print_report(
    report: DiagnosticReport,
    *,
    as_json: bool = False,
) -> None:
    """Print a report as text or JSON."""

    if as_json:
        print(report.format_json())
    else:
        print(report.format_text())


__all__ = [
    "DiagnosticReport",
    "ReportLine",
    "ReportStatus",
    "format_float",
    "format_json",
    "format_key_values",
    "format_percentage",
    "format_voltage",
    "make_report",
    "make_report_line",
    "normalize_report_status",
    "print_report",
    "report_status_from_ok",
    "safe_text",
    "to_jsonable",
]
