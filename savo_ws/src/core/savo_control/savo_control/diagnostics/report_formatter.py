# -*- coding: utf-8 -*-

"""Small report-formatting helpers for diagnostics and CLI tools."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


STATUS_OK = "OK"
STATUS_WARN = "WARN"
STATUS_ERROR = "ERROR"
STATUS_STALE = "STALE"
STATUS_UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class DiagnosticItem:
    name: str
    status: str
    value: object = ""
    note: str = ""

    def normalized_status(self) -> str:
        status = str(self.status).strip().upper()
        if status in {STATUS_OK, STATUS_WARN, STATUS_ERROR, STATUS_STALE}:
            return status
        return STATUS_UNKNOWN

    def is_ok(self) -> bool:
        return self.normalized_status() == STATUS_OK

    def to_line(self) -> str:
        parts = [f"{self.name}: {self.normalized_status()}"]

        if self.value != "":
            parts.append(f"value={self.value}")

        if self.note:
            parts.append(f"note={self.note}")

        return " | ".join(parts)


def status_from_bool(ok: bool, *, stale: bool = False) -> str:
    if stale:
        return STATUS_STALE
    return STATUS_OK if ok else STATUS_ERROR


def status_from_age(age_s: float, stale_after_s: float) -> str:
    if age_s < 0.0:
        return STATUS_UNKNOWN
    if age_s > stale_after_s:
        return STATUS_STALE
    return STATUS_OK


def format_seconds(value: float) -> str:
    return f"{value:.3f}s"


def format_float(value: float, *, unit: str = "", digits: int = 3) -> str:
    text = f"{value:.{digits}f}"
    return f"{text}{unit}" if unit else text


def format_bool(value: bool) -> str:
    return "true" if value else "false"


def format_key_values(values: dict[str, object]) -> str:
    return "; ".join(f"{key}={value}" for key, value in values.items())


def format_section(title: str, lines: Iterable[str]) -> str:
    clean_lines = [line for line in lines if line]
    if not clean_lines:
        return title
    return "\n".join([title, *clean_lines])


def summarize_items(items: Iterable[DiagnosticItem]) -> str:
    item_list = list(items)

    if not item_list:
        return STATUS_UNKNOWN

    statuses = [item.normalized_status() for item in item_list]

    if STATUS_ERROR in statuses:
        return STATUS_ERROR
    if STATUS_STALE in statuses:
        return STATUS_STALE
    if STATUS_WARN in statuses:
        return STATUS_WARN
    if all(status == STATUS_OK for status in statuses):
        return STATUS_OK

    return STATUS_UNKNOWN


def format_report(title: str, items: Iterable[DiagnosticItem]) -> str:
    item_list = list(items)
    overall = summarize_items(item_list)

    lines = [
        f"{title}: {overall}",
        *[item.to_line() for item in item_list],
    ]

    return "\n".join(lines)


__all__ = [
    "DiagnosticItem",
    "STATUS_ERROR",
    "STATUS_OK",
    "STATUS_STALE",
    "STATUS_UNKNOWN",
    "STATUS_WARN",
    "format_bool",
    "format_float",
    "format_key_values",
    "format_report",
    "format_seconds",
    "format_section",
    "status_from_age",
    "status_from_bool",
    "summarize_items",
]
