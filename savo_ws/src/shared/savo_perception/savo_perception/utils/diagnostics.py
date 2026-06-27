#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Small diagnostic helpers for perception fallback nodes and CLI tools."""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional

from savo_perception.constants import (
    PACKAGE_NAME,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
)


def now_mono_s() -> float:
    return time.monotonic()


@dataclass(frozen=True)
class DiagnosticEvent:
    level: str
    name: str
    message: str
    stamp_mono_s: float = field(default_factory=now_mono_s)
    data: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def ok(cls, name: str, message: str, data: Optional[Dict[str, Any]] = None) -> "DiagnosticEvent":
        return cls(level=STATUS_OK, name=name, message=message, data=dict(data or {}))

    @classmethod
    def stale(cls, name: str, message: str, data: Optional[Dict[str, Any]] = None) -> "DiagnosticEvent":
        return cls(level=STATUS_STALE, name=name, message=message, data=dict(data or {}))

    @classmethod
    def error(cls, name: str, message: str, data: Optional[Dict[str, Any]] = None) -> "DiagnosticEvent":
        return cls(level=STATUS_ERROR, name=name, message=message, data=dict(data or {}))

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))


@dataclass
class DiagnosticSummary:
    name: str
    events: List[DiagnosticEvent] = field(default_factory=list)
    stamp_mono_s: float = field(default_factory=now_mono_s)

    def add(self, event: DiagnosticEvent) -> None:
        self.events.append(event)

    def add_ok(self, name: str, message: str, data: Optional[Dict[str, Any]] = None) -> None:
        self.add(DiagnosticEvent.ok(name, message, data))

    def add_stale(self, name: str, message: str, data: Optional[Dict[str, Any]] = None) -> None:
        self.add(DiagnosticEvent.stale(name, message, data))

    def add_error(self, name: str, message: str, data: Optional[Dict[str, Any]] = None) -> None:
        self.add(DiagnosticEvent.error(name, message, data))

    @property
    def ok(self) -> bool:
        return all(event.level == STATUS_OK for event in self.events)

    @property
    def status(self) -> str:
        if self.ok:
            return STATUS_OK
        if any(event.level == STATUS_ERROR for event in self.events):
            return STATUS_ERROR
        return STATUS_STALE

    def to_dict(self) -> Dict[str, Any]:
        return {
            "package": PACKAGE_NAME,
            "name": self.name,
            "ok": self.ok,
            "status": self.status,
            "stamp_mono_s": self.stamp_mono_s,
            "events": [event.to_dict() for event in self.events],
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))


def format_status_line(name: str, ok: bool, message: str) -> str:
    status = STATUS_OK if ok else STATUS_ERROR
    return f"[{PACKAGE_NAME}] {name}: {status} - {message}"


def event_from_exception(name: str, exc: BaseException) -> DiagnosticEvent:
    return DiagnosticEvent.error(
        name=name,
        message=str(exc),
        data={"exception_type": type(exc).__name__},
    )


def merge_summaries(name: str, summaries: List[DiagnosticSummary]) -> DiagnosticSummary:
    merged = DiagnosticSummary(name=name)
    for summary in summaries:
        for event in summary.events:
            merged.add(event)
    return merged


__all__ = [
    "DiagnosticEvent",
    "DiagnosticSummary",
    "now_mono_s",
    "format_status_line",
    "event_from_exception",
    "merge_summaries",
]