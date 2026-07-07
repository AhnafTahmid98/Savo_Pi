"""Diagnostic helpers for Robot Savo power Python tools."""

from __future__ import annotations

import importlib
import json
from dataclasses import asdict, dataclass, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, Iterable, Mapping

from savo_power.utils.formatting import (
    compact_key_values,
    format_table,
    safe_text,
)
from savo_power.utils.timing import format_duration_s, utc_now_iso


DiagnosticCallback = Callable[[], object]


class DiagnosticStatus(str, Enum):
    """Stable diagnostic status names."""

    OK = "OK"
    WARN = "WARN"
    ERROR = "ERROR"
    MISSING = "MISSING"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class DiagnosticCheck:
    """One diagnostic check result."""

    name: str
    status: DiagnosticStatus = DiagnosticStatus.UNKNOWN
    detail: str = ""
    metadata: tuple[tuple[str, object], ...] = ()

    @property
    def ok(self) -> bool:
        return self.status in {
            DiagnosticStatus.OK,
            DiagnosticStatus.WARN,
        }

    @property
    def failed(self) -> bool:
        return not self.ok

    @property
    def warning(self) -> bool:
        return self.status == DiagnosticStatus.WARN

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "status": self.status.value,
            "ok": self.ok,
            "failed": self.failed,
            "detail": self.detail,
            "metadata": {
                key: to_jsonable(value)
                for key, value in self.metadata
            },
        }

    def format_line(self) -> str:
        detail = f" — {self.detail}" if self.detail else ""
        metadata = (
            f" ({compact_key_values(self.metadata)})"
            if self.metadata
            else ""
        )

        return f"[{self.status.value}] {self.name}{detail}{metadata}"


@dataclass(frozen=True)
class DiagnosticSummary:
    """Summary of multiple diagnostic checks."""

    title: str
    checks: tuple[DiagnosticCheck, ...]
    started_at_utc: str = ""
    duration_s: float = 0.0
    metadata: tuple[tuple[str, object], ...] = ()

    @property
    def ok(self) -> bool:
        return all(check.ok for check in self.checks)

    @property
    def check_count(self) -> int:
        return len(self.checks)

    @property
    def warning_count(self) -> int:
        return sum(1 for check in self.checks if check.status == DiagnosticStatus.WARN)

    @property
    def error_count(self) -> int:
        return sum(1 for check in self.checks if check.status == DiagnosticStatus.ERROR)

    @property
    def missing_count(self) -> int:
        return sum(1 for check in self.checks if check.status == DiagnosticStatus.MISSING)

    @property
    def unknown_count(self) -> int:
        return sum(1 for check in self.checks if check.status == DiagnosticStatus.UNKNOWN)

    @property
    def failure_count(self) -> int:
        return sum(1 for check in self.checks if check.failed)

    def to_dict(self) -> dict[str, object]:
        return {
            "title": self.title,
            "ok": self.ok,
            "check_count": self.check_count,
            "warning_count": self.warning_count,
            "error_count": self.error_count,
            "missing_count": self.missing_count,
            "unknown_count": self.unknown_count,
            "failure_count": self.failure_count,
            "started_at_utc": self.started_at_utc,
            "duration_s": self.duration_s,
            "metadata": {
                key: to_jsonable(value)
                for key, value in self.metadata
            },
            "checks": [check.to_dict() for check in self.checks],
        }

    def format_text(self) -> str:
        lines = [self.title]

        if self.started_at_utc:
            lines.append(f"started_at_utc={self.started_at_utc}")

        if self.duration_s > 0.0:
            lines.append(f"duration={format_duration_s(self.duration_s)}")

        if self.metadata:
            lines.append(compact_key_values(self.metadata))

        for check in self.checks:
            lines.append(check.format_line())

        if self.ok:
            lines.append("Result: PASS")
        else:
            lines.append(
                "Result: FAIL "
                f"checks={self.check_count} "
                f"warnings={self.warning_count} "
                f"errors={self.error_count} "
                f"missing={self.missing_count} "
                f"unknown={self.unknown_count}"
            )

        return "\n".join(lines)

    def format_json(self, *, indent: int = 2, sort_keys: bool = True) -> str:
        return json.dumps(
            self.to_dict(),
            indent=indent,
            sort_keys=sort_keys,
        )

    def format_table(self) -> str:
        rows = [
            [
                check.status.value,
                check.name,
                check.detail,
            ]
            for check in self.checks
        ]

        return format_table(
            rows,
            headers=["status", "name", "detail"],
        )


def normalize_diagnostic_status(
    value: str | DiagnosticStatus | None,
) -> DiagnosticStatus:
    """Normalize diagnostic status value."""

    if isinstance(value, DiagnosticStatus):
        return value

    if value is None:
        return DiagnosticStatus.UNKNOWN

    normalized = str(value).strip().upper()

    if normalized in {"PASS", "PASSED", "TRUE"}:
        return DiagnosticStatus.OK

    if normalized in {"FAIL", "FAILED", "FALSE"}:
        return DiagnosticStatus.ERROR

    if normalized == "WARNING":
        return DiagnosticStatus.WARN

    for status in DiagnosticStatus:
        if normalized == status.value:
            return status

    return DiagnosticStatus.UNKNOWN


def status_from_ok(
    ok: bool | None,
    *,
    missing: bool = False,
    warning: bool = False,
) -> DiagnosticStatus:
    """Create status from simple booleans."""

    if missing:
        return DiagnosticStatus.MISSING

    if warning:
        return DiagnosticStatus.WARN

    if ok is True:
        return DiagnosticStatus.OK

    if ok is False:
        return DiagnosticStatus.ERROR

    return DiagnosticStatus.UNKNOWN


def make_check(
    name: str,
    *,
    status: str | DiagnosticStatus | None = None,
    ok: bool | None = None,
    detail: object = "",
    metadata: Mapping[str, object] | Iterable[tuple[str, object]] = (),
    missing: bool = False,
    warning: bool = False,
) -> DiagnosticCheck:
    """Create one diagnostic check."""

    if status is None:
        normalized_status = status_from_ok(
            ok,
            missing=missing,
            warning=warning,
        )
    else:
        normalized_status = normalize_diagnostic_status(status)

    if isinstance(metadata, Mapping):
        metadata_tuple = tuple(metadata.items())
    else:
        metadata_tuple = tuple(metadata)

    return DiagnosticCheck(
        name=str(name),
        status=normalized_status,
        detail=safe_text(detail, unknown=""),
        metadata=metadata_tuple,
    )


def ok_check(
    name: str,
    detail: object = "",
    *,
    metadata: Mapping[str, object] | Iterable[tuple[str, object]] = (),
) -> DiagnosticCheck:
    """Create OK diagnostic check."""

    return make_check(
        name,
        status=DiagnosticStatus.OK,
        detail=detail,
        metadata=metadata,
    )


def warn_check(
    name: str,
    detail: object = "",
    *,
    metadata: Mapping[str, object] | Iterable[tuple[str, object]] = (),
) -> DiagnosticCheck:
    """Create warning diagnostic check."""

    return make_check(
        name,
        status=DiagnosticStatus.WARN,
        detail=detail,
        metadata=metadata,
    )


def error_check(
    name: str,
    detail: object = "",
    *,
    metadata: Mapping[str, object] | Iterable[tuple[str, object]] = (),
) -> DiagnosticCheck:
    """Create error diagnostic check."""

    return make_check(
        name,
        status=DiagnosticStatus.ERROR,
        detail=detail,
        metadata=metadata,
    )


def missing_check(
    name: str,
    detail: object = "",
    *,
    metadata: Mapping[str, object] | Iterable[tuple[str, object]] = (),
) -> DiagnosticCheck:
    """Create missing diagnostic check."""

    return make_check(
        name,
        status=DiagnosticStatus.MISSING,
        detail=detail,
        metadata=metadata,
    )


def unknown_check(
    name: str,
    detail: object = "",
    *,
    metadata: Mapping[str, object] | Iterable[tuple[str, object]] = (),
) -> DiagnosticCheck:
    """Create unknown diagnostic check."""

    return make_check(
        name,
        status=DiagnosticStatus.UNKNOWN,
        detail=detail,
        metadata=metadata,
    )


def summarize_checks(
    title: str,
    checks: Iterable[DiagnosticCheck],
    *,
    started_at_utc: str | None = None,
    duration_s: float = 0.0,
    metadata: Mapping[str, object] | Iterable[tuple[str, object]] = (),
) -> DiagnosticSummary:
    """Create diagnostic summary."""

    if isinstance(metadata, Mapping):
        metadata_tuple = tuple(metadata.items())
    else:
        metadata_tuple = tuple(metadata)

    return DiagnosticSummary(
        title=str(title),
        checks=tuple(checks),
        started_at_utc=started_at_utc or utc_now_iso(),
        duration_s=max(0.0, float(duration_s)),
        metadata=metadata_tuple,
    )


def run_diagnostic_check(
    name: str,
    callback: DiagnosticCallback,
) -> DiagnosticCheck:
    """Run one diagnostic callback and convert result to DiagnosticCheck.

    Supported callback returns:
    - DiagnosticCheck
    - bool
    - tuple[bool, detail]
    - tuple[DiagnosticStatus, detail]
    - None, treated as OK
    """

    try:
        result = callback()
    except Exception as exc:  # noqa: BLE001 - diagnostics should preserve root error
        return error_check(
            name,
            f"{type(exc).__name__}: {exc}",
        )

    if isinstance(result, DiagnosticCheck):
        return result

    if result is None:
        return ok_check(name)

    if isinstance(result, bool):
        return make_check(
            name,
            ok=result,
            detail="ok" if result else "failed",
        )

    if isinstance(result, tuple) and len(result) >= 2:
        status_or_ok = result[0]
        detail = result[1]

        if isinstance(status_or_ok, bool):
            return make_check(
                name,
                ok=status_or_ok,
                detail=detail,
            )

        return make_check(
            name,
            status=normalize_diagnostic_status(status_or_ok),
            detail=detail,
        )

    return ok_check(
        name,
        safe_text(result),
    )


def run_diagnostic_checks(
    title: str,
    checks: Iterable[tuple[str, DiagnosticCallback]],
    *,
    metadata: Mapping[str, object] | Iterable[tuple[str, object]] = (),
) -> DiagnosticSummary:
    """Run several diagnostic callbacks and summarize them."""

    started = utc_now_iso()
    results = [
        run_diagnostic_check(name, callback)
        for name, callback in checks
    ]

    return summarize_checks(
        title,
        results,
        started_at_utc=started,
        metadata=metadata,
    )


def check_import_available(
    module_name: str,
    *,
    name: str | None = None,
    required: bool = True,
) -> DiagnosticCheck:
    """Check whether a Python module can be imported."""

    check_name = name or f"Python import {module_name}"

    try:
        importlib.import_module(module_name)
    except ImportError as exc:
        if required:
            return missing_check(check_name, str(exc))

        return warn_check(check_name, str(exc))

    return ok_check(check_name, "available")


def check_path_exists(
    path: str | Path,
    *,
    name: str | None = None,
    required: bool = True,
) -> DiagnosticCheck:
    """Check whether a filesystem path exists."""

    selected_path = Path(path)
    check_name = name or f"Path {selected_path}"

    if selected_path.exists():
        return ok_check(
            check_name,
            "exists",
            metadata={"path": str(selected_path)},
        )

    if required:
        return missing_check(
            check_name,
            "not found",
            metadata={"path": str(selected_path)},
        )

    return warn_check(
        check_name,
        "not found",
        metadata={"path": str(selected_path)},
    )


def check_file_readable(
    path: str | Path,
    *,
    name: str | None = None,
    required: bool = True,
) -> DiagnosticCheck:
    """Check whether a file exists and can be opened for reading."""

    selected_path = Path(path)
    check_name = name or f"Readable file {selected_path}"

    if not selected_path.exists():
        return (
            missing_check(check_name, "not found", metadata={"path": str(selected_path)})
            if required
            else warn_check(check_name, "not found", metadata={"path": str(selected_path)})
        )

    if not selected_path.is_file():
        return error_check(
            check_name,
            "not a file",
            metadata={"path": str(selected_path)},
        )

    try:
        with selected_path.open("rb"):
            pass
    except OSError as exc:
        return error_check(
            check_name,
            str(exc),
            metadata={"path": str(selected_path)},
        )

    return ok_check(
        check_name,
        "readable",
        metadata={"path": str(selected_path)},
    )


def check_value_range(
    name: str,
    value: float | int | None,
    *,
    minimum: float,
    maximum: float,
    unit: str = "",
    warning_only: bool = False,
) -> DiagnosticCheck:
    """Check whether numeric value is inside a range."""

    metadata = {
        "value": value,
        "minimum": minimum,
        "maximum": maximum,
        "unit": unit,
    }

    if value is None:
        return unknown_check(
            name,
            "value is unknown",
            metadata=metadata,
        )

    number = float(value)

    if float(minimum) <= number <= float(maximum):
        detail = f"{number:g}{unit} inside {minimum:g}..{maximum:g}{unit}"
        return ok_check(name, detail, metadata=metadata)

    detail = f"{number:g}{unit} outside {minimum:g}..{maximum:g}{unit}"

    if warning_only:
        return warn_check(name, detail, metadata=metadata)

    return error_check(name, detail, metadata=metadata)


def check_not_none(
    name: str,
    value: object,
    *,
    detail: str = "value present",
) -> DiagnosticCheck:
    """Check that a value is not None."""

    if value is None:
        return unknown_check(name, "value is None")

    return ok_check(name, detail)


def checks_to_table(checks: Iterable[DiagnosticCheck]) -> str:
    """Format checks as a small plain-text table."""

    rows = [
        [
            check.status.value,
            check.name,
            check.detail,
        ]
        for check in checks
    ]

    return format_table(
        rows,
        headers=["status", "name", "detail"],
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

    if isinstance(value, (tuple, list)):
        return [
            to_jsonable(item)
            for item in value
        ]

    if isinstance(value, set):
        return sorted(to_jsonable(item) for item in value)

    return value


__all__ = [
    "DiagnosticCallback",
    "DiagnosticCheck",
    "DiagnosticStatus",
    "DiagnosticSummary",
    "check_file_readable",
    "check_import_available",
    "check_not_none",
    "check_path_exists",
    "check_value_range",
    "checks_to_table",
    "error_check",
    "make_check",
    "missing_check",
    "normalize_diagnostic_status",
    "ok_check",
    "run_diagnostic_check",
    "run_diagnostic_checks",
    "status_from_ok",
    "summarize_checks",
    "to_jsonable",
    "unknown_check",
    "warn_check",
]
