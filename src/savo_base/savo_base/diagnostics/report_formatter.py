#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/diagnostics/report_formatter.py
------------------------------------------------------
Professional formatting utilities for `savo_base` diagnostics scripts.

Purpose
-------
This module standardizes how diagnostics scripts print, summarize, and export
results (PASS/WARN/FAIL, metadata, observations, recommendations) so all
`savo_base/diagnostics/*.py` tools look consistent and are easy to review in:

- terminal output
- logs
- CI / automated checks (JSON)
- internship / engineering documentation screenshots

Design goals
------------
- Zero ROS dependency (pure Python)
- Reusable across all diagnostics scripts
- Human-friendly terminal formatting + machine-friendly JSON
- Safe default behavior (never crashes on non-serializable values)

Typical usage
-------------
from savo_base.diagnostics.report_formatter import (
    DiagnosticReport,
    DiagnosticCheckResult,
    Severity,
    print_report,
    report_to_json_str,
)

report = DiagnosticReport(
    tool="wheel_direction_check",
    robot_name="Robot Savo",
    target="Freenove PCA9685 mecanum base",
)
report.add_result(DiagnosticCheckResult(
    name="FL forward spin",
    passed=True,
    severity=Severity.INFO,
    observed="Wheel rotates as expected",
))
print_report(report)
print(report_to_json_str(report))

Notes
-----
- This module is a formatter/helper only; it does not access hardware.
- Individual diagnostic scripts should collect raw measurements and convert them
  into `DiagnosticCheckResult` entries.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence


# =============================================================================
# Severity / status enums
# =============================================================================
class Severity(str, Enum):
    """Severity level for an individual diagnostic check result."""
    INFO = "INFO"
    WARN = "WARN"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


class ReportStatus(str, Enum):
    """Overall report status."""
    PASS = "PASS"
    WARN = "WARN"
    FAIL = "FAIL"


# =============================================================================
# Data models
# =============================================================================
@dataclass
class DiagnosticCheckResult:
    """
    One diagnostic check line-item (atomic result).

    Examples
    --------
    - "PCA9685 detected on I2C bus 1"
    - "FL wheel forward direction"
    - "Emergency stop topic observed"
    """
    name: str
    passed: bool
    severity: Severity = Severity.INFO
    observed: str = ""
    expected: str = ""
    recommendation: str = ""
    value: Any = None
    unit: str = ""
    tags: List[str] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)
    duration_s: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["severity"] = self.severity.value
        return _json_safe(d)


@dataclass
class DiagnosticReport:
    """
    Structured diagnostic report for one tool execution.
    """
    tool: str
    robot_name: str = "Robot Savo"
    target: str = ""
    version: str = "1.0"
    operator: str = ""
    host: str = ""
    started_at_utc: str = field(default_factory=lambda: _utc_now_iso())
    ended_at_utc: str = ""
    duration_s: Optional[float] = None

    environment: Dict[str, Any] = field(default_factory=dict)
    config: Dict[str, Any] = field(default_factory=dict)
    observations: List[str] = field(default_factory=list)
    recommendations: List[str] = field(default_factory=list)
    results: List[DiagnosticCheckResult] = field(default_factory=list)
    extras: Dict[str, Any] = field(default_factory=dict)

    _t0_mono: float = field(default_factory=time.monotonic, repr=False)

    def add_result(self, result: DiagnosticCheckResult) -> None:
        self.results.append(result)

    def add_observation(self, text: str) -> None:
        text = str(text).strip()
        if text:
            self.observations.append(text)

    def add_recommendation(self, text: str) -> None:
        text = str(text).strip()
        if text:
            self.recommendations.append(text)

    def finalize(self) -> "DiagnosticReport":
        if not self.ended_at_utc:
            self.ended_at_utc = _utc_now_iso()
        if self.duration_s is None:
            self.duration_s = max(0.0, time.monotonic() - self._t0_mono)
        return self

    @property
    def pass_count(self) -> int:
        return sum(1 for r in self.results if r.passed)

    @property
    def fail_count(self) -> int:
        return sum(1 for r in self.results if not r.passed)

    @property
    def warn_count(self) -> int:
        # "Warn count" tracks failed WARN items + passed WARN notes
        return sum(1 for r in self.results if r.severity == Severity.WARN)

    @property
    def error_count(self) -> int:
        return sum(1 for r in self.results if r.severity in (Severity.ERROR, Severity.CRITICAL))

    @property
    def total_count(self) -> int:
        return len(self.results)

    @property
    def overall_status(self) -> ReportStatus:
        """
        Professional aggregation rule:
        - FAIL if any failed ERROR/CRITICAL item exists, OR any failed item with severity != INFO? (strict)
        - WARN if no hard failures but there are warnings / non-critical failed checks
        - PASS otherwise
        """
        any_fail = any(not r.passed for r in self.results)
        any_hard_fail = any((not r.passed) and (r.severity in (Severity.ERROR, Severity.CRITICAL)) for r in self.results)
        any_warnish = any(
            (r.severity == Severity.WARN) or ((not r.passed) and r.severity == Severity.INFO)
            for r in self.results
        )

        if any_hard_fail:
            return ReportStatus.FAIL
        if any_fail or any_warnish:
            return ReportStatus.WARN
        return ReportStatus.PASS

    def summary_dict(self) -> Dict[str, Any]:
        self.finalize()
        return _json_safe({
            "tool": self.tool,
            "robot_name": self.robot_name,
            "target": self.target,
            "version": self.version,
            "status": self.overall_status.value,
            "counts": {
                "total": self.total_count,
                "passed": self.pass_count,
                "failed": self.fail_count,
                "warnings": self.warn_count,
                "errors": self.error_count,
            },
            "started_at_utc": self.started_at_utc,
            "ended_at_utc": self.ended_at_utc,
            "duration_s": self.duration_s,
        })

    def to_dict(self) -> Dict[str, Any]:
        self.finalize()
        return _json_safe({
            "summary": self.summary_dict(),
            "environment": self.environment,
            "config": self.config,
            "observations": list(self.observations),
            "recommendations": list(self.recommendations),
            "results": [r.to_dict() for r in self.results],
            "extras": self.extras,
        })


# =============================================================================
# Builder helpers (optional convenience)
# =============================================================================
def make_pass(
    name: str,
    *,
    observed: str = "",
    expected: str = "",
    value: Any = None,
    unit: str = "",
    tags: Optional[Sequence[str]] = None,
    metadata: Optional[Mapping[str, Any]] = None,
    duration_s: Optional[float] = None,
) -> DiagnosticCheckResult:
    return DiagnosticCheckResult(
        name=name,
        passed=True,
        severity=Severity.INFO,
        observed=observed,
        expected=expected,
        value=value,
        unit=unit,
        tags=list(tags or []),
        metadata=dict(metadata or {}),
        duration_s=duration_s,
    )


def make_warn(
    name: str,
    *,
    observed: str = "",
    expected: str = "",
    recommendation: str = "",
    value: Any = None,
    unit: str = "",
    tags: Optional[Sequence[str]] = None,
    metadata: Optional[Mapping[str, Any]] = None,
    duration_s: Optional[float] = None,
    passed: bool = False,
) -> DiagnosticCheckResult:
    return DiagnosticCheckResult(
        name=name,
        passed=bool(passed),
        severity=Severity.WARN,
        observed=observed,
        expected=expected,
        recommendation=recommendation,
        value=value,
        unit=unit,
        tags=list(tags or []),
        metadata=dict(metadata or {}),
        duration_s=duration_s,
    )


def make_fail(
    name: str,
    *,
    observed: str = "",
    expected: str = "",
    recommendation: str = "",
    severity: Severity = Severity.ERROR,
    value: Any = None,
    unit: str = "",
    tags: Optional[Sequence[str]] = None,
    metadata: Optional[Mapping[str, Any]] = None,
    duration_s: Optional[float] = None,
) -> DiagnosticCheckResult:
    if severity == Severity.INFO:
        severity = Severity.ERROR
    return DiagnosticCheckResult(
        name=name,
        passed=False,
        severity=severity,
        observed=observed,
        expected=expected,
        recommendation=recommendation,
        value=value,
        unit=unit,
        tags=list(tags or []),
        metadata=dict(metadata or {}),
        duration_s=duration_s,
    )


# =============================================================================
# Terminal formatting
# =============================================================================
@dataclass(frozen=True)
class TerminalStyle:
    """
    Terminal style toggles. ANSI colors are optional and safe to disable.
    """
    use_color: bool = True
    line_width: int = 88
    show_expected: bool = True
    show_recommendation: bool = True
    show_metadata: bool = False
    show_config: bool = True
    show_environment: bool = True
    show_observations: bool = True
    show_results: bool = True
    show_recommendations: bool = True


def print_report(report: DiagnosticReport, style: Optional[TerminalStyle] = None) -> None:
    """
    Pretty-print a diagnostic report to stdout.

    This is intended for human operators during bringup/bench testing.
    """
    s = style or TerminalStyle()
    report.finalize()

    width = max(60, int(s.line_width))
    hr = "=" * width
    sr = "-" * width

    # Header
    summary = report.summary_dict()
    status = summary["status"]
    status_colored = _fmt_status(status, s.use_color)

    print(hr)
    print(_center_text(f"{report.robot_name} — Diagnostic Report", width))
    print(_center_text(f"Tool: {report.tool}", width))
    print(hr)
    print(f"Status      : {status_colored}")
    print(f"Target      : {report.target or '-'}")
    print(f"Version     : {report.version}")
    print(f"Operator    : {report.operator or '-'}")
    print(f"Host        : {report.host or '-'}")
    print(f"Started UTC : {report.started_at_utc}")
    print(f"Ended UTC   : {report.ended_at_utc}")
    print(f"Duration    : {_fmt_seconds(report.duration_s)}")
    print(sr)

    counts = summary["counts"]
    print(
        "Counts      : "
        f"total={counts['total']}  "
        f"passed={_fmt_passfail_num(counts['passed'], True, s.use_color)}  "
        f"failed={_fmt_passfail_num(counts['failed'], False, s.use_color)}  "
        f"warnings={counts['warnings']}  errors={counts['errors']}"
    )

    if s.show_config and report.config:
        print(sr)
        print("CONFIG")
        for k, v in _flatten_map(report.config).items():
            print(f"  - {k}: {v}")

    if s.show_environment and report.environment:
        print(sr)
        print("ENVIRONMENT")
        for k, v in _flatten_map(report.environment).items():
            print(f"  - {k}: {v}")

    if s.show_observations and report.observations:
        print(sr)
        print("OBSERVATIONS")
        for i, obs in enumerate(report.observations, start=1):
            print(f"  {i:02d}. {obs}")

    if s.show_results and report.results:
        print(sr)
        print("RESULTS")
        for i, r in enumerate(report.results, start=1):
            print(_format_result_block(i=i, result=r, style=s))

    if s.show_recommendations:
        merged_recs = list(report.recommendations)
        # Auto-pull recommendations from failed/warn result items if not duplicated
        for r in report.results:
            rec = (r.recommendation or "").strip()
            if rec and rec not in merged_recs:
                merged_recs.append(rec)

        if merged_recs:
            print(sr)
            print("RECOMMENDATIONS")
            for i, rec in enumerate(merged_recs, start=1):
                print(f"  {i:02d}. {rec}")

    print(hr)


def _format_result_block(i: int, result: DiagnosticCheckResult, style: TerminalStyle) -> str:
    icon, label = _result_badge(result, style.use_color)
    lines: List[str] = []

    line1 = f"  [{i:02d}] {icon} {result.name}  ({label})"
    lines.append(line1)

    if result.observed:
        lines.append(f"       observed : {result.observed}")
    if style.show_expected and result.expected:
        lines.append(f"       expected : {result.expected}")

    if result.value is not None:
        vtxt = _safe_str(result.value)
        if result.unit:
            vtxt = f"{vtxt} {result.unit}"
        lines.append(f"       value    : {vtxt}")

    if result.duration_s is not None:
        lines.append(f"       duration : {_fmt_seconds(result.duration_s)}")

    if result.tags:
        lines.append(f"       tags     : {', '.join(map(str, result.tags))}")

    if style.show_recommendation and result.recommendation:
        lines.append(f"       action   : {result.recommendation}")

    if style.show_metadata and result.metadata:
        for mk, mv in _flatten_map(result.metadata, prefix="metadata").items():
            lines.append(f"       {mk:<8}: {mv}")

    return "\n".join(lines)


# =============================================================================
# JSON / export helpers
# =============================================================================
def report_to_json_dict(report: DiagnosticReport) -> Dict[str, Any]:
    return report.to_dict()


def report_to_json_str(report: DiagnosticReport, *, indent: int = 2, sort_keys: bool = False) -> str:
    return json.dumps(report.to_dict(), ensure_ascii=False, indent=indent, sort_keys=sort_keys, default=_json_default)


def write_report_json(report: DiagnosticReport, path: str, *, indent: int = 2, sort_keys: bool = False) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(report_to_json_str(report, indent=indent, sort_keys=sort_keys))
        f.write("\n")


def report_to_text(report: DiagnosticReport, style: Optional[TerminalStyle] = None) -> str:
    """
    Build a plain-text report string (same content style as print_report) without printing.

    Useful for:
    - saving .txt logs
    - attaching to emails / Teams messages
    """
    # Reuse formatter internals without importing io/contextlib complexity into user scripts
    # by composing lines directly.
    s = style or TerminalStyle()
    report.finalize()

    width = max(60, int(s.line_width))
    hr = "=" * width
    sr = "-" * width
    summary = report.summary_dict()

    out: List[str] = []
    out.append(hr)
    out.append(_center_text(f"{report.robot_name} — Diagnostic Report", width))
    out.append(_center_text(f"Tool: {report.tool}", width))
    out.append(hr)
    out.append(f"Status      : {summary['status']}")
    out.append(f"Target      : {report.target or '-'}")
    out.append(f"Version     : {report.version}")
    out.append(f"Operator    : {report.operator or '-'}")
    out.append(f"Host        : {report.host or '-'}")
    out.append(f"Started UTC : {report.started_at_utc}")
    out.append(f"Ended UTC   : {report.ended_at_utc}")
    out.append(f"Duration    : {_fmt_seconds(report.duration_s)}")
    out.append(sr)

    counts = summary["counts"]
    out.append(
        "Counts      : "
        f"total={counts['total']}  passed={counts['passed']}  failed={counts['failed']}  "
        f"warnings={counts['warnings']}  errors={counts['errors']}"
    )

    if s.show_config and report.config:
        out.append(sr)
        out.append("CONFIG")
        for k, v in _flatten_map(report.config).items():
            out.append(f"  - {k}: {v}")

    if s.show_environment and report.environment:
        out.append(sr)
        out.append("ENVIRONMENT")
        for k, v in _flatten_map(report.environment).items():
            out.append(f"  - {k}: {v}")

    if s.show_observations and report.observations:
        out.append(sr)
        out.append("OBSERVATIONS")
        for i, obs in enumerate(report.observations, start=1):
            out.append(f"  {i:02d}. {obs}")

    if s.show_results and report.results:
        out.append(sr)
        out.append("RESULTS")
        # disable color in text export
        s_txt = TerminalStyle(**{**s.__dict__, "use_color": False})
        for i, r in enumerate(report.results, start=1):
            out.append(_format_result_block(i=i, result=r, style=s_txt))

    if s.show_recommendations:
        merged_recs = list(report.recommendations)
        for r in report.results:
            rec = (r.recommendation or "").strip()
            if rec and rec not in merged_recs:
                merged_recs.append(rec)
        if merged_recs:
            out.append(sr)
            out.append("RECOMMENDATIONS")
            for i, rec in enumerate(merged_recs, start=1):
                out.append(f"  {i:02d}. {rec}")

    out.append(hr)
    return "\n".join(out)


def write_report_text(report: DiagnosticReport, path: str, style: Optional[TerminalStyle] = None) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(report_to_text(report, style=style))
        f.write("\n")


# =============================================================================
# Internal formatting helpers
# =============================================================================
def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _fmt_seconds(val: Optional[float]) -> str:
    if val is None:
        return "-"
    try:
        x = float(val)
    except Exception:
        return str(val)
    return f"{x:.3f} s"


def _center_text(text: str, width: int) -> str:
    s = str(text)
    if len(s) >= width:
        return s
    pad = width - len(s)
    left = pad // 2
    right = pad - left
    return (" " * left) + s + (" " * right)


def _result_badge(result: DiagnosticCheckResult, use_color: bool) -> tuple[str, str]:
    sev = result.severity.value
    if result.passed:
        icon = _color("PASS", "green", use_color)
        return icon, sev
    if result.severity == Severity.WARN:
        icon = _color("WARN", "yellow", use_color)
        return icon, sev
    if result.severity == Severity.CRITICAL:
        icon = _color("FAIL", "red_bold", use_color)
        return icon, sev
    return _color("FAIL", "red", use_color), sev


def _fmt_status(status: str, use_color: bool) -> str:
    s = str(status).upper()
    if s == ReportStatus.PASS.value:
        return _color(s, "green", use_color)
    if s == ReportStatus.WARN.value:
        return _color(s, "yellow", use_color)
    if s == ReportStatus.FAIL.value:
        return _color(s, "red_bold", use_color)
    return s


def _fmt_passfail_num(n: int, is_pass: bool, use_color: bool) -> str:
    txt = str(int(n))
    if not use_color:
        return txt
    return _color(txt, "green" if is_pass else "red", use_color)


def _color(text: str, style: str, enabled: bool) -> str:
    if not enabled:
        return text
    code_map = {
        "green": "\033[32m",
        "yellow": "\033[33m",
        "red": "\033[31m",
        "red_bold": "\033[1;31m",
        "cyan": "\033[36m",
        "dim": "\033[2m",
        "reset": "\033[0m",
    }
    prefix = code_map.get(style, "")
    reset = code_map["reset"] if prefix else ""
    return f"{prefix}{text}{reset}"


def _safe_str(v: Any) -> str:
    try:
        return str(v)
    except Exception:
        return repr(v)


def _json_default(obj: Any) -> Any:
    if isinstance(obj, Enum):
        return obj.value
    return _safe_str(obj)


def _json_safe(value: Any) -> Any:
    """
    Convert nested structure into a JSON-safe structure without raising.

    Strategy:
    - keep JSON-native primitives
    - recurse dict/list/tuple/set
    - convert Enum to value
    - fallback to str()
    """
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Mapping):
        out: Dict[str, Any] = {}
        for k, v in value.items():
            out[str(k)] = _json_safe(v)
        return out
    if isinstance(value, (list, tuple, set)):
        return [_json_safe(v) for v in value]
    return _safe_str(value)


def _flatten_map(data: Mapping[str, Any], prefix: str = "") -> Dict[str, Any]:
    """
    Flatten nested mappings for cleaner terminal printing.
    Example:
      {"a":{"b":1}} -> {"a.b":1}
    """
    out: Dict[str, Any] = {}

    def _walk(obj: Any, p: str) -> None:
        if isinstance(obj, Mapping):
            for k, v in obj.items():
                np = f"{p}.{k}" if p else str(k)
                _walk(v, np)
        else:
            out[p] = _json_safe(obj)

    _walk(dict(data), prefix)
    return out


# =============================================================================
# CLI self-test / demo (optional)
# =============================================================================
def _demo() -> int:
    """
    Quick demo so you can run this file directly during development:
        python3 report_formatter.py
    """
    rep = DiagnosticReport(
        tool="report_formatter_demo",
        target="Formatting layer only",
        version="1.0.0",
        operator="developer",
        host="local-dev",
    )
    rep.environment = {
        "python": "3.x",
        "platform": "linux",
    }
    rep.config = {
        "dry_run": True,
        "line_width": 88,
    }

    rep.add_result(make_pass(
        "Formatter module import",
        observed="Module loaded successfully",
        expected="No import errors",
        tags=["sanity", "formatter"],
    ))

    rep.add_result(make_warn(
        "ANSI color support",
        observed="Terminal may not support ANSI colors",
        expected="Colors optional",
        recommendation="Use TerminalStyle(use_color=False) for plain logs/CI.",
        passed=True,
        tags=["ux", "terminal"],
    ))

    rep.add_result(make_fail(
        "Example failed check",
        observed="Demonstration failure",
        expected="Pass condition",
        recommendation="This is a demo only; remove in production diagnostics.",
        severity=Severity.ERROR,
        tags=["demo"],
    ))

    rep.add_observation("This demo validates formatting and export helpers.")
    rep.add_recommendation("Integrate report_formatter into all savo_base diagnostics scripts.")
    rep.finalize()

    print_report(rep)
    print("\nJSON preview:\n")
    print(report_to_json_str(rep, indent=2))

    return 0


__all__ = [
    # enums
    "Severity",
    "ReportStatus",
    # dataclasses
    "DiagnosticCheckResult",
    "DiagnosticReport",
    "TerminalStyle",
    # builders
    "make_pass",
    "make_warn",
    "make_fail",
    # format/print/export
    "print_report",
    "report_to_json_dict",
    "report_to_json_str",
    "write_report_json",
    "report_to_text",
    "write_report_text",
]


if __name__ == "__main__":
    raise SystemExit(_demo())