#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/diagnostics/__init__.py
---------------------------------------------
Diagnostics package exports for `savo_base`.

Purpose
-------
Provides a clean import surface for shared diagnostics utilities
(e.g., report formatter models/helpers) used by hardware validation scripts.

Examples
--------
from savo_base.diagnostics import (
    DiagnosticReport,
    DiagnosticCheckResult,
    Severity,
    ReportStatus,
    TerminalStyle,
    print_report,
    report_to_json_str,
    write_report_json,
    report_to_text,
    write_report_text,
    make_pass,
    make_warn,
    make_fail,
)
"""

from __future__ import annotations

# Re-export shared report formatting utilities
from .report_formatter import (
    Severity,
    ReportStatus,
    DiagnosticCheckResult,
    DiagnosticReport,
    TerminalStyle,
    make_pass,
    make_warn,
    make_fail,
    print_report,
    report_to_json_dict,
    report_to_json_str,
    write_report_json,
    report_to_text,
    write_report_text,
)

# Optional package version fallback (diagnostics-local)
__diagnostics_api_version__ = "1.0.0"

__all__ = [
    # enums
    "Severity",
    "ReportStatus",
    # dataclasses / models
    "DiagnosticCheckResult",
    "DiagnosticReport",
    "TerminalStyle",
    # builders
    "make_pass",
    "make_warn",
    "make_fail",
    # formatter / exporters
    "print_report",
    "report_to_json_dict",
    "report_to_json_str",
    "write_report_json",
    "report_to_text",
    "write_report_text",
    # diagnostics package metadata
    "__diagnostics_api_version__",
]