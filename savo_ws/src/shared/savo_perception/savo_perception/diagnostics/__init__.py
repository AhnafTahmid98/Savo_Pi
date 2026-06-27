#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Diagnostic helpers and entry metadata for savo_perception."""

from __future__ import annotations

from typing import Final

from savo_perception.diagnostics.report_formatter import (
    DiagnosticReport,
    ReportRow,
    format_report_text,
    format_row_text,
    print_report,
    report_from_rows,
    status_mark,
)


TOF_MUX_CHECK: Final[str] = "tof_mux_check"
ULTRASONIC_CHECK: Final[str] = "ultrasonic_check"
SAFETY_FUSION_CHECK: Final[str] = "safety_fusion_check"
RANGE_HEALTH_CHECK: Final[str] = "range_health_check"

HARDWARE_DIAGNOSTICS: Final[tuple[str, ...]] = (
    TOF_MUX_CHECK,
    ULTRASONIC_CHECK,
)

OFFLINE_DIAGNOSTICS: Final[tuple[str, ...]] = (
    SAFETY_FUSION_CHECK,
    RANGE_HEALTH_CHECK,
)

ALL_DIAGNOSTICS: Final[tuple[str, ...]] = (
    HARDWARE_DIAGNOSTICS + OFFLINE_DIAGNOSTICS
)


def list_hardware_diagnostics() -> tuple[str, ...]:
    return HARDWARE_DIAGNOSTICS


def list_offline_diagnostics() -> tuple[str, ...]:
    return OFFLINE_DIAGNOSTICS


def list_all_diagnostics() -> tuple[str, ...]:
    return ALL_DIAGNOSTICS


__all__ = [
    "TOF_MUX_CHECK",
    "ULTRASONIC_CHECK",
    "SAFETY_FUSION_CHECK",
    "RANGE_HEALTH_CHECK",
    "HARDWARE_DIAGNOSTICS",
    "OFFLINE_DIAGNOSTICS",
    "ALL_DIAGNOSTICS",
    "list_hardware_diagnostics",
    "list_offline_diagnostics",
    "list_all_diagnostics",
    "DiagnosticReport",
    "ReportRow",
    "status_mark",
    "format_row_text",
    "format_report_text",
    "report_from_rows",
    "print_report",
]