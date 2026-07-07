"""Python diagnostics for Robot Savo power monitoring."""

from __future__ import annotations

from savo_power.diagnostics.i2c_power_check import (
    I2cDeviceCheck,
    I2cPowerCheckSummary,
    check_ads7830,
    check_ups_hat,
    main as i2c_power_check_main,
    make_arg_parser as make_i2c_power_check_arg_parser,
    run_power_i2c_check,
)

from savo_power.diagnostics.kit_battery_check import (
    KitBatteryCheckSummary,
    main as kit_battery_check_main,
    make_arg_parser as make_kit_battery_check_arg_parser,
    read_kit_battery_samples,
    run_kit_battery_check,
)

from savo_power.diagnostics.report_formatter import (
    DiagnosticReport,
    ReportLine,
    ReportStatus,
    format_float,
    format_json,
    format_key_values,
    format_percentage,
    format_voltage,
    make_report,
    make_report_line,
    normalize_report_status,
    print_report,
    report_status_from_ok,
    safe_text,
    to_jsonable,
)

from savo_power.diagnostics.ups_check import (
    UpsCheckSummary,
    main as ups_check_main,
    make_arg_parser as make_ups_check_arg_parser,
    read_ups_samples,
    run_ups_check,
)


__all__ = [
    "I2cDeviceCheck",
    "I2cPowerCheckSummary",
    "check_ads7830",
    "check_ups_hat",
    "i2c_power_check_main",
    "make_i2c_power_check_arg_parser",
    "run_power_i2c_check",
    "KitBatteryCheckSummary",
    "kit_battery_check_main",
    "make_kit_battery_check_arg_parser",
    "read_kit_battery_samples",
    "run_kit_battery_check",
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
    "UpsCheckSummary",
    "ups_check_main",
    "make_ups_check_arg_parser",
    "read_ups_samples",
    "run_ups_check",
]
