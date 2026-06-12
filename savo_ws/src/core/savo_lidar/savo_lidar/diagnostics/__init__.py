"""Diagnostic helpers for Robot Savo LiDAR bringup and health checks."""

from .frame_id_check import FrameIdCheckResult, check_frame_id, normalize_frame_id
from .motor_spin_check import MotorSpinCheckResult, check_motor_spin
from .port_check import PortCheckResult, check_lidar_port
from .range_quality_check import RangeQualityCheckResult, check_range_quality
from .report_formatter import (
    compact_status_line,
    format_bool,
    format_json_report,
    format_key_value_report,
    format_optional_float,
)
from .scan_rate_check import ScanRateCheckResult, run_scan_rate_check

__all__ = [
    "FrameIdCheckResult",
    "MotorSpinCheckResult",
    "PortCheckResult",
    "RangeQualityCheckResult",
    "ScanRateCheckResult",
    "check_frame_id",
    "check_lidar_port",
    "check_motor_spin",
    "check_range_quality",
    "compact_status_line",
    "format_bool",
    "format_json_report",
    "format_key_value_report",
    "format_optional_float",
    "normalize_frame_id",
    "run_scan_rate_check",
]