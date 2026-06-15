# -*- coding: utf-8 -*-

from math import inf, nan

from savo_lidar.constants import (
    DEFAULT_SERIAL_PORT,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_WARN,
)
from savo_lidar.diagnostics import (
    check_frame_id,
    check_motor_spin,
    check_range_quality,
    compact_status_line,
    format_bool,
    format_json_report,
    format_key_value_report,
    format_optional_float,
    normalize_frame_id,
    run_scan_rate_check,
)


def test_normalize_frame_id_strips_slash_and_spaces():
    assert normalize_frame_id(" /laser ") == "laser"


def test_check_frame_id_accepts_expected_frame():
    result = check_frame_id(frame_id="/laser", expected_frame_id="laser")

    assert result.ok
    assert result.frame_id == "laser"
    assert result.expected_frame_id == "laser"
    assert result.message == "LiDAR frame_id matches expected TF frame"


def test_check_frame_id_rejects_empty_frame():
    result = check_frame_id(frame_id="", expected_frame_id="laser")

    assert not result.ok
    assert result.frame_id == ""
    assert result.message == "LiDAR frame_id is empty"


def test_check_frame_id_rejects_wrong_frame():
    result = check_frame_id(frame_id="base_link", expected_frame_id="laser")

    assert not result.ok
    assert result.frame_id == "base_link"
    assert result.expected_frame_id == "laser"
    assert result.message == "LiDAR frame_id does not match expected TF frame"


def test_check_motor_spin_fails_when_driver_not_running():
    result = check_motor_spin(
        driver_running=False,
        scan_count=0,
        scan_rate_hz=0.0,
        min_scan_rate_hz=3.0,
    )

    assert not result.ok
    assert not result.driver_running
    assert result.message == "LiDAR driver is not running"


def test_check_motor_spin_fails_when_no_scans_received():
    result = check_motor_spin(
        driver_running=True,
        scan_count=0,
        scan_rate_hz=0.0,
        min_scan_rate_hz=3.0,
    )

    assert not result.ok
    assert result.driver_running
    assert result.scan_count == 0
    assert result.message == "LiDAR motor may not be spinning; no scans received"


def test_check_motor_spin_fails_when_rate_is_too_low():
    result = check_motor_spin(
        driver_running=True,
        scan_count=10,
        scan_rate_hz=1.0,
        min_scan_rate_hz=3.0,
    )

    assert not result.ok
    assert result.scan_count == 10
    assert result.scan_rate_hz == 1.0
    assert result.message == "LiDAR scan stream is slower than expected"


def test_check_motor_spin_accepts_healthy_stream():
    result = check_motor_spin(
        driver_running=True,
        scan_count=10,
        scan_rate_hz=5.5,
        min_scan_rate_hz=3.0,
    )

    assert result.ok
    assert result.driver_running
    assert result.scan_count == 10
    assert result.scan_rate_hz == 5.5
    assert result.message == "LiDAR scan stream looks healthy"


def test_check_motor_spin_clamps_negative_inputs():
    result = check_motor_spin(
        driver_running=True,
        scan_count=-10,
        scan_rate_hz=-5.0,
        min_scan_rate_hz=-1.0,
    )

    assert not result.ok
    assert result.scan_count == 0
    assert result.scan_rate_hz == 0.0
    assert result.min_scan_rate_hz == 0.0


def test_check_range_quality_reports_empty_scan_as_error():
    result = check_range_quality([])

    assert not result.ok
    assert result.status == STATUS_ERROR
    assert result.total_points == 0
    assert result.valid_points == 0
    assert result.invalid_points == 0
    assert result.valid_ratio == 0.0
    assert result.message == "no range samples"


def test_check_range_quality_reports_error_for_too_few_valid_points():
    result = check_range_quality(
        [0.10, 13.0, inf, nan, 1.0],
        min_range_m=0.15,
        max_range_m=12.0,
        warn_ratio=0.60,
        error_ratio=0.30,
    )

    assert not result.ok
    assert result.status == STATUS_ERROR
    assert result.total_points == 5
    assert result.valid_points == 1
    assert result.invalid_points == 4
    assert result.valid_ratio == 0.2
    assert result.message == "too few valid range samples"


def test_check_range_quality_reports_warn_for_low_quality_scan():
    result = check_range_quality(
        [0.10, 0.20, 1.0, 13.0, inf],
        min_range_m=0.15,
        max_range_m=12.0,
        warn_ratio=0.60,
        error_ratio=0.30,
    )

    assert not result.ok
    assert result.status == STATUS_WARN
    assert result.total_points == 5
    assert result.valid_points == 2
    assert result.invalid_points == 3
    assert result.valid_ratio == 0.4
    assert result.message == "range quality is low"


def test_check_range_quality_reports_ok_for_good_scan():
    result = check_range_quality(
        [0.20, 1.0, 2.0, 3.0, inf],
        min_range_m=0.15,
        max_range_m=12.0,
        warn_ratio=0.60,
        error_ratio=0.30,
    )

    assert result.ok
    assert result.status == STATUS_OK
    assert result.total_points == 5
    assert result.valid_points == 4
    assert result.invalid_points == 1
    assert result.valid_ratio == 0.8
    assert result.min_range_m == 0.20
    assert result.max_range_m == 3.0
    assert result.mean_range_m == 1.55
    assert result.message == "range quality is good"


def test_check_range_quality_clamps_ratios():
    result = check_range_quality(
        [1.0],
        warn_ratio=2.0,
        error_ratio=-1.0,
    )

    assert result.warn_ratio == 1.0
    assert result.error_ratio == 0.0


def test_check_range_quality_adjusts_error_ratio_if_greater_than_warn_ratio():
    result = check_range_quality(
        [0.10, 1.0],
        min_range_m=0.15,
        max_range_m=12.0,
        warn_ratio=0.40,
        error_ratio=0.80,
    )

    assert result.warn_ratio == 0.40
    assert result.error_ratio == 0.40


def test_range_quality_to_dict_contains_expected_fields():
    result = check_range_quality([1.0, 2.0])
    data = result.to_dict()

    assert data["total_points"] == 2
    assert data["valid_points"] == 2
    assert data["invalid_points"] == 0
    assert data["ok"] is True
    assert data["status"] == STATUS_OK


def test_run_scan_rate_check_accepts_healthy_rate():
    result = run_scan_rate_check(
        measured_rate_hz=5.5,
        expected_rate_hz=5.5,
        tolerance_ratio=0.50,
    )

    assert result.ok
    assert result.rate_hz == 5.5
    assert result.expected_hz == 5.5
    assert result.min_allowed_hz == 2.75
    assert result.message == "LiDAR scan rate healthy"


def test_run_scan_rate_check_rejects_slow_rate():
    result = run_scan_rate_check(
        measured_rate_hz=1.0,
        expected_rate_hz=5.5,
        tolerance_ratio=0.50,
    )

    assert not result.ok
    assert result.rate_hz == 1.0
    assert result.expected_hz == 5.5
    assert result.min_allowed_hz == 2.75
    assert result.message == "LiDAR scan rate below expected range"


def test_scan_rate_check_to_dict_contains_expected_fields():
    result = run_scan_rate_check(measured_rate_hz=5.5)
    data = result.to_dict()

    assert data["rate_hz"] == 5.5
    assert data["ok"] is True
    assert data["message"] == "LiDAR scan rate healthy"


def test_format_bool_outputs_yes_no():
    assert format_bool(True) == "yes"
    assert format_bool(False) == "no"


def test_format_optional_float_formats_value():
    assert format_optional_float(1.23456, digits=2, suffix=" m") == "1.23 m"


def test_format_optional_float_handles_none():
    assert format_optional_float(None) == "n/a"


def test_format_key_value_report_formats_values():
    report = format_key_value_report(
        "LiDAR",
        {
            "ok": True,
            "rate_hz": 5.5,
            "port": DEFAULT_SERIAL_PORT,
        },
    )

    assert "LiDAR" in report
    assert "-----" in report
    assert "ok: yes" in report
    assert "rate_hz: 5.500" in report
    assert f"port: {DEFAULT_SERIAL_PORT}" in report


def test_format_json_report_formats_nested_values():
    report = format_json_report(
        {
            "scan": {
                "ok": True,
                "points": [1, 2, 3],
            }
        }
    )

    assert '"scan"' in report
    assert '"ok": true' in report
    assert '"points"' in report


def test_compact_status_line_formats_success():
    line = compact_status_line(
        name="scan",
        ok=True,
        message="healthy",
        rate_hz=5.5,
        unused=None,
    )

    assert line == "scan: OK | healthy | rate_hz=5.500"


def test_compact_status_line_formats_failure():
    line = compact_status_line(
        name="port",
        ok=False,
        message="not found",
        preferred=DEFAULT_SERIAL_PORT,
    )

    assert line == f"port: FAIL | not found | preferred={DEFAULT_SERIAL_PORT}"
