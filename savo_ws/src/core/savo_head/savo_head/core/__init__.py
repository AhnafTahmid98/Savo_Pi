# -*- coding: utf-8 -*-

"""Core Python support for Robot Savo head fallback tools and tests."""

from __future__ import annotations

from savo_head.core.calibration import (
    DEFAULT_SERVO_CALIBRATION,
    SERVO_DIRECTION_NORMAL,
    SERVO_DIRECTION_REVERSED,
    HeadServoCalibration,
    ServoChannelCalibration,
    angle_to_pulse_us,
    angle_to_servo_output,
    angle_to_ticks,
    calibration_for_axis,
    clamp_pulse_us,
    logical_channel_to_pca9685_channel,
    pulse_us_to_ticks,
)
from savo_head.core.scan_pattern import (
    SCAN_PARAM_PREFIX,
    ScanRuntime,
    ScanStepResult,
    load_scan_profile_yaml,
    make_scan_runtime,
    pan_targets_from_preview,
    preview_scan,
    profile_from_params,
    profile_to_params,
    validate_scan_profile,
)
from savo_head.core.validation import (
    Severity,
    ValidationIssue,
    ValidationResult,
    load_yaml_params,
    validate_apriltag_semantics_params,
    validate_camera_stream_params,
    validate_config_files,
    validate_diagnostics_params,
    validate_head_frames_params,
    validate_head_hardware_params,
    validate_head_topics_params,
    validate_parameter_contract,
    validate_scan_profile_params,
)

__all__ = [
    # Calibration
    "SERVO_DIRECTION_NORMAL",
    "SERVO_DIRECTION_REVERSED",
    "ServoChannelCalibration",
    "HeadServoCalibration",
    "DEFAULT_SERVO_CALIBRATION",
    "clamp_pulse_us",
    "angle_to_pulse_us",
    "pulse_us_to_ticks",
    "angle_to_ticks",
    "logical_channel_to_pca9685_channel",
    "calibration_for_axis",
    "angle_to_servo_output",
    # Scan pattern
    "SCAN_PARAM_PREFIX",
    "ScanStepResult",
    "ScanRuntime",
    "profile_from_params",
    "profile_to_params",
    "load_scan_profile_yaml",
    "validate_scan_profile",
    "make_scan_runtime",
    "preview_scan",
    "pan_targets_from_preview",
    # Validation
    "Severity",
    "ValidationIssue",
    "ValidationResult",
    "load_yaml_params",
    "validate_head_hardware_params",
    "validate_scan_profile_params",
    "validate_head_topics_params",
    "validate_head_frames_params",
    "validate_camera_stream_params",
    "validate_apriltag_semantics_params",
    "validate_diagnostics_params",
    "validate_parameter_contract",
    "validate_config_files",
]
