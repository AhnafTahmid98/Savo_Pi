# -*- coding: utf-8 -*-

"""Config and contract validation for Robot Savo head fallback tools."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Literal

import yaml

from savo_head.constants import (
    CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC,
    FREENOVE_LOGICAL_TO_PCA9685_CHANNEL,
    PAN_CENTER_DEG_DEFAULT,
    PAN_LOGICAL_CHANNEL_DEFAULT,
    PAN_PCA9685_CHANNEL_DEFAULT,
    TILT_CENTER_DEG_DEFAULT,
    TILT_LOGICAL_CHANNEL_DEFAULT,
    TILT_MAX_DEG_DEFAULT,
    TILT_PCA9685_CHANNEL_DEFAULT,
)
from savo_head.contracts.frame_names import FRAMES
from savo_head.contracts.parameter_names import PARAMETERS, is_known_parameter
from savo_head.contracts.topic_names import TOPICS
from savo_head.core.scan_pattern import profile_from_params


Severity = Literal["error", "warning"]


@dataclass(frozen=True)
class ValidationIssue:
    severity: Severity
    source: str
    message: str
    key: str = ""

    def to_dict(self) -> dict:
        return {
            "severity": self.severity,
            "source": self.source,
            "key": self.key,
            "message": self.message,
        }


@dataclass(frozen=True)
class ValidationResult:
    issues: tuple[ValidationIssue, ...] = field(default_factory=tuple)

    def errors(self) -> tuple[ValidationIssue, ...]:
        return tuple(item for item in self.issues if item.severity == "error")

    def warnings(self) -> tuple[ValidationIssue, ...]:
        return tuple(item for item in self.issues if item.severity == "warning")

    def ok(self) -> bool:
        return not self.errors()

    def merge(self, *others: "ValidationResult") -> "ValidationResult":
        issues = list(self.issues)
        for item in others:
            issues.extend(item.issues)
        return ValidationResult(tuple(issues))

    def to_dict(self) -> dict:
        return {
            "ok": self.ok(),
            "error_count": len(self.errors()),
            "warning_count": len(self.warnings()),
            "issues": [item.to_dict() for item in self.issues],
        }

    def raise_if_errors(self) -> None:
        errors = self.errors()
        if not errors:
            return
        text = "\n".join(f"{item.source}: {item.key}: {item.message}" for item in errors)
        raise ValueError(text)


def _issue(severity: Severity, source: str, message: str, key: str = "") -> ValidationIssue:
    return ValidationIssue(severity=severity, source=source, key=key, message=message)


def _require(params: dict[str, Any], source: str, keys: Iterable[str]) -> list[ValidationIssue]:
    return [
        _issue("error", source, "missing required parameter", key)
        for key in keys
        if key not in params
    ]


def _as_params(data: dict[str, Any], node_name: str = "savo_head") -> dict[str, Any]:
    return dict(data.get(node_name, {}).get("ros__parameters", {}) or {})


def load_yaml_params(path: str | Path, node_name: str = "savo_head") -> dict[str, Any]:
    data = yaml.safe_load(Path(path).read_text()) or {}
    return _as_params(data, node_name=node_name)


def validate_head_hardware_params(params: dict[str, Any], source: str = "head_hardware") -> ValidationResult:
    issues: list[ValidationIssue] = []

    required = (
        "hardware_profile",
        "backend",
        "i2c_bus",
        "pca9685_address",
        "pwm_frequency_hz",
        "pan_logical_channel",
        "tilt_logical_channel",
        "pan_pca9685_channel",
        "tilt_pca9685_channel",
        "allow_channel_override",
        "pan_min_deg",
        "pan_center_deg",
        "pan_max_deg",
        "tilt_min_deg",
        "tilt_center_deg",
        "tilt_max_deg",
    )
    issues.extend(_require(params, source, required))

    if issues:
        return ValidationResult(tuple(issues))

    if params["backend"] not in ("pca9685", "dryrun"):
        issues.append(_issue("error", source, "backend must be pca9685 or dryrun", "backend"))

    if int(params["i2c_bus"]) != 1:
        issues.append(_issue("warning", source, "Robot Savo head is validated on I2C bus 1", "i2c_bus"))

    if int(params["pca9685_address"]) != 0x40:
        issues.append(_issue("error", source, "PCA9685 address must be 0x40", "pca9685_address"))

    pan_logical = str(params["pan_logical_channel"])
    tilt_logical = str(params["tilt_logical_channel"])

    if pan_logical != PAN_LOGICAL_CHANNEL_DEFAULT:
        issues.append(_issue("error", source, "pan logical channel must stay fixed to 7", "pan_logical_channel"))

    if tilt_logical != TILT_LOGICAL_CHANNEL_DEFAULT:
        issues.append(_issue("error", source, "tilt logical channel must stay fixed to 6", "tilt_logical_channel"))

    if int(params["pan_pca9685_channel"]) != PAN_PCA9685_CHANNEL_DEFAULT:
        issues.append(_issue("error", source, "pan PCA9685 channel must stay fixed to 15", "pan_pca9685_channel"))

    if int(params["tilt_pca9685_channel"]) != TILT_PCA9685_CHANNEL_DEFAULT:
        issues.append(_issue("error", source, "tilt PCA9685 channel must stay fixed to 14", "tilt_pca9685_channel"))

    if FREENOVE_LOGICAL_TO_PCA9685_CHANNEL.get(pan_logical) != int(params["pan_pca9685_channel"]):
        issues.append(_issue("error", source, "pan logical/PCA9685 mapping mismatch", "pan_pca9685_channel"))

    if FREENOVE_LOGICAL_TO_PCA9685_CHANNEL.get(tilt_logical) != int(params["tilt_pca9685_channel"]):
        issues.append(_issue("error", source, "tilt logical/PCA9685 mapping mismatch", "tilt_pca9685_channel"))

    if bool(params["allow_channel_override"]):
        issues.append(_issue("error", source, "production config must not allow channel override", "allow_channel_override"))

    pan_min = int(params["pan_min_deg"])
    pan_center = int(params["pan_center_deg"])
    pan_max = int(params["pan_max_deg"])

    tilt_min = int(params["tilt_min_deg"])
    tilt_center = int(params["tilt_center_deg"])
    tilt_max = int(params["tilt_max_deg"])

    if not pan_min < pan_center < pan_max:
        issues.append(_issue("error", source, "pan center must be inside min/max range", "pan_center_deg"))

    if not tilt_min <= tilt_center <= tilt_max:
        issues.append(_issue("error", source, "tilt center must be inside min/max range", "tilt_center_deg"))

    if pan_center != PAN_CENTER_DEG_DEFAULT:
        issues.append(_issue("error", source, "pan center must stay locked to 72", "pan_center_deg"))

    if tilt_center != TILT_CENTER_DEG_DEFAULT:
        issues.append(_issue("error", source, "tilt center must stay locked to 55", "tilt_center_deg"))

    if tilt_max != TILT_MAX_DEG_DEFAULT:
        issues.append(_issue("error", source, "tilt max must stay locked to 130", "tilt_max_deg"))

    return ValidationResult(tuple(issues))


def validate_scan_profile_params(params: dict[str, Any], source: str = "scan_profiles") -> ValidationResult:
    issues: list[ValidationIssue] = []

    profile = profile_from_params(params)
    for message in profile.validation_errors():
        issues.append(_issue("error", source, message))

    if profile.pan_targets_deg != (72, 170, 72, 0):
        issues.append(_issue("error", source, "semantic scan pan targets must be [72, 170, 72, 0]", "semantic_scan_pan_targets_deg"))

    if profile.tilt_sweep_pan_targets_deg != (72,):
        issues.append(_issue("error", source, "tilt sweep must happen at pan center 72", "semantic_scan_tilt_sweep_pan_targets_deg"))

    if profile.tilt_max_deg != TILT_MAX_DEG_DEFAULT:
        issues.append(_issue("error", source, "tilt max must stay locked to 130", "semantic_scan_tilt_max_deg"))

    return ValidationResult(tuple(issues))


def validate_head_topics_params(params: dict[str, Any], source: str = "head_topics") -> ValidationResult:
    issues: list[ValidationIssue] = []

    expected = {
        "pan_tilt_cmd_topic": TOPICS.pan_tilt_cmd,
        "pan_tilt_state_topic": TOPICS.pan_tilt_state,
        "scan_cmd_topic": TOPICS.scan_cmd,
        "scan_state_topic": TOPICS.scan_state,
        "status_topic": TOPICS.status,
        "dashboard_text_topic": TOPICS.dashboard_text,
        "apriltag_detections_topic": TOPICS.apriltag_detections,
        "semantic_confirmations_topic": TOPICS.semantic_confirmations,
    }

    issues.extend(_require(params, source, expected.keys()))

    for key, value in expected.items():
        if key in params and str(params[key]) != value:
            issues.append(_issue("error", source, f"expected {value}", key))

    return ValidationResult(tuple(issues))


def validate_head_frames_params(params: dict[str, Any], source: str = "head_frames") -> ValidationResult:
    issues: list[ValidationIssue] = []

    expected = {
        "base_frame": FRAMES.base_link,
        "pan_frame": FRAMES.pantilt_pan_link,
        "tilt_frame": FRAMES.pantilt_tilt_link,
        "camera_frame": FRAMES.pi_camera_link,
        "camera_optical_frame": FRAMES.pi_camera_optical_frame,
        "pan_joint_name": FRAMES.head_pan_joint,
        "tilt_joint_name": FRAMES.head_tilt_joint,
    }

    issues.extend(_require(params, source, expected.keys()))

    for key, value in expected.items():
        if key in params and str(params[key]) != value:
            issues.append(_issue("error", source, f"expected {value}", key))

    for key in (
        "base_to_pan_xyz_m",
        "base_to_pan_rpy_rad",
        "pan_to_tilt_xyz_m",
        "pan_to_tilt_rpy_rad",
        "tilt_to_camera_xyz_m",
        "tilt_to_camera_rpy_rad",
        "camera_to_optical_xyz_m",
        "camera_to_optical_rpy_rad",
    ):
        value = params.get(key)
        if not isinstance(value, list) or len(value) != 3:
            issues.append(_issue("error", source, "must be a 3-value list", key))

    return ValidationResult(tuple(issues))


def validate_camera_stream_params(params: dict[str, Any], source: str = "camera_stream") -> ValidationResult:
    issues: list[ValidationIssue] = []

    required = (
        "camera_backend",
        "udp_port",
        "width",
        "height",
        "fps",
        "format",
        "bitrate_kbps",
        "gst_binary",
        "gst_source",
        "gst_encoder",
        "gst_payloader",
        "gst_sink",
    )
    issues.extend(_require(params, source, required))

    if issues:
        return ValidationResult(tuple(issues))

    if params["camera_backend"] != CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC:
        issues.append(_issue("error", source, "camera backend must remain gstreamer_libcamerasrc", "camera_backend"))

    if params["gst_source"] != "libcamerasrc":
        issues.append(_issue("error", source, "validated Pi Camera source is libcamerasrc", "gst_source"))

    if params["gst_encoder"] != "x264enc":
        issues.append(_issue("error", source, "validated encoder is x264enc", "gst_encoder"))

    if params["gst_payloader"] != "rtph264pay":
        issues.append(_issue("error", source, "validated payloader is rtph264pay", "gst_payloader"))

    if params["gst_sink"] != "udpsink":
        issues.append(_issue("error", source, "validated sink is udpsink", "gst_sink"))

    if int(params["width"]) <= 0 or int(params["height"]) <= 0:
        issues.append(_issue("error", source, "camera resolution must be positive", "width/height"))

    if int(params["fps"]) <= 0:
        issues.append(_issue("error", source, "fps must be positive", "fps"))

    if not 1 <= int(params["udp_port"]) <= 65535:
        issues.append(_issue("error", source, "udp_port must be in range 1..65535", "udp_port"))

    if bool(params.get("stream_enabled_on_start", False)) and not str(params.get("udp_host", "")).strip():
        issues.append(_issue("error", source, "udp_host is required when stream starts on launch", "udp_host"))

    return ValidationResult(tuple(issues))


def validate_apriltag_semantics_params(params: dict[str, Any], source: str = "apriltag_semantics") -> ValidationResult:
    issues: list[ValidationIssue] = []

    required = (
        "apriltag_enabled",
        "apriltag_family",
        "confirmation_source",
        "allow_unknown_tags",
        "registered_tag_ids_csv",
        "min_stable_frames",
        "min_detection_confidence",
        "max_detection_distance_m",
        "require_tf_available",
        "require_robot_stationary",
        "require_localization_ok",
        "require_lidar_map_pose",
        "require_semantic_label",
    )
    issues.extend(_require(params, source, required))

    if issues:
        return ValidationResult(tuple(issues))

    if params["apriltag_family"] != "tag36h11":
        issues.append(_issue("warning", source, "default validated family is tag36h11", "apriltag_family"))

    if bool(params["allow_unknown_tags"]):
        issues.append(_issue("error", source, "unknown tags must not be accepted in production", "allow_unknown_tags"))

    if int(params["min_stable_frames"]) < 1:
        issues.append(_issue("error", source, "min_stable_frames must be positive", "min_stable_frames"))

    confidence = float(params["min_detection_confidence"])
    if not 0.0 <= confidence <= 1.0:
        issues.append(_issue("error", source, "confidence must be in range 0..1", "min_detection_confidence"))

    if float(params["max_detection_distance_m"]) <= 0.0:
        issues.append(_issue("error", source, "max_detection_distance_m must be positive", "max_detection_distance_m"))

    return ValidationResult(tuple(issues))


def validate_diagnostics_params(params: dict[str, Any], source: str = "diagnostics") -> ValidationResult:
    issues: list[ValidationIssue] = []

    required = (
        "diagnostics_enabled",
        "diagnostics_topic",
        "status_topic",
        "dashboard_text_topic",
        "status_publish_hz",
        "dashboard_publish_hz",
        "require_hardware_for_ok",
        "require_tf_for_ok",
        "require_camera_for_ok",
        "require_apriltag_for_ok",
    )
    issues.extend(_require(params, source, required))

    if issues:
        return ValidationResult(tuple(issues))

    if float(params["status_publish_hz"]) <= 0.0:
        issues.append(_issue("error", source, "status_publish_hz must be positive", "status_publish_hz"))

    if not bool(params["require_hardware_for_ok"]):
        issues.append(_issue("error", source, "hardware must be required for OK status", "require_hardware_for_ok"))

    if not bool(params["require_tf_for_ok"]):
        issues.append(_issue("error", source, "TF must be required for OK status", "require_tf_for_ok"))

    if bool(params["require_camera_for_ok"]):
        issues.append(_issue("warning", source, "camera stream is optional in default bringup", "require_camera_for_ok"))

    if bool(params["require_apriltag_for_ok"]):
        issues.append(_issue("warning", source, "AprilTag visibility is optional in default bringup", "require_apriltag_for_ok"))

    return ValidationResult(tuple(issues))


def validate_parameter_contract(source: str = "parameter_contract") -> ValidationResult:
    issues: list[ValidationIssue] = []

    all_params = PARAMETERS.all()
    if len(all_params) != len(set(all_params)):
        issues.append(_issue("error", source, "PARAMETERS.all() must return unique names"))

    for name in all_params:
        if not is_known_parameter(name):
            issues.append(_issue("error", source, "parameter missing from lookup", name))

    return ValidationResult(tuple(issues))


def validate_config_files(package_root: str | Path) -> ValidationResult:
    root = Path(package_root)
    config = root / "config"

    checks = (
        ("head_hardware.yaml", validate_head_hardware_params),
        ("scan_profiles.yaml", validate_scan_profile_params),
        ("head_topics.yaml", validate_head_topics_params),
        ("head_frames.yaml", validate_head_frames_params),
        ("camera_stream.yaml", validate_camera_stream_params),
        ("apriltag_semantics.yaml", validate_apriltag_semantics_params),
        ("diagnostics.yaml", validate_diagnostics_params),
    )

    result = validate_parameter_contract()

    for filename, validator in checks:
        path = config / filename
        if not path.exists():
            result = result.merge(
                ValidationResult((_issue("error", filename, "config file is missing"),))
            )
            continue

        params = load_yaml_params(path)
        result = result.merge(validator(params, source=filename))

    return result


__all__ = [
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
