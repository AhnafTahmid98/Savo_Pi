# -*- coding: utf-8 -*-

"""Python fallback and diagnostic support for Robot Savo active head."""

from __future__ import annotations

from savo_head.constants import (
    CAMERA_BACKEND_DEFAULT,
    CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC,
    DEFAULTS,
    FRAME_BASE,
    FRAME_PANTILT_PAN,
    FRAME_PANTILT_TILT,
    FRAME_PI_CAMERA,
    FRAME_PI_CAMERA_OPTICAL,
    HEAD_HARDWARE_PROFILE_DEFAULT,
    PAN_CENTER_DEG_DEFAULT,
    PAN_LOGICAL_CHANNEL_DEFAULT,
    PAN_MAX_DEG_DEFAULT,
    PAN_MIN_DEG_DEFAULT,
    PAN_PCA9685_CHANNEL_DEFAULT,
    PACKAGE_NAME,
    ROBOT_NAME,
    SCAN_PAN_TARGETS_DEFAULT,
    TILT_CENTER_DEG_DEFAULT,
    TILT_LOGICAL_CHANNEL_DEFAULT,
    TILT_MAX_DEG_DEFAULT,
    TILT_MIN_DEG_DEFAULT,
    TILT_PCA9685_CHANNEL_DEFAULT,
    TOPIC_APRILTAG_DETECTIONS,
    TOPIC_CAMERA_INFO,
    TOPIC_CAMERA_STATUS,
    TOPIC_DASHBOARD_TEXT,
    TOPIC_IMAGE_RAW,
    TOPIC_PAN_TILT_CMD,
    TOPIC_PAN_TILT_STATE,
    TOPIC_SCAN_CMD,
    TOPIC_SCAN_STATE,
    TOPIC_SEMANTIC_CONFIRMATIONS,
    TOPIC_STATUS,
    HeadDefaults,
    get_head_defaults,
)
from savo_head.version import (
    VERSION,
    VERSION_TUPLE,
    __version__,
    get_package_version_info,
    get_version,
    get_version_tuple,
)

__all__ = [
    # Version / identity
    "__version__",
    "VERSION",
    "VERSION_TUPLE",
    "get_version",
    "get_version_tuple",
    "get_package_version_info",
    "PACKAGE_NAME",
    "ROBOT_NAME",
    # Structured defaults
    "DEFAULTS",
    "HeadDefaults",
    "get_head_defaults",
    # Hardware profile
    "HEAD_HARDWARE_PROFILE_DEFAULT",
    "PAN_LOGICAL_CHANNEL_DEFAULT",
    "TILT_LOGICAL_CHANNEL_DEFAULT",
    "PAN_PCA9685_CHANNEL_DEFAULT",
    "TILT_PCA9685_CHANNEL_DEFAULT",
    # Calibration
    "PAN_MIN_DEG_DEFAULT",
    "PAN_CENTER_DEG_DEFAULT",
    "PAN_MAX_DEG_DEFAULT",
    "TILT_MIN_DEG_DEFAULT",
    "TILT_CENTER_DEG_DEFAULT",
    "TILT_MAX_DEG_DEFAULT",
    # Scan
    "SCAN_PAN_TARGETS_DEFAULT",
    # Camera
    "CAMERA_BACKEND_DEFAULT",
    "CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC",
    # Topics
    "TOPIC_PAN_TILT_CMD",
    "TOPIC_PAN_TILT_STATE",
    "TOPIC_SCAN_CMD",
    "TOPIC_SCAN_STATE",
    "TOPIC_STATUS",
    "TOPIC_DASHBOARD_TEXT",
    "TOPIC_IMAGE_RAW",
    "TOPIC_CAMERA_INFO",
    "TOPIC_CAMERA_STATUS",
    "TOPIC_APRILTAG_DETECTIONS",
    "TOPIC_SEMANTIC_CONFIRMATIONS",
    # Frames
    "FRAME_BASE",
    "FRAME_PANTILT_PAN",
    "FRAME_PANTILT_TILT",
    "FRAME_PI_CAMERA",
    "FRAME_PI_CAMERA_OPTICAL",
]
