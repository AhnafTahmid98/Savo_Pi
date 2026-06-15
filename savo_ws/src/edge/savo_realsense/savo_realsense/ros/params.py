# Copyright 2026 Ahnaf Tahmid
from dataclasses import dataclass

from savo_realsense.constants import (
    DEFAULT_EXPECTED_CAMERA_INFO_HZ,
    DEFAULT_EXPECTED_COLOR_HZ,
    DEFAULT_EXPECTED_DEPTH_HZ,
    DEFAULT_EXPECTED_POINTCLOUD_HZ,
    DEFAULT_STALE_TIMEOUT_S,
    DEFAULT_STATUS_HZ,
)


@dataclass(frozen=True)
class StreamMonitorParams:
    expected_color_hz: float = DEFAULT_EXPECTED_COLOR_HZ
    expected_depth_hz: float = DEFAULT_EXPECTED_DEPTH_HZ
    expected_camera_info_hz: float = DEFAULT_EXPECTED_CAMERA_INFO_HZ
    expected_pointcloud_hz: float = DEFAULT_EXPECTED_POINTCLOUD_HZ
    stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S


@dataclass(frozen=True)
class CameraHealthParams:
    status_hz: float = DEFAULT_STATUS_HZ
    stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S
    require_pointcloud: bool = False


def clamp_positive(value: float, fallback: float) -> float:
    if value <= 0.0:
        return fallback
    return value


def load_stream_monitor_params(node) -> StreamMonitorParams:
    node.declare_parameter("expected_color_hz", DEFAULT_EXPECTED_COLOR_HZ)
    node.declare_parameter("expected_depth_hz", DEFAULT_EXPECTED_DEPTH_HZ)
    node.declare_parameter("expected_camera_info_hz", DEFAULT_EXPECTED_CAMERA_INFO_HZ)
    node.declare_parameter("expected_pointcloud_hz", DEFAULT_EXPECTED_POINTCLOUD_HZ)
    node.declare_parameter("stale_timeout_s", DEFAULT_STALE_TIMEOUT_S)

    return StreamMonitorParams(
        expected_color_hz=clamp_positive(
            float(node.get_parameter("expected_color_hz").value),
            DEFAULT_EXPECTED_COLOR_HZ,
        ),
        expected_depth_hz=clamp_positive(
            float(node.get_parameter("expected_depth_hz").value),
            DEFAULT_EXPECTED_DEPTH_HZ,
        ),
        expected_camera_info_hz=clamp_positive(
            float(node.get_parameter("expected_camera_info_hz").value),
            DEFAULT_EXPECTED_CAMERA_INFO_HZ,
        ),
        expected_pointcloud_hz=clamp_positive(
            float(node.get_parameter("expected_pointcloud_hz").value),
            DEFAULT_EXPECTED_POINTCLOUD_HZ,
        ),
        stale_timeout_s=clamp_positive(
            float(node.get_parameter("stale_timeout_s").value),
            DEFAULT_STALE_TIMEOUT_S,
        ),
    )


def load_camera_health_params(node) -> CameraHealthParams:
    node.declare_parameter("status_hz", DEFAULT_STATUS_HZ)
    node.declare_parameter("stale_timeout_s", DEFAULT_STALE_TIMEOUT_S)
    node.declare_parameter("require_pointcloud", False)

    return CameraHealthParams(
        status_hz=clamp_positive(
            float(node.get_parameter("status_hz").value),
            DEFAULT_STATUS_HZ,
        ),
        stale_timeout_s=clamp_positive(
            float(node.get_parameter("stale_timeout_s").value),
            DEFAULT_STALE_TIMEOUT_S,
        ),
        require_pointcloud=bool(node.get_parameter("require_pointcloud").value),
    )
