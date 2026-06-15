#!/usr/bin/env python3
# Copyright 2026 Ahnaf Tahmid

import json
import sys

from savo_realsense.constants import (
    DEFAULT_EXPECTED_CAMERA_INFO_HZ,
    DEFAULT_EXPECTED_COLOR_HZ,
    DEFAULT_EXPECTED_DEPTH_HZ,
    DEFAULT_EXPECTED_POINTCLOUD_HZ,
    DEFAULT_FRONT_DEPTH_PERCENTILE,
    DEFAULT_MAX_VALID_DEPTH_M,
    DEFAULT_MIN_VALID_DEPTH_M,
    DEFAULT_STALE_TIMEOUT_S,
    DEFAULT_STATUS_HZ,
)
from savo_realsense.ros.frame_contract import DEFAULT_FRAMES
from savo_realsense.ros.topic_contract import DEFAULT_TOPICS


def build_params_snapshot() -> dict:
    return {
        "topics": {
            "color_image": DEFAULT_TOPICS.color_image,
            "color_info": DEFAULT_TOPICS.color_info,
            "depth_image": DEFAULT_TOPICS.depth_image,
            "depth_info": DEFAULT_TOPICS.depth_info,
            "pointcloud": DEFAULT_TOPICS.pointcloud,
            "status": DEFAULT_TOPICS.status,
            "diagnostics": DEFAULT_TOPICS.diagnostics,
        },
        "frames": {
            "base_frame": DEFAULT_FRAMES.base_frame,
            "camera_link": DEFAULT_FRAMES.camera_link,
            "color_optical_frame": DEFAULT_FRAMES.color_optical_frame,
            "depth_optical_frame": DEFAULT_FRAMES.depth_optical_frame,
        },
        "monitoring": {
            "status_hz": DEFAULT_STATUS_HZ,
            "stale_timeout_s": DEFAULT_STALE_TIMEOUT_S,
            "expected_color_hz": DEFAULT_EXPECTED_COLOR_HZ,
            "expected_depth_hz": DEFAULT_EXPECTED_DEPTH_HZ,
            "expected_camera_info_hz": DEFAULT_EXPECTED_CAMERA_INFO_HZ,
            "expected_pointcloud_hz": DEFAULT_EXPECTED_POINTCLOUD_HZ,
        },
        "depth": {
            "min_valid_m": DEFAULT_MIN_VALID_DEPTH_M,
            "max_valid_m": DEFAULT_MAX_VALID_DEPTH_M,
            "front_depth_percentile": DEFAULT_FRONT_DEPTH_PERCENTILE,
        },
    }


def main() -> int:
    print(json.dumps(build_params_snapshot(), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
