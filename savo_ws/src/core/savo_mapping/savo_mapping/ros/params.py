#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Parameter helpers for Robot Savo mapping Python nodes. No ROS runtime required."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional

from savo_mapping.constants import (
    DEFAULT_MAP_STALE_TIMEOUT_S,
    DEFAULT_ODOM_STALE_TIMEOUT_S,
    DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
    DEFAULT_PUBLISH_RATE_HZ,
    DEFAULT_SCAN_STALE_TIMEOUT_S,
    DEFAULT_TF_TIMEOUT_S,
    FRAME_BASE_LINK,
    FRAME_CAMERA_DEPTH,
    FRAME_LASER,
    FRAME_MAP,
    FRAME_ODOM,
    TOPIC_MAP,
    TOPIC_MAP_METADATA,
    TOPIC_ODOM,
    TOPIC_REALSENSE_POINTS,
    TOPIC_SCAN,
)
from savo_mapping.utils.param_loader import get_nested


# =============================================================================
# Mapping supervisor params
# =============================================================================
@dataclass(frozen=True)
class MappingSupervisorParams:
    scan_topic: str = TOPIC_SCAN
    odom_topic: str = TOPIC_ODOM
    map_topic: str = TOPIC_MAP
    map_metadata_topic: str = TOPIC_MAP_METADATA
    pointcloud_topic: str = TOPIC_REALSENSE_POINTS

    map_frame: str = FRAME_MAP
    odom_frame: str = FRAME_ODOM
    base_frame: str = FRAME_BASE_LINK
    laser_frame: str = FRAME_LASER
    camera_depth_frame: str = FRAME_CAMERA_DEPTH

    publish_rate_hz: float = DEFAULT_PUBLISH_RATE_HZ

    require_scan: bool = True
    require_odom: bool = True
    require_tf: bool = True
    require_map: bool = False
    require_pointcloud: bool = False

    scan_stale_timeout_s: float = DEFAULT_SCAN_STALE_TIMEOUT_S
    odom_stale_timeout_s: float = DEFAULT_ODOM_STALE_TIMEOUT_S
    tf_timeout_s: float = DEFAULT_TF_TIMEOUT_S
    map_stale_timeout_s: float = DEFAULT_MAP_STALE_TIMEOUT_S
    pointcloud_stale_timeout_s: float = DEFAULT_POINTCLOUD_STALE_TIMEOUT_S

    def to_dict(self) -> dict:
        return {
            "scan_topic": self.scan_topic,
            "odom_topic": self.odom_topic,
            "map_topic": self.map_topic,
            "map_metadata_topic": self.map_metadata_topic,
            "pointcloud_topic": self.pointcloud_topic,
            "map_frame": self.map_frame,
            "odom_frame": self.odom_frame,
            "base_frame": self.base_frame,
            "laser_frame": self.laser_frame,
            "camera_depth_frame": self.camera_depth_frame,
            "publish_rate_hz": self.publish_rate_hz,
            "require_scan": self.require_scan,
            "require_odom": self.require_odom,
            "require_tf": self.require_tf,
            "require_map": self.require_map,
            "require_pointcloud": self.require_pointcloud,
            "scan_stale_timeout_s": self.scan_stale_timeout_s,
            "odom_stale_timeout_s": self.odom_stale_timeout_s,
            "tf_timeout_s": self.tf_timeout_s,
            "map_stale_timeout_s": self.map_stale_timeout_s,
            "pointcloud_stale_timeout_s": self.pointcloud_stale_timeout_s,
        }


# =============================================================================
# Mode manager params
# =============================================================================
@dataclass(frozen=True)
class MappingModeManagerParams:
    default_mode: str = "idle"
    allow_autonomous_mapping: bool = False
    allow_manual_mapping: bool = True
    allow_map_save: bool = True
    publish_rate_hz: float = DEFAULT_PUBLISH_RATE_HZ

    def to_dict(self) -> dict:
        return {
            "default_mode": self.default_mode,
            "allow_autonomous_mapping": self.allow_autonomous_mapping,
            "allow_manual_mapping": self.allow_manual_mapping,
            "allow_map_save": self.allow_map_save,
            "publish_rate_hz": self.publish_rate_hz,
        }


# =============================================================================
# Frontier explorer params
# =============================================================================
@dataclass(frozen=True)
class FrontierExplorerParams:
    enabled: bool = False
    map_topic: str = TOPIC_MAP
    map_frame: str = FRAME_MAP
    base_frame: str = FRAME_BASE_LINK

    min_frontier_size_cells: int = 5
    goal_clearance_m: float = 0.35
    goal_reached_radius_m: float = 0.35
    failed_goal_blacklist_radius_m: float = 0.60

    max_failed_goals: int = 20
    publish_rate_hz: float = 1.0

    def to_dict(self) -> dict:
        return {
            "enabled": self.enabled,
            "map_topic": self.map_topic,
            "map_frame": self.map_frame,
            "base_frame": self.base_frame,
            "min_frontier_size_cells": self.min_frontier_size_cells,
            "goal_clearance_m": self.goal_clearance_m,
            "goal_reached_radius_m": self.goal_reached_radius_m,
            "failed_goal_blacklist_radius_m": self.failed_goal_blacklist_radius_m,
            "max_failed_goals": self.max_failed_goals,
            "publish_rate_hz": self.publish_rate_hz,
        }


# =============================================================================
# Pointcloud monitor params
# =============================================================================
@dataclass(frozen=True)
class PointcloudMonitorParams:
    enabled: bool = False
    pointcloud_topic: str = TOPIC_REALSENSE_POINTS
    expected_frame: str = FRAME_CAMERA_DEPTH
    stale_timeout_s: float = DEFAULT_POINTCLOUD_STALE_TIMEOUT_S
    min_rate_hz: float = 3.0
    publish_rate_hz: float = DEFAULT_PUBLISH_RATE_HZ

    def to_dict(self) -> dict:
        return {
            "enabled": self.enabled,
            "pointcloud_topic": self.pointcloud_topic,
            "expected_frame": self.expected_frame,
            "stale_timeout_s": self.stale_timeout_s,
            "min_rate_hz": self.min_rate_hz,
            "publish_rate_hz": self.publish_rate_hz,
        }


# =============================================================================
# Load helpers
# =============================================================================
def _get_bool(data: Mapping[str, Any], path: str, default: bool) -> bool:
    return bool(get_nested(data, path, default))


def _get_int(data: Mapping[str, Any], path: str, default: int) -> int:
    return int(get_nested(data, path, default))


def _get_float(data: Mapping[str, Any], path: str, default: float) -> float:
    return float(get_nested(data, path, default))


def _get_str(data: Mapping[str, Any], path: str, default: str) -> str:
    return str(get_nested(data, path, default))


def mapping_supervisor_params_from_dict(
    data: Optional[Mapping[str, Any]],
) -> MappingSupervisorParams:
    values = data or {}

    return MappingSupervisorParams(
        scan_topic=_get_str(values, "scan_topic", TOPIC_SCAN),
        odom_topic=_get_str(values, "odom_topic", TOPIC_ODOM),
        map_topic=_get_str(values, "map_topic", TOPIC_MAP),
        map_metadata_topic=_get_str(values, "map_metadata_topic", TOPIC_MAP_METADATA),
        pointcloud_topic=_get_str(values, "pointcloud_topic", TOPIC_REALSENSE_POINTS),
        map_frame=_get_str(values, "map_frame", FRAME_MAP),
        odom_frame=_get_str(values, "odom_frame", FRAME_ODOM),
        base_frame=_get_str(values, "base_frame", FRAME_BASE_LINK),
        laser_frame=_get_str(values, "laser_frame", FRAME_LASER),
        camera_depth_frame=_get_str(values, "camera_depth_frame", FRAME_CAMERA_DEPTH),
        publish_rate_hz=_get_float(values, "publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ),
        require_scan=_get_bool(values, "require_scan", True),
        require_odom=_get_bool(values, "require_odom", True),
        require_tf=_get_bool(values, "require_tf", True),
        require_map=_get_bool(values, "require_map", False),
        require_pointcloud=_get_bool(values, "require_pointcloud", False),
        scan_stale_timeout_s=_get_float(
            values,
            "scan_stale_timeout_s",
            DEFAULT_SCAN_STALE_TIMEOUT_S,
        ),
        odom_stale_timeout_s=_get_float(
            values,
            "odom_stale_timeout_s",
            DEFAULT_ODOM_STALE_TIMEOUT_S,
        ),
        tf_timeout_s=_get_float(values, "tf_timeout_s", DEFAULT_TF_TIMEOUT_S),
        map_stale_timeout_s=_get_float(
            values,
            "map_stale_timeout_s",
            DEFAULT_MAP_STALE_TIMEOUT_S,
        ),
        pointcloud_stale_timeout_s=_get_float(
            values,
            "pointcloud_stale_timeout_s",
            DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
        ),
    )


def mapping_mode_manager_params_from_dict(
    data: Optional[Mapping[str, Any]],
) -> MappingModeManagerParams:
    values = data or {}

    return MappingModeManagerParams(
        default_mode=_get_str(values, "default_mode", "idle"),
        allow_autonomous_mapping=_get_bool(values, "allow_autonomous_mapping", False),
        allow_manual_mapping=_get_bool(values, "allow_manual_mapping", True),
        allow_map_save=_get_bool(values, "allow_map_save", True),
        publish_rate_hz=_get_float(values, "publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ),
    )


def frontier_explorer_params_from_dict(
    data: Optional[Mapping[str, Any]],
) -> FrontierExplorerParams:
    values = data or {}

    return FrontierExplorerParams(
        enabled=_get_bool(values, "enabled", False),
        map_topic=_get_str(values, "map_topic", TOPIC_MAP),
        map_frame=_get_str(values, "map_frame", FRAME_MAP),
        base_frame=_get_str(values, "base_frame", FRAME_BASE_LINK),
        min_frontier_size_cells=_get_int(values, "min_frontier_size_cells", 5),
        goal_clearance_m=_get_float(values, "goal_clearance_m", 0.35),
        goal_reached_radius_m=_get_float(values, "goal_reached_radius_m", 0.35),
        failed_goal_blacklist_radius_m=_get_float(
            values,
            "failed_goal_blacklist_radius_m",
            0.60,
        ),
        max_failed_goals=_get_int(values, "max_failed_goals", 20),
        publish_rate_hz=_get_float(values, "publish_rate_hz", 1.0),
    )


def pointcloud_monitor_params_from_dict(
    data: Optional[Mapping[str, Any]],
) -> PointcloudMonitorParams:
    values = data or {}

    return PointcloudMonitorParams(
        enabled=_get_bool(values, "enabled", False),
        pointcloud_topic=_get_str(values, "pointcloud_topic", TOPIC_REALSENSE_POINTS),
        expected_frame=_get_str(values, "expected_frame", FRAME_CAMERA_DEPTH),
        stale_timeout_s=_get_float(
            values,
            "stale_timeout_s",
            DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
        ),
        min_rate_hz=_get_float(values, "min_rate_hz", 3.0),
        publish_rate_hz=_get_float(values, "publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ),
    )


def build_default_param_bundle() -> Dict[str, dict]:
    return {
        "mapping_supervisor": MappingSupervisorParams().to_dict(),
        "mapping_mode_manager": MappingModeManagerParams().to_dict(),
        "frontier_explorer": FrontierExplorerParams().to_dict(),
        "pointcloud_monitor": PointcloudMonitorParams().to_dict(),
    }


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    bundle = build_default_param_bundle()

    for name, params in bundle.items():
        print(f"[{name}]")
        for key in sorted(params.keys()):
            print(f"{key}: {params[key]}")
        print()


if __name__ == "__main__":
    main()


__all__ = [
    "MappingSupervisorParams",
    "MappingModeManagerParams",
    "FrontierExplorerParams",
    "PointcloudMonitorParams",
    "mapping_supervisor_params_from_dict",
    "mapping_mode_manager_params_from_dict",
    "frontier_explorer_params_from_dict",
    "pointcloud_monitor_params_from_dict",
    "build_default_param_bundle",
    "main",
]