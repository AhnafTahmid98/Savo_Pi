#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS message adapters for Robot Savo mapping Python nodes."""

from __future__ import annotations

import json
from typing import Any, Mapping, Optional


# =============================================================================
# Generic helpers
# =============================================================================
def to_json_string(data: Mapping[str, Any], *, indent: Optional[int] = None) -> str:
    return json.dumps(dict(data), indent=indent, sort_keys=True)


def from_json_string(text: str) -> dict:
    value = str(text).strip()

    if not value:
        return {}

    loaded = json.loads(value)

    if not isinstance(loaded, dict):
        raise ValueError("JSON payload must decode to a dictionary.")

    return loaded


# =============================================================================
# std_msgs adapters
# =============================================================================
def bool_msg(value: bool):
    from std_msgs.msg import Bool

    msg = Bool()
    msg.data = bool(value)
    return msg


def string_msg(value: str):
    from std_msgs.msg import String

    msg = String()
    msg.data = str(value)
    return msg


def json_msg(data: Mapping[str, Any], *, indent: Optional[int] = None):
    return string_msg(to_json_string(data, indent=indent))


def bool_from_msg(msg: Any) -> bool:
    return bool(msg.data)


def string_from_msg(msg: Any) -> str:
    return str(msg.data)


def json_from_msg(msg: Any) -> dict:
    return from_json_string(str(msg.data))


# =============================================================================
# geometry_msgs adapters
# =============================================================================
def pose_stamped_msg(
    x: float,
    y: float,
    yaw: float = 0.0,
    frame_id: str = "map",
    stamp: Optional[Any] = None,
):
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.frame_id = str(frame_id)

    if stamp is not None:
        msg.header.stamp = stamp

    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = 0.0

    qz, qw = yaw_to_quaternion_z_w(float(yaw))
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw

    return msg


def yaw_to_quaternion_z_w(yaw: float) -> tuple[float, float]:
    import math

    half = float(yaw) * 0.5
    return math.sin(half), math.cos(half)


def pose_stamped_to_dict(msg: Any) -> dict:
    return {
        "frame_id": str(msg.header.frame_id),
        "x": float(msg.pose.position.x),
        "y": float(msg.pose.position.y),
        "z": float(msg.pose.position.z),
        "orientation": {
            "x": float(msg.pose.orientation.x),
            "y": float(msg.pose.orientation.y),
            "z": float(msg.pose.orientation.z),
            "w": float(msg.pose.orientation.w),
        },
    }


# =============================================================================
# Model adapters
# =============================================================================
def mapping_status_msg(status: Any):
    return json_msg(status.to_dict())


def readiness_state_msg(readiness: Any):
    return json_msg(readiness.to_dict())


def map_quality_msg(quality: Any):
    return json_msg(quality.to_dict())


def pointcloud_status_msg(status: Any):
    return json_msg(status.to_dict())


def exploration_status_msg(status: Any):
    return json_msg(status.to_dict())


def apriltag_status_msg(status: Any):
    return json_msg(status.to_dict())


def semantic_landmark_msg(landmark: Any):
    return json_msg(landmark.to_dict())


def location_bridge_status_msg(status: Any):
    return json_msg(status.to_dict())


def exploration_goal_to_pose_msg(goal: Any, stamp: Optional[Any] = None):
    return pose_stamped_msg(
        x=goal.x,
        y=goal.y,
        yaw=getattr(goal, "yaw", 0.0),
        frame_id=getattr(goal, "frame_id", "map"),
        stamp=stamp,
    )


# =============================================================================
# OccupancyGrid helpers
# =============================================================================
def occupancy_grid_summary(msg: Any) -> dict:
    width = int(msg.info.width)
    height = int(msg.info.height)
    resolution = float(msg.info.resolution)
    values = list(msg.data)

    free = 0
    occupied = 0
    unknown = 0

    for value in values:
        cell = int(value)

        if cell < 0:
            unknown += 1
        elif cell >= 50:
            occupied += 1
        else:
            free += 1

    return {
        "frame_id": str(msg.header.frame_id),
        "width_cells": width,
        "height_cells": height,
        "resolution_m": resolution,
        "free_cells": free,
        "occupied_cells": occupied,
        "unknown_cells": unknown,
        "cell_count": len(values),
    }


def map_quality_from_occupancy_grid(msg: Any):
    from savo_mapping.models.map_quality import calculate_map_quality

    summary = occupancy_grid_summary(msg)

    return calculate_map_quality(
        width_cells=summary["width_cells"],
        height_cells=summary["height_cells"],
        resolution_m=summary["resolution_m"],
        free_cells=summary["free_cells"],
        occupied_cells=summary["occupied_cells"],
        unknown_cells=summary["unknown_cells"],
        extra={
            "frame_id": summary["frame_id"],
            "cell_count": summary["cell_count"],
        },
    )


def map_metadata_from_occupancy_grid(
    msg: Any,
    name: str,
    image_file: Optional[str] = None,
    yaml_file: Optional[str] = None,
    session_id: Optional[str] = None,
):
    from savo_mapping.models.map_metadata import MapOrigin, make_map_metadata

    origin = MapOrigin(
        x=float(msg.info.origin.position.x),
        y=float(msg.info.origin.position.y),
        yaw=0.0,
    )

    return make_map_metadata(
        name=name,
        width_cells=int(msg.info.width),
        height_cells=int(msg.info.height),
        resolution_m=float(msg.info.resolution),
        frame_id=str(msg.header.frame_id) or "map",
        origin=origin,
        image_file=image_file,
        yaml_file=yaml_file,
        session_id=session_id,
    )


# =============================================================================
# LaserScan / PointCloud helpers
# =============================================================================
def scan_summary(msg: Any) -> dict:
    ranges = list(msg.ranges)
    finite = []

    for value in ranges:
        try:
            distance = float(value)
        except (TypeError, ValueError):
            continue

        if distance == distance and distance not in (float("inf"), float("-inf")):
            finite.append(distance)

    return {
        "frame_id": str(msg.header.frame_id),
        "range_count": len(ranges),
        "finite_count": len(finite),
        "range_min_m": float(msg.range_min),
        "range_max_m": float(msg.range_max),
        "min_observed_m": min(finite) if finite else None,
        "max_observed_m": max(finite) if finite else None,
    }


def pointcloud_summary(msg: Any) -> dict:
    width = int(getattr(msg, "width", 0))
    height = int(getattr(msg, "height", 0))

    return {
        "frame_id": str(msg.header.frame_id),
        "width": width,
        "height": height,
        "point_count": width * height,
        "is_dense": bool(getattr(msg, "is_dense", False)),
        "row_step": int(getattr(msg, "row_step", 0)),
        "point_step": int(getattr(msg, "point_step", 0)),
    }


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    print(to_json_string({"package": "savo_mapping", "adapter": "ok"}))


if __name__ == "__main__":
    main()


__all__ = [
    "to_json_string",
    "from_json_string",
    "bool_msg",
    "string_msg",
    "json_msg",
    "bool_from_msg",
    "string_from_msg",
    "json_from_msg",
    "pose_stamped_msg",
    "yaw_to_quaternion_z_w",
    "pose_stamped_to_dict",
    "mapping_status_msg",
    "readiness_state_msg",
    "map_quality_msg",
    "pointcloud_status_msg",
    "exploration_status_msg",
    "apriltag_status_msg",
    "semantic_landmark_msg",
    "location_bridge_status_msg",
    "exploration_goal_to_pose_msg",
    "occupancy_grid_summary",
    "map_quality_from_occupancy_grid",
    "map_metadata_from_occupancy_grid",
    "scan_summary",
    "pointcloud_summary",
    "main",
]