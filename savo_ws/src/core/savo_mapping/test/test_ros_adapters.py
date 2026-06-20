#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unit tests for Robot Savo mapping ROS adapters."""

from __future__ import annotations

import json
import math

import pytest

from savo_mapping.models.exploration_status import ExplorationGoal
from savo_mapping.models.mapping_status import make_idle_status
from savo_mapping.models.readiness_state import build_idle_readiness_state
from savo_mapping.ros.adapters import (
    bool_from_msg,
    bool_msg,
    exploration_goal_to_pose_msg,
    from_json_string,
    json_from_msg,
    json_msg,
    map_metadata_from_occupancy_grid,
    map_quality_from_occupancy_grid,
    mapping_status_msg,
    occupancy_grid_summary,
    pointcloud_summary,
    pose_stamped_msg,
    pose_stamped_to_dict,
    readiness_state_msg,
    scan_summary,
    string_from_msg,
    string_msg,
    to_json_string,
    yaw_to_quaternion_z_w,
)


# =============================================================================
# JSON helpers
# =============================================================================
def test_to_json_string_sorts_keys() -> None:
    text = to_json_string({"ready": True, "mode": "manual_mapping"})

    assert text == '{"mode": "manual_mapping", "ready": true}'


def test_from_json_string_returns_dict() -> None:
    data = from_json_string('{"ready": true, "mode": "manual_mapping"}')

    assert data["ready"] is True
    assert data["mode"] == "manual_mapping"


def test_from_json_string_empty_returns_empty_dict() -> None:
    assert from_json_string("") == {}
    assert from_json_string("   ") == {}


def test_from_json_string_rejects_non_dict_json() -> None:
    with pytest.raises(ValueError):
        from_json_string("[1, 2, 3]")


# =============================================================================
# std_msgs adapters
# =============================================================================
def test_bool_msg_roundtrip() -> None:
    msg = bool_msg(True)

    assert msg.data is True
    assert bool_from_msg(msg) is True


def test_string_msg_roundtrip() -> None:
    msg = string_msg("Robot Savo")

    assert msg.data == "Robot Savo"
    assert string_from_msg(msg) == "Robot Savo"


def test_json_msg_roundtrip() -> None:
    msg = json_msg({"ready": True, "mode": "manual_mapping"})

    data = json_from_msg(msg)

    assert data["ready"] is True
    assert data["mode"] == "manual_mapping"


# =============================================================================
# Pose adapters
# =============================================================================
def test_yaw_to_quaternion_zero() -> None:
    qz, qw = yaw_to_quaternion_z_w(0.0)

    assert qz == pytest.approx(0.0)
    assert qw == pytest.approx(1.0)


def test_yaw_to_quaternion_pi() -> None:
    qz, qw = yaw_to_quaternion_z_w(math.pi)

    assert qz == pytest.approx(1.0)
    assert qw == pytest.approx(0.0)


def test_pose_stamped_msg() -> None:
    msg = pose_stamped_msg(
        x=2.0,
        y=3.5,
        yaw=math.pi / 2.0,
        frame_id="map",
    )

    assert msg.header.frame_id == "map"
    assert msg.pose.position.x == pytest.approx(2.0)
    assert msg.pose.position.y == pytest.approx(3.5)
    assert msg.pose.position.z == pytest.approx(0.0)
    assert msg.pose.orientation.z == pytest.approx(math.sin(math.pi / 4.0))
    assert msg.pose.orientation.w == pytest.approx(math.cos(math.pi / 4.0))


def test_pose_stamped_to_dict() -> None:
    msg = pose_stamped_msg(
        x=2.0,
        y=3.5,
        yaw=0.0,
        frame_id="map",
    )

    data = pose_stamped_to_dict(msg)

    assert data["frame_id"] == "map"
    assert data["x"] == pytest.approx(2.0)
    assert data["y"] == pytest.approx(3.5)
    assert data["orientation"]["w"] == pytest.approx(1.0)


def test_exploration_goal_to_pose_msg() -> None:
    goal = ExplorationGoal(
        x=4.25,
        y=8.70,
        yaw=1.57,
        frame_id="map",
        score=0.82,
        reason="closest_frontier",
    )

    msg = exploration_goal_to_pose_msg(goal)

    assert msg.header.frame_id == "map"
    assert msg.pose.position.x == pytest.approx(4.25)
    assert msg.pose.position.y == pytest.approx(8.70)


# =============================================================================
# Model message adapters
# =============================================================================
def test_mapping_status_msg_contains_json_status() -> None:
    status = make_idle_status()

    msg = mapping_status_msg(status)
    data = json.loads(msg.data)

    assert data["mode"] == "idle"
    assert data["ready"] is True


def test_readiness_state_msg_contains_json_readiness() -> None:
    readiness = build_idle_readiness_state()

    msg = readiness_state_msg(readiness)
    data = json.loads(msg.data)

    assert data["ready"] is True
    assert data["degraded"] is False


# =============================================================================
# OccupancyGrid adapters
# =============================================================================
def _make_occupancy_grid():
    from nav_msgs.msg import OccupancyGrid

    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.width = 100
    msg.info.height = 80
    msg.info.resolution = 0.05
    msg.info.origin.position.x = -1.0
    msg.info.origin.position.y = -2.0

    msg.data = [-1] * 2500 + [0] * 5000 + [100] * 500

    return msg


def test_occupancy_grid_summary() -> None:
    msg = _make_occupancy_grid()

    summary = occupancy_grid_summary(msg)

    assert summary["frame_id"] == "map"
    assert summary["width_cells"] == 100
    assert summary["height_cells"] == 80
    assert summary["resolution_m"] == pytest.approx(0.05)
    assert summary["free_cells"] == 5000
    assert summary["occupied_cells"] == 500
    assert summary["unknown_cells"] == 2500
    assert summary["cell_count"] == 8000


def test_map_quality_from_occupancy_grid() -> None:
    msg = _make_occupancy_grid()

    quality = map_quality_from_occupancy_grid(msg)

    assert quality.ok is True
    assert quality.width_cells == 100
    assert quality.height_cells == 80
    assert quality.free_cells == 5000
    assert quality.occupied_cells == 500
    assert quality.unknown_cells == 2500


def test_map_metadata_from_occupancy_grid() -> None:
    msg = _make_occupancy_grid()

    metadata = map_metadata_from_occupancy_grid(
        msg,
        name="Savonia Campus Heart",
        image_file="maps/saved/savonia_campus_heart.pgm",
        yaml_file="maps/saved/savonia_campus_heart.yaml",
        session_id="session_001",
    )

    assert metadata.name == "savonia_campus_heart"
    assert metadata.valid is True
    assert metadata.frame_id == "map"
    assert metadata.width_cells == 100
    assert metadata.height_cells == 80
    assert metadata.resolution_m == pytest.approx(0.05)
    assert metadata.origin.x == pytest.approx(-1.0)
    assert metadata.origin.y == pytest.approx(-2.0)
    assert metadata.session_id == "session_001"


# =============================================================================
# LaserScan adapters
# =============================================================================
def test_scan_summary() -> None:
    from sensor_msgs.msg import LaserScan

    msg = LaserScan()
    msg.header.frame_id = "laser"
    msg.range_min = 0.15
    msg.range_max = 12.0
    msg.ranges = [
        1.0,
        2.0,
        float("inf"),
        float("nan"),
        0.5,
    ]

    summary = scan_summary(msg)

    assert summary["frame_id"] == "laser"
    assert summary["range_count"] == 5
    assert summary["finite_count"] == 3
    assert summary["range_min_m"] == pytest.approx(0.15)
    assert summary["range_max_m"] == pytest.approx(12.0)
    assert summary["min_observed_m"] == pytest.approx(0.5)
    assert summary["max_observed_m"] == pytest.approx(2.0)


# =============================================================================
# PointCloud2 adapters
# =============================================================================
def test_pointcloud_summary() -> None:
    from sensor_msgs.msg import PointCloud2

    msg = PointCloud2()
    msg.header.frame_id = "camera_depth_optical_frame"
    msg.width = 160
    msg.height = 75
    msg.is_dense = False
    msg.point_step = 16
    msg.row_step = msg.width * msg.point_step

    summary = pointcloud_summary(msg)

    assert summary["frame_id"] == "camera_depth_optical_frame"
    assert summary["width"] == 160
    assert summary["height"] == 75
    assert summary["point_count"] == 12000
    assert summary["is_dense"] is False
    assert summary["point_step"] == 16
    assert summary["row_step"] == 2560