#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unit tests for Robot Savo mapping data models."""

from __future__ import annotations

import json

import pytest

from savo_mapping.models import (
    AprilTagObservation,
    LandmarkState,
    MappingMode,
    build_autonomous_mapping_ready_state,
    build_default_not_ready_state,
    build_idle_readiness_state,
    build_manual_mapping_ready_state,
    make_apriltag_status,
    make_autonomous_mapping_status,
    make_idle_status,
    make_landmark_key,
    make_location_confirmed_status,
    make_location_saved_status,
    make_manual_mapping_status,
    make_map_metadata,
    make_pointcloud_disabled_status,
    make_pointcloud_status,
    make_saved_map_paths,
    make_semantic_landmark,
    map_quality_from_occupancy_values,
    require_valid_mapping_mode,
    sanitize_map_name,
)


# =============================================================================
# Mapping mode
# =============================================================================
def test_mapping_mode_accepts_common_input_forms() -> None:
    assert require_valid_mapping_mode("manual mapping") == "manual_mapping"
    assert require_valid_mapping_mode("autonomous-mapping") == "autonomous_mapping"
    assert MappingMode.from_string("MAP SAVING") == MappingMode.MAP_SAVING


def test_mapping_mode_rejects_invalid_mode() -> None:
    with pytest.raises(ValueError):
        require_valid_mapping_mode("drive_fast_mode")


# =============================================================================
# Readiness states
# =============================================================================
def test_idle_readiness_is_not_degraded() -> None:
    state = build_idle_readiness_state()

    assert state.ready is True
    assert state.degraded is False
    assert state.failed_required_checks == ()
    assert state.failed_optional_checks == ()


def test_default_not_ready_state_fails_required_core_inputs() -> None:
    state = build_default_not_ready_state()

    assert state.ready is False
    assert state.degraded is False
    assert state.failed_required_checks == ("scan", "odom", "tf")


def test_manual_mapping_ready_state_is_clean() -> None:
    state = build_manual_mapping_ready_state()

    assert state.ready is True
    assert state.degraded is False
    assert state.failed_required_checks == ()
    assert state.failed_optional_checks == ()


def test_autonomous_mapping_ready_state_with_pointcloud_enabled() -> None:
    state = build_autonomous_mapping_ready_state(use_pointcloud=True)

    assert state.ready is True
    assert state.degraded is False
    assert state.get("nav2") is not None
    assert state.get("pointcloud").enabled is True


def test_autonomous_mapping_ready_state_with_pointcloud_disabled() -> None:
    state = build_autonomous_mapping_ready_state(use_pointcloud=False)

    assert state.ready is True
    assert state.degraded is False
    assert state.get("pointcloud").enabled is False


# =============================================================================
# Mapping status
# =============================================================================
def test_idle_status_uses_idle_mode() -> None:
    status = make_idle_status()

    assert status.mode == "idle"
    assert status.ready is True
    assert status.degraded is False
    assert status.active is False


def test_manual_mapping_status_tracks_readiness() -> None:
    status = make_manual_mapping_status(
        readiness=build_manual_mapping_ready_state(),
        map_name="test_map",
        session_id="session_001",
    )

    assert status.mode == "manual_mapping"
    assert status.ready is True
    assert status.active is True
    assert status.map_name == "test_map"
    assert status.session_id == "session_001"


def test_autonomous_mapping_status_tracks_readiness() -> None:
    status = make_autonomous_mapping_status(
        readiness=build_autonomous_mapping_ready_state(),
        map_name="auto_map",
    )

    assert status.mode == "autonomous_mapping"
    assert status.ready is True
    assert status.active is True
    assert status.map_name == "auto_map"


def test_mapping_status_json_is_valid() -> None:
    status = make_idle_status()
    data = json.loads(status.to_json())

    assert data["mode"] == "idle"
    assert data["ready"] is True


# =============================================================================
# Map metadata
# =============================================================================
def test_sanitize_map_name() -> None:
    assert sanitize_map_name("Savonia Campus Heart") == "savonia_campus_heart"
    assert sanitize_map_name(" A201 / Robot Lab!! ") == "a201_robot_lab"


def test_sanitize_map_name_rejects_empty_name() -> None:
    with pytest.raises(ValueError):
        sanitize_map_name("")


def test_make_map_metadata_valid() -> None:
    metadata = make_map_metadata(
        name="Savonia Campus Heart",
        width_cells=400,
        height_cells=250,
        resolution_m=0.05,
    )

    assert metadata.name == "savonia_campus_heart"
    assert metadata.valid is True
    assert metadata.width_m == pytest.approx(20.0)
    assert metadata.height_m == pytest.approx(12.5)


def test_saved_map_paths_are_stable() -> None:
    paths = make_saved_map_paths(
        "Savonia Campus Heart",
        maps_dir="maps/saved",
    )

    assert paths["name"] == "savonia_campus_heart"
    assert paths["yaml_file"].endswith("savonia_campus_heart.yaml")
    assert paths["image_file"].endswith("savonia_campus_heart.pgm")
    assert paths["metadata_file"].endswith("savonia_campus_heart.metadata.json")


# =============================================================================
# Map quality
# =============================================================================
def test_map_quality_from_good_occupancy_values() -> None:
    values = [-1] * 20 + [0] * 600 + [100] * 200

    quality = map_quality_from_occupancy_values(
        width_cells=41,
        height_cells=20,
        resolution_m=0.10,
        values=values,
    )

    assert quality.ok is True
    assert quality.known_ratio == pytest.approx(800 / 820)
    assert quality.occupied_cells == 200
    assert quality.free_cells == 600
    assert quality.unknown_cells == 20
    assert quality.width_m == pytest.approx(4.1)
    assert quality.height_m == pytest.approx(2.0)


def test_map_quality_from_weak_occupancy_values() -> None:
    values = [-1] * 78 + [0] * 20 + [100] * 2

    quality = map_quality_from_occupancy_values(
        width_cells=10,
        height_cells=10,
        resolution_m=0.05,
        values=values,
    )

    assert quality.ok is False
    assert "occupied_cells_low" in quality.extra["failures"]


# =============================================================================
# Pointcloud status
# =============================================================================
def test_pointcloud_disabled_status() -> None:
    status = make_pointcloud_disabled_status()

    assert status.enabled is False
    assert status.ok is False
    assert status.stale is True


def test_pointcloud_ok_status() -> None:
    status = make_pointcloud_status(
        enabled=True,
        msg_count=10,
        rate_hz=8.0,
        age_s=0.05,
        point_count=12000,
    )

    assert status.enabled is True
    assert status.ok is True
    assert status.stale is False
    assert status.message == "Pointcloud OK."


def test_pointcloud_stale_status() -> None:
    status = make_pointcloud_status(
        enabled=True,
        msg_count=10,
        rate_hz=8.0,
        age_s=5.0,
        point_count=12000,
        stale_timeout_s=1.0,
    )

    assert status.ok is False
    assert status.stale is True
    assert status.message == "Pointcloud stale."


# =============================================================================
# AprilTag status
# =============================================================================
def test_apriltag_status_ok() -> None:
    observation = AprilTagObservation(
        tag_id=21,
        label="A201",
        confidence=0.92,
    )

    status = make_apriltag_status(
        enabled=True,
        msg_count=3,
        detection_count=1,
        unique_tag_count=1,
        last_tag_id=21,
        last_label="A201",
        age_s=0.03,
        observations=[observation],
    )

    assert status.enabled is True
    assert status.ok is True
    assert status.last_tag_id == 21
    assert status.last_label == "A201"
    assert status.observations[0].label == "A201"


def test_apriltag_status_disabled() -> None:
    status = make_apriltag_status(enabled=False)

    assert status.enabled is False
    assert status.ok is False
    assert status.stale is True


# =============================================================================
# Semantic landmarks
# =============================================================================
def test_make_landmark_key() -> None:
    assert make_landmark_key("Info Desk") == "info_desk"
    assert make_landmark_key("A201") == "a201"
    assert make_landmark_key("Elevator 2F") == "elevator_2f"


def test_semantic_landmark_candidate_to_confirmed() -> None:
    landmark = make_semantic_landmark(
        label="Info Desk",
        x=2.0,
        y=3.5,
        yaw=1.57,
        tag_id=5,
        confidence=0.88,
    )

    confirmed = landmark.with_state(LandmarkState.CONFIRMED.value)

    assert landmark.key == "info_desk"
    assert landmark.confirmed is False
    assert confirmed.confirmed is True
    assert confirmed.pose.x == pytest.approx(2.0)
    assert confirmed.pose.y == pytest.approx(3.5)


def test_semantic_landmark_rejects_empty_label() -> None:
    with pytest.raises(ValueError):
        make_semantic_landmark(
            label="",
            x=0.0,
            y=0.0,
        )


# =============================================================================
# Location bridge
# =============================================================================
def test_location_saved_status() -> None:
    status = make_location_saved_status(
        landmark_key="a201",
        label="A201",
        location_id="loc_a201",
    )

    assert status.enabled is True
    assert status.ok is True
    assert status.saved_count == 1
    assert status.last_result.saved is True
    assert status.last_result.confirmed is False


def test_location_confirmed_status() -> None:
    status = make_location_confirmed_status(
        landmark_key="info_desk",
        label="Info Desk",
        location_id="loc_info_desk",
    )

    assert status.enabled is True
    assert status.ok is True
    assert status.confirmed_count == 1
    assert status.last_result.confirmed is True
