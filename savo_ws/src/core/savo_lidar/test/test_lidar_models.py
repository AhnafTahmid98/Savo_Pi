# -*- coding: utf-8 -*-

import pytest

from savo_lidar.constants import (
    BACKEND_DRYRUN,
    BACKEND_REAL,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_OFFLINE,
    STATUS_STALE,
    STATUS_WARN,
)
from savo_lidar.models import (
    LidarDriverConfig,
    LidarHealth,
    LidarState,
    make_driver_config,
    make_lidar_state,
    make_scan_quality,
    make_sector_scan,
    quality_status,
)


def test_lidar_driver_config_accepts_default_dryrun_config():
    config = LidarDriverConfig()

    config.validate()

    assert config.backend == BACKEND_DRYRUN
    assert config.model == "rplidar_a1"
    assert config.frame_id == "laser"
    assert config.scan_topic == "/scan"
    assert config.driver_state_topic == "/savo_lidar/state"
    assert config.heartbeat_topic == "/savo_lidar/heartbeat"
    assert config.is_dryrun_backend
    assert not config.is_real_backend


def test_lidar_driver_config_accepts_real_backend_with_serial_port():
    config = LidarDriverConfig(
        backend=BACKEND_REAL,
        serial_port="/dev/ttyUSB0",
    )

    config.validate()

    assert config.is_real_backend
    assert not config.is_dryrun_backend


def test_lidar_driver_config_rejects_invalid_backend():
    with pytest.raises(ValueError):
        LidarDriverConfig(backend="bad_backend").validate()


def test_lidar_driver_config_rejects_invalid_range_limits():
    with pytest.raises(ValueError):
        LidarDriverConfig(min_range_m=1.0, max_range_m=0.5).validate()


def test_lidar_driver_config_rejects_invalid_driver_state_topic():
    with pytest.raises(ValueError):
        LidarDriverConfig(
            publish_driver_state=True,
            driver_state_topic="",
        ).validate()


def test_lidar_driver_config_rejects_invalid_heartbeat_hz():
    with pytest.raises(ValueError):
        LidarDriverConfig(heartbeat_hz=0.0).validate()


def test_make_driver_config_validates_overrides():
    config = make_driver_config(backend=BACKEND_DRYRUN, frame_id="laser")

    assert config.backend == BACKEND_DRYRUN
    assert config.frame_id == "laser"


def test_lidar_driver_config_to_dict_contains_runtime_topics():
    config = make_driver_config()

    data = config.to_dict()

    assert data["scan_topic"] == "/scan"
    assert data["driver_state_topic"] == "/savo_lidar/state"
    assert data["heartbeat_topic"] == "/savo_lidar/heartbeat"


def test_lidar_state_default_is_offline():
    state = LidarState(node="lidar_driver_node")

    assert state.status == STATUS_OFFLINE
    assert not state.connected
    assert not state.motor_running
    assert not state.publishing_scan
    assert state.scan_count == 0
    assert not state.has_scan()


def test_lidar_state_reports_ok_scan_state():
    state = make_lidar_state(
        status=STATUS_OK,
        connected=True,
        motor_running=True,
        publishing_scan=True,
        scan_count=10,
        scan_rate_hz=5.5,
        valid_points=300,
        total_points=360,
        valid_ratio=300 / 360,
        front_min_m=0.70,
    )

    assert state.is_ok()
    assert state.has_scan()
    assert state.connected
    assert state.motor_running
    assert state.publishing_scan
    assert state.scan_count == 10
    assert state.front_min_m == 0.70


def test_lidar_state_reports_stale_state():
    state = make_lidar_state(
        status=STATUS_STALE,
        publishing_scan=False,
        scan_count=10,
        last_scan_age_s=2.0,
        detail="scan stopped",
    )

    assert state.is_stale()
    assert not state.has_scan()
    assert state.detail == "scan stopped"


def test_lidar_state_to_dict_contains_scan_metrics():
    state = make_lidar_state(
        status=STATUS_OK,
        scan_count=3,
        scan_rate_hz=5.5,
        valid_ratio=0.9,
    )

    data = state.to_dict()

    assert data["status"] == STATUS_OK
    assert data["scan_count"] == 3
    assert data["scan_rate_hz"] == 5.5
    assert data["valid_ratio"] == 0.9


def test_lidar_health_updates_to_ok_from_good_scan():
    health = LidarHealth()

    health.update_from_scan(
        scan_rate_hz=5.5,
        valid_ratio=0.90,
        stale=False,
        hardware_ok=True,
    )

    assert health.status == STATUS_OK
    assert health.hardware_ok
    assert health.scan_ok
    assert not health.stale
    assert not health.needs_attention()


def test_lidar_health_updates_to_stale():
    health = LidarHealth()

    health.update_from_scan(
        scan_rate_hz=0.0,
        valid_ratio=0.0,
        stale=True,
        hardware_ok=True,
    )

    assert health.status == STATUS_STALE
    assert not health.scan_ok
    assert health.needs_attention()


def test_lidar_health_marks_offline():
    health = LidarHealth()
    health.mark_offline("usb disconnected")

    assert health.status == STATUS_OFFLINE
    assert not health.hardware_ok
    assert health.message == "usb disconnected"
    assert health.needs_attention()


def test_lidar_health_latches_and_clears_fault():
    health = LidarHealth()

    health.latch_fault("driver crashed")

    assert health.status == STATUS_ERROR
    assert health.fault_latched
    assert health.fault_reason == "driver crashed"

    health.clear_fault()

    assert not health.fault_latched


def test_lidar_health_to_dict_contains_status_fields():
    health = LidarHealth()
    health.update_from_scan(
        scan_rate_hz=5.5,
        valid_ratio=0.90,
        stale=False,
        hardware_ok=True,
    )

    data = health.to_dict()

    assert data["status"] == STATUS_OK
    assert data["hardware_ok"]
    assert data["scan_ok"]
    assert data["valid_ratio"] == 0.90


def test_quality_status_returns_error_warn_and_ok():
    assert quality_status(0.20)[0] == STATUS_ERROR
    assert quality_status(0.50)[0] == STATUS_WARN
    assert quality_status(0.90)[0] == STATUS_OK


def test_quality_status_clamps_ratios():
    assert quality_status(-1.0)[0] == STATUS_ERROR
    assert quality_status(2.0)[0] == STATUS_OK


def test_make_scan_quality_builds_summary():
    quality = make_scan_quality(
        total_points=10,
        valid_points=8,
        min_range_m=0.5,
        max_range_m=4.0,
        mean_range_m=2.0,
        scan_rate_hz=5.5,
    )

    assert quality.total_points == 10
    assert quality.valid_points == 8
    assert quality.valid_ratio == 0.8
    assert quality.ok


def test_make_scan_quality_clamps_valid_points_to_total():
    quality = make_scan_quality(
        total_points=10,
        valid_points=20,
        min_range_m=0.5,
        max_range_m=4.0,
        mean_range_m=2.0,
        scan_rate_hz=5.5,
    )

    assert quality.total_points == 10
    assert quality.valid_points == 10
    assert quality.valid_ratio == 1.0
    assert quality.ok


def test_make_sector_scan_reports_blocked_when_nearest_range_is_close():
    sector = make_sector_scan(
        name="front",
        angle_min_deg=-45.0,
        angle_max_deg=45.0,
        ranges_m=[0.30, 1.0, 2.0],
        blocked_distance_m=0.50,
    )

    assert sector.name == "front"
    assert sector.valid_points == 3
    assert sector.nearest_range_m == 0.30
    assert sector.blocked
    assert sector.message == "sector blocked"


def test_make_sector_scan_reports_clear_when_nearest_range_is_far():
    sector = make_sector_scan(
        name="front",
        angle_min_deg=-45.0,
        angle_max_deg=45.0,
        ranges_m=[0.80, 1.0, 2.0],
        blocked_distance_m=0.50,
    )

    assert sector.name == "front"
    assert sector.valid_points == 3
    assert sector.nearest_range_m == 0.80
    assert not sector.blocked
    assert sector.clear
    assert sector.message == "sector clear"


def test_make_sector_scan_ignores_invalid_ranges():
    sector = make_sector_scan(
        name="front",
        angle_min_deg=-45.0,
        angle_max_deg=45.0,
        ranges_m=[float("inf"), float("nan"), 0.0, 0.8],
        blocked_distance_m=0.50,
    )

    assert sector.total_points == 4
    assert sector.valid_points == 1
    assert sector.nearest_range_m == 0.8
    assert not sector.blocked
