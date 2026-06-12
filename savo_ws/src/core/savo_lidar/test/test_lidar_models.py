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
    assert config.is_dryrun_backend
    assert not config.is_real_backend


def test_lidar_driver_config_accepts_real_backend_with_serial_port():
    config = LidarDriverConfig(
        backend=BACKEND_REAL,
        serial_port="/dev/ttyUSB0",
    )

    config.validate()

    assert config.is_real_backend


def test_lidar_driver_config_rejects_invalid_backend():
    with pytest.raises(ValueError):
        LidarDriverConfig(backend="bad_backend").validate()


def test_lidar_driver_config_rejects_invalid_range_limits():
    with pytest.raises(ValueError):
        LidarDriverConfig(min_range_m=1.0, max_range_m=0.5).validate()


def test_make_driver_config_validates_overrides():
    config = make_driver_config(backend=BACKEND_DRYRUN, frame_id="laser")

    assert config.backend == BACKEND_DRYRUN
    assert config.frame_id == "laser"


def test_lidar_state_marks_scan():
    state = LidarState(node="lidar_driver_node")

    state.mark_scan(
        scan_rate_hz=5.5,
        valid_ratio=0.95,
        last_scan_age_s=0.0,
    )

    assert state.scan_count == 1
    assert state.scan_ok
    assert state.scan_rate_hz == 5.5
    assert state.valid_ratio == 0.95


def test_lidar_state_marks_stale():
    state = LidarState(node="lidar_driver_node")

    state.mark_stale("scan stopped")

    assert not state.scan_ok
    assert state.message == "scan stopped"


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


def test_lidar_health_marks_offline():
    health = LidarHealth()
    health.mark_offline("usb disconnected")

    assert health.status == STATUS_OFFLINE
    assert not health.hardware_ok
    assert health.message == "usb disconnected"


def test_lidar_health_latches_and_clears_fault():
    health = LidarHealth()

    health.latch_fault("driver crashed")

    assert health.status == STATUS_ERROR
    assert health.fault_latched
    assert health.fault_reason == "driver crashed"

    health.clear_fault()

    assert not health.fault_latched


def test_quality_status_returns_error_warn_and_ok():
    assert quality_status(0.20)[0] == STATUS_ERROR
    assert quality_status(0.50)[0] == STATUS_WARN
    assert quality_status(0.90)[0] == STATUS_OK


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