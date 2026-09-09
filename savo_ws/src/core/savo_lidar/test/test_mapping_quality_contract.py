"""Regression contracts for the hardware-validated mapping LiDAR quality floor."""

from pathlib import Path

import yaml

from savo_lidar.constants import STATUS_ERROR, STATUS_WARN
from savo_lidar.models import lidar_rate_quality, quality_status
from savo_lidar.safety import LidarHealthPolicy


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = PACKAGE_ROOT / "config/profiles/mapping_rplidar_a1.yaml"
HARD_ERROR_RATIO = 0.30
MAPPING_WARN_RATIO = 0.70
MEASURED_GOOD_A1_RATIO = 0.3138888889
MEASURED_GOOD_A1_RATE_HZ = 6.905


def _mapping_profile() -> dict:
    return yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))


def _mapping_policy() -> LidarHealthPolicy:
    return LidarHealthPolicy(
        min_valid_ratio_warn=MAPPING_WARN_RATIO,
        min_valid_ratio_error=HARD_ERROR_RATIO,
        min_scan_rate_hz=3.0,
    )


def test_mapping_profile_separates_warning_from_hard_error_floor() -> None:
    """Mapping remains warning-strict without inventing a harder failure floor."""
    profile = _mapping_profile()
    filter_params = profile["lidar_filter_node"]["ros__parameters"]
    health_params = profile["lidar_health_node"]["ros__parameters"]

    assert filter_params["warn_valid_ratio"] == MAPPING_WARN_RATIO
    assert filter_params["error_valid_ratio"] == HARD_ERROR_RATIO
    assert health_params["min_valid_ratio_warn"] == MAPPING_WARN_RATIO
    assert health_params["min_valid_ratio_error"] == HARD_ERROR_RATIO
    assert health_params["min_scan_rate_hz"] == 3.0
    assert health_params["good_scan_rate_hz"] == 5.0
    assert health_params["excellent_scan_rate_hz"] == 6.5


def test_mapping_rate_quality_boundaries_are_independent_of_valid_ratio() -> None:
    assert lidar_rate_quality(2.999) == "BELOW_MINIMUM"
    assert lidar_rate_quality(3.0) == "MINIMUM"
    assert lidar_rate_quality(4.999) == "MINIMUM"
    assert lidar_rate_quality(5.0) == "GOOD"
    assert lidar_rate_quality(6.499) == "GOOD"
    assert lidar_rate_quality(6.5) == "EXCELLENT"
    assert lidar_rate_quality(MEASURED_GOOD_A1_RATE_HZ) == "EXCELLENT"


def test_measured_good_a1_ratio_is_operational_warning() -> None:
    """The measured 0.3139 ratio remains visible as WARN but is not an error."""
    status, _message = quality_status(
        MEASURED_GOOD_A1_RATIO,
        warn_ratio=MAPPING_WARN_RATIO,
        error_ratio=HARD_ERROR_RATIO,
    )
    decision = _mapping_policy().evaluate(
        hardware_ok=True,
        driver_running=True,
        stale=False,
        scan_rate_hz=MEASURED_GOOD_A1_RATE_HZ,
        valid_ratio=MEASURED_GOOD_A1_RATIO,
    )

    assert status == STATUS_WARN
    assert decision.status == STATUS_WARN
    assert decision.hardware_ok
    assert decision.scan_ok


def test_validated_error_floor_boundary_is_inclusive_operational_warning() -> None:
    """The exact 0.30 floor is operational; only values below it are errors."""
    at_floor = _mapping_policy().evaluate(
        hardware_ok=True,
        driver_running=True,
        stale=False,
        scan_rate_hz=MEASURED_GOOD_A1_RATE_HZ,
        valid_ratio=HARD_ERROR_RATIO,
    )
    below_floor = _mapping_policy().evaluate(
        hardware_ok=True,
        driver_running=True,
        stale=False,
        scan_rate_hz=MEASURED_GOOD_A1_RATE_HZ,
        valid_ratio=HARD_ERROR_RATIO - 0.000001,
    )

    assert at_floor.status == STATUS_WARN
    assert at_floor.scan_ok
    assert below_floor.status == STATUS_ERROR
    assert not below_floor.scan_ok


def test_hardware_and_freshness_failures_remain_fail_closed() -> None:
    """The ratio correction cannot mask offline, stopped, or stale LiDAR."""
    common = {
        "scan_rate_hz": MEASURED_GOOD_A1_RATE_HZ,
        "valid_ratio": MEASURED_GOOD_A1_RATIO,
    }
    decisions = (
        _mapping_policy().evaluate(
            hardware_ok=False, driver_running=True, stale=False, **common
        ),
        _mapping_policy().evaluate(
            hardware_ok=True, driver_running=False, stale=False, **common
        ),
        _mapping_policy().evaluate(
            hardware_ok=True, driver_running=True, stale=True, **common
        ),
    )

    assert all(not decision.scan_ok for decision in decisions)
