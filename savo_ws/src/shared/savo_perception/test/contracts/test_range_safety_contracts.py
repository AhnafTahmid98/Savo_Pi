"""Regression contracts for production range safety and lightweight bringup."""

from __future__ import annotations

import math
import re
import time
from pathlib import Path

from savo_perception.models.range_sample import (
    RangeSample,
    RangeSnapshot,
    is_valid_distance,
)
from savo_perception.models.sensor_health import SensorHealth
from savo_perception.safety.range_fusion import RangeFusionConfig, fuse_range_snapshot


PACKAGE = Path(__file__).resolve().parents[2]
SRC = Path(__file__).resolve().parents[4]


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _sample(name: str, distance_m: float = 0.6) -> RangeSample:
    return RangeSample.now(sensor_name=name, distance_m=distance_m, source="test")


def _invalid(name: str, value: float = math.nan) -> RangeSample:
    return RangeSample(
        sensor_name=name,
        distance_m=value,
        stamp_mono_s=time.monotonic(),
        valid=True,
        source="test",
    )


def _stale(name: str, distance_m: float = 0.6) -> RangeSample:
    return RangeSample(
        sensor_name=name,
        distance_m=distance_m,
        stamp_mono_s=time.monotonic() - 2.0,
        valid=True,
        source="test",
    )


def _snapshot() -> RangeSnapshot:
    return RangeSnapshot(
        depth_front=_sample("depth_front", 1.5),
        tof_left=_sample("tof_left"),
        tof_right=_sample("tof_right"),
        ultrasonic_front=_sample("ultrasonic_front", 1.0),
    )


def test_common_quality_reuses_existing_required_range_health() -> None:
    source = _read(PACKAGE / "src/nodes/range_health_node.cpp")

    assert 'overall_ok(health) ? "MINIMUM" : "BELOW_MINIMUM"' in source
    assert '\\"quality_reason\\":\\"required_range_health_only\\"' in source


def test_left_nan_is_a_required_invalid_stop() -> None:
    snapshot = _snapshot()
    snapshot = RangeSnapshot(snapshot.depth_front, _invalid("tof_left"), snapshot.tof_right,
                             snapshot.ultrasonic_front)
    decision = fuse_range_snapshot(snapshot).decision
    assert decision.stop_required
    assert decision.reason == "required_sensor_invalid"


def test_right_nan_is_a_required_invalid_stop() -> None:
    snapshot = _snapshot()
    snapshot = RangeSnapshot(snapshot.depth_front, snapshot.tof_left, _invalid("tof_right"),
                             snapshot.ultrasonic_front)
    decision = fuse_range_snapshot(snapshot).decision
    assert decision.stop_required
    assert decision.reason == "required_sensor_invalid"


def test_required_tof_staleness_stops() -> None:
    snapshot = _snapshot()
    snapshot = RangeSnapshot(snapshot.depth_front, _stale("tof_left"), snapshot.tof_right,
                             snapshot.ultrasonic_front)
    decision = fuse_range_snapshot(snapshot).decision
    assert decision.stop_required
    assert decision.reason == "required_sensor_stale"


def test_healthy_tofs_do_not_cause_required_sensor_stop() -> None:
    decision = fuse_range_snapshot(_snapshot()).decision
    assert not decision.stop_required
    assert not decision.reason.startswith("required_sensor_")


def test_disabled_ultrasonic_missing_is_ignored() -> None:
    snapshot = _snapshot()
    snapshot = RangeSnapshot(snapshot.depth_front, snapshot.tof_left, snapshot.tof_right,
                             RangeSample.invalid(sensor_name="ultrasonic_front", error="missing"))
    result = fuse_range_snapshot(snapshot, RangeFusionConfig(use_ultrasonic=False))
    assert not result.decision.stop_required
    assert "ultrasonic_front" not in result.invalid_sensors


def test_disabled_ultrasonic_stale_is_ignored() -> None:
    snapshot = _snapshot()
    snapshot = RangeSnapshot(snapshot.depth_front, snapshot.tof_left, snapshot.tof_right,
                             _stale("ultrasonic_front", 0.01))
    result = fuse_range_snapshot(snapshot, RangeFusionConfig(use_ultrasonic=False))
    assert not result.decision.stop_required
    assert "ultrasonic_front" not in result.stale_sensors


def test_optional_ultrasonic_nan_is_reported_but_does_not_stop() -> None:
    snapshot = _snapshot()
    snapshot = RangeSnapshot(snapshot.depth_front, snapshot.tof_left, snapshot.tof_right,
                             _invalid("ultrasonic_front"))
    result = fuse_range_snapshot(snapshot)
    assert "ultrasonic_front" in result.invalid_sensors
    assert result.decision.reason != "required_sensor_invalid"
    assert not result.decision.stop_required


def test_optional_ultrasonic_stale_is_reported_but_does_not_stop() -> None:
    snapshot = _snapshot()
    snapshot = RangeSnapshot(snapshot.depth_front, snapshot.tof_left, snapshot.tof_right,
                             _stale("ultrasonic_front"))
    result = fuse_range_snapshot(snapshot)
    assert "ultrasonic_front" in result.stale_sensors
    assert result.decision.reason != "required_sensor_stale"
    assert not result.decision.stop_required


def test_enabled_clear_ultrasonic_does_not_stop() -> None:
    assert not fuse_range_snapshot(_snapshot()).decision.stop_required


def test_enabled_close_ultrasonic_stops() -> None:
    snapshot = _snapshot()
    snapshot = RangeSnapshot(snapshot.depth_front, snapshot.tof_left, snapshot.tof_right,
                             _sample("ultrasonic_front", 0.05))
    result = fuse_range_snapshot(snapshot)
    assert result.decision.stop_required
    assert result.decision.reason == "ultrasonic_close_stop"


def test_nan_is_invalid() -> None:
    assert not is_valid_distance(math.nan)


def test_positive_infinity_is_invalid() -> None:
    assert not is_valid_distance(math.inf)


def test_negative_infinity_is_invalid() -> None:
    assert not is_valid_distance(-math.inf)


def test_zero_is_invalid() -> None:
    assert not is_valid_distance(0.0)


def test_negative_is_invalid() -> None:
    assert not is_valid_distance(-0.1)


def test_finite_positive_range_is_valid() -> None:
    assert is_valid_distance(0.5)


def test_health_never_reports_valid_with_null_distance() -> None:
    health = SensorHealth.from_sample(_invalid("tof_left", math.inf), stale_timeout_s=1.0)
    assert not health.valid
    assert not health.ok
    assert health.last_distance_m is None


def test_perception_launch_disables_driver_and_passes_flag_to_consumers() -> None:
    launch = _read(PACKAGE / "launch" / "perception_bringup.launch.py")
    assert "if use_ultrasonic:" in launch
    assert launch.count('{"use_ultrasonic": use_ultrasonic}') == 2


def test_core_and_robot_bringup_propagate_ultrasonic_flag() -> None:
    for path in (
        SRC / "shared" / "savo_bringup" / "launch" / "core_bringup.launch.py",
        SRC / "shared" / "savo_bringup" / "launch" / "robot_bringup.launch.py",
    ):
        launch = _read(path)
        assert "perception_use_ultrasonic" in launch
        assert 'default_value="true"' in launch


def test_autonomous_mapping_propagates_ultrasonic_flag() -> None:
    launch = _read(SRC / "shared" / "savo_bringup" / "launch" /
                   "autonomous_mapping.launch.py")
    assert '"use_ultrasonic": LaunchConfiguration(' in launch
    assert '"perception_use_ultrasonic"' in launch


def test_control_python_status_monitors_default_off() -> None:
    launch = _read(SRC / "core" / "savo_control" / "launch" /
                   "control_bringup.launch.py")
    for name in ("use_control_status", "use_recovery_status"):
        match = re.search(
            rf'DeclareLaunchArgument\(\s*"{name}",\s*default_value="([^"]+)"',
            launch,
        )
        assert match and match.group(1) == "false"


def test_cpp_control_status_producer_remains() -> None:
    source = _read(SRC / "core" / "savo_control" / "src" / "nodes" /
                   "control_mode_manager_node.cpp")
    topics = _read(SRC / "core" / "savo_control" / "include" / "savo_control" /
                   "topic_names.hpp")
    assert "control_status_pub_->publish" in source
    assert 'CONTROL_STATUS = "/savo_control/control_status"' in topics


def test_cpp_mux_and_shaper_status_producers_remain() -> None:
    mux = _read(SRC / "core" / "savo_control" / "src" / "nodes" / "twist_mux_node.cpp")
    shaper = _read(SRC / "core" / "savo_control" / "src" / "nodes" /
                   "cmd_vel_shaper_node.cpp")
    assert "/savo_control/twist_mux/status" in mux
    assert "/savo_control/cmd_vel_shaper/status" in shaper
    assert "status_pub_->publish" in mux
    assert "status_pub_->publish" in shaper


def test_control_startup_mode_remains_stop() -> None:
    launch = _read(SRC / "core" / "savo_control" / "launch" /
                   "control_bringup.launch.py")
    assert re.search(
        r'DeclareLaunchArgument\(\s*"startup_mode",\s*default_value="STOP"',
        launch,
    )


def test_tof_status_is_bounded_throttled_and_keeps_real_errors() -> None:
    node = _read(PACKAGE / "src" / "nodes" / "vl53_mux_node.cpp")
    driver = _read(PACKAGE / "src" / "drivers" / "vl53l1x_driver.cpp")
    assert "KeepLast(1)" in node
    assert "transient_local" in node
    assert "state_changed" in node and "periodic_due" in node
    for error in (
        "mux_select_failed",
        "sensor_init_failed",
        "start_ranging_failed",
        "data_ready_check_failed",
        "data_not_ready",
        "get_distance_failed",
        "clear_interrupt_failed",
        "distance_out_of_valid_range",
    ):
        assert error in driver
    assert "worker_stale" in node
    assert "read_exception:" in node


def test_real_tof_topology_is_unchanged() -> None:
    config = _read(PACKAGE / "config" / "profiles" / "core_real_robot_v1.yaml")
    for token in (
        "bus: 1",
        "tca_addr: 0x70",
        "vl53_addr: 0x29",
        "left_channel: 2",
        "right_channel: 3",
    ):
        assert token in config


def test_required_tofs_remain_required_and_ultrasonic_optional() -> None:
    config = _read(PACKAGE / "config" / "core" / "perception_core.yaml")
    assert "required_sensors:\n      - tof_left\n      - tof_right" in config
    assert "optional_sensors:\n      - depth_front\n      - ultrasonic_front" in config
