"""Regression contracts for production range safety and lightweight bringup."""

from __future__ import annotations

import copy
import math
import re
import runpy
import time
from pathlib import Path

import pytest
import yaml

from savo_perception.diagnostics.tof_mux_check import build_arg_parser
from savo_perception.drivers.vl53_mux_driver import Vl53MuxConfig
from savo_perception.ros.params import load_vl53_mux_params
from savo_perception.models.range_sample import (
    RangeSample,
    RangeSnapshot,
    is_valid_distance,
)
from savo_perception.models.sensor_health import (
    SensorHealth,
    evaluate_required_range_health,
)
from savo_perception.nodes.range_health_node_py import (
    RangeHealthNodePy,
    fresh_ultrasonic_diagnostic_error,
    update_ultrasonic_diagnostic,
)
from savo_perception.nodes.safety_stop_node_py import SafetyStopNodePy
from savo_perception.ros import params as ros_params
from savo_perception.safety.range_fusion import (
    RangeFusionConfig,
    fuse_range_snapshot,
    slowdown_from_distance,
)


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


def test_degraded_left_nan_is_reported_without_required_sensor_stop() -> None:
    snapshot = _snapshot()
    snapshot = RangeSnapshot(
        snapshot.depth_front,
        _invalid("tof_left"),
        snapshot.tof_right,
        snapshot.ultrasonic_front,
    )
    result = fuse_range_snapshot(
        snapshot,
        RangeFusionConfig(required_sensors=()),
    )

    assert "tof_left" in result.invalid_sensors
    assert result.decision.reason != "required_sensor_invalid"
    assert not result.decision.stop_required


def test_degraded_right_nan_is_reported_without_required_sensor_stop() -> None:
    snapshot = _snapshot()
    snapshot = RangeSnapshot(
        snapshot.depth_front,
        snapshot.tof_left,
        _invalid("tof_right"),
        snapshot.ultrasonic_front,
    )
    result = fuse_range_snapshot(
        snapshot,
        RangeFusionConfig(required_sensors=()),
    )

    assert "tof_right" in result.invalid_sensors
    assert result.decision.reason != "required_sensor_invalid"
    assert not result.decision.stop_required


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


def test_degraded_required_health_ignores_optional_tof_error() -> None:
    health = {
        "tof_left": SensorHealth.from_sample(
            RangeSample.invalid(
                sensor_name="tof_left",
                error="invalid_distance",
            ),
            stale_timeout_s=1.0,
        ),
        "tof_right": SensorHealth.from_sample(
            _sample("tof_right"),
            stale_timeout_s=1.0,
        ),
    }

    result = evaluate_required_range_health(health, required_sensors=[])

    assert health["tof_left"].status == "ERROR"
    assert not health["tof_left"].valid
    assert health["tof_left"].error == "invalid_distance"
    assert result == {
        "ok": True,
        "status": "OK",
        "stale_required_sensors": [],
        "error_required_sensors": [],
    }


def test_production_required_health_fails_on_tof_error() -> None:
    health = {
        "tof_left": SensorHealth.from_sample(
            RangeSample.invalid(
                sensor_name="tof_left",
                error="invalid_distance",
            ),
            stale_timeout_s=1.0,
        ),
        "tof_right": SensorHealth.from_sample(
            _sample("tof_right"),
            stale_timeout_s=1.0,
        ),
    }

    result = evaluate_required_range_health(
        health,
        required_sensors=["tof_left", "tof_right"],
    )

    assert not result["ok"]
    assert result["status"] == "ERROR"
    assert result["error_required_sensors"] == ["tof_left"]


def test_received_invalid_tof_is_error_not_stale_in_python_fallbacks() -> None:
    for node_type in (RangeHealthNodePy, SafetyStopNodePy):
        for invalid_value in (math.nan, math.inf, -math.inf, 0.0, -0.1):
            sample = node_type._sample_from_value(
                "tof_left",
                invalid_value,
                required=True,
            )
            health = SensorHealth.from_sample(sample, stale_timeout_s=1.0)

            assert not sample.valid
            assert sample.distance_m is None
            assert health.status == "ERROR"
            assert not health.stale


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


def test_real_tof_topology_uses_rewired_left_channel() -> None:
    for path in ("core/perception_core.yaml", "core/tof_mux.yaml",
                 "profiles/core_real_robot_v1.yaml"):
        config = yaml.safe_load(_read(PACKAGE / "config" / path))
        for node in ("vl53_mux_node", "vl53_mux_node_py"):
            params = load_vl53_mux_params(config[node]["ros__parameters"])
            assert (params.left_channel, params.right_channel) == (7, 3), (path, node)
            assert (params.bus, params.tca_addr, params.vl53_addr) == (1, 0x70, 0x29)


def test_python_tof_default_paths_use_rewired_left_channel() -> None:
    for params in (Vl53MuxConfig(), load_vl53_mux_params({})):
        assert (params.left_channel, params.right_channel) == (7, 3)
        assert (params.bus, params.tca_addr, params.vl53_addr) == (1, 0x70, 0x29)


def test_tof_diagnostic_defaults_follow_physical_mapping() -> None:
    scan_cli = runpy.run_path(str(PACKAGE / "scripts/tof_mux_scan_cli.py"))
    for parser in (build_arg_parser(), scan_cli["build_arg_parser"]()):
        args = parser.parse_args([])
        assert (args.left_ch, args.right_ch) == (7, 3)
        assert (args.bus, args.tca_addr, args.vl53_addr) == (1, 0x70, 0x29)
    assert scan_cli["expected_label"](7, right_ch=3, left_ch=7) == "LEFT expected"
    config = yaml.safe_load(_read(PACKAGE / "config/diagnostics.yaml"))
    params = load_vl53_mux_params(config["diagnostics"]["tof_mux_check"])
    assert (params.left_channel, params.right_channel) == (7, 3)


def test_real_profile_uses_tuned_slowdown_and_unchanged_stop_thresholds() -> None:
    profile_path = (
        PACKAGE / "config" / "profiles" / "core_real_robot_v1.yaml"
    )
    profile = yaml.safe_load(_read(profile_path))

    for node_name in ("safety_stop_node", "safety_stop_node_py"):
        params = profile[node_name]["ros__parameters"]
        assert math.isclose(params["front_slow_m"], 0.40)
        assert math.isclose(params["side_slow_m"], 0.12)
        assert math.isclose(params["front_stop_m"], 0.25)
        assert math.isclose(params["side_stop_m"], 0.08)
        assert params["front_slow_m"] > params["front_stop_m"]
        assert params["side_slow_m"] > params["side_stop_m"]


def test_autonomous_mapping_selects_production_perception_authority() -> None:
    bringup = PACKAGE.parents[1] / "shared" / "savo_bringup"
    launch = _read(bringup / "launch" / "autonomous_mapping.launch.py")
    profiles = _read(
        bringup / "savo_bringup" / "autonomous_mapping_profiles.py"
    )
    generic = _read(PACKAGE / "config/core/perception_core.yaml")
    generic_safety = _read(PACKAGE / "config/core/range_safety.yaml")

    assert 'perception_config_filename="core_real_robot_v1.yaml"' in profiles
    assert '"autonomous_mapping_profile"' in launch
    assert "resolve_autonomous_mapping_profile(" in launch
    assert '"config_file": LaunchConfiguration("perception_config_file")' in launch
    for source in (generic, generic_safety):
        assert "not the autonomous-mapping threshold authority" in source


def test_tuned_production_slowdown_boundaries() -> None:
    assert math.isclose(
        slowdown_from_distance(0.401, stop_m=0.25, slow_m=0.40),
        1.0,
    )
    assert slowdown_from_distance(0.399, stop_m=0.25, slow_m=0.40) < 1.0
    assert math.isclose(
        slowdown_from_distance(0.25, stop_m=0.25, slow_m=0.40),
        0.0,
    )

    assert math.isclose(
        slowdown_from_distance(0.121, stop_m=0.08, slow_m=0.12),
        1.0,
    )
    assert slowdown_from_distance(0.119, stop_m=0.08, slow_m=0.12) < 1.0
    assert math.isclose(
        slowdown_from_distance(0.08, stop_m=0.08, slow_m=0.12),
        0.0,
    )


def test_side_hard_stop_overrides_front_slowdown() -> None:
    snapshot = _snapshot()
    snapshot = RangeSnapshot(
        _sample("depth_front", 0.30),
        _sample("tof_left", 0.08),
        snapshot.tof_right,
        snapshot.ultrasonic_front,
    )
    config = RangeFusionConfig(
        front_stop_m=0.25,
        front_slow_m=0.40,
        side_stop_m=0.08,
        side_slow_m=0.12,
    )

    decision = fuse_range_snapshot(snapshot, config).decision
    assert decision.stop_required
    assert decision.reason == "side_stop_zone"
    assert math.isclose(decision.slowdown_factor, 0.0)


def test_required_tofs_remain_required_and_ultrasonic_optional() -> None:
    config = _read(PACKAGE / "config" / "core" / "perception_core.yaml")
    assert "required_sensors:\n      - tof_left\n      - tof_right" in config
    assert "optional_sensors:\n      - depth_front\n      - ultrasonic_front" in config


def test_production_and_degraded_profiles_only_change_sensor_optionality() -> None:
    profiles = PACKAGE / "config" / "profiles"
    production = yaml.safe_load(_read(profiles / "core_real_robot_v1.yaml"))
    degraded = yaml.safe_load(
        _read(profiles / "core_lidar_mapping_degraded.yaml")
    )
    policy_nodes = (
        "safety_stop_node",
        "safety_stop_node_py",
        "range_health_node",
        "range_health_node_py",
    )
    optional_sensors = [
        "tof_left",
        "tof_right",
        "depth_front",
        "ultrasonic_front",
    ]

    expected = copy.deepcopy(production)
    for node_name in policy_nodes:
        production_params = production[node_name]["ros__parameters"]
        degraded_params = degraded[node_name]["ros__parameters"]

        assert production_params["required_sensors"] == [
            "tof_left",
            "tof_right",
        ]
        assert degraded_params["required_sensors"] == ["__none__"]
        assert ros_params.normalize_required_sensor_names(
            degraded_params["required_sensors"]
        ) == ()
        assert degraded_params["optional_sensors"] == optional_sensors

        expected[node_name]["ros__parameters"]["required_sensors"] = [
            "__none__"
        ]
        expected[node_name]["ros__parameters"]["optional_sensors"] = (
            optional_sensors
        )

    assert degraded == expected

    driver = degraded["vl53_mux_node"]["ros__parameters"]
    assert driver["tca_addr"] == 0x70
    assert driver["vl53_addr"] == 0x29
    assert driver["left_channel"] == 7
    assert driver["right_channel"] == 3
    assert driver["publish_nan_on_error"] is True
    assert (
        degraded["cmd_vel_safety_gate"]["ros__parameters"][
            "cmd_vel_safe_topic"
        ]
        == "/cmd_vel_safe"
    )


def test_required_sensor_transport_sentinel_rejects_mixed_configuration() -> None:
    with pytest.raises(ValueError, match="must be the only configured value"):
        ros_params.normalize_required_sensor_names(
            ["__none__", "tof_left"]
        )


def test_ultrasonic_driver_causes_reach_range_health_status() -> None:
    driver = _read(PACKAGE / "src/drivers/ultrasonic_reader.cpp")
    producer = _read(PACKAGE / "src/nodes/ultrasonic_node.cpp")
    producer_py = _read(
        PACKAGE / "savo_perception" / "nodes" / "ultrasonic_node_py.py"
    )
    health = _read(PACKAGE / "src/nodes/range_health_node.cpp")
    health_py = _read(
        PACKAGE / "savo_perception" / "nodes" / "range_health_node_py.py"
    )
    topics = _read(PACKAGE / "include/savo_perception/topic_names.hpp")

    for cause in (
        "echo_idle_timeout",
        "echo_start_timeout",
        "echo_end_timeout",
        "echo_read_failed",
        "distance_out_of_valid_range",
    ):
        assert cause in driver
    assert "kUltrasonicStatus" in topics
    assert "publish_status(reading.error)" in producer
    assert "qos_state_string(depth=1)" in producer_py
    assert "on_ultrasonic_status" in health
    assert "qos_state_string(depth=1)" in health_py
    assert "ultrasonic_error_" in health
    assert "ultrasonic_status_receipt_" in health
    assert "current_ultrasonic_error(" in health
    assert '"invalid_distance"' in health


def test_ultrasonic_diagnostic_cause_is_used_only_while_fresh() -> None:
    assert fresh_ultrasonic_diagnostic_error(
        "echo_end_timeout",
        received_mono_s=10.0,
        now_mono_s=10.2,
        stale_timeout_s=0.3,
    ) == "echo_end_timeout"
    assert fresh_ultrasonic_diagnostic_error(
        "echo_end_timeout",
        received_mono_s=10.0,
        now_mono_s=10.31,
        stale_timeout_s=0.3,
    ) == ""
    assert fresh_ultrasonic_diagnostic_error(
        "echo_end_timeout",
        received_mono_s=10.0,
        now_mono_s=9.9,
        stale_timeout_s=0.3,
    ) == ""
    assert fresh_ultrasonic_diagnostic_error(
        "",
        received_mono_s=10.0,
        now_mono_s=10.1,
        stale_timeout_s=0.3,
    ) == ""


def test_ultrasonic_diagnostic_updates_do_not_erase_failure_on_empty_status() -> None:
    assert update_ultrasonic_diagnostic(
        "echo_end_timeout",
        received_mono_s=10.0,
        incoming="",
        now_mono_s=10.1,
    ) == ("echo_end_timeout", 10.0)
    assert update_ultrasonic_diagnostic(
        "echo_end_timeout",
        received_mono_s=10.0,
        incoming="ok",
        now_mono_s=10.1,
    ) == ("", 10.1)


def test_ultrasonic_diagnostic_is_not_attached_to_uncorrelated_range_sample() -> None:
    health = _read(PACKAGE / "src" / "nodes" / "range_health_node.cpp")
    health_py = _read(
        PACKAGE / "savo_perception" / "nodes" / "range_health_node_py.py"
    )

    assert "diagnostic_error" in health
    assert '"diagnostic_error"' in health_py
    assert "ultrasonic_sample.error =" not in health
    assert "replace(\n                ultrasonic_sample" not in health_py


def test_ultrasonic_status_is_part_of_the_topic_contract() -> None:
    topics = yaml.safe_load(_read(PACKAGE / "config" / "topics.yaml"))
    topic_name = "/savo_perception/ultrasonic_status"

    assert topics["topics"]["health"]["ultrasonic_status"] == topic_name
    assert topics["ownership"]["ultrasonic_status"]["topic"] == topic_name

    names = _read(PACKAGE / "savo_perception" / "utils" / "topic_names.py")
    assert "TOPIC_ULTRASONIC_STATUS as _TOPIC_ULTRASONIC_STATUS" in names
    assert "ultrasonic_status: str = ULTRASONIC_STATUS" in names


def test_range_rate_quality_is_package_local_and_preserves_optionality() -> None:
    config = _read(PACKAGE / "config" / "core" / "perception_core.yaml")
    cpp = _read(PACKAGE / "src" / "nodes" / "range_health_node.cpp")
    fallback = _read(
        PACKAGE / "savo_perception" / "nodes" / "range_health_node_py.py"
    )

    for token in (
        "tof_minimum_rate_hz: 5.0",
        "tof_good_rate_hz: 8.0",
        "tof_excellent_rate_hz: 9.0",
        "depth_minimum_rate_hz: 5.0",
        "depth_good_rate_hz: 10.0",
        "depth_excellent_rate_hz: 12.0",
    ):
        assert token in config

    for source in (cpp, fallback):
        assert "BELOW_MINIMUM" in source
        assert "MINIMUM" in source
        assert "GOOD" in source
        assert "EXCELLENT" in source

    assert 'include_depth_in_overall_ok_ &&' in cpp
    assert "include_depth_in_overall_ok" in fallback
    assert 'sensor_name == "ultrasonic_front"' in cpp
    assert 'sensor_name == "ultrasonic_front"' in fallback
