import json
from pathlib import Path
from dataclasses import is_dataclass

from savo_power.models.power_status import BatterySource, PowerState
from savo_power.nodes import power_aggregator_node_py as aggregator


class FakeString:
    def __init__(self, data):
        self.data = data


def make_ok_reading(source, voltage=4.1, capacity=80.0):
    return {
        "source": source,
        "state": "ok",
        "voltage_v": voltage,
        "capacity_pct": capacity,
    }


def make_full_ok_memory(now_s=10.0):
    memory = aggregator.PowerAggregatorMemory()

    memory.update("core_ups", make_ok_reading("core_ups", 4.10, 85.0), now_s=now_s)
    memory.update("edge_ups", make_ok_reading("edge_ups", 4.08, 82.0), now_s=now_s)
    memory.update(
        "base_battery",
        {
            "source": "base_battery",
            "state": "ok",
            "voltage_v": 8.10,
            "soc_pct": 80.0,
        },
        now_s=now_s,
    )

    return memory


def test_power_aggregator_module_exports_core_symbols():
    for name in [
        "PowerAggregatorMemory",
        "PowerAggregatorNodeState",
        "parse_reading_payload",
        "source_from_reading_dict",
        "state_from_reading_dict",
        "expected_sources_from_params",
        "build_source_statuses",
        "compute_overall_state",
        "aggregate_memory",
        "status_topic",
        "aggregator_python_node_name",
    ]:
        assert hasattr(aggregator, name), name


def test_power_aggregator_memory_is_dataclass():
    assert is_dataclass(aggregator.PowerAggregatorMemory)

    memory = aggregator.PowerAggregatorMemory()

    assert memory.parse_error_count == 0
    assert memory.last_parse_error == ""
    assert memory.to_dict()["parse_error_count"] == 0


def test_parse_reading_payload_accepts_dict_json_and_string_message():
    reading = make_ok_reading("core_ups")

    parsed_from_dict = aggregator.parse_reading_payload(reading)
    parsed_from_json = aggregator.parse_reading_payload(json.dumps(reading))
    parsed_from_msg = aggregator.parse_reading_payload(FakeString(json.dumps(reading)))

    assert parsed_from_dict["source"] == "core_ups"
    assert parsed_from_json["source"] == "core_ups"
    assert parsed_from_msg["source"] == "core_ups"


def test_source_and_state_helpers_normalize_values():
    reading = make_ok_reading("edge_ups")

    assert aggregator.source_from_reading_dict(reading) == BatterySource.EDGE_UPS
    assert aggregator.state_from_reading_dict(reading) == PowerState.OK


def test_memory_update_stores_reading_and_timestamp():
    memory = aggregator.PowerAggregatorMemory()

    source = memory.update(
        "core_ups",
        make_ok_reading("core_ups"),
        now_s=12.0,
    )

    assert source == BatterySource.CORE_UPS
    assert memory.source_seen(BatterySource.CORE_UPS) is True
    assert memory.reading_for_source(BatterySource.CORE_UPS)["source"] == "core_ups"
    assert memory.source_age_s(BatterySource.CORE_UPS, now_s=14.0) == 2.0


def test_memory_update_from_payload_uses_payload_source():
    memory = aggregator.PowerAggregatorMemory()

    payload = FakeString(json.dumps(make_ok_reading("edge_ups")))

    source = memory.update_from_payload(payload, now_s=20.0)

    assert source == BatterySource.EDGE_UPS
    assert memory.source_seen(BatterySource.EDGE_UPS) is True
    assert memory.reading_for_source(BatterySource.EDGE_UPS)["source"] == "edge_ups"


def test_memory_parse_error_tracking():
    memory = aggregator.PowerAggregatorMemory()

    memory.mark_parse_error("bad payload")

    assert memory.parse_error_count == 1
    assert "bad payload" in memory.last_parse_error


def test_build_source_statuses_marks_all_seen_sources_ok():
    memory = make_full_ok_memory(now_s=10.0)

    statuses = aggregator.build_source_statuses(
        memory,
        expected_sources=(
            BatterySource.CORE_UPS,
            BatterySource.EDGE_UPS,
            BatterySource.BASE_BATTERY,
        ),
        stale_timeout_s=5.0,
        now_s=12.0,
    )

    assert len(statuses) == 3

    for status in statuses:
        assert status.expected is True
        assert status.seen is True
        assert status.state == PowerState.OK


def test_compute_overall_state_returns_ok_for_all_ok_sources():
    memory = make_full_ok_memory(now_s=10.0)

    statuses = aggregator.build_source_statuses(
        memory,
        expected_sources=(
            BatterySource.CORE_UPS,
            BatterySource.EDGE_UPS,
            BatterySource.BASE_BATTERY,
        ),
        stale_timeout_s=5.0,
        now_s=12.0,
    )

    assert aggregator.compute_overall_state(statuses) == PowerState.OK


def test_aggregate_memory_builds_ok_summary_for_full_valid_memory():
    memory = make_full_ok_memory(now_s=10.0)

    summary = aggregator.aggregate_memory(
        memory,
        expected_sources=(
            BatterySource.CORE_UPS,
            BatterySource.EDGE_UPS,
            BatterySource.BASE_BATTERY,
        ),
        stale_timeout_s=5.0,
        now_s=12.0,
    )

    state = getattr(summary, "overall_state", getattr(summary, "state", None))

    assert state == PowerState.OK
    assert aggregator.summary_status_ok(summary) is True
    assert len(aggregator.summary_source_statuses(summary)) == 3


def test_missing_expected_source_is_visible_in_statuses():
    memory = aggregator.PowerAggregatorMemory()
    memory.update("core_ups", make_ok_reading("core_ups"), now_s=10.0)

    statuses = aggregator.build_source_statuses(
        memory,
        expected_sources=(
            BatterySource.CORE_UPS,
            BatterySource.EDGE_UPS,
            BatterySource.BASE_BATTERY,
        ),
        stale_timeout_s=5.0,
        now_s=12.0,
    )

    by_source = {
        status.source: status
        for status in statuses
    }

    assert by_source[BatterySource.CORE_UPS].seen is True
    assert by_source[BatterySource.EDGE_UPS].seen is False
    assert by_source[BatterySource.BASE_BATTERY].seen is False


def test_aggregator_node_names_and_topics_are_locked():
    assert aggregator.aggregator_python_node_name() == "power_aggregator_node_py"
    assert aggregator.status_topic() == "/savo_power/status"


def test_power_aggregator_does_not_enable_shutdown_or_motor_control():
    files = [
        "savo_power/nodes/power_aggregator_node_py.py",
        "src/aggregation/power_aggregator.cpp",
        "include/savo_power/power_aggregator.hpp",
        "src/nodes/power_aggregator_node.cpp",
    ]

    combined = "\n".join(
        Path(path).read_text(errors="ignore")
        for path in files
        if Path(path).is_file()
    ).lower()

    for forbidden in [
        "automatic_shutdown_enabled: true",
        "automatic_shutdown_enabled=true",
        "shutdown:=true",
        "/cmd_vel",
        "motor_pwm",
        "wheel_speed",
    ]:
        assert forbidden not in combined
