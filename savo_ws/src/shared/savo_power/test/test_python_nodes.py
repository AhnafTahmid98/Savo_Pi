import json

from savo_power import constants as c
from savo_power.models.power_status import BatterySource, PowerState
from savo_power.nodes import (
    kit_battery_node_py,
    power_aggregator_node_py,
    power_dashboard_node_py,
    power_health_node_py,
    ups_hat_node_py,
)


class FakeString:
    def __init__(self, data):
        self.data = data


def test_python_node_modules_import_without_ros_spin():
    assert isinstance(ups_hat_node_py.RCLPY_AVAILABLE, bool)
    assert isinstance(kit_battery_node_py.RCLPY_AVAILABLE, bool)
    assert isinstance(power_aggregator_node_py.RCLPY_AVAILABLE, bool)
    assert isinstance(power_health_node_py.RCLPY_AVAILABLE, bool)
    assert isinstance(power_dashboard_node_py.RCLPY_AVAILABLE, bool)


def test_ups_python_node_names_and_topics():
    assert ups_hat_node_py.source_to_python_node_name("core_ups") == "core_ups_node_py"
    assert ups_hat_node_py.source_to_python_node_name("edge_ups") == "edge_ups_node_py"

    assert ups_hat_node_py.source_to_topic("core_ups") == c.CORE_UPS_TOPIC
    assert ups_hat_node_py.source_to_topic("edge_ups") == c.EDGE_UPS_TOPIC

    assert ups_hat_node_py.validate_ups_source("core_ups") == BatterySource.CORE_UPS
    assert ups_hat_node_py.validate_ups_source("edge_ups") == BatterySource.EDGE_UPS


def test_kit_battery_python_node_name_and_topic():
    assert kit_battery_node_py.base_battery_python_node_name() == "base_battery_node_py"
    assert kit_battery_node_py.base_battery_topic() == c.BASE_BATTERY_TOPIC


def test_status_health_dashboard_node_names_and_topics():
    assert power_aggregator_node_py.aggregator_python_node_name() == "power_aggregator_node_py"
    assert power_aggregator_node_py.status_topic() == c.STATUS_TOPIC

    assert power_health_node_py.health_python_node_name() == "power_health_node_py"
    assert power_health_node_py.health_topic() == c.HEALTH_TOPIC
    assert power_health_node_py.shutdown_request_topic() == c.SHUTDOWN_REQUEST_TOPIC

    assert power_dashboard_node_py.dashboard_python_node_name() == "power_dashboard_node_py"
    assert power_dashboard_node_py.dashboard_topic() == c.DASHBOARD_TOPIC
    assert power_dashboard_node_py.dashboard_text_topic() == c.DASHBOARD_TEXT_TOPIC


def test_python_node_timer_period_helpers():
    assert ups_hat_node_py.create_timer_period_s(2.0) == 0.5
    assert kit_battery_node_py.create_timer_period_s(2.0) == 0.5
    assert power_aggregator_node_py.create_timer_period_s(2.0) == 0.5
    assert power_health_node_py.create_timer_period_s(2.0) == 0.5
    assert power_dashboard_node_py.create_timer_period_s(2.0) == 0.5


def test_aggregator_parse_reading_payload():
    payload = {
        "source": "core_ups",
        "state": "ok",
        "voltage_v": 4.1,
        "capacity_pct": 85.0,
    }

    parsed = power_aggregator_node_py.parse_reading_payload(
        FakeString(json.dumps(payload))
    )

    assert parsed["source"] == "core_ups"
    assert parsed["state"] == "ok"
    assert parsed["voltage_v"] == 4.1


def test_aggregator_source_and_state_helpers():
    payload = {
        "source": "base_battery",
        "state": "low",
    }

    assert (
        power_aggregator_node_py.source_from_reading_dict(payload)
        == BatterySource.BASE_BATTERY
    )
    assert power_aggregator_node_py.state_from_reading_dict(payload) == PowerState.LOW


def test_aggregator_memory_builds_status_summary():
    memory = power_aggregator_node_py.PowerAggregatorMemory()

    readings = {
        "core_ups": {
            "source": "core_ups",
            "state": "ok",
            "voltage_v": 4.1,
            "capacity_pct": 85.0,
        },
        "edge_ups": {
            "source": "edge_ups",
            "state": "ok",
            "voltage_v": 4.08,
            "capacity_pct": 82.0,
        },
        "base_battery": {
            "source": "base_battery",
            "state": "ok",
            "voltage_v": 8.1,
            "soc_pct": 80.0,
        },
    }

    for source, reading in readings.items():
        memory.update(source, reading, now_s=10.0)

    summary = power_aggregator_node_py.aggregate_memory(
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



def test_health_parse_and_evaluate_status_payload():
    status = {
        "overall_state": "ok",
        "ok": True,
        "sources": [],
    }

    parsed = power_health_node_py.parse_status_payload(
        FakeString(json.dumps(status))
    )

    assert parsed["overall_state"] == "ok"

    health = power_health_node_py.evaluate_status_dict(
        parsed,
        automatic_shutdown_enabled=False,
    )

    assert health.ok is True
    assert health.shutdown_requested is False


def test_health_never_requests_shutdown_when_disabled():
    status = {
        "overall_state": "critical",
        "ok": False,
        "sources": [],
    }

    health = power_health_node_py.evaluate_status_dict(
        status,
        automatic_shutdown_enabled=False,
    )

    assert health.shutdown_requested is False


def test_dashboard_parse_json_payload():
    payload = {
        "source": "core_ups",
        "state": "ok",
        "voltage_v": 4.1,
        "capacity_pct": 85.0,
    }

    parsed = power_dashboard_node_py.parse_json_payload(
        FakeString(json.dumps(payload))
    )

    assert parsed["source"] == "core_ups"
    assert parsed["state"] == "ok"


def test_dashboard_memory_builds_snapshot():
    memory = power_dashboard_node_py.PowerDashboardMemory()

    memory.update_reading(
        "core_ups",
        FakeString(
            json.dumps(
                {
                    "source": "core_ups",
                    "state": "ok",
                    "voltage_v": 4.1,
                    "capacity_pct": 85.0,
                }
            )
        ),
        now_s=10.0,
    )
    memory.update_status(
        FakeString(
            json.dumps(
                {
                    "overall_state": "ok",
                    "ok": True,
                }
            )
        ),
        now_s=11.0,
    )
    memory.update_health(
        FakeString(
            json.dumps(
                {
                    "level": "OK",
                    "state": "ok",
                    "ok": True,
                    "shutdown_requested": False,
                }
            )
        ),
        now_s=12.0,
    )

    snapshot = power_dashboard_node_py.build_dashboard_snapshot(
        memory,
        now_s=15.0,
    )

    assert snapshot.readings["core_ups"]["state"] == "ok"
    assert snapshot.status["overall_state"] == "ok"
    assert snapshot.health["level"] == "OK"
    assert "Robot Savo power dashboard" in snapshot.text
