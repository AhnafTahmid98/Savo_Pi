from pathlib import Path

import pytest

from savo_power import constants as c
from savo_power.ros import topic_contract


yaml = pytest.importorskip("yaml")


LOCKED_TOPICS = {
    "CORE_UPS_TOPIC": "/savo_power/core/ups",
    "EDGE_UPS_TOPIC": "/savo_power/edge/ups",
    "BASE_BATTERY_TOPIC": "/savo_power/base/battery",
    "STATUS_TOPIC": "/savo_power/status",
    "HEALTH_TOPIC": "/savo_power/health",
    "DASHBOARD_TOPIC": "/savo_power/dashboard",
    "DASHBOARD_TEXT_TOPIC": "/savo_power/dashboard_text",
    "SHUTDOWN_REQUEST_TOPIC": "/savo_power/shutdown_request",
}


def topic_values():
    return {
        name: getattr(c, name)
        for name in LOCKED_TOPICS
    }


def load_global_params(path):
    data = yaml.safe_load(Path(path).read_text())

    assert isinstance(data, dict)
    assert "/**" in data
    assert "ros__parameters" in data["/**"]

    return data["/**"]["ros__parameters"]



def contract_topic_name(contract):
    for attr in ("name", "topic", "topic_name"):
        value = getattr(contract, attr, None)

        if isinstance(value, str) and value.startswith("/"):
            return value

    if hasattr(contract, "to_dict"):
        data = contract.to_dict()

        for key in ("name", "topic", "topic_name"):
            value = data.get(key)

            if isinstance(value, str) and value.startswith("/"):
                return value

    if isinstance(contract, dict):
        for key in ("name", "topic", "topic_name"):
            value = contract.get(key)

            if isinstance(value, str) and value.startswith("/"):
                return value

    return ""


def test_locked_topic_constants_exist():
    for name in LOCKED_TOPICS:
        assert hasattr(c, name), name


def test_locked_topic_constants_match_expected_names():
    for name, expected in LOCKED_TOPICS.items():
        assert getattr(c, name) == expected


def test_all_topics_are_absolute_savo_power_topics():
    for name, topic in topic_values().items():
        assert topic.startswith("/savo_power/"), name
        assert "//" not in topic, name
        assert topic == topic.strip(), name


def test_topic_names_are_unique():
    values = list(topic_values().values())

    assert len(values) == len(set(values))


def test_sensor_topics_are_separate_from_summary_topics():
    sensor_topics = {
        c.CORE_UPS_TOPIC,
        c.EDGE_UPS_TOPIC,
        c.BASE_BATTERY_TOPIC,
    }
    summary_topics = {
        c.STATUS_TOPIC,
        c.HEALTH_TOPIC,
        c.DASHBOARD_TOPIC,
        c.DASHBOARD_TEXT_TOPIC,
        c.SHUTDOWN_REQUEST_TOPIC,
    }

    assert sensor_topics.isdisjoint(summary_topics)


def test_shutdown_request_topic_is_not_a_sensor_or_dashboard_topic():
    assert c.SHUTDOWN_REQUEST_TOPIC not in {
        c.CORE_UPS_TOPIC,
        c.EDGE_UPS_TOPIC,
        c.BASE_BATTERY_TOPIC,
        c.STATUS_TOPIC,
        c.HEALTH_TOPIC,
        c.DASHBOARD_TOPIC,
        c.DASHBOARD_TEXT_TOPIC,
    }


def test_topics_yaml_matches_constants():
    params = load_global_params("config/topics.yaml")

    expected_yaml_keys = {
        "core_ups_topic": c.CORE_UPS_TOPIC,
        "edge_ups_topic": c.EDGE_UPS_TOPIC,
        "base_battery_topic": c.BASE_BATTERY_TOPIC,
        "status_topic": c.STATUS_TOPIC,
        "health_topic": c.HEALTH_TOPIC,
        "dashboard_topic": c.DASHBOARD_TOPIC,
        "dashboard_text_topic": c.DASHBOARD_TEXT_TOPIC,
        "shutdown_request_topic": c.SHUTDOWN_REQUEST_TOPIC,
    }

    for key, expected in expected_yaml_keys.items():
        assert params[key] == expected


def test_topic_contract_module_exports_expected_helpers():
    for name in [
        "all_topic_contracts",
        "sensor_topic_contracts",
        "status_topic_contracts",
        "topic_names",
        "topic_contract_by_name",
        "topic_contract_for_source",
    ]:
        assert hasattr(topic_contract, name), name


def test_topic_contract_names_include_locked_topics():
    names = set(topic_contract.topic_names())

    for topic in topic_values().values():
        assert topic in names


def test_topic_contract_lookup_returns_each_locked_topic():
    for topic in topic_values().values():
        contract = topic_contract.topic_contract_by_name(topic)

        assert contract is not None
        assert contract_topic_name(contract) == topic


def test_sensor_topic_contracts_match_sensor_topics():
    contracts = topic_contract.sensor_topic_contracts()
    names = {
        contract_topic_name(contract)
        for contract in contracts
    }

    assert c.CORE_UPS_TOPIC in names
    assert c.EDGE_UPS_TOPIC in names
    assert c.BASE_BATTERY_TOPIC in names


def test_status_topic_contracts_include_status_health_dashboard_topics():
    contracts = topic_contract.status_topic_contracts()
    names = {
        contract_topic_name(contract)
        for contract in contracts
    }

    assert c.STATUS_TOPIC in names
    assert c.HEALTH_TOPIC in names
    assert c.DASHBOARD_TOPIC in names
    assert c.DASHBOARD_TEXT_TOPIC in names


def test_topic_names_do_not_overlap_with_motion_control():
    for topic in topic_values().values():
        assert "/cmd_vel" not in topic
        assert "motor" not in topic
        assert "wheel" not in topic


def test_runtime_files_reference_locked_topics_without_typo():
    files = [
        "config/topics.yaml",
        "savo_power/constants.py",
        "savo_power/ros/topic_contract.py",
    ]

    combined = "\n".join(
        Path(path).read_text(errors="ignore")
        for path in files
        if Path(path).is_file()
    )

    for topic in topic_values().values():
        assert topic in combined
