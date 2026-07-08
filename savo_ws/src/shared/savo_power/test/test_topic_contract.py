import dataclasses

import pytest

from savo_power import constants as c
from savo_power.models.power_status import BatterySource
from savo_power.ros import topic_contract as tc


EXPECTED_TOPICS = {
    c.CORE_UPS_TOPIC,
    c.EDGE_UPS_TOPIC,
    c.BASE_BATTERY_TOPIC,
    c.STATUS_TOPIC,
    c.HEALTH_TOPIC,
    c.DASHBOARD_TOPIC,
    c.DASHBOARD_TEXT_TOPIC,
    c.SHUTDOWN_REQUEST_TOPIC,
}


def contract_topic_name(contract):
    if isinstance(contract, dict):
        return contract.get("name") or contract.get("topic") or contract.get("topic_name")

    return (
        getattr(contract, "name", None)
        or getattr(contract, "topic", None)
        or getattr(contract, "topic_name", None)
    )


def contract_names(contracts):
    return {
        contract_topic_name(contract)
        for contract in contracts
    }


def test_topic_contract_model_exists():
    assert dataclasses.is_dataclass(tc.TopicContract)
    assert hasattr(tc, "TopicDirection")
    assert hasattr(tc, "TopicPurpose")


def test_all_topic_contracts_contains_expected_topics():
    contracts = tc.all_topic_contracts()
    names = contract_names(contracts)

    assert EXPECTED_TOPICS.issubset(names)


def test_topic_names_contains_expected_topics():
    names = set(tc.topic_names())

    assert EXPECTED_TOPICS.issubset(names)


@pytest.mark.parametrize(
    "topic_name",
    sorted(EXPECTED_TOPICS),
)
def test_topic_contract_lookup_by_name(topic_name):
    contract = tc.topic_contract_by_name(topic_name)

    assert contract is not None
    assert contract_topic_name(contract) == topic_name


@pytest.mark.parametrize(
    ("source", "expected_topic"),
    [
        (BatterySource.CORE_UPS, c.CORE_UPS_TOPIC),
        (BatterySource.EDGE_UPS, c.EDGE_UPS_TOPIC),
        (BatterySource.BASE_BATTERY, c.BASE_BATTERY_TOPIC),
    ],
)
def test_topic_contract_lookup_by_source(source, expected_topic):
    contract = tc.topic_contract_for_source(source)

    assert contract is not None
    assert contract_topic_name(contract) == expected_topic


def test_sensor_contracts_are_sensor_topics():
    names = contract_names(tc.sensor_topic_contracts())

    assert c.CORE_UPS_TOPIC in names
    assert c.EDGE_UPS_TOPIC in names
    assert c.BASE_BATTERY_TOPIC in names


def test_status_contracts_are_status_outputs():
    names = contract_names(tc.status_topic_contracts())

    assert c.STATUS_TOPIC in names
    assert c.HEALTH_TOPIC in names
    assert c.DASHBOARD_TOPIC in names
    assert c.DASHBOARD_TEXT_TOPIC in names
    assert c.SHUTDOWN_REQUEST_TOPIC in names


@pytest.mark.parametrize(
    ("node_name", "expected_topic"),
    [
        ("core_ups_node", c.CORE_UPS_TOPIC),
        ("edge_ups_node", c.EDGE_UPS_TOPIC),
        ("base_battery_node", c.BASE_BATTERY_TOPIC),
        ("power_aggregator_node", c.STATUS_TOPIC),
        ("power_health_node", c.HEALTH_TOPIC),
        ("power_dashboard_node", c.DASHBOARD_TOPIC),
    ],
)
def test_publish_contracts_for_node(node_name, expected_topic):
    names = contract_names(tc.publish_contracts_for_node(node_name))

    assert expected_topic in names


@pytest.mark.parametrize(
    ("node_name", "expected_topic"),
    [
        ("power_aggregator_node", c.CORE_UPS_TOPIC),
        ("power_aggregator_node", c.EDGE_UPS_TOPIC),
        ("power_aggregator_node", c.BASE_BATTERY_TOPIC),
        ("power_health_node", c.STATUS_TOPIC),
        ("power_dashboard_node", c.STATUS_TOPIC),
        ("power_dashboard_node", c.HEALTH_TOPIC),
    ],
)
def test_subscribe_contracts_for_node(node_name, expected_topic):
    names = contract_names(tc.subscribe_contracts_for_node(node_name))

    assert expected_topic in names


def test_contracts_for_node_combines_publish_and_subscribe_topics():
    names = contract_names(tc.contracts_for_node("power_dashboard_node"))

    assert c.STATUS_TOPIC in names
    assert c.HEALTH_TOPIC in names
    assert c.DASHBOARD_TOPIC in names
    assert c.DASHBOARD_TEXT_TOPIC in names


def test_format_topic_contracts_contains_expected_topic_text():
    text = tc.format_topic_contracts(tc.all_topic_contracts())

    assert c.CORE_UPS_TOPIC in text
    assert c.EDGE_UPS_TOPIC in text
    assert c.BASE_BATTERY_TOPIC in text
    assert c.STATUS_TOPIC in text
    assert c.HEALTH_TOPIC in text


def test_topic_contracts_to_dict_contains_expected_topics():
    data = tc.topic_contracts_to_dict(tc.all_topic_contracts())
    text = str(data)

    for topic_name in EXPECTED_TOPICS:
        assert topic_name in text
