"""ROS topic contract helpers for Robot Savo power monitoring."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Iterable

from savo_power import constants as c
from savo_power.models.power_status import (
    BatterySource,
    normalize_battery_source,
)


class TopicDirection(str, Enum):
    """Topic direction from the point of view of a node."""

    PUBLISH = "publish"
    SUBSCRIBE = "subscribe"
    BOTH = "both"


class TopicPurpose(str, Enum):
    """Stable purpose tags for power topics."""

    SENSOR_READING = "sensor_reading"
    AGGREGATED_STATUS = "aggregated_status"
    HEALTH = "health"
    DASHBOARD = "dashboard"
    SHUTDOWN_REQUEST = "shutdown_request"
    DEBUG = "debug"


@dataclass(frozen=True)
class TopicContract:
    """One ROS topic contract entry."""

    topic: str
    message_type: str
    purpose: TopicPurpose
    direction: TopicDirection
    qos_profile: str
    description: str = ""

    def to_dict(self) -> dict[str, object]:
        data = asdict(self)
        data["purpose"] = self.purpose.value
        data["direction"] = self.direction.value
        return data

    @property
    def package_name(self) -> str:
        parts = self.message_type.split("/")
        return parts[0] if parts else ""

    @property
    def type_name(self) -> str:
        parts = self.message_type.split("/")
        return parts[-1] if parts else ""

    def format_line(self) -> str:
        suffix = f" — {self.description}" if self.description else ""
        return (
            f"{self.topic} [{self.message_type}] "
            f"purpose={self.purpose.value} "
            f"direction={self.direction.value} "
            f"qos={self.qos_profile}"
            f"{suffix}"
        )


STD_STRING = "std_msgs/msg/String"
STD_BOOL = "std_msgs/msg/Bool"


CORE_UPS_CONTRACT = TopicContract(
    topic=c.CORE_UPS_TOPIC,
    message_type=STD_STRING,
    purpose=TopicPurpose.SENSOR_READING,
    direction=TopicDirection.PUBLISH,
    qos_profile="power_sensor",
    description="Core Pi UPS HAT reading.",
)

EDGE_UPS_CONTRACT = TopicContract(
    topic=c.EDGE_UPS_TOPIC,
    message_type=STD_STRING,
    purpose=TopicPurpose.SENSOR_READING,
    direction=TopicDirection.PUBLISH,
    qos_profile="power_sensor",
    description="Edge Pi UPS HAT reading.",
)

BASE_BATTERY_CONTRACT = TopicContract(
    topic=c.BASE_BATTERY_TOPIC,
    message_type=STD_STRING,
    purpose=TopicPurpose.SENSOR_READING,
    direction=TopicDirection.PUBLISH,
    qos_profile="power_sensor",
    description="Freenove/base battery ADS7830 reading.",
)

STATUS_CONTRACT = TopicContract(
    topic=c.STATUS_TOPIC,
    message_type=STD_STRING,
    purpose=TopicPurpose.AGGREGATED_STATUS,
    direction=TopicDirection.PUBLISH,
    qos_profile="power_status",
    description="Aggregated Robot Savo power status.",
)

HEALTH_CONTRACT = TopicContract(
    topic=c.HEALTH_TOPIC,
    message_type=STD_STRING,
    purpose=TopicPurpose.HEALTH,
    direction=TopicDirection.PUBLISH,
    qos_profile="power_health",
    description="Robot Savo power health result.",
)

DASHBOARD_CONTRACT = TopicContract(
    topic=c.DASHBOARD_TOPIC,
    message_type=STD_STRING,
    purpose=TopicPurpose.DASHBOARD,
    direction=TopicDirection.PUBLISH,
    qos_profile="power_dashboard",
    description="Machine-readable dashboard power output.",
)

DASHBOARD_TEXT_CONTRACT = TopicContract(
    topic=c.DASHBOARD_TEXT_TOPIC,
    message_type=STD_STRING,
    purpose=TopicPurpose.DASHBOARD,
    direction=TopicDirection.PUBLISH,
    qos_profile="power_dashboard",
    description="Human-readable dashboard power output.",
)

SHUTDOWN_REQUEST_CONTRACT = TopicContract(
    topic=c.SHUTDOWN_REQUEST_TOPIC,
    message_type=STD_BOOL,
    purpose=TopicPurpose.SHUTDOWN_REQUEST,
    direction=TopicDirection.PUBLISH,
    qos_profile="latched_status",
    description="Optional shutdown request. Disabled by default policy.",
)


POWER_SENSOR_CONTRACTS = (
    CORE_UPS_CONTRACT,
    EDGE_UPS_CONTRACT,
    BASE_BATTERY_CONTRACT,
)

POWER_STATUS_CONTRACTS = (
    STATUS_CONTRACT,
    HEALTH_CONTRACT,
    DASHBOARD_CONTRACT,
    DASHBOARD_TEXT_CONTRACT,
    SHUTDOWN_REQUEST_CONTRACT,
)

ALL_TOPIC_CONTRACTS = POWER_SENSOR_CONTRACTS + POWER_STATUS_CONTRACTS


NODE_PUBLISH_CONTRACTS: dict[str, tuple[TopicContract, ...]] = {
    c.CORE_UPS_NODE_NAME: (CORE_UPS_CONTRACT,),
    c.EDGE_UPS_NODE_NAME: (EDGE_UPS_CONTRACT,),
    c.BASE_BATTERY_NODE_NAME: (BASE_BATTERY_CONTRACT,),
    c.POWER_AGGREGATOR_NODE_NAME: (STATUS_CONTRACT,),
    c.POWER_HEALTH_NODE_NAME: (HEALTH_CONTRACT, SHUTDOWN_REQUEST_CONTRACT),
    c.POWER_DASHBOARD_NODE_NAME: (DASHBOARD_CONTRACT, DASHBOARD_TEXT_CONTRACT),
}


NODE_SUBSCRIBE_CONTRACTS: dict[str, tuple[TopicContract, ...]] = {
    c.POWER_AGGREGATOR_NODE_NAME: POWER_SENSOR_CONTRACTS,
    c.POWER_HEALTH_NODE_NAME: (STATUS_CONTRACT,),
    c.POWER_DASHBOARD_NODE_NAME: (
        CORE_UPS_CONTRACT,
        EDGE_UPS_CONTRACT,
        BASE_BATTERY_CONTRACT,
        STATUS_CONTRACT,
        HEALTH_CONTRACT,
    ),
}


def all_topic_contracts() -> tuple[TopicContract, ...]:
    """Return all power topic contracts."""

    return ALL_TOPIC_CONTRACTS


def sensor_topic_contracts() -> tuple[TopicContract, ...]:
    """Return direct sensor reading topic contracts."""

    return POWER_SENSOR_CONTRACTS


def status_topic_contracts() -> tuple[TopicContract, ...]:
    """Return status/health/dashboard topic contracts."""

    return POWER_STATUS_CONTRACTS


def topic_names(
    contracts: Iterable[TopicContract] | None = None,
) -> tuple[str, ...]:
    """Return topic names from contracts."""

    selected = tuple(contracts) if contracts is not None else ALL_TOPIC_CONTRACTS
    return tuple(contract.topic for contract in selected)


def topic_contract_by_name(topic: str) -> TopicContract | None:
    """Return contract for a topic name."""

    normalized = str(topic).strip()

    for contract in ALL_TOPIC_CONTRACTS:
        if contract.topic == normalized:
            return contract

    return None


def topic_contract_for_source(
    source: str | BatterySource,
) -> TopicContract:
    """Return sensor topic contract for a battery source."""

    normalized = normalize_battery_source(source)

    if normalized == BatterySource.CORE_UPS:
        return CORE_UPS_CONTRACT

    if normalized == BatterySource.EDGE_UPS:
        return EDGE_UPS_CONTRACT

    if normalized == BatterySource.BASE_BATTERY:
        return BASE_BATTERY_CONTRACT

    raise ValueError(f"No power topic contract for source: {source}")


def publish_contracts_for_node(node_name: str) -> tuple[TopicContract, ...]:
    """Return contracts published by a node."""

    return NODE_PUBLISH_CONTRACTS.get(str(node_name), ())


def subscribe_contracts_for_node(node_name: str) -> tuple[TopicContract, ...]:
    """Return contracts subscribed by a node."""

    return NODE_SUBSCRIBE_CONTRACTS.get(str(node_name), ())


def contracts_for_node(node_name: str) -> tuple[TopicContract, ...]:
    """Return all publish/subscribe contracts for a node."""

    publish = publish_contracts_for_node(node_name)
    subscribe = subscribe_contracts_for_node(node_name)

    merged: list[TopicContract] = []
    seen: set[str] = set()

    for contract in publish + subscribe:
        if contract.topic in seen:
            continue

        merged.append(contract)
        seen.add(contract.topic)

    return tuple(merged)


def validate_topic_name(topic: str) -> bool:
    """Return True when topic has a ROS-like absolute topic form."""

    value = str(topic).strip()

    if not value.startswith("/"):
        return False

    if "//" in value:
        return False

    if len(value) <= 1:
        return False

    return all(part for part in value.split("/")[1:])


def validate_contract(contract: TopicContract) -> tuple[bool, str]:
    """Validate one topic contract."""

    if not validate_topic_name(contract.topic):
        return False, f"invalid topic name: {contract.topic}"

    if "/msg/" not in contract.message_type:
        return False, f"invalid message type: {contract.message_type}"

    if not contract.qos_profile:
        return False, f"missing qos profile for {contract.topic}"

    return True, "ok"


def validate_all_contracts(
    contracts: Iterable[TopicContract] | None = None,
) -> tuple[bool, tuple[str, ...]]:
    """Validate all topic contracts."""

    selected = tuple(contracts) if contracts is not None else ALL_TOPIC_CONTRACTS
    errors: list[str] = []

    seen: set[str] = set()

    for contract in selected:
        ok, reason = validate_contract(contract)

        if not ok:
            errors.append(reason)

        if contract.topic in seen:
            errors.append(f"duplicate topic contract: {contract.topic}")

        seen.add(contract.topic)

    return len(errors) == 0, tuple(errors)


def format_topic_contracts(
    contracts: Iterable[TopicContract] | None = None,
) -> str:
    """Format topic contracts as readable text."""

    selected = tuple(contracts) if contracts is not None else ALL_TOPIC_CONTRACTS
    lines = ["Robot Savo power topic contract:"]

    for contract in selected:
        lines.append(contract.format_line())

    ok, errors = validate_all_contracts(selected)

    if ok:
        lines.append("Result: PASS")
    else:
        lines.append("Result: FAIL")
        lines.extend(f"- {error}" for error in errors)

    return "\n".join(lines)


def topic_contracts_to_dict(
    contracts: Iterable[TopicContract] | None = None,
) -> list[dict[str, object]]:
    """Convert contracts to dictionaries."""

    selected = tuple(contracts) if contracts is not None else ALL_TOPIC_CONTRACTS
    return [contract.to_dict() for contract in selected]


__all__ = [
    "ALL_TOPIC_CONTRACTS",
    "BASE_BATTERY_CONTRACT",
    "CORE_UPS_CONTRACT",
    "DASHBOARD_CONTRACT",
    "DASHBOARD_TEXT_CONTRACT",
    "EDGE_UPS_CONTRACT",
    "HEALTH_CONTRACT",
    "NODE_PUBLISH_CONTRACTS",
    "NODE_SUBSCRIBE_CONTRACTS",
    "POWER_SENSOR_CONTRACTS",
    "POWER_STATUS_CONTRACTS",
    "SHUTDOWN_REQUEST_CONTRACT",
    "STATUS_CONTRACT",
    "STD_BOOL",
    "STD_STRING",
    "TopicContract",
    "TopicDirection",
    "TopicPurpose",
    "all_topic_contracts",
    "contracts_for_node",
    "format_topic_contracts",
    "publish_contracts_for_node",
    "sensor_topic_contracts",
    "status_topic_contracts",
    "subscribe_contracts_for_node",
    "topic_contract_by_name",
    "topic_contract_for_source",
    "topic_contracts_to_dict",
    "topic_names",
    "validate_all_contracts",
    "validate_contract",
    "validate_topic_name",
]
