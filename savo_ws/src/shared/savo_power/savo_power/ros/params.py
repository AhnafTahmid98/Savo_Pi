"""ROS parameter helpers for Robot Savo Python power fallback nodes."""

from __future__ import annotations

from dataclasses import asdict, dataclass, is_dataclass
from enum import Enum
from typing import Any, Mapping

from savo_power import constants as c
from savo_power.models.kit_battery_reading import normalize_pcb_version
from savo_power.models.power_status import (
    BatterySource,
    normalize_battery_source,
)
from savo_power.policy.power_policy import PowerPolicyThresholds


@dataclass(frozen=True)
class UpsNodeParams:
    """Parameters for a UPS HAT Python fallback node."""

    source: BatterySource
    i2c_bus: int = c.DEFAULT_I2C_BUS
    address: int = c.UPS_HAT_DEFAULT_ADDRESS
    sample_rate_hz: float = c.DEFAULT_SAMPLE_RATE_HZ
    publish_rate_hz: float = c.DEFAULT_PUBLISH_RATE_HZ

    def to_dict(self) -> dict[str, object]:
        return to_jsonable(self)  # type: ignore[return-value]


@dataclass(frozen=True)
class KitBatteryNodeParams:
    """Parameters for the base battery ADS7830 Python fallback node."""

    i2c_bus: int = c.DEFAULT_I2C_BUS
    address: int = c.ADS7830_DEFAULT_ADDRESS
    channel: int = c.ADS7830_DEFAULT_CHANNEL
    pcb_version: str = c.ADS7830_DEFAULT_PCB_VERSION
    sample_rate_hz: float = c.DEFAULT_SAMPLE_RATE_HZ
    publish_rate_hz: float = c.DEFAULT_PUBLISH_RATE_HZ

    def to_dict(self) -> dict[str, object]:
        return to_jsonable(self)  # type: ignore[return-value]


@dataclass(frozen=True)
class PowerAggregatorNodeParams:
    """Parameters for the Python power aggregator fallback node."""

    stale_timeout_s: float = c.DEFAULT_STALE_TIMEOUT_S

    core_ups_expected: bool = True
    edge_ups_expected: bool = True
    base_battery_expected: bool = True

    publish_rate_hz: float = c.DEFAULT_PUBLISH_RATE_HZ

    def to_dict(self) -> dict[str, object]:
        return to_jsonable(self)  # type: ignore[return-value]


@dataclass(frozen=True)
class PowerHealthNodeParams:
    """Parameters for the Python power health fallback node."""

    stale_timeout_s: float = c.DEFAULT_STALE_TIMEOUT_S
    publish_rate_hz: float = c.DEFAULT_HEALTH_PUBLISH_HZ

    automatic_shutdown_enabled: bool = c.AUTOMATIC_SHUTDOWN_ENABLED_DEFAULT

    def to_dict(self) -> dict[str, object]:
        return to_jsonable(self)  # type: ignore[return-value]


@dataclass(frozen=True)
class PowerDashboardNodeParams:
    """Parameters for the Python power dashboard fallback node."""

    publish_rate_hz: float = c.DEFAULT_DASHBOARD_PUBLISH_HZ

    def to_dict(self) -> dict[str, object]:
        return to_jsonable(self)  # type: ignore[return-value]


def to_jsonable(value: object) -> object:
    """Convert dataclasses/enums/containers into JSON-safe objects."""

    if isinstance(value, Enum):
        return value.value

    if is_dataclass(value) and not isinstance(value, type):
        return to_jsonable(asdict(value))

    if isinstance(value, Mapping):
        return {
            str(key): to_jsonable(item)
            for key, item in value.items()
        }

    if isinstance(value, tuple | list):
        return [
            to_jsonable(item)
            for item in value
        ]

    if isinstance(value, set):
        return sorted(to_jsonable(item) for item in value)

    return value


def ros_parameter_to_python_value(parameter: object, default: object = None) -> object:
    """Extract a Python value from a ROS parameter-like object."""

    if parameter is None:
        return default

    if hasattr(parameter, "value"):
        return getattr(parameter, "value")

    if hasattr(parameter, "get_parameter_value"):
        parameter_value = parameter.get_parameter_value()

        if isinstance(default, bool):
            return bool(getattr(parameter_value, "bool_value", default))

        if isinstance(default, int) and not isinstance(default, bool):
            return int(getattr(parameter_value, "integer_value", default))

        if isinstance(default, float):
            return float(getattr(parameter_value, "double_value", default))

        if isinstance(default, str):
            return str(getattr(parameter_value, "string_value", default))

    return parameter


def declare_and_get_parameter(
    node: object,
    name: str,
    default: object,
) -> object:
    """Declare a parameter if possible, then return its value."""

    if node is None:
        return default

    if hasattr(node, "declare_parameter"):
        try:
            declared = node.declare_parameter(name, default)
            return ros_parameter_to_python_value(declared, default)
        except Exception:
            pass

    if hasattr(node, "get_parameter"):
        try:
            parameter = node.get_parameter(name)
            value = ros_parameter_to_python_value(parameter, default)
            return default if value is None else value
        except Exception:
            pass

    return default


def as_bool(value: object, default: bool = False) -> bool:
    """Convert common parameter values into bool."""

    if isinstance(value, bool):
        return value

    if value is None:
        return bool(default)

    if isinstance(value, str):
        text = value.strip().lower()

        if text in {"1", "true", "yes", "y", "on"}:
            return True

        if text in {"0", "false", "no", "n", "off"}:
            return False

        return bool(default)

    return bool(value)


def as_int(value: object, default: int = 0) -> int:
    """Convert common parameter values into int."""

    if value is None:
        return int(default)

    try:
        if isinstance(value, str):
            return int(value, 0)

        return int(value)
    except (TypeError, ValueError):
        return int(default)


def as_float(value: object, default: float = 0.0) -> float:
    """Convert common parameter values into float."""

    if value is None:
        return float(default)

    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def as_str(value: object, default: str = "") -> str:
    """Convert common parameter values into string."""

    if value is None:
        return str(default)

    return str(value)


def clamp_float(value: float, minimum: float, maximum: float) -> float:
    """Clamp float value into range."""

    return max(float(minimum), min(float(maximum), float(value)))


def normalize_rate_hz(value: object, default: float = c.DEFAULT_PUBLISH_RATE_HZ) -> float:
    """Normalize publish/sample rate into safe range."""

    rate = as_float(value, default)

    if rate <= 0.0:
        rate = float(default)

    return clamp_float(
        rate,
        c.MIN_PUBLISH_RATE_HZ,
        c.MAX_PUBLISH_RATE_HZ,
    )


def get_bool_param(node: object, name: str, default: bool) -> bool:
    """Read bool parameter from node."""

    return as_bool(
        declare_and_get_parameter(node, name, default),
        default,
    )


def get_int_param(node: object, name: str, default: int) -> int:
    """Read int parameter from node."""

    return as_int(
        declare_and_get_parameter(node, name, default),
        default,
    )


def get_float_param(node: object, name: str, default: float) -> float:
    """Read float parameter from node."""

    return as_float(
        declare_and_get_parameter(node, name, default),
        default,
    )


def get_str_param(node: object, name: str, default: str) -> str:
    """Read string parameter from node."""

    return as_str(
        declare_and_get_parameter(node, name, default),
        default,
    )


def read_power_policy_thresholds(node: object) -> PowerPolicyThresholds:
    """Read power policy thresholds from ROS parameters."""

    return PowerPolicyThresholds(
        ups_low_voltage_v=get_float_param(
            node,
            c.PARAM_UPS_LOW_VOLTAGE,
            c.UPS_LOW_VOLTAGE,
        ),
        ups_critical_voltage_v=get_float_param(
            node,
            c.PARAM_UPS_CRITICAL_VOLTAGE,
            c.UPS_CRITICAL_VOLTAGE,
        ),
        base_empty_voltage_v=get_float_param(
            node,
            c.PARAM_BASE_EMPTY_VOLTAGE,
            c.BASE_BATTERY_EMPTY_VOLTAGE,
        ),
        base_low_voltage_v=get_float_param(
            node,
            c.PARAM_BASE_LOW_VOLTAGE,
            c.BASE_BATTERY_LOW_VOLTAGE,
        ),
        base_full_voltage_v=get_float_param(
            node,
            c.PARAM_BASE_FULL_VOLTAGE,
            c.BASE_BATTERY_FULL_VOLTAGE,
        ),
        base_low_soc_pct=get_float_param(
            node,
            c.PARAM_BASE_LOW_SOC,
            c.BASE_BATTERY_LOW_SOC,
        ),
        base_full_soc_pct=get_float_param(
            node,
            c.PARAM_BASE_FULL_SOC,
            c.BASE_BATTERY_FULL_SOC,
        ),
        full_capacity_pct=get_float_param(
            node,
            c.PARAM_FULL_CAPACITY,
            c.FULL_CAPACITY_PERCENT,
        ),
        automatic_shutdown_enabled=get_bool_param(
            node,
            c.PARAM_AUTOMATIC_SHUTDOWN_ENABLED,
            c.AUTOMATIC_SHUTDOWN_ENABLED_DEFAULT,
        ),
    )


def read_ups_node_params(
    node: object,
    *,
    source: str | BatterySource,
) -> UpsNodeParams:
    """Read UPS HAT node parameters."""

    return UpsNodeParams(
        source=normalize_battery_source(source),
        i2c_bus=get_int_param(
            node,
            c.PARAM_I2C_BUS,
            c.DEFAULT_I2C_BUS,
        ),
        address=get_int_param(
            node,
            c.PARAM_UPS_ADDRESS,
            c.UPS_HAT_DEFAULT_ADDRESS,
        ),
        sample_rate_hz=normalize_rate_hz(
            declare_and_get_parameter(
                node,
                c.PARAM_SAMPLE_RATE_HZ,
                c.DEFAULT_SAMPLE_RATE_HZ,
            ),
            c.DEFAULT_SAMPLE_RATE_HZ,
        ),
        publish_rate_hz=normalize_rate_hz(
            declare_and_get_parameter(
                node,
                c.PARAM_PUBLISH_RATE_HZ,
                c.DEFAULT_PUBLISH_RATE_HZ,
            ),
            c.DEFAULT_PUBLISH_RATE_HZ,
        ),
    )


def read_kit_battery_node_params(node: object) -> KitBatteryNodeParams:
    """Read base battery ADS7830 node parameters."""

    return KitBatteryNodeParams(
        i2c_bus=get_int_param(
            node,
            c.PARAM_I2C_BUS,
            c.DEFAULT_I2C_BUS,
        ),
        address=get_int_param(
            node,
            c.PARAM_ADS7830_ADDRESS,
            c.ADS7830_DEFAULT_ADDRESS,
        ),
        channel=get_int_param(
            node,
            c.PARAM_ADS7830_CHANNEL,
            c.ADS7830_DEFAULT_CHANNEL,
        ),
        pcb_version=normalize_pcb_version(
            get_str_param(
                node,
                c.PARAM_ADS7830_PCB_VERSION,
                c.ADS7830_DEFAULT_PCB_VERSION,
            )
        ),
        sample_rate_hz=normalize_rate_hz(
            declare_and_get_parameter(
                node,
                c.PARAM_SAMPLE_RATE_HZ,
                c.DEFAULT_SAMPLE_RATE_HZ,
            ),
            c.DEFAULT_SAMPLE_RATE_HZ,
        ),
        publish_rate_hz=normalize_rate_hz(
            declare_and_get_parameter(
                node,
                c.PARAM_PUBLISH_RATE_HZ,
                c.DEFAULT_PUBLISH_RATE_HZ,
            ),
            c.DEFAULT_PUBLISH_RATE_HZ,
        ),
    )


def read_power_aggregator_node_params(node: object) -> PowerAggregatorNodeParams:
    """Read power aggregator node parameters."""

    return PowerAggregatorNodeParams(
        stale_timeout_s=get_float_param(
            node,
            c.PARAM_STALE_TIMEOUT_S,
            c.DEFAULT_STALE_TIMEOUT_S,
        ),
        core_ups_expected=get_bool_param(
            node,
            c.PARAM_CORE_UPS_EXPECTED,
            True,
        ),
        edge_ups_expected=get_bool_param(
            node,
            c.PARAM_EDGE_UPS_EXPECTED,
            True,
        ),
        base_battery_expected=get_bool_param(
            node,
            c.PARAM_BASE_BATTERY_EXPECTED,
            True,
        ),
        publish_rate_hz=normalize_rate_hz(
            declare_and_get_parameter(
                node,
                c.PARAM_PUBLISH_RATE_HZ,
                c.DEFAULT_PUBLISH_RATE_HZ,
            ),
            c.DEFAULT_PUBLISH_RATE_HZ,
        ),
    )


def read_power_health_node_params(node: object) -> PowerHealthNodeParams:
    """Read power health node parameters."""

    return PowerHealthNodeParams(
        stale_timeout_s=get_float_param(
            node,
            c.PARAM_STALE_TIMEOUT_S,
            c.DEFAULT_STALE_TIMEOUT_S,
        ),
        publish_rate_hz=normalize_rate_hz(
            declare_and_get_parameter(
                node,
                c.PARAM_PUBLISH_RATE_HZ,
                c.DEFAULT_HEALTH_PUBLISH_HZ,
            ),
            c.DEFAULT_HEALTH_PUBLISH_HZ,
        ),
        automatic_shutdown_enabled=get_bool_param(
            node,
            c.PARAM_AUTOMATIC_SHUTDOWN_ENABLED,
            c.AUTOMATIC_SHUTDOWN_ENABLED_DEFAULT,
        ),
    )


def read_power_dashboard_node_params(node: object) -> PowerDashboardNodeParams:
    """Read power dashboard node parameters."""

    return PowerDashboardNodeParams(
        publish_rate_hz=normalize_rate_hz(
            declare_and_get_parameter(
                node,
                c.PARAM_PUBLISH_RATE_HZ,
                c.DEFAULT_DASHBOARD_PUBLISH_HZ,
            ),
            c.DEFAULT_DASHBOARD_PUBLISH_HZ,
        ),
    )


def node_params_to_ros_dict(params: object) -> dict[str, object]:
    """Convert parameter dataclass into ROS launch-compatible dictionary."""

    converted = to_jsonable(params)

    if not isinstance(converted, dict):
        return {}

    return converted


__all__ = [
    "KitBatteryNodeParams",
    "PowerAggregatorNodeParams",
    "PowerDashboardNodeParams",
    "PowerHealthNodeParams",
    "UpsNodeParams",
    "as_bool",
    "as_float",
    "as_int",
    "as_str",
    "clamp_float",
    "declare_and_get_parameter",
    "get_bool_param",
    "get_float_param",
    "get_int_param",
    "get_str_param",
    "node_params_to_ros_dict",
    "normalize_rate_hz",
    "read_kit_battery_node_params",
    "read_power_aggregator_node_params",
    "read_power_dashboard_node_params",
    "read_power_health_node_params",
    "read_power_policy_thresholds",
    "read_ups_node_params",
    "ros_parameter_to_python_value",
    "to_jsonable",
]
