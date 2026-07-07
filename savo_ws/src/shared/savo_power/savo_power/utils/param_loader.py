"""Parameter loading helpers for Robot Savo power Python tools."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Iterable, Mapping, MutableMapping

from savo_power import constants as c
from savo_power.models.kit_battery_reading import normalize_pcb_version
from savo_power.models.power_status import (
    BatterySource,
    normalize_battery_source,
)
from savo_power.utils.clamp import (
    clamp_i2c_7bit_address,
    clamp_publish_rate_hz,
    clamp_raw_byte,
    to_float,
    to_int,
)


class ParamLoadError(RuntimeError):
    """Raised when a parameter file cannot be loaded."""


@dataclass(frozen=True)
class ParamLoadResult:
    """Result from loading one parameter source."""

    path: str
    exists: bool
    data: dict[str, object]
    error: str = ""

    @property
    def ok(self) -> bool:
        return self.exists and not self.error

    def to_dict(self) -> dict[str, object]:
        return {
            "path": self.path,
            "exists": self.exists,
            "ok": self.ok,
            "error": self.error,
            "data": to_plain_data(self.data),
        }


def yaml_available() -> bool:
    """Return True when PyYAML is available."""

    try:
        import yaml  # noqa: F401
    except ImportError:
        return False

    return True


def to_plain_data(value: object) -> object:
    """Convert dataclasses/enums/containers into plain JSON-safe data."""

    if isinstance(value, Enum):
        return value.value

    if is_dataclass(value) and not isinstance(value, type):
        return to_plain_data(asdict(value))

    if isinstance(value, Mapping):
        return {
            str(key): to_plain_data(item)
            for key, item in value.items()
        }

    if isinstance(value, (tuple, list)):
        return [
            to_plain_data(item)
            for item in value
        ]

    if isinstance(value, set):
        return sorted(to_plain_data(item) for item in value)

    return value


def ensure_mapping(value: object, *, name: str = "value") -> dict[str, object]:
    """Return value as a plain dictionary or raise ParamLoadError."""

    if value is None:
        return {}

    if isinstance(value, Mapping):
        return {
            str(key): item
            for key, item in value.items()
        }

    raise ParamLoadError(f"{name} must be a mapping, got {type(value).__name__}")


def _load_json_text(text: str) -> object:
    return json.loads(text)


def _load_yaml_text(text: str) -> object:
    try:
        import yaml
    except ImportError as exc:
        raise ParamLoadError(
            "PyYAML is not installed. Install python3-yaml to load YAML files."
        ) from exc

    loaded = yaml.safe_load(text)

    return {} if loaded is None else loaded


def load_parameter_text(text: str, *, source_name: str = "<text>") -> dict[str, object]:
    """Load JSON/YAML parameter text into a dictionary."""

    clean = str(text)

    if not clean.strip():
        return {}

    if source_name.endswith(".json"):
        return ensure_mapping(_load_json_text(clean), name=source_name)

    if source_name.endswith((".yaml", ".yml")):
        return ensure_mapping(_load_yaml_text(clean), name=source_name)

    try:
        return ensure_mapping(_load_json_text(clean), name=source_name)
    except json.JSONDecodeError:
        return ensure_mapping(_load_yaml_text(clean), name=source_name)


def load_parameter_file(path: str | Path, *, required: bool = True) -> ParamLoadResult:
    """Load a JSON/YAML parameter file."""

    selected_path = Path(path)

    if not selected_path.exists():
        error = "file not found"
        if required:
            return ParamLoadResult(
                path=str(selected_path),
                exists=False,
                data={},
                error=error,
            )

        return ParamLoadResult(
            path=str(selected_path),
            exists=False,
            data={},
            error="",
        )

    if not selected_path.is_file():
        return ParamLoadResult(
            path=str(selected_path),
            exists=True,
            data={},
            error="path is not a file",
        )

    try:
        data = load_parameter_text(
            selected_path.read_text(),
            source_name=str(selected_path),
        )
    except Exception as exc:  # noqa: BLE001 - keep config errors visible
        return ParamLoadResult(
            path=str(selected_path),
            exists=True,
            data={},
            error=f"{type(exc).__name__}: {exc}",
        )

    return ParamLoadResult(
        path=str(selected_path),
        exists=True,
        data=data,
    )


def deep_merge_dicts(
    base: Mapping[str, object],
    override: Mapping[str, object],
) -> dict[str, object]:
    """Deep merge dictionaries. Override wins."""

    result: dict[str, object] = {
        str(key): to_plain_data(value)
        for key, value in base.items()
    }

    for key, value in override.items():
        key_text = str(key)

        if (
            key_text in result
            and isinstance(result[key_text], Mapping)
            and isinstance(value, Mapping)
        ):
            result[key_text] = deep_merge_dicts(
                result[key_text],  # type: ignore[arg-type]
                value,
            )
        else:
            result[key_text] = to_plain_data(value)

    return result


def merge_many_dicts(
    mappings: Iterable[Mapping[str, object]],
) -> dict[str, object]:
    """Merge many dictionaries from left to right."""

    result: dict[str, object] = {}

    for mapping in mappings:
        result = deep_merge_dicts(result, mapping)

    return result


def get_nested(
    data: Mapping[str, object],
    path: str,
    *,
    default: object = None,
    separator: str = ".",
) -> object:
    """Read nested value using dot path."""

    if not path:
        return data

    current: object = data

    for part in path.split(separator):
        if not isinstance(current, Mapping):
            return default

        if part not in current:
            return default

        current = current[part]

    return current


def set_nested(
    data: MutableMapping[str, object],
    path: str,
    value: object,
    *,
    separator: str = ".",
) -> None:
    """Set nested value using dot path."""

    parts = [part for part in path.split(separator) if part]

    if not parts:
        return

    current: MutableMapping[str, object] = data

    for part in parts[:-1]:
        existing = current.get(part)

        if not isinstance(existing, MutableMapping):
            existing = {}
            current[part] = existing

        current = existing

    current[parts[-1]] = value


def flatten_mapping(
    data: Mapping[str, object],
    *,
    prefix: str = "",
    separator: str = ".",
) -> dict[str, object]:
    """Flatten nested mapping into dot-path dictionary."""

    result: dict[str, object] = {}

    for key, value in data.items():
        key_text = str(key)
        full_key = f"{prefix}{separator}{key_text}" if prefix else key_text

        if isinstance(value, Mapping):
            result.update(
                flatten_mapping(
                    value,
                    prefix=full_key,
                    separator=separator,
                )
            )
        else:
            result[full_key] = value

    return result


def unflatten_mapping(
    data: Mapping[str, object],
    *,
    separator: str = ".",
) -> dict[str, object]:
    """Convert dot-path dictionary into nested dictionary."""

    result: dict[str, object] = {}

    for key, value in data.items():
        set_nested(
            result,
            str(key),
            value,
            separator=separator,
        )

    return result


def extract_ros_parameters(
    data: Mapping[str, object],
    *,
    node_name: str | None = None,
) -> dict[str, object]:
    """Extract ROS 2 style ros__parameters mapping.

    Supported forms:

    plain:
      publish_rate_hz: 1.0

    ROS 2:
      node_name:
        ros__parameters:
          publish_rate_hz: 1.0

    wildcard:
      /**:
        ros__parameters:
          publish_rate_hz: 1.0
    """

    mapping = ensure_mapping(data)

    if "ros__parameters" in mapping:
        return ensure_mapping(
            mapping.get("ros__parameters"),
            name="ros__parameters",
        )

    if node_name:
        candidates = (
            node_name,
            f"/{node_name}",
            node_name.lstrip("/"),
        )

        for candidate in candidates:
            node_block = mapping.get(candidate)

            if isinstance(node_block, Mapping):
                if "ros__parameters" in node_block:
                    return ensure_mapping(
                        node_block.get("ros__parameters"),
                        name=f"{candidate}.ros__parameters",
                    )

                return ensure_mapping(node_block, name=candidate)

    wildcard = mapping.get("/**")

    if isinstance(wildcard, Mapping) and "ros__parameters" in wildcard:
        return ensure_mapping(
            wildcard.get("ros__parameters"),
            name="/**.ros__parameters",
        )

    return mapping


def coerce_bool(value: object, *, default: bool = False) -> bool:
    """Coerce common values to bool."""

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


def coerce_power_param_value(name: str, value: object) -> object:
    """Coerce known power parameter values to stable Python types."""

    if name in {
        c.PARAM_I2C_BUS,
        c.PARAM_ADS7830_CHANNEL,
    }:
        return to_int(value)

    if name in {
        c.PARAM_UPS_ADDRESS,
        c.PARAM_ADS7830_ADDRESS,
    }:
        return clamp_i2c_7bit_address(value)

    if name == c.PARAM_ADS7830_PCB_VERSION:
        return normalize_pcb_version(str(value))

    if name in {
        c.PARAM_SAMPLE_RATE_HZ,
        c.PARAM_PUBLISH_RATE_HZ,
    }:
        return clamp_publish_rate_hz(value)

    if name in {
        c.PARAM_STALE_TIMEOUT_S,
        c.PARAM_UPS_LOW_VOLTAGE,
        c.PARAM_UPS_CRITICAL_VOLTAGE,
        c.PARAM_BASE_EMPTY_VOLTAGE,
        c.PARAM_BASE_LOW_VOLTAGE,
        c.PARAM_BASE_FULL_VOLTAGE,
        c.PARAM_BASE_LOW_SOC,
        c.PARAM_BASE_FULL_SOC,
        c.PARAM_FULL_CAPACITY,
    }:
        return to_float(value)

    if name in {
        c.PARAM_AUTOMATIC_SHUTDOWN_ENABLED,
        c.PARAM_CORE_UPS_EXPECTED,
        c.PARAM_EDGE_UPS_EXPECTED,
        c.PARAM_BASE_BATTERY_EXPECTED,
    }:
        return coerce_bool(value)

    return value


def coerce_power_params(params: Mapping[str, object]) -> dict[str, object]:
    """Coerce known power parameters inside a flat mapping."""

    result: dict[str, object] = {}

    for key, value in params.items():
        result[str(key)] = coerce_power_param_value(str(key), value)

    return result


def default_ups_params(
    source: str | BatterySource,
) -> dict[str, object]:
    """Default parameters for a UPS HAT node."""

    normalized = normalize_battery_source(source)

    return {
        "source": normalized.value,
        c.PARAM_I2C_BUS: c.DEFAULT_I2C_BUS,
        c.PARAM_UPS_ADDRESS: c.UPS_HAT_DEFAULT_ADDRESS,
        c.PARAM_SAMPLE_RATE_HZ: c.DEFAULT_SAMPLE_RATE_HZ,
        c.PARAM_PUBLISH_RATE_HZ: c.DEFAULT_PUBLISH_RATE_HZ,
    }


def default_kit_battery_params() -> dict[str, object]:
    """Default parameters for ADS7830/base battery node."""

    return {
        c.PARAM_I2C_BUS: c.DEFAULT_I2C_BUS,
        c.PARAM_ADS7830_ADDRESS: c.ADS7830_DEFAULT_ADDRESS,
        c.PARAM_ADS7830_CHANNEL: c.ADS7830_DEFAULT_CHANNEL,
        c.PARAM_ADS7830_PCB_VERSION: c.ADS7830_DEFAULT_PCB_VERSION,
        c.PARAM_SAMPLE_RATE_HZ: c.DEFAULT_SAMPLE_RATE_HZ,
        c.PARAM_PUBLISH_RATE_HZ: c.DEFAULT_PUBLISH_RATE_HZ,
    }


def default_power_aggregator_params() -> dict[str, object]:
    """Default parameters for power aggregator node."""

    return {
        c.PARAM_STALE_TIMEOUT_S: c.DEFAULT_STALE_TIMEOUT_S,
        c.PARAM_CORE_UPS_EXPECTED: True,
        c.PARAM_EDGE_UPS_EXPECTED: True,
        c.PARAM_BASE_BATTERY_EXPECTED: True,
        c.PARAM_PUBLISH_RATE_HZ: c.DEFAULT_PUBLISH_RATE_HZ,
    }


def default_power_health_params() -> dict[str, object]:
    """Default parameters for power health node."""

    return {
        c.PARAM_STALE_TIMEOUT_S: c.DEFAULT_STALE_TIMEOUT_S,
        c.PARAM_PUBLISH_RATE_HZ: c.DEFAULT_HEALTH_PUBLISH_HZ,
        c.PARAM_AUTOMATIC_SHUTDOWN_ENABLED: c.AUTOMATIC_SHUTDOWN_ENABLED_DEFAULT,
    }


def default_power_dashboard_params() -> dict[str, object]:
    """Default parameters for power dashboard node."""

    return {
        c.PARAM_PUBLISH_RATE_HZ: c.DEFAULT_DASHBOARD_PUBLISH_HZ,
    }


def default_power_policy_params() -> dict[str, object]:
    """Default power policy threshold parameters."""

    return {
        c.PARAM_UPS_LOW_VOLTAGE: c.UPS_LOW_VOLTAGE,
        c.PARAM_UPS_CRITICAL_VOLTAGE: c.UPS_CRITICAL_VOLTAGE,
        c.PARAM_BASE_EMPTY_VOLTAGE: c.BASE_BATTERY_EMPTY_VOLTAGE,
        c.PARAM_BASE_LOW_VOLTAGE: c.BASE_BATTERY_LOW_VOLTAGE,
        c.PARAM_BASE_FULL_VOLTAGE: c.BASE_BATTERY_FULL_VOLTAGE,
        c.PARAM_BASE_LOW_SOC: c.BASE_BATTERY_LOW_SOC,
        c.PARAM_BASE_FULL_SOC: c.BASE_BATTERY_FULL_SOC,
        c.PARAM_FULL_CAPACITY: c.FULL_CAPACITY_PERCENT,
        c.PARAM_AUTOMATIC_SHUTDOWN_ENABLED: c.AUTOMATIC_SHUTDOWN_ENABLED_DEFAULT,
    }


def merge_with_defaults(
    defaults: Mapping[str, object],
    overrides: Mapping[str, object] | None = None,
    *,
    coerce: bool = True,
) -> dict[str, object]:
    """Merge override parameters over defaults."""

    merged = deep_merge_dicts(
        defaults,
        overrides or {},
    )

    if coerce:
        return coerce_power_params(merged)

    return merged


def load_node_params_from_file(
    path: str | Path,
    *,
    node_name: str | None = None,
    defaults: Mapping[str, object] | None = None,
    required: bool = True,
    coerce: bool = True,
) -> ParamLoadResult:
    """Load node parameters from JSON/YAML/ROS parameter file."""

    result = load_parameter_file(
        path,
        required=required,
    )

    if not result.ok:
        return result

    try:
        extracted = extract_ros_parameters(
            result.data,
            node_name=node_name,
        )
        merged = merge_with_defaults(
            defaults or {},
            extracted,
            coerce=coerce,
        )
    except Exception as exc:  # noqa: BLE001
        return ParamLoadResult(
            path=result.path,
            exists=result.exists,
            data={},
            error=f"{type(exc).__name__}: {exc}",
        )

    return ParamLoadResult(
        path=result.path,
        exists=result.exists,
        data=merged,
    )


def params_to_ros2_yaml_dict(
    node_name: str,
    params: Mapping[str, object],
) -> dict[str, object]:
    """Convert params to ROS 2 parameter YAML structure."""

    return {
        str(node_name): {
            "ros__parameters": to_plain_data(params),
        }
    }


def params_to_launch_list(params: Mapping[str, object]) -> list[dict[str, object]]:
    """Convert params to launch Node(parameters=[...]) compatible list."""

    return [dict(to_plain_data(params))]  # type: ignore[arg-type]


def dump_params_json(params: Mapping[str, object], *, indent: int = 2) -> str:
    """Dump parameters as JSON text."""

    return json.dumps(
        to_plain_data(params),
        indent=indent,
        sort_keys=True,
    )


def write_params_json(path: str | Path, params: Mapping[str, object]) -> Path:
    """Write parameters to JSON file."""

    selected_path = Path(path)
    selected_path.parent.mkdir(parents=True, exist_ok=True)
    selected_path.write_text(dump_params_json(params) + "\n")
    return selected_path


__all__ = [
    "ParamLoadError",
    "ParamLoadResult",
    "coerce_bool",
    "coerce_power_param_value",
    "coerce_power_params",
    "deep_merge_dicts",
    "default_kit_battery_params",
    "default_power_aggregator_params",
    "default_power_dashboard_params",
    "default_power_health_params",
    "default_power_policy_params",
    "default_ups_params",
    "dump_params_json",
    "ensure_mapping",
    "extract_ros_parameters",
    "flatten_mapping",
    "get_nested",
    "load_node_params_from_file",
    "load_parameter_file",
    "load_parameter_text",
    "merge_many_dicts",
    "merge_with_defaults",
    "params_to_launch_list",
    "params_to_ros2_yaml_dict",
    "set_nested",
    "to_plain_data",
    "unflatten_mapping",
    "write_params_json",
    "yaml_available",
]
