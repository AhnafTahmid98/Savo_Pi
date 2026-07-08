import dataclasses
import inspect
import json
from pathlib import Path

import pytest

import savo_power.utils.param_loader as param_loader


def public_callables():
    return {
        name: value
        for name, value in vars(param_loader).items()
        if callable(value) and not name.startswith("_")
    }


def first_callable(*names):
    callables = public_callables()

    for name in names:
        value = callables.get(name)
        if value is not None:
            return value

    return None


def callable_containing(*parts):
    parts = tuple(part.lower() for part in parts)

    for name, value in public_callables().items():
        lower = name.lower()

        if all(part in lower for part in parts):
            return value

    return None


def test_param_loader_module_imports():
    assert param_loader is not None


def test_param_loader_result_and_error_models_exist():
    assert hasattr(param_loader, "ParamLoadError")
    assert hasattr(param_loader, "ParamLoadResult")

    assert dataclasses.is_dataclass(param_loader.ParamLoadResult)


def test_param_load_result_can_represent_success():
    cls = param_loader.ParamLoadResult
    signature = inspect.signature(cls)
    kwargs = {}

    for name, parameter in signature.parameters.items():
        if parameter.default is not inspect.Parameter.empty:
            continue

        if name in {"ok", "success", "valid"}:
            kwargs[name] = True
        elif name in {"path", "source", "filename"}:
            kwargs[name] = "test.yaml"
        elif name in {"data", "params", "parameters", "value"}:
            kwargs[name] = {"sample_rate_hz": 1.0}
        elif name in {"errors", "warnings"}:
            kwargs[name] = []
        elif name in {"message", "error"}:
            kwargs[name] = ""
        else:
            kwargs[name] = None

    result = cls(**kwargs)

    assert dataclasses.is_dataclass(result)


def test_deep_merge_preserves_nested_values():
    deep_merge = first_callable(
        "deep_merge",
        "merge_nested_dicts",
        "merge_dicts",
    ) or callable_containing("merge")

    if deep_merge is None:
        pytest.skip("No deep merge helper found")

    base = {
        "ros__parameters": {
            "i2c_bus": 1,
            "publish_rate_hz": 1.0,
            "automatic_shutdown_enabled": False,
        }
    }
    override = {
        "ros__parameters": {
            "publish_rate_hz": 2.0,
            "stale_timeout_s": 5.0,
        }
    }

    merged = deep_merge(base, override)

    assert merged["ros__parameters"]["i2c_bus"] == 1
    assert merged["ros__parameters"]["publish_rate_hz"] == 2.0
    assert merged["ros__parameters"]["stale_timeout_s"] == 5.0
    assert merged["ros__parameters"]["automatic_shutdown_enabled"] is False


def test_nested_get_and_set_helpers_work_if_available():
    nested_get = first_callable(
        "nested_get",
        "get_nested",
        "get_nested_value",
    )
    nested_set = first_callable(
        "nested_set",
        "set_nested",
        "set_nested_value",
    )

    if nested_get is None or nested_set is None:
        pytest.skip("Nested get/set helpers not found")

    data = {}

    nested_set(data, "ros__parameters.i2c_bus", 1)
    value = nested_get(data, "ros__parameters.i2c_bus")

    assert value == 1


def test_flatten_and_unflatten_helpers_work_if_available():
    flatten = first_callable(
        "flatten_dict",
        "flatten_nested_dict",
        "flatten",
    )
    unflatten = first_callable(
        "unflatten_dict",
        "unflatten_nested_dict",
        "unflatten",
    )

    if flatten is None or unflatten is None:
        pytest.skip("Flatten/unflatten helpers not found")

    data = {
        "ros__parameters": {
            "i2c_bus": 1,
            "publish_rate_hz": 1.0,
        }
    }

    flat = flatten(data)

    assert isinstance(flat, dict)
    assert any("i2c_bus" in key for key in flat)

    rebuilt = unflatten(flat)

    assert isinstance(rebuilt, dict)
    assert "ros__parameters" in str(rebuilt)


def test_yaml_loader_can_read_ros_param_file_if_available(tmp_path):
    loader = first_callable(
        "load_yaml_file",
        "load_yaml",
        "read_yaml_file",
    ) or callable_containing("yaml", "load")

    if loader is None:
        pytest.skip("YAML loader helper not found")

    path = tmp_path / "power.yaml"
    path.write_text(
        '"/**":\n'
        "  ros__parameters:\n"
        "    i2c_bus: 1\n"
        "    automatic_shutdown_enabled: false\n"
    )

    data = loader(path)

    assert isinstance(data, dict)
    assert "automatic_shutdown_enabled" in str(data)
    assert "false" in str(data).lower() or "False" in str(data)


def test_json_loader_can_read_config_if_available(tmp_path):
    loader = first_callable(
        "load_json_file",
        "load_json",
        "read_json_file",
    ) or callable_containing("json", "load")

    if loader is None:
        pytest.skip("JSON loader helper not found")

    path = tmp_path / "power.json"
    path.write_text(
        json.dumps(
            {
                "ros__parameters": {
                    "i2c_bus": 1,
                    "automatic_shutdown_enabled": False,
                }
            }
        )
    )

    data = loader(path)

    assert isinstance(data, dict)
    assert data["ros__parameters"]["i2c_bus"] == 1
    assert data["ros__parameters"]["automatic_shutdown_enabled"] is False


def test_default_param_helpers_exist_for_power_nodes():
    names = set(public_callables())

    expected_keywords = [
        "ups",
        "kit",
        "aggregator",
        "health",
        "dashboard",
    ]

    for keyword in expected_keywords:
        assert any(
            "default" in name.lower() and keyword in name.lower()
            for name in names
        ), f"Missing default params helper for {keyword}"


def test_default_policy_keeps_shutdown_disabled_if_available():
    helper = (
        callable_containing("default", "policy")
        or callable_containing("default", "power")
    )

    if helper is None:
        pytest.skip("Default policy helper not found")

    data = helper()

    if dataclasses.is_dataclass(data):
        text = str(dataclasses.asdict(data))
    else:
        text = str(data)

    assert "automatic_shutdown_enabled" in text
    assert "False" in text or "false" in text


def test_config_files_are_present():
    required = [
        "config/power_common.yaml",
        "config/topics.yaml",
        "config/core/core_ups.yaml",
        "config/core/kit_battery.yaml",
        "config/core/power_aggregator.yaml",
        "config/core/power_health.yaml",
        "config/core/dashboard.yaml",
        "config/edge/edge_ups.yaml",
        "config/profiles/core_real_robot_v1.yaml",
        "config/profiles/edge_real_robot_v1.yaml",
        "config/profiles/dryrun_sim.yaml",
        "config/diagnostic.yaml",
    ]

    for item in required:
        assert Path(item).is_file(), item


def test_config_files_keep_shutdown_disabled_by_default():
    for path in Path("config").rglob("*.yaml"):
        text = path.read_text()

        if "automatic_shutdown_enabled" in text:
            assert "automatic_shutdown_enabled: false" in text
