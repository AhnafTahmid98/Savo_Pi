import dataclasses
import inspect

import pytest

import savo_power.policy.power_policy as power_policy
from savo_power.models.power_status import PowerState


def public_callables(module):
    return {
        name: value
        for name, value in vars(module).items()
        if callable(value) and not name.startswith("_")
    }


def public_dataclasses(module):
    result = {}

    for name, value in vars(module).items():
        if name.startswith("_"):
            continue
        if inspect.isclass(value) and dataclasses.is_dataclass(value):
            result[name] = value

    return result


def instantiate_dataclass(cls):
    signature = inspect.signature(cls)
    kwargs = {}

    for name, parameter in signature.parameters.items():
        if parameter.default is not inspect.Parameter.empty:
            continue

        annotation = parameter.annotation

        if annotation is bool:
            kwargs[name] = False
        elif annotation in (int, float):
            kwargs[name] = annotation()
        elif annotation is str:
            kwargs[name] = ""
        elif name.endswith("_s") or name.endswith("_hz"):
            kwargs[name] = 1.0
        elif "state" in name:
            kwargs[name] = PowerState.OK
        else:
            raise TypeError(f"Cannot auto-fill required parameter {name}")

    return cls(**kwargs)


def test_power_policy_module_imports():
    assert power_policy is not None


def test_power_policy_exposes_policy_api():
    names = set(public_callables(power_policy))

    assert any("policy" in name.lower() for name in names)
    assert any("shutdown" in name.lower() for name in names)


def test_power_policy_has_dataclass_models():
    dataclass_names = set(public_dataclasses(power_policy))

    assert dataclass_names
    assert any("policy" in name.lower() for name in dataclass_names)


def test_automatic_shutdown_defaults_to_false():
    checked = []

    for name, cls in public_dataclasses(power_policy).items():
        fields = getattr(cls, "__dataclass_fields__", {})

        if "automatic_shutdown_enabled" not in fields:
            continue

        instance = instantiate_dataclass(cls)
        checked.append(name)

        assert instance.automatic_shutdown_enabled is False

    assert checked, "No policy dataclass has automatic_shutdown_enabled"


def test_shutdown_request_is_not_enabled_by_default():
    checked = []

    for name, value in public_callables(power_policy).items():
        if "shutdown" not in name.lower():
            continue

        signature = inspect.signature(value)

        kwargs = {}
        supported = True

        for param_name, parameter in signature.parameters.items():
            if parameter.default is not inspect.Parameter.empty:
                continue

            if param_name == "automatic_shutdown_enabled":
                kwargs[param_name] = False
            elif param_name in {"state", "overall_state", "power_state"}:
                kwargs[param_name] = PowerState.CRITICAL
            elif param_name.endswith("_enabled"):
                kwargs[param_name] = False
            elif param_name.endswith("_s"):
                kwargs[param_name] = 1.0
            else:
                supported = False
                break

        if not supported:
            continue

        try:
            result = value(**kwargs)
        except TypeError:
            continue

        checked.append(name)

        if isinstance(result, bool):
            assert result is False

        if hasattr(result, "shutdown_requested"):
            assert result.shutdown_requested is False

        if hasattr(result, "request_shutdown"):
            assert result.request_shutdown is False

    assert checked, "No shutdown policy function could be checked"


@pytest.mark.parametrize(
    "state",
    [
        PowerState.OK,
        PowerState.LOW,
        PowerState.CRITICAL,
        PowerState.STALE,
        PowerState.ERROR,
    ],
)
def test_power_state_values_are_available_for_policy(state):
    assert isinstance(state.value, str)
    assert state.value
