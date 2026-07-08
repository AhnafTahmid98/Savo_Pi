import dataclasses
import enum
import inspect

from savo_power.models import power_health


def test_power_health_module_exports_core_symbols():
    for name in [
        "PowerHealthLevel",
        "PowerHealthConfig",
        "PowerHealthInput",
        "PowerHealthResult",
        "evaluate_power_health",
    ]:
        assert hasattr(power_health, name), name


def test_power_health_level_is_enum_like():
    level_cls = power_health.PowerHealthLevel

    assert issubclass(level_cls, enum.Enum)

    values = {
        str(member.value).lower()
        for member in level_cls
    }

    assert values
    assert "ok" in values or "healthy" in values
    assert "critical" in values or "error" in values


def test_power_health_dataclasses_have_fields():
    for name in [
        "PowerHealthConfig",
        "PowerHealthInput",
        "PowerHealthResult",
    ]:
        cls = getattr(power_health, name)

        assert dataclasses.is_dataclass(cls), name
        assert dataclasses.fields(cls), name


def test_power_health_config_keeps_shutdown_disabled_by_default_when_present():
    config_cls = power_health.PowerHealthConfig
    config = config_cls()

    if hasattr(config, "automatic_shutdown_enabled"):
        assert config.automatic_shutdown_enabled is False


def test_power_health_result_exposes_level_or_ok_state():
    result_cls = power_health.PowerHealthResult
    field_names = {
        field.name
        for field in dataclasses.fields(result_cls)
    }

    assert (
        "level" in field_names
        or "state" in field_names
        or "ok" in field_names
    )


def test_evaluate_power_health_is_callable_and_typed():
    func = power_health.evaluate_power_health

    assert callable(func)

    signature = inspect.signature(func)

    assert signature.parameters
    assert signature.return_annotation is not inspect.Signature.empty


def test_power_health_config_has_timing_or_shutdown_safety_field():
    config_cls = power_health.PowerHealthConfig
    field_names = {
        field.name
        for field in dataclasses.fields(config_cls)
    }

    assert (
        "stale_timeout_s" in field_names
        or "automatic_shutdown_enabled" in field_names
        or "shutdown_grace_s" in field_names
    )
