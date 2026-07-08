import dataclasses
from enum import Enum

import pytest

import savo_power.models.kit_battery_reading as kit_model
import savo_power.models.power_health as health_model
import savo_power.models.power_status as status_model
import savo_power.models.ups_reading as ups_model


def enum_names(enum_cls):
    return {item.name for item in enum_cls}


def enum_values(enum_cls):
    return {item.value for item in enum_cls}


def field_names(dataclass_cls):
    return {field.name for field in dataclasses.fields(dataclass_cls)}


def test_power_state_has_required_safety_states():
    assert issubclass(status_model.PowerState, Enum)

    names = enum_names(status_model.PowerState)

    assert "OK" in names
    assert "LOW" in names
    assert "CRITICAL" in names
    assert "ERROR" in names
    assert "UNKNOWN" in names


def test_power_state_values_are_strings():
    for state in status_model.PowerState:
        assert isinstance(state.value, str)
        assert state.value


def test_battery_source_has_robot_sources():
    assert issubclass(status_model.BatterySource, Enum)

    names = enum_names(status_model.BatterySource)

    assert "CORE_UPS" in names
    assert "EDGE_UPS" in names
    assert "BASE_BATTERY" in names


def test_battery_source_values_are_strings():
    for source in status_model.BatterySource:
        assert isinstance(source.value, str)
        assert source.value


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("ok", "OK"),
        ("low", "LOW"),
        ("critical", "CRITICAL"),
        ("error", "ERROR"),
        ("unknown", "UNKNOWN"),
    ],
)
def test_power_state_normalizer_accepts_text(text, expected):
    state = status_model.normalize_power_state(text)

    assert state is getattr(status_model.PowerState, expected)


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("core_ups", "CORE_UPS"),
        ("edge_ups", "EDGE_UPS"),
        ("base_battery", "BASE_BATTERY"),
    ],
)
def test_battery_source_normalizer_accepts_text(text, expected):
    source = status_model.normalize_battery_source(text)

    assert source is getattr(status_model.BatterySource, expected)


def test_power_status_dataclasses_exist():
    assert dataclasses.is_dataclass(status_model.PowerSourceStatus)
    assert dataclasses.is_dataclass(status_model.PowerStatusSummary)


def test_power_source_status_has_core_fields():
    fields = field_names(status_model.PowerSourceStatus)

    assert "source" in fields
    assert "state" in fields
    assert "expected" in fields
    assert "seen" in fields
    assert "stale" in fields


def test_power_status_summary_has_core_fields():
    fields = field_names(status_model.PowerStatusSummary)

    assert "overall_state" in fields
    assert "core_ups" in fields
    assert "edge_ups" in fields
    assert "base_battery" in fields
    assert "shutdown_requested" in fields


def test_ups_reading_dataclass_has_voltage_and_capacity():
    assert dataclasses.is_dataclass(ups_model.UpsReading)

    fields = field_names(ups_model.UpsReading)

    assert "source" in fields
    assert "state" in fields
    assert "voltage_v" in fields
    assert "capacity_pct" in fields


def test_kit_battery_reading_dataclass_has_voltage_and_soc():
    assert dataclasses.is_dataclass(kit_model.KitBatteryReading)

    fields = field_names(kit_model.KitBatteryReading)

    assert "source" in fields
    assert "state" in fields
    assert "voltage_v" in fields
    assert "soc_pct" in fields


def test_power_health_level_has_required_values():
    assert issubclass(health_model.PowerHealthLevel, Enum)

    names = enum_names(health_model.PowerHealthLevel)

    assert "OK" in names
    assert "WARN" in names
    assert "ERROR" in names
    assert "UNKNOWN" in names


def test_power_health_level_values_are_strings():
    for level in health_model.PowerHealthLevel:
        assert isinstance(level.value, str)
        assert level.value


def test_power_health_result_model_exists():
    assert hasattr(health_model, "PowerHealthResult")
    assert dataclasses.is_dataclass(health_model.PowerHealthResult)


def test_model_modules_export_public_factories():
    assert any(name.startswith("make_ups") for name in vars(ups_model))
    assert any(name.startswith("make_kit") for name in vars(kit_model))
    assert any("health" in name.lower() for name in vars(health_model))


def test_enum_values_are_unique():
    assert len(enum_values(status_model.PowerState)) == len(list(status_model.PowerState))
    assert len(enum_values(status_model.BatterySource)) == len(list(status_model.BatterySource))
    assert len(enum_values(health_model.PowerHealthLevel)) == len(list(health_model.PowerHealthLevel))
