import math
import inspect

import pytest

from savo_power.models.kit_battery_reading import (
    ads7830_battery_voltage_from_byte,
    ads7830_channel_command,
    estimate_linear_soc_pct,
)


def call_battery_voltage_from_byte(raw_byte, pcb_version):
    signature = inspect.signature(ads7830_battery_voltage_from_byte)

    if "pcb_version" in signature.parameters:
        return ads7830_battery_voltage_from_byte(
            raw_byte,
            pcb_version=pcb_version,
        )

    if len(signature.parameters) == 2:
        return ads7830_battery_voltage_from_byte(raw_byte, pcb_version)

    return ads7830_battery_voltage_from_byte(raw_byte)


def call_soc(voltage_v, empty_voltage_v, full_voltage_v):
    signature = inspect.signature(estimate_linear_soc_pct)

    kwargs = {
        "voltage_v": voltage_v,
        "empty_voltage_v": empty_voltage_v,
        "full_voltage_v": full_voltage_v,
    }

    accepted = {
        name: value
        for name, value in kwargs.items()
        if name in signature.parameters
    }

    if accepted:
        return estimate_linear_soc_pct(**accepted)

    return estimate_linear_soc_pct(
        voltage_v,
        empty_voltage_v,
        full_voltage_v,
    )


def test_ads7830_channel_2_command_is_locked():
    assert ads7830_channel_command(2) == 0x94


@pytest.mark.parametrize(
    ("channel", "expected"),
    [
        (0, 0x84),
        (1, 0xC4),
        (2, 0x94),
        (3, 0xD4),
        (4, 0xA4),
        (5, 0xE4),
        (6, 0xB4),
        (7, 0xF4),
    ],
)
def test_ads7830_command_for_all_channels(channel, expected):
    assert ads7830_channel_command(channel) == expected


@pytest.mark.parametrize("channel", [-1, 8])
def test_ads7830_rejects_invalid_channel(channel):
    with pytest.raises(ValueError):
        ads7830_channel_command(channel)


def test_ads7830_pcb_v2_voltage_formula_full_scale():
    voltage = call_battery_voltage_from_byte(
        raw_byte=255,
        pcb_version="v2",
    )

    assert math.isclose(voltage, 10.4, rel_tol=0.0, abs_tol=0.05)


def test_base_battery_soc_linear_mapping():
    assert math.isclose(call_soc(6.40, 6.40, 8.40), 0.0, abs_tol=0.01)
    assert math.isclose(call_soc(7.40, 6.40, 8.40), 50.0, abs_tol=0.01)
    assert math.isclose(call_soc(8.40, 6.40, 8.40), 100.0, abs_tol=0.01)


def test_base_battery_soc_is_clamped():
    assert call_soc(6.0, 6.40, 8.40) == 0.0
    assert call_soc(9.0, 6.40, 8.40) == 100.0
