"""Clamp and range helpers for Robot Savo power Python tools."""

from __future__ import annotations

import math
from dataclasses import dataclass

from savo_power import constants as c


@dataclass(frozen=True)
class ClampRange:
    """Simple numeric range."""

    minimum: float
    maximum: float

    @property
    def low(self) -> float:
        return min(float(self.minimum), float(self.maximum))

    @property
    def high(self) -> float:
        return max(float(self.minimum), float(self.maximum))

    def contains(self, value: float | int) -> bool:
        return self.low <= float(value) <= self.high

    def clamp(self, value: float | int, *, default: float | None = None) -> float:
        return clamp_float(
            value,
            self.low,
            self.high,
            default=default,
        )

    def to_tuple(self) -> tuple[float, float]:
        return (self.low, self.high)


PERCENT_RANGE = ClampRange(0.0, 100.0)
UNIT_RANGE = ClampRange(0.0, 1.0)
RAW_BYTE_RANGE = ClampRange(0.0, 255.0)
RAW_WORD_RANGE = ClampRange(0.0, 65535.0)
I2C_7BIT_RANGE = ClampRange(0.0, 127.0)
PUBLISH_RATE_RANGE = ClampRange(c.MIN_PUBLISH_RATE_HZ, c.MAX_PUBLISH_RATE_HZ)


def is_finite_number(value: object) -> bool:
    """Return True when value can be converted to a finite float."""

    try:
        number = float(value)
    except (TypeError, ValueError):
        return False

    return math.isfinite(number)


def to_float(value: object, *, default: float = 0.0) -> float:
    """Convert value to finite float, otherwise return default."""

    if not is_finite_number(value):
        return float(default)

    return float(value)


def to_int(value: object, *, default: int = 0) -> int:
    """Convert value to int, supporting decimal and 0x string values."""

    if value is None:
        return int(default)

    try:
        if isinstance(value, str):
            return int(value, 0)

        return int(value)
    except (TypeError, ValueError):
        return int(default)


def normalized_limits(
    minimum: float | int,
    maximum: float | int,
) -> tuple[float, float]:
    """Return ordered low/high limits."""

    low = float(minimum)
    high = float(maximum)

    if low <= high:
        return low, high

    return high, low


def clamp_float(
    value: object,
    minimum: float | int,
    maximum: float | int,
    *,
    default: float | None = None,
) -> float:
    """Clamp value into a float range."""

    low, high = normalized_limits(minimum, maximum)

    if not is_finite_number(value):
        number = low if default is None else float(default)
    else:
        number = float(value)

    return max(low, min(high, number))


def clamp_int(
    value: object,
    minimum: int,
    maximum: int,
    *,
    default: int | None = None,
) -> int:
    """Clamp value into an int range."""

    low, high = normalized_limits(minimum, maximum)

    if value is None:
        number = int(low if default is None else default)
    else:
        number = to_int(value, default=int(low if default is None else default))

    return int(max(low, min(high, number)))


def clamp_optional_float(
    value: object,
    minimum: float | int,
    maximum: float | int,
) -> float | None:
    """Clamp optional float. Invalid/None values remain None."""

    if value is None or not is_finite_number(value):
        return None

    return clamp_float(value, minimum, maximum)


def clamp_optional_int(
    value: object,
    minimum: int,
    maximum: int,
) -> int | None:
    """Clamp optional int. None remains None."""

    if value is None:
        return None

    try:
        return clamp_int(value, minimum, maximum)
    except (TypeError, ValueError):
        return None


def clamp_percentage(value: object, *, default: float = 0.0) -> float:
    """Clamp value to 0..100 percentage."""

    return clamp_float(
        value,
        PERCENT_RANGE.low,
        PERCENT_RANGE.high,
        default=default,
    )


def clamp_soc_pct(value: object, *, default: float = 0.0) -> float:
    """Clamp battery state-of-charge percentage."""

    return clamp_percentage(value, default=default)


def clamp_unit_interval(value: object, *, default: float = 0.0) -> float:
    """Clamp value to 0..1."""

    return clamp_float(
        value,
        UNIT_RANGE.low,
        UNIT_RANGE.high,
        default=default,
    )


def clamp_raw_byte(value: object, *, default: int = 0) -> int:
    """Clamp raw 8-bit value."""

    return clamp_int(
        value,
        int(RAW_BYTE_RANGE.low),
        int(RAW_BYTE_RANGE.high),
        default=default,
    )


def clamp_raw_word(value: object, *, default: int = 0) -> int:
    """Clamp raw 16-bit value."""

    return clamp_int(
        value,
        int(RAW_WORD_RANGE.low),
        int(RAW_WORD_RANGE.high),
        default=default,
    )


def clamp_i2c_7bit_address(value: object, *, default: int = 0x00) -> int:
    """Clamp value to 7-bit I2C address range."""

    return clamp_int(
        value,
        int(I2C_7BIT_RANGE.low),
        int(I2C_7BIT_RANGE.high),
        default=default,
    )


def clamp_publish_rate_hz(
    value: object,
    *,
    default: float = c.DEFAULT_PUBLISH_RATE_HZ,
) -> float:
    """Clamp publish/sample rate to package-supported range."""

    rate = to_float(value, default=default)

    if rate <= 0.0:
        rate = float(default)

    return clamp_float(
        rate,
        PUBLISH_RATE_RANGE.low,
        PUBLISH_RATE_RANGE.high,
        default=default,
    )


def clamp_voltage_nonnegative(
    value: object,
    *,
    maximum_v: float = 60.0,
    default: float = 0.0,
) -> float:
    """Clamp voltage to a non-negative diagnostic range."""

    return clamp_float(
        value,
        0.0,
        maximum_v,
        default=default,
    )


def saturating_linear_map(
    value: object,
    *,
    in_min: float,
    in_max: float,
    out_min: float,
    out_max: float,
    default: float | None = None,
) -> float:
    """Map input range to output range with input and output saturation."""

    input_low, input_high = normalized_limits(in_min, in_max)
    output_low, output_high = normalized_limits(out_min, out_max)

    if input_high == input_low:
        return output_low if default is None else clamp_float(default, output_low, output_high)

    input_value = clamp_float(
        value,
        input_low,
        input_high,
        default=input_low if default is None else default,
    )

    fraction = (input_value - input_low) / (input_high - input_low)
    output_value = output_low + fraction * (output_high - output_low)

    return clamp_float(
        output_value,
        output_low,
        output_high,
    )


def value_in_range(
    value: object,
    minimum: float | int,
    maximum: float | int,
) -> bool:
    """Return True when value is finite and inside range."""

    if not is_finite_number(value):
        return False

    low, high = normalized_limits(minimum, maximum)
    number = float(value)

    return low <= number <= high


def percentage_in_range(value: object) -> bool:
    """Return True when value is valid 0..100 percentage."""

    return value_in_range(value, PERCENT_RANGE.low, PERCENT_RANGE.high)


def raw_byte_in_range(value: object) -> bool:
    """Return True when value is valid 8-bit raw byte."""

    return value_in_range(value, RAW_BYTE_RANGE.low, RAW_BYTE_RANGE.high)


def raw_word_in_range(value: object) -> bool:
    """Return True when value is valid 16-bit raw word."""

    return value_in_range(value, RAW_WORD_RANGE.low, RAW_WORD_RANGE.high)


__all__ = [
    "ClampRange",
    "I2C_7BIT_RANGE",
    "PERCENT_RANGE",
    "PUBLISH_RATE_RANGE",
    "RAW_BYTE_RANGE",
    "RAW_WORD_RANGE",
    "UNIT_RANGE",
    "clamp_float",
    "clamp_i2c_7bit_address",
    "clamp_int",
    "clamp_optional_float",
    "clamp_optional_int",
    "clamp_percentage",
    "clamp_publish_rate_hz",
    "clamp_raw_byte",
    "clamp_raw_word",
    "clamp_soc_pct",
    "clamp_unit_interval",
    "clamp_voltage_nonnegative",
    "is_finite_number",
    "normalized_limits",
    "percentage_in_range",
    "raw_byte_in_range",
    "raw_word_in_range",
    "saturating_linear_map",
    "to_float",
    "to_int",
    "value_in_range",
]
