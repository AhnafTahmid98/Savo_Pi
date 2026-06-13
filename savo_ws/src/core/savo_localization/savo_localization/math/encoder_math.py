#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Encoder math helpers for Robot Savo localization."""

from __future__ import annotations

import math
from dataclasses import dataclass

from savo_localization.constants import (
    DEFAULT_CPR,
    DEFAULT_DECODING_FACTOR,
    DEFAULT_GEAR_RATIO,
    DEFAULT_WHEEL_DIAMETER_M,
)


@dataclass(frozen=True)
class EncoderGeometry:
    wheel_diameter_m: float = DEFAULT_WHEEL_DIAMETER_M
    cpr: int = DEFAULT_CPR
    decoding: int = DEFAULT_DECODING_FACTOR
    gear_ratio: float = DEFAULT_GEAR_RATIO

    @property
    def wheel_radius_m(self) -> float:
        return self.wheel_diameter_m / 2.0

    @property
    def wheel_circumference_m(self) -> float:
        return wheel_circumference_m(self.wheel_diameter_m)

    @property
    def counts_per_wheel_rev(self) -> int:
        return counts_per_wheel_rev(
            cpr=self.cpr,
            decoding=self.decoding,
            gear_ratio=self.gear_ratio,
        )

    @property
    def metres_per_count(self) -> float:
        return metres_per_count(
            wheel_diameter_m=self.wheel_diameter_m,
            counts_per_rev=self.counts_per_wheel_rev,
        )


def valid_decoding_factor(decoding: int) -> bool:
    return int(decoding) in (1, 2, 4)


def require_valid_decoding_factor(decoding: int) -> int:
    value = int(decoding)

    if not valid_decoding_factor(value):
        raise ValueError("decoding must be one of: 1, 2, 4")

    return value


def wheel_circumference_m(wheel_diameter_m: float) -> float:
    diameter = float(wheel_diameter_m)

    if diameter <= 0.0:
        raise ValueError("wheel_diameter_m must be > 0.0")

    return math.pi * diameter


def counts_per_wheel_rev(
    cpr: int = DEFAULT_CPR,
    decoding: int = DEFAULT_DECODING_FACTOR,
    gear_ratio: float = DEFAULT_GEAR_RATIO,
) -> int:
    cpr_value = int(cpr)
    decoding_value = require_valid_decoding_factor(decoding)
    gear_value = float(gear_ratio)

    if cpr_value <= 0:
        raise ValueError("cpr must be > 0")

    if gear_value <= 0.0:
        raise ValueError("gear_ratio must be > 0.0")

    return int(round(cpr_value * decoding_value * gear_value))


def counts_to_revolutions(
    counts: int | float,
    counts_per_rev: int | float | None = None,
    *,
    cpr: int = DEFAULT_CPR,
    decoding: int = DEFAULT_DECODING_FACTOR,
    gear_ratio: float = DEFAULT_GEAR_RATIO,
) -> float:
    if counts_per_rev is None:
        counts_per_rev = counts_per_wheel_rev(
            cpr=cpr,
            decoding=decoding,
            gear_ratio=gear_ratio,
        )

    counts_per_rev_value = float(counts_per_rev)

    if counts_per_rev_value <= 0.0:
        raise ValueError("counts_per_rev must be > 0.0")

    return float(counts) / counts_per_rev_value


def revolutions_to_counts(
    revolutions: float,
    counts_per_rev: int | float | None = None,
    *,
    cpr: int = DEFAULT_CPR,
    decoding: int = DEFAULT_DECODING_FACTOR,
    gear_ratio: float = DEFAULT_GEAR_RATIO,
) -> int:
    if counts_per_rev is None:
        counts_per_rev = counts_per_wheel_rev(
            cpr=cpr,
            decoding=decoding,
            gear_ratio=gear_ratio,
        )

    counts_per_rev_value = float(counts_per_rev)

    if counts_per_rev_value <= 0.0:
        raise ValueError("counts_per_rev must be > 0.0")

    return int(round(float(revolutions) * counts_per_rev_value))


def metres_per_count(
    wheel_diameter_m: float = DEFAULT_WHEEL_DIAMETER_M,
    counts_per_rev: int | float | None = None,
    *,
    cpr: int = DEFAULT_CPR,
    decoding: int = DEFAULT_DECODING_FACTOR,
    gear_ratio: float = DEFAULT_GEAR_RATIO,
) -> float:
    if counts_per_rev is None:
        counts_per_rev = counts_per_wheel_rev(
            cpr=cpr,
            decoding=decoding,
            gear_ratio=gear_ratio,
        )

    counts_per_rev_value = float(counts_per_rev)

    if counts_per_rev_value <= 0.0:
        raise ValueError("counts_per_rev must be > 0.0")

    return wheel_circumference_m(wheel_diameter_m) / counts_per_rev_value


def counts_to_distance_m(
    counts: int | float,
    wheel_diameter_m: float = DEFAULT_WHEEL_DIAMETER_M,
    counts_per_rev: int | float | None = None,
    *,
    cpr: int = DEFAULT_CPR,
    decoding: int = DEFAULT_DECODING_FACTOR,
    gear_ratio: float = DEFAULT_GEAR_RATIO,
) -> float:
    return counts_to_revolutions(
        counts,
        counts_per_rev,
        cpr=cpr,
        decoding=decoding,
        gear_ratio=gear_ratio,
    ) * wheel_circumference_m(wheel_diameter_m)


def distance_m_to_counts(
    distance_m: float,
    wheel_diameter_m: float = DEFAULT_WHEEL_DIAMETER_M,
    counts_per_rev: int | float | None = None,
    *,
    cpr: int = DEFAULT_CPR,
    decoding: int = DEFAULT_DECODING_FACTOR,
    gear_ratio: float = DEFAULT_GEAR_RATIO,
) -> int:
    metres = metres_per_count(
        wheel_diameter_m=wheel_diameter_m,
        counts_per_rev=counts_per_rev,
        cpr=cpr,
        decoding=decoding,
        gear_ratio=gear_ratio,
    )

    return int(round(float(distance_m) / metres))


def delta_count(previous_count: int, current_count: int) -> int:
    return int(current_count) - int(previous_count)


def direction_from_delta(delta: int | float) -> int:
    value = float(delta)

    if value > 0.0:
        return 1

    if value < 0.0:
        return -1

    return 0


def direction_symbol(direction: int | float) -> str:
    value = direction_from_delta(direction)

    if value > 0:
        return "+"

    if value < 0:
        return "-"

    return "0"


def apply_encoder_inversion(value: int | float, inverted: bool) -> float:
    return -float(value) if inverted else float(value)


def delta_counts_to_speed_mps(
    delta_counts: int | float,
    dt_s: float,
    wheel_diameter_m: float = DEFAULT_WHEEL_DIAMETER_M,
    counts_per_rev: int | float | None = None,
    *,
    cpr: int = DEFAULT_CPR,
    decoding: int = DEFAULT_DECODING_FACTOR,
    gear_ratio: float = DEFAULT_GEAR_RATIO,
) -> float:
    if dt_s <= 0.0:
        raise ValueError("dt_s must be > 0.0")

    distance_m = counts_to_distance_m(
        delta_counts,
        wheel_diameter_m=wheel_diameter_m,
        counts_per_rev=counts_per_rev,
        cpr=cpr,
        decoding=decoding,
        gear_ratio=gear_ratio,
    )

    return distance_m / float(dt_s)


def counts_per_second_to_speed_mps(
    counts_per_second: float,
    wheel_diameter_m: float = DEFAULT_WHEEL_DIAMETER_M,
    counts_per_rev: int | float | None = None,
    *,
    cpr: int = DEFAULT_CPR,
    decoding: int = DEFAULT_DECODING_FACTOR,
    gear_ratio: float = DEFAULT_GEAR_RATIO,
) -> float:
    return counts_to_distance_m(
        counts_per_second,
        wheel_diameter_m=wheel_diameter_m,
        counts_per_rev=counts_per_rev,
        cpr=cpr,
        decoding=decoding,
        gear_ratio=gear_ratio,
    )


def speed_mps_to_counts_per_second(
    speed_mps: float,
    wheel_diameter_m: float = DEFAULT_WHEEL_DIAMETER_M,
    counts_per_rev: int | float | None = None,
    *,
    cpr: int = DEFAULT_CPR,
    decoding: int = DEFAULT_DECODING_FACTOR,
    gear_ratio: float = DEFAULT_GEAR_RATIO,
) -> float:
    metres = metres_per_count(
        wheel_diameter_m=wheel_diameter_m,
        counts_per_rev=counts_per_rev,
        cpr=cpr,
        decoding=decoding,
        gear_ratio=gear_ratio,
    )

    return float(speed_mps) / metres


def wheel_angular_to_linear_mps(
    angular_rad_s: float,
    wheel_diameter_m: float = DEFAULT_WHEEL_DIAMETER_M,
) -> float:
    return float(angular_rad_s) * (float(wheel_diameter_m) / 2.0)


def wheel_linear_to_angular_rad_s(
    linear_mps: float,
    wheel_diameter_m: float = DEFAULT_WHEEL_DIAMETER_M,
) -> float:
    radius = float(wheel_diameter_m) / 2.0

    if radius <= 0.0:
        raise ValueError("wheel radius must be > 0.0")

    return float(linear_mps) / radius


__all__ = [
    "EncoderGeometry",
    "valid_decoding_factor",
    "require_valid_decoding_factor",
    "wheel_circumference_m",
    "counts_per_wheel_rev",
    "counts_to_revolutions",
    "revolutions_to_counts",
    "metres_per_count",
    "counts_to_distance_m",
    "distance_m_to_counts",
    "delta_count",
    "direction_from_delta",
    "direction_symbol",
    "apply_encoder_inversion",
    "delta_counts_to_speed_mps",
    "counts_per_second_to_speed_mps",
    "speed_mps_to_counts_per_second",
    "wheel_angular_to_linear_mps",
    "wheel_linear_to_angular_rad_s",
]
