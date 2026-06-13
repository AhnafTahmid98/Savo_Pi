#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Encoder math helpers for Robot Savo four-wheel odometry. No ROS imports."""

from __future__ import annotations

import math
from dataclasses import dataclass

from savo_localization.constants import (
    DEFAULT_ENCODER_CPR,
    DEFAULT_ENCODER_DECODING,
    DEFAULT_GEAR_RATIO,
    DEFAULT_WHEEL_DIAMETER_M,
)


@dataclass(frozen=True)
class EncoderGeometry:
    wheel_diameter_m: float = DEFAULT_WHEEL_DIAMETER_M
    cpr: int = DEFAULT_ENCODER_CPR
    decoding: int = DEFAULT_ENCODER_DECODING
    gear_ratio: float = DEFAULT_GEAR_RATIO

    def validate(self) -> None:
        if self.wheel_diameter_m <= 0.0:
            raise ValueError(
                f"wheel_diameter_m must be > 0.0, got {self.wheel_diameter_m}"
            )

        if self.cpr <= 0:
            raise ValueError(f"cpr must be > 0, got {self.cpr}")

        if self.decoding <= 0:
            raise ValueError(f"decoding must be > 0, got {self.decoding}")

        if self.gear_ratio <= 0.0:
            raise ValueError(f"gear_ratio must be > 0.0, got {self.gear_ratio}")

    @property
    def wheel_circumference_m(self) -> float:
        return math.pi * self.wheel_diameter_m

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


def counts_per_wheel_rev(
    *,
    cpr: int,
    decoding: int,
    gear_ratio: float = 1.0,
) -> int:
    cpr = int(cpr)
    decoding = int(decoding)
    gear_ratio = float(gear_ratio)

    if cpr <= 0:
        raise ValueError(f"cpr must be > 0, got {cpr}")

    if decoding <= 0:
        raise ValueError(f"decoding must be > 0, got {decoding}")

    if gear_ratio <= 0.0:
        raise ValueError(f"gear_ratio must be > 0.0, got {gear_ratio}")

    return max(1, int(cpr * decoding * gear_ratio))


def wheel_circumference_m(wheel_diameter_m: float) -> float:
    wheel_diameter_m = float(wheel_diameter_m)

    if wheel_diameter_m <= 0.0:
        raise ValueError(f"wheel_diameter_m must be > 0.0, got {wheel_diameter_m}")

    return math.pi * wheel_diameter_m


def metres_per_count(
    *,
    wheel_diameter_m: float,
    counts_per_rev: int,
) -> float:
    counts_per_rev = int(counts_per_rev)

    if counts_per_rev <= 0:
        raise ValueError(f"counts_per_rev must be > 0, got {counts_per_rev}")

    return wheel_circumference_m(wheel_diameter_m) / float(counts_per_rev)


def counts_to_revolutions(
    counts: int | float,
    *,
    counts_per_rev: int,
) -> float:
    counts_per_rev = int(counts_per_rev)

    if counts_per_rev <= 0:
        raise ValueError(f"counts_per_rev must be > 0, got {counts_per_rev}")

    return float(counts) / float(counts_per_rev)


def revolutions_to_counts(
    revolutions: float,
    *,
    counts_per_rev: int,
) -> int:
    counts_per_rev = int(counts_per_rev)

    if counts_per_rev <= 0:
        raise ValueError(f"counts_per_rev must be > 0, got {counts_per_rev}")

    return int(round(float(revolutions) * float(counts_per_rev)))


def counts_to_distance_m(
    counts: int | float,
    *,
    wheel_diameter_m: float,
    counts_per_rev: int,
) -> float:
    return counts_to_revolutions(
        counts,
        counts_per_rev=counts_per_rev,
    ) * wheel_circumference_m(wheel_diameter_m)


def distance_m_to_counts(
    distance_m: float,
    *,
    wheel_diameter_m: float,
    counts_per_rev: int,
) -> int:
    metres_per_tick = metres_per_count(
        wheel_diameter_m=wheel_diameter_m,
        counts_per_rev=counts_per_rev,
    )

    return int(round(float(distance_m) / metres_per_tick))


def delta_count(
    current_count: int,
    previous_count: int,
) -> int:
    return int(current_count) - int(previous_count)


def delta_counts_to_speed_mps(
    delta_counts: int | float,
    dt_s: float,
    *,
    wheel_diameter_m: float,
    counts_per_rev: int,
) -> float:
    dt_s = max(float(dt_s), 1e-9)
    distance_m = counts_to_distance_m(
        delta_counts,
        wheel_diameter_m=wheel_diameter_m,
        counts_per_rev=counts_per_rev,
    )

    return distance_m / dt_s


def counts_per_second_to_speed_mps(
    counts_per_second: float,
    *,
    wheel_diameter_m: float,
    counts_per_rev: int,
) -> float:
    rev_per_second = counts_to_revolutions(
        counts_per_second,
        counts_per_rev=counts_per_rev,
    )

    return rev_per_second * wheel_circumference_m(wheel_diameter_m)


def speed_mps_to_counts_per_second(
    speed_mps: float,
    *,
    wheel_diameter_m: float,
    counts_per_rev: int,
) -> float:
    metres_per_tick = metres_per_count(
        wheel_diameter_m=wheel_diameter_m,
        counts_per_rev=counts_per_rev,
    )

    return float(speed_mps) / metres_per_tick


def wheel_linear_to_angular_rad_s(
    speed_mps: float,
    *,
    wheel_diameter_m: float,
) -> float:
    radius_m = float(wheel_diameter_m) / 2.0

    if radius_m <= 0.0:
        raise ValueError(f"wheel_diameter_m must be > 0.0, got {wheel_diameter_m}")

    return float(speed_mps) / radius_m


def wheel_angular_to_linear_mps(
    angular_rad_s: float,
    *,
    wheel_diameter_m: float,
) -> float:
    radius_m = float(wheel_diameter_m) / 2.0

    if radius_m <= 0.0:
        raise ValueError(f"wheel_diameter_m must be > 0.0, got {wheel_diameter_m}")

    return float(angular_rad_s) * radius_m


def apply_encoder_inversion(
    count_or_delta: int | float,
    *,
    inverted: bool,
) -> int | float:
    return -count_or_delta if bool(inverted) else count_or_delta


def direction_from_delta(delta: int | float) -> int:
    delta = float(delta)

    if delta > 0.0:
        return 1

    if delta < 0.0:
        return -1

    return 0


def direction_symbol(direction: int | float) -> str:
    direction = direction_from_delta(direction)

    if direction > 0:
        return "→"

    if direction < 0:
        return "←"

    return "•"


def valid_decoding_factor(decoding: int) -> bool:
    return int(decoding) in (1, 2, 4)


def require_valid_decoding_factor(decoding: int) -> None:
    if not valid_decoding_factor(decoding):
        raise ValueError(f"decoding must be one of 1, 2, or 4, got {decoding}")