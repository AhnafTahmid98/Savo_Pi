#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Encoder state models for Robot Savo four-wheel odometry. No ROS imports."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable

from savo_localization.constants import (
    DEFAULT_MAX_ILLEGAL_TRANSITIONS,
    DEFAULT_MIN_ENCODER_RATE_CPS,
    ENCODER_MODEL_QUADRATURE,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_WARN,
    WHEEL_FL,
    WHEEL_FR,
    WHEEL_ORDER,
    WHEEL_RL,
    WHEEL_RR,
)
from savo_localization.math.encoder_math import (
    direction_from_delta,
    direction_symbol,
)


@dataclass
class EncoderWheelSnapshot:
    name: str = WHEEL_FL
    count: int = 0
    delta_count: int = 0
    counts_per_second: float = 0.0
    speed_mps: float = 0.0
    illegal_transitions: int = 0
    direction: int | None = None
    active: bool | None = None

    def __post_init__(self) -> None:
        if self.direction is None:
            self.direction = direction_from_delta(self.delta_count)

        if self.active is None:
            self.active = (
                self.delta_count != 0
                or abs(float(self.counts_per_second)) > 0.0
                or abs(float(self.speed_mps)) > 0.0
            )

    @property
    def direction_symbol(self) -> str:
        return direction_symbol(int(self.direction or 0))

    @property
    def moving_forward(self) -> bool:
        return int(self.direction or 0) > 0

    @property
    def moving_reverse(self) -> bool:
        return int(self.direction or 0) < 0

    @property
    def stopped(self) -> bool:
        return int(self.direction or 0) == 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "count": int(self.count),
            "delta_count": int(self.delta_count),
            "counts_per_second": float(self.counts_per_second),
            "speed_mps": float(self.speed_mps),
            "direction": int(self.direction or 0),
            "direction_symbol": self.direction_symbol,
            "illegal_transitions": int(self.illegal_transitions),
            "active": bool(self.active),
            "moving_forward": self.moving_forward,
            "moving_reverse": self.moving_reverse,
            "stopped": self.stopped,
        }


@dataclass
class WheelEncoderState:
    name: str
    count: int = 0
    previous_count: int = 0
    delta_count: int = 0
    counts_per_second: float = 0.0
    speed_mps: float = 0.0
    direction: int = 0
    illegal_transitions: int = 0
    active: bool = False
    last_update_s: float | None = None

    def mark_count(
        self,
        *,
        count: int,
        dt_s: float,
        speed_mps: float = 0.0,
        illegal_transitions: int = 0,
        stamp_s: float | None = None,
    ) -> None:
        if dt_s <= 0.0:
            raise ValueError("dt_s must be > 0.0")

        self.previous_count = int(self.count)
        self.count = int(count)
        self.delta_count = self.count - self.previous_count
        self.counts_per_second = float(self.delta_count) / float(dt_s)
        self.speed_mps = float(speed_mps)
        self.direction = direction_from_delta(self.delta_count)
        self.illegal_transitions = int(illegal_transitions)
        self.active = self.delta_count != 0 or abs(self.speed_mps) > 0.0
        self.last_update_s = stamp_s

    def update(
        self,
        *,
        count: int,
        dt_s: float,
        speed_mps: float = 0.0,
        stamp_s: float | None = None,
        illegal_transitions: int | None = None,
    ) -> None:
        self.mark_count(
            count=count,
            dt_s=dt_s,
            speed_mps=speed_mps,
            illegal_transitions=(
                self.illegal_transitions
                if illegal_transitions is None
                else illegal_transitions
            ),
            stamp_s=stamp_s,
        )

    def snapshot(self) -> EncoderWheelSnapshot:
        return EncoderWheelSnapshot(
            name=self.name,
            count=self.count,
            delta_count=self.delta_count,
            counts_per_second=self.counts_per_second,
            speed_mps=self.speed_mps,
            illegal_transitions=self.illegal_transitions,
            direction=self.direction,
            active=self.active,
        )

    @property
    def direction_symbol(self) -> str:
        return direction_symbol(self.direction)

    @property
    def moving_forward(self) -> bool:
        return self.direction > 0

    @property
    def moving_reverse(self) -> bool:
        return self.direction < 0

    @property
    def stopped(self) -> bool:
        return self.direction == 0

    @property
    def has_illegal_transitions(self) -> bool:
        return self.illegal_transitions > 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "count": int(self.count),
            "previous_count": int(self.previous_count),
            "delta_count": int(self.delta_count),
            "counts_per_second": float(self.counts_per_second),
            "speed_mps": float(self.speed_mps),
            "direction": int(self.direction),
            "direction_symbol": self.direction_symbol,
            "illegal_transitions": int(self.illegal_transitions),
            "active": bool(self.active),
            "last_update_s": self.last_update_s,
            "moving_forward": self.moving_forward,
            "moving_reverse": self.moving_reverse,
            "stopped": self.stopped,
        }


def _default_wheels() -> tuple[EncoderWheelSnapshot, ...]:
    return (
        EncoderWheelSnapshot(name=WHEEL_FL),
        EncoderWheelSnapshot(name=WHEEL_FR),
        EncoderWheelSnapshot(name=WHEEL_RL),
        EncoderWheelSnapshot(name=WHEEL_RR),
    )


@dataclass
class EncoderSample:
    stamp_s: float = 0.0
    dt_s: float = 0.0
    wheels: tuple[EncoderWheelSnapshot, ...] = field(default_factory=_default_wheels)
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    omega_rad_s: float = 0.0

    def __post_init__(self) -> None:
        self.wheels = tuple(self.wheels)

    @property
    def wheel_map(self) -> dict[str, EncoderWheelSnapshot]:
        return {
            wheel.name: wheel
            for wheel in self.wheels
        }

    @property
    def fl(self) -> EncoderWheelSnapshot:
        return self.wheel_map.get(WHEEL_FL, EncoderWheelSnapshot(name=WHEEL_FL))

    @property
    def fr(self) -> EncoderWheelSnapshot:
        return self.wheel_map.get(WHEEL_FR, EncoderWheelSnapshot(name=WHEEL_FR))

    @property
    def rl(self) -> EncoderWheelSnapshot:
        return self.wheel_map.get(WHEEL_RL, EncoderWheelSnapshot(name=WHEEL_RL))

    @property
    def rr(self) -> EncoderWheelSnapshot:
        return self.wheel_map.get(WHEEL_RR, EncoderWheelSnapshot(name=WHEEL_RR))

    @property
    def total_illegal_transitions(self) -> int:
        return sum(int(wheel.illegal_transitions) for wheel in self.wheels)

    @property
    def active_wheel_count(self) -> int:
        return sum(1 for wheel in self.wheels if wheel.active)

    @property
    def all_wheels_active(self) -> bool:
        return self.active_wheel_count == len(WHEEL_ORDER)

    @property
    def linear_speed_mps(self) -> float:
        return math.hypot(float(self.vx_mps), float(self.vy_mps))

    def to_dict(self) -> dict[str, Any]:
        return {
            "stamp_s": float(self.stamp_s),
            "dt_s": float(self.dt_s),
            "wheels": {
                wheel.name: wheel.to_dict()
                for wheel in self.wheels
            },
            "active_wheel_count": self.active_wheel_count,
            "all_wheels_active": self.all_wheels_active,
            "total_illegal_transitions": self.total_illegal_transitions,
            "vx_mps": float(self.vx_mps),
            "vy_mps": float(self.vy_mps),
            "omega_rad_s": float(self.omega_rad_s),
            "linear_speed_mps": self.linear_speed_mps,
        }


@dataclass
class EncoderHealthState:
    status: str = STATUS_UNKNOWN
    message: str = "encoder state unknown"
    reasons: list[str] = field(default_factory=list)
    hardware_ok: bool = False
    data_ok: bool = False
    all_wheels_seen: bool = False
    illegal_transition_ok: bool = True
    rate_ok: bool = True
    active_wheel_count: int = 0
    total_illegal_transitions: int = 0
    min_counts_per_second: float = 0.0
    max_counts_per_second: float = 0.0

    @property
    def ready(self) -> bool:
        return (
            self.status == STATUS_OK
            and self.hardware_ok
            and self.data_ok
            and self.all_wheels_seen
            and self.illegal_transition_ok
            and self.rate_ok
        )

    @property
    def ok(self) -> bool:
        return self.ready

    def mark_ok(self, message: str = "encoders healthy") -> None:
        self.status = STATUS_OK
        self.message = message
        self.reasons.clear()
        self.hardware_ok = True
        self.data_ok = True
        self.all_wheels_seen = True
        self.illegal_transition_ok = True
        self.rate_ok = True

    def mark_warn(self, message: str, reasons: list[str] | None = None) -> None:
        self.status = STATUS_WARN
        self.message = message
        self.reasons = list(reasons or [])
        self.hardware_ok = True
        self.data_ok = True

    def mark_error(self, message: str, reasons: list[str] | None = None) -> None:
        self.status = STATUS_ERROR
        self.message = message
        self.reasons = list(reasons or [])
        self.hardware_ok = False
        self.data_ok = False

    def mark_stale(self, message: str, reasons: list[str] | None = None) -> None:
        self.status = STATUS_STALE
        self.message = message
        self.reasons = list(reasons or [])
        self.data_ok = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "message": self.message,
            "reasons": list(self.reasons),
            "hardware_ok": self.hardware_ok,
            "data_ok": self.data_ok,
            "all_wheels_seen": self.all_wheels_seen,
            "illegal_transition_ok": self.illegal_transition_ok,
            "rate_ok": self.rate_ok,
            "active_wheel_count": self.active_wheel_count,
            "total_illegal_transitions": self.total_illegal_transitions,
            "min_counts_per_second": self.min_counts_per_second,
            "max_counts_per_second": self.max_counts_per_second,
            "ready": self.ready,
            "ok": self.ok,
        }


@dataclass
class EncoderState:
    model: str = ENCODER_MODEL_QUADRATURE
    fl: WheelEncoderState = field(default_factory=lambda: WheelEncoderState(WHEEL_FL))
    fr: WheelEncoderState = field(default_factory=lambda: WheelEncoderState(WHEEL_FR))
    rl: WheelEncoderState = field(default_factory=lambda: WheelEncoderState(WHEEL_RL))
    rr: WheelEncoderState = field(default_factory=lambda: WheelEncoderState(WHEEL_RR))
    sample_count: int = 0
    last_sample: EncoderSample | None = None
    last_sample_age_s: float | None = None
    health: EncoderHealthState = field(default_factory=EncoderHealthState)

    @property
    def wheels(self) -> tuple[WheelEncoderState, WheelEncoderState, WheelEncoderState, WheelEncoderState]:
        return (self.fl, self.fr, self.rl, self.rr)

    @property
    def wheel_map(self) -> dict[str, WheelEncoderState]:
        return {
            wheel.name: wheel
            for wheel in self.wheels
        }

    @property
    def ready(self) -> bool:
        return self.health.ready

    def mark_sample(
        self,
        sample: EncoderSample,
        *,
        now_s: float | None = None,
    ) -> None:
        self.last_sample = sample
        self.sample_count += 1

        if now_s is None:
            self.last_sample_age_s = 0.0
        else:
            self.last_sample_age_s = max(
                0.0,
                float(now_s) - float(sample.stamp_s),
            )

        self.health.active_wheel_count = sample.active_wheel_count
        self.health.total_illegal_transitions = sample.total_illegal_transitions
        self.health.all_wheels_seen = set(sample.wheel_map.keys()) == set(WHEEL_ORDER)
        self.health.illegal_transition_ok = (
            sample.total_illegal_transitions <= DEFAULT_MAX_ILLEGAL_TRANSITIONS
        )

        rates = [abs(wheel.counts_per_second) for wheel in sample.wheels]
        self.health.min_counts_per_second = min(rates) if rates else 0.0
        self.health.max_counts_per_second = max(rates) if rates else 0.0
        self.health.rate_ok = (
            sample.active_wheel_count == 0
            or self.health.max_counts_per_second >= DEFAULT_MIN_ENCODER_RATE_CPS
        )

        if self.health.all_wheels_seen and self.health.illegal_transition_ok:
            self.health.mark_ok()
        else:
            reasons: list[str] = []
            if not self.health.all_wheels_seen:
                reasons.append("not all wheel encoders are present")
            if not self.health.illegal_transition_ok:
                reasons.append("too many illegal encoder transitions")
            self.health.mark_warn("encoder sample usable with notes", reasons)

        self.health.active_wheel_count = sample.active_wheel_count
        self.health.total_illegal_transitions = sample.total_illegal_transitions

    def mark_stale(self, age_s: float) -> None:
        self.last_sample_age_s = float(age_s)
        self.health.mark_stale(
            "encoder state stale",
            [f"last sample age_s={float(age_s):.3f}"],
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "model": self.model,
            "ready": self.ready,
            "sample_count": self.sample_count,
            "last_sample_age_s": self.last_sample_age_s,
            "health": self.health.to_dict(),
            "wheels": {
                wheel.name: wheel.to_dict()
                for wheel in self.wheels
            },
            "last_sample": (
                self.last_sample.to_dict()
                if self.last_sample is not None
                else None
            ),
        }


def make_wheel_snapshot(
    *,
    name: str,
    count: int = 0,
    delta_count: int = 0,
    counts_per_second: float = 0.0,
    speed_mps: float = 0.0,
    illegal_transitions: int = 0,
    direction: int | None = None,
    active: bool | None = None,
) -> EncoderWheelSnapshot:
    return EncoderWheelSnapshot(
        name=name,
        count=count,
        delta_count=delta_count,
        counts_per_second=counts_per_second,
        speed_mps=speed_mps,
        illegal_transitions=illegal_transitions,
        direction=direction,
        active=active,
    )


def make_encoder_sample(
    *,
    stamp_s: float = 0.0,
    dt_s: float = 0.0,
    wheels: Iterable[EncoderWheelSnapshot] | None = None,
    vx_mps: float = 0.0,
    vy_mps: float = 0.0,
    omega_rad_s: float = 0.0,
) -> EncoderSample:
    return EncoderSample(
        stamp_s=stamp_s,
        dt_s=dt_s,
        wheels=tuple(wheels) if wheels is not None else _default_wheels(),
        vx_mps=vx_mps,
        vy_mps=vy_mps,
        omega_rad_s=omega_rad_s,
    )


def make_encoder_sample_from_counts(
    *,
    stamp_s: float,
    dt_s: float,
    current_counts: dict[str, int],
    previous_counts: dict[str, int],
    wheel_speeds_mps: dict[str, float] | None = None,
    illegal_transitions: dict[str, int] | None = None,
    vx_mps: float = 0.0,
    vy_mps: float = 0.0,
    omega_rad_s: float = 0.0,
) -> EncoderSample:
    if dt_s <= 0.0:
        raise ValueError("dt_s must be > 0.0")

    wheel_speeds_mps = wheel_speeds_mps or {}
    illegal_transitions = illegal_transitions or {}

    wheels: list[EncoderWheelSnapshot] = []

    for wheel_name in WHEEL_ORDER:
        current = int(current_counts.get(wheel_name, 0))
        previous = int(previous_counts.get(wheel_name, current))
        delta = current - previous

        wheels.append(
            make_wheel_snapshot(
                name=wheel_name,
                count=current,
                delta_count=delta,
                counts_per_second=float(delta) / float(dt_s),
                speed_mps=float(wheel_speeds_mps.get(wheel_name, 0.0)),
                illegal_transitions=int(illegal_transitions.get(wheel_name, 0)),
            )
        )

    return make_encoder_sample(
        stamp_s=stamp_s,
        dt_s=dt_s,
        wheels=wheels,
        vx_mps=vx_mps,
        vy_mps=vy_mps,
        omega_rad_s=omega_rad_s,
    )


__all__ = [
    "WheelEncoderState",
    "EncoderWheelSnapshot",
    "EncoderSample",
    "EncoderHealthState",
    "EncoderState",
    "make_wheel_snapshot",
    "make_encoder_sample",
    "make_encoder_sample_from_counts",
]
