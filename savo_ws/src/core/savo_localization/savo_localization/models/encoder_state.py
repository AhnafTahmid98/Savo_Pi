#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Encoder state models for Robot Savo four-wheel odometry. No ROS imports."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any

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
from savo_localization.math.encoder_math import direction_from_delta, direction_symbol


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

    def update(
        self,
        *,
        count: int,
        dt_s: float,
        speed_mps: float = 0.0,
        stamp_s: float | None = None,
        illegal_transitions: int | None = None,
    ) -> None:
        dt = max(float(dt_s), 1e-9)

        self.previous_count = int(self.count)
        self.count = int(count)
        self.delta_count = self.count - self.previous_count

        self.counts_per_second = float(self.delta_count) / dt
        self.speed_mps = float(speed_mps)
        self.direction = direction_from_delta(self.delta_count)
        self.active = self.delta_count != 0

        if illegal_transitions is not None:
            self.illegal_transitions = int(illegal_transitions)

        self.last_update_s = stamp_s

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


@dataclass(frozen=True)
class EncoderWheelSnapshot:
    name: str
    count: int
    delta_count: int
    counts_per_second: float
    speed_mps: float
    direction: int
    illegal_transitions: int
    active: bool

    @property
    def direction_symbol(self) -> str:
        return direction_symbol(self.direction)

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["direction_symbol"] = self.direction_symbol
        return data


@dataclass(frozen=True)
class EncoderSample:
    stamp_s: float
    dt_s: float

    fl: EncoderWheelSnapshot
    fr: EncoderWheelSnapshot
    rl: EncoderWheelSnapshot
    rr: EncoderWheelSnapshot

    vx_mps: float = 0.0
    vy_mps: float = 0.0
    omega_rad_s: float = 0.0

    @property
    def wheels(self) -> tuple[EncoderWheelSnapshot, EncoderWheelSnapshot, EncoderWheelSnapshot, EncoderWheelSnapshot]:
        return (self.fl, self.fr, self.rl, self.rr)

    @property
    def wheel_map(self) -> dict[str, EncoderWheelSnapshot]:
        return {
            self.fl.name: self.fl,
            self.fr.name: self.fr,
            self.rl.name: self.rl,
            self.rr.name: self.rr,
        }

    @property
    def total_illegal_transitions(self) -> int:
        return sum(int(wheel.illegal_transitions) for wheel in self.wheels)

    @property
    def active_wheel_count(self) -> int:
        return sum(1 for wheel in self.wheels if wheel.active)

    @property
    def all_wheels_active(self) -> bool:
        return self.active_wheel_count == len(self.wheels)

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

    def mark_error(self, message: str, reasons: list[str] | None = None) -> None:
        self.status = STATUS_ERROR
        self.message = message
        self.reasons = list(reasons or [])
        self.hardware_ok = False

    def mark_stale(self, message: str, reasons: list[str] | None = None) -> None:
        self.status = STATUS_STALE
        self.message = message
        self.reasons = list(reasons or [])
        self.data_ok = False

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


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
            self.fl.name: self.fl,
            self.fr.name: self.fr,
            self.rl.name: self.rl,
            self.rr.name: self.rr,
        }

    @property
    def ready(self) -> bool:
        return self.health.hardware_ok and self.health.data_ok

    def mark_sample(self, sample: EncoderSample) -> None:
        self.last_sample = sample
        self.sample_count += 1
        self.last_sample_age_s = 0.0

        self.health.active_wheel_count = sample.active_wheel_count
        self.health.total_illegal_transitions = sample.total_illegal_transitions
        self.health.all_wheels_seen = set(sample.wheel_map.keys()) == set(WHEEL_ORDER)

        rates = [abs(wheel.counts_per_second) for wheel in sample.wheels]
        self.health.min_counts_per_second = min(rates) if rates else 0.0
        self.health.max_counts_per_second = max(rates) if rates else 0.0

        reasons: list[str] = []

        if not self.health.all_wheels_seen:
            reasons.append("not all wheel encoder states are present")

        if sample.total_illegal_transitions > DEFAULT_MAX_ILLEGAL_TRANSITIONS:
            reasons.append(
                "too many illegal encoder transitions: "
                f"{sample.total_illegal_transitions}"
            )
            self.health.illegal_transition_ok = False
        else:
            self.health.illegal_transition_ok = True

        if sample.active_wheel_count == 0 and DEFAULT_MIN_ENCODER_RATE_CPS > 0.0:
            reasons.append("no active encoder movement detected")
            self.health.rate_ok = False
        else:
            self.health.rate_ok = True

        if reasons:
            self.health.mark_warn("encoder data usable with notes", reasons=reasons)
            self.health.data_ok = True
            return

        self.health.mark_ok()

    def mark_stale(self, age_s: float | None) -> None:
        self.last_sample_age_s = age_s
        self.health.mark_stale(
            "encoder sample stale",
            reasons=[
                "no sample age available" if age_s is None else f"age_s={age_s:.3f}"
            ],
        )

    def mark_error(self, message: str, reasons: list[str] | None = None) -> None:
        self.health.mark_error(message, reasons=reasons)

    def to_dict(self) -> dict[str, Any]:
        return {
            "model": self.model,
            "sample_count": int(self.sample_count),
            "last_sample_age_s": self.last_sample_age_s,
            "wheels": {
                wheel.name: wheel.to_dict()
                for wheel in self.wheels
            },
            "last_sample": self.last_sample.to_dict() if self.last_sample else None,
            "health": self.health.to_dict(),
            "ready": self.ready,
        }


def make_wheel_snapshot(
    *,
    name: str,
    count: int,
    delta_count: int,
    counts_per_second: float = 0.0,
    speed_mps: float = 0.0,
    direction: int | None = None,
    illegal_transitions: int = 0,
    active: bool | None = None,
) -> EncoderWheelSnapshot:
    if name not in WHEEL_ORDER:
        raise ValueError(f"Unsupported wheel name: {name!r}")

    if direction is None:
        direction = direction_from_delta(delta_count)

    if active is None:
        active = int(delta_count) != 0

    return EncoderWheelSnapshot(
        name=name,
        count=int(count),
        delta_count=int(delta_count),
        counts_per_second=float(counts_per_second),
        speed_mps=float(speed_mps),
        direction=int(direction),
        illegal_transitions=int(illegal_transitions),
        active=bool(active),
    )


def make_encoder_sample(
    *,
    stamp_s: float,
    dt_s: float,
    fl: EncoderWheelSnapshot,
    fr: EncoderWheelSnapshot,
    rl: EncoderWheelSnapshot,
    rr: EncoderWheelSnapshot,
    vx_mps: float = 0.0,
    vy_mps: float = 0.0,
    omega_rad_s: float = 0.0,
) -> EncoderSample:
    if dt_s <= 0.0:
        raise ValueError(f"dt_s must be > 0.0, got {dt_s}")

    _require_wheel_name(fl, WHEEL_FL)
    _require_wheel_name(fr, WHEEL_FR)
    _require_wheel_name(rl, WHEEL_RL)
    _require_wheel_name(rr, WHEEL_RR)

    return EncoderSample(
        stamp_s=float(stamp_s),
        dt_s=float(dt_s),
        fl=fl,
        fr=fr,
        rl=rl,
        rr=rr,
        vx_mps=float(vx_mps),
        vy_mps=float(vy_mps),
        omega_rad_s=float(omega_rad_s),
    )


def make_encoder_sample_from_counts(
    *,
    stamp_s: float,
    dt_s: float,
    previous_counts: dict[str, int],
    current_counts: dict[str, int],
    speeds_mps: dict[str, float] | None = None,
    illegal_transitions: dict[str, int] | None = None,
    vx_mps: float = 0.0,
    vy_mps: float = 0.0,
    omega_rad_s: float = 0.0,
) -> EncoderSample:
    speeds_mps = speeds_mps or {}
    illegal_transitions = illegal_transitions or {}

    snapshots = {}

    for wheel_name in WHEEL_ORDER:
        previous = int(previous_counts.get(wheel_name, 0))
        current = int(current_counts.get(wheel_name, 0))
        delta = current - previous
        cps = float(delta) / max(float(dt_s), 1e-9)

        snapshots[wheel_name] = make_wheel_snapshot(
            name=wheel_name,
            count=current,
            delta_count=delta,
            counts_per_second=cps,
            speed_mps=float(speeds_mps.get(wheel_name, 0.0)),
            illegal_transitions=int(illegal_transitions.get(wheel_name, 0)),
        )

    return make_encoder_sample(
        stamp_s=stamp_s,
        dt_s=dt_s,
        fl=snapshots[WHEEL_FL],
        fr=snapshots[WHEEL_FR],
        rl=snapshots[WHEEL_RL],
        rr=snapshots[WHEEL_RR],
        vx_mps=vx_mps,
        vy_mps=vy_mps,
        omega_rad_s=omega_rad_s,
    )


def _require_wheel_name(snapshot: EncoderWheelSnapshot, expected: str) -> None:
    if snapshot.name != expected:
        raise ValueError(f"expected {expected} snapshot, got {snapshot.name}")