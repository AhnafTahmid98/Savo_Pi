#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Encoder direction checks for Robot Savo four-wheel odometry. No ROS imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Mapping

from savo_localization.constants import (
    STATUS_ERROR,
    STATUS_OK,
    STATUS_WARN,
    WHEEL_FL,
    WHEEL_FR,
    WHEEL_ORDER,
    WHEEL_RL,
    WHEEL_RR,
)
from savo_localization.math.encoder_math import direction_from_delta, direction_symbol
from savo_localization.models.encoder_state import EncoderSample, EncoderWheelSnapshot


DIRECTION_FORWARD = "forward"
DIRECTION_REVERSE = "reverse"
DIRECTION_STRAFE_LEFT = "strafe_left"
DIRECTION_STRAFE_RIGHT = "strafe_right"
DIRECTION_ROTATE_CCW = "rotate_ccw"
DIRECTION_ROTATE_CW = "rotate_cw"
DIRECTION_STOP = "stop"

EXPECTED_DIRECTION_PATTERNS: dict[str, dict[str, int]] = {
    DIRECTION_FORWARD: {
        WHEEL_FL: +1,
        WHEEL_FR: +1,
        WHEEL_RL: +1,
        WHEEL_RR: +1,
    },
    DIRECTION_REVERSE: {
        WHEEL_FL: -1,
        WHEEL_FR: -1,
        WHEEL_RL: -1,
        WHEEL_RR: -1,
    },
    DIRECTION_STRAFE_LEFT: {
        WHEEL_FL: -1,
        WHEEL_FR: +1,
        WHEEL_RL: +1,
        WHEEL_RR: -1,
    },
    DIRECTION_STRAFE_RIGHT: {
        WHEEL_FL: +1,
        WHEEL_FR: -1,
        WHEEL_RL: -1,
        WHEEL_RR: +1,
    },
    DIRECTION_ROTATE_CCW: {
        WHEEL_FL: -1,
        WHEEL_FR: +1,
        WHEEL_RL: -1,
        WHEEL_RR: +1,
    },
    DIRECTION_ROTATE_CW: {
        WHEEL_FL: +1,
        WHEEL_FR: -1,
        WHEEL_RL: +1,
        WHEEL_RR: -1,
    },
    DIRECTION_STOP: {
        WHEEL_FL: 0,
        WHEEL_FR: 0,
        WHEEL_RL: 0,
        WHEEL_RR: 0,
    },
}


@dataclass(frozen=True)
class WheelDirectionResult:
    name: str
    expected: int
    observed: int
    delta_count: int
    ok: bool
    message: str

    @property
    def expected_symbol(self) -> str:
        return direction_symbol(self.expected)

    @property
    def observed_symbol(self) -> str:
        return direction_symbol(self.observed)

    def to_dict(self) -> dict[str, object]:
        data = asdict(self)
        data["expected_symbol"] = self.expected_symbol
        data["observed_symbol"] = self.observed_symbol
        return data


@dataclass(frozen=True)
class EncoderDirectionCheckResult:
    command: str
    status: str
    ok: bool
    message: str
    reasons: list[str] = field(default_factory=list)
    wheel_results: dict[str, WheelDirectionResult] = field(default_factory=dict)

    @property
    def matched_wheel_count(self) -> int:
        return sum(1 for result in self.wheel_results.values() if result.ok)

    @property
    def checked_wheel_count(self) -> int:
        return len(self.wheel_results)

    def to_dict(self) -> dict[str, object]:
        return {
            "command": self.command,
            "status": self.status,
            "ok": bool(self.ok),
            "message": self.message,
            "reasons": list(self.reasons),
            "matched_wheel_count": self.matched_wheel_count,
            "checked_wheel_count": self.checked_wheel_count,
            "wheel_results": {
                name: result.to_dict()
                for name, result in self.wheel_results.items()
            },
        }


def expected_direction_pattern(command: str) -> dict[str, int]:
    key = normalize_command_name(command)

    if key not in EXPECTED_DIRECTION_PATTERNS:
        raise ValueError(f"Unsupported encoder direction command: {command!r}")

    return dict(EXPECTED_DIRECTION_PATTERNS[key])


def normalize_command_name(command: str) -> str:
    return str(command).strip().lower().replace("-", "_").replace(" ", "_")


def check_encoder_direction_sample(
    sample: EncoderSample,
    *,
    command: str,
    min_delta_count: int = 1,
    allow_partial: bool = False,
) -> EncoderDirectionCheckResult:
    pattern = expected_direction_pattern(command)
    return check_encoder_direction_map(
        wheel_snapshots=sample.wheel_map,
        expected_pattern=pattern,
        command=command,
        min_delta_count=min_delta_count,
        allow_partial=allow_partial,
    )


def check_encoder_direction_map(
    *,
    wheel_snapshots: Mapping[str, EncoderWheelSnapshot],
    expected_pattern: Mapping[str, int],
    command: str,
    min_delta_count: int = 1,
    allow_partial: bool = False,
) -> EncoderDirectionCheckResult:
    if min_delta_count < 0:
        raise ValueError(f"min_delta_count must be >= 0, got {min_delta_count}")

    wheel_results: dict[str, WheelDirectionResult] = {}
    reasons: list[str] = []

    for wheel_name in WHEEL_ORDER:
        if wheel_name not in wheel_snapshots:
            reasons.append(f"{wheel_name}: missing wheel snapshot")
            continue

        expected = int(expected_pattern.get(wheel_name, 0))
        wheel_results[wheel_name] = check_wheel_direction(
            wheel_snapshots[wheel_name],
            expected=expected,
            min_delta_count=min_delta_count,
        )

    for wheel_name, result in wheel_results.items():
        if not result.ok:
            reasons.append(
                f"{wheel_name}: expected {result.expected_symbol}, "
                f"observed {result.observed_symbol}, delta={result.delta_count}"
            )

    missing_count = len(WHEEL_ORDER) - len(wheel_results)
    failed_count = sum(1 for result in wheel_results.values() if not result.ok)

    if missing_count > 0:
        return EncoderDirectionCheckResult(
            command=normalize_command_name(command),
            status=STATUS_ERROR,
            ok=False,
            message="encoder direction check missing wheel data",
            reasons=reasons,
            wheel_results=wheel_results,
        )

    if failed_count == 0:
        return EncoderDirectionCheckResult(
            command=normalize_command_name(command),
            status=STATUS_OK,
            ok=True,
            message="encoder directions match expected pattern",
            reasons=[],
            wheel_results=wheel_results,
        )

    if allow_partial and failed_count < len(WHEEL_ORDER):
        return EncoderDirectionCheckResult(
            command=normalize_command_name(command),
            status=STATUS_WARN,
            ok=True,
            message="encoder directions partially match expected pattern",
            reasons=reasons,
            wheel_results=wheel_results,
        )

    return EncoderDirectionCheckResult(
        command=normalize_command_name(command),
        status=STATUS_ERROR,
        ok=False,
        message="encoder direction check failed",
        reasons=reasons,
        wheel_results=wheel_results,
    )


def check_wheel_direction(
    wheel: EncoderWheelSnapshot,
    *,
    expected: int,
    min_delta_count: int = 1,
) -> WheelDirectionResult:
    expected_direction = _normalize_expected_direction(expected)
    observed_direction = observed_direction_from_delta(
        wheel.delta_count,
        min_delta_count=min_delta_count,
    )

    ok = observed_direction == expected_direction

    if ok:
        message = "direction matched"
    else:
        message = "direction mismatch"

    return WheelDirectionResult(
        name=wheel.name,
        expected=expected_direction,
        observed=observed_direction,
        delta_count=int(wheel.delta_count),
        ok=ok,
        message=message,
    )


def observed_direction_from_delta(
    delta_count: int | float,
    *,
    min_delta_count: int = 1,
) -> int:
    if abs(float(delta_count)) < int(min_delta_count):
        return 0

    return direction_from_delta(delta_count)


def make_custom_direction_pattern(
    *,
    fl: int,
    fr: int,
    rl: int,
    rr: int,
) -> dict[str, int]:
    pattern = {
        WHEEL_FL: _normalize_expected_direction(fl),
        WHEEL_FR: _normalize_expected_direction(fr),
        WHEEL_RL: _normalize_expected_direction(rl),
        WHEEL_RR: _normalize_expected_direction(rr),
    }
    return pattern


def direction_pattern_to_string(pattern: Mapping[str, int]) -> str:
    parts: list[str] = []

    for wheel_name in WHEEL_ORDER:
        direction = _normalize_expected_direction(pattern.get(wheel_name, 0))
        parts.append(f"{wheel_name}={direction_symbol(direction)}")

    return " ".join(parts)


def available_direction_commands() -> tuple[str, ...]:
    return tuple(EXPECTED_DIRECTION_PATTERNS.keys())


def _normalize_expected_direction(value: int | float) -> int:
    value = int(value)

    if value > 0:
        return +1

    if value < 0:
        return -1

    return 0
