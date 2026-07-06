#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Stateful perception safety policy: fusion + debounce + smoothing."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Optional

from savo_perception.constants import (
    CLEAR_DEBOUNCE_COUNT_DEFAULT,
    FRONT_CLEAR_HYSTERESIS_M_DEFAULT,
    SLOWDOWN_DEFAULT,
    SLOWDOWN_EMA_ALPHA_DEFAULT,
    STOP_DEBOUNCE_COUNT_DEFAULT,
    SIDE_CLEAR_HYSTERESIS_M_DEFAULT,
)
from savo_perception.models.range_sample import RangeSnapshot
from savo_perception.models.safety_state import (
    SafetyDecision,
    SafetyState,
    clamp_slowdown_factor,
)
from savo_perception.safety.range_fusion import (
    RangeFusionConfig,
    RangeFusionResult,
    fuse_range_snapshot,
)


@dataclass(frozen=True)
class SafetyPolicyConfig:
    fusion: RangeFusionConfig = field(default_factory=RangeFusionConfig)
    stop_debounce_count: int = STOP_DEBOUNCE_COUNT_DEFAULT
    clear_debounce_count: int = CLEAR_DEBOUNCE_COUNT_DEFAULT
    front_clear_hysteresis_m: float = FRONT_CLEAR_HYSTERESIS_M_DEFAULT
    side_clear_hysteresis_m: float = SIDE_CLEAR_HYSTERESIS_M_DEFAULT
    slowdown_ema_alpha: float = SLOWDOWN_EMA_ALPHA_DEFAULT

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["fusion"] = self.fusion.to_dict()
        return data


@dataclass(frozen=True)
class SafetyPolicyUpdate:
    raw_result: RangeFusionResult
    state: SafetyState
    published_decision: SafetyDecision
    stop_count: int
    clear_count: int

    def to_dict(self) -> Dict[str, Any]:
        return {
            "raw_result": self.raw_result.to_dict(),
            "state": self.state.to_dict(),
            "published_decision": self.published_decision.to_dict(),
            "stop_count": self.stop_count,
            "clear_count": self.clear_count,
        }


def _safe_count(value: int, *, minimum: int = 1) -> int:
    return max(minimum, int(value))


def _ema(new_value: float, old_value: float, alpha: float) -> float:
    a = max(0.0, min(1.0, float(alpha)))
    return (a * float(new_value)) + ((1.0 - a) * float(old_value))


def fusion_config_for_state(
    config: SafetyPolicyConfig,
    state: SafetyState,
) -> RangeFusionConfig:
    fusion = config.fusion

    if not state.stop_required:
        return fusion

    return RangeFusionConfig(
        front_stop_m=fusion.front_stop_m + config.front_clear_hysteresis_m,
        front_slow_m=fusion.front_slow_m,
        side_stop_m=fusion.side_stop_m + config.side_clear_hysteresis_m,
        side_slow_m=fusion.side_slow_m,
        stale_timeout_s=fusion.stale_timeout_s,
        fail_safe_on_stale=fusion.fail_safe_on_stale,
        required_sensors=fusion.required_sensors,
    )


class SafetyPolicy:
    def __init__(
        self,
        config: Optional[SafetyPolicyConfig] = None,
        initial_state: Optional[SafetyState] = None,
    ) -> None:
        self.config = config or SafetyPolicyConfig()
        self.state = initial_state or SafetyState.initial()
        self._stop_count = 0
        self._clear_count = 0
        self._slowdown_filtered = SLOWDOWN_DEFAULT

    @property
    def stop_count(self) -> int:
        return self._stop_count

    @property
    def clear_count(self) -> int:
        return self._clear_count

    @property
    def slowdown_filtered(self) -> float:
        return self._slowdown_filtered

    def reset(self) -> None:
        self.state = SafetyState.initial()
        self._stop_count = 0
        self._clear_count = 0
        self._slowdown_filtered = SLOWDOWN_DEFAULT

    def update(self, snapshot: RangeSnapshot) -> SafetyPolicyUpdate:
        fusion_cfg = fusion_config_for_state(self.config, self.state)
        raw_result = fuse_range_snapshot(snapshot, fusion_cfg)
        raw_decision = raw_result.decision

        if raw_decision.stop_required:
            self._stop_count += 1
            self._clear_count = 0
        else:
            self._clear_count += 1
            self._stop_count = 0

        published = self._apply_debounce(raw_decision)
        published = self._apply_slowdown_filter(published)

        self.state = self.state.with_decision(
            published,
            stop_count=self._stop_count,
            clear_count=self._clear_count,
        )

        return SafetyPolicyUpdate(
            raw_result=raw_result,
            state=self.state,
            published_decision=published,
            stop_count=self._stop_count,
            clear_count=self._clear_count,
        )

    def _apply_debounce(self, raw_decision: SafetyDecision) -> SafetyDecision:
        stop_needed = _safe_count(self.config.stop_debounce_count)
        clear_needed = _safe_count(self.config.clear_debounce_count)

        if self.state.stop_required:
            if raw_decision.stop_required:
                return raw_decision

            if self._clear_count >= clear_needed:
                return raw_decision

            return SafetyDecision.stop(
                reason="holding_stop_until_clear_debounce",
                front_distance_m=raw_decision.front_distance_m,
                side_distance_m=raw_decision.side_distance_m,
                stale_sensors=raw_decision.stale_sensors,
                invalid_sensors=raw_decision.invalid_sensors,
            )

        if raw_decision.stop_required:
            if self._stop_count >= stop_needed:
                return raw_decision

            return SafetyDecision.slow(
                slowdown_factor=0.2,
                reason="pre_stop_debounce",
                front_distance_m=raw_decision.front_distance_m,
                side_distance_m=raw_decision.side_distance_m,
            )

        return raw_decision

    def _apply_slowdown_filter(self, decision: SafetyDecision) -> SafetyDecision:
        if decision.stop_required:
            self._slowdown_filtered = 0.0
            return decision

        target = clamp_slowdown_factor(decision.slowdown_factor)

        if self.state.stop_required:
            self._slowdown_filtered = target
        else:
            previous = self._slowdown_filtered
            if previous <= 0.0:
                previous = target
            self._slowdown_filtered = clamp_slowdown_factor(
                _ema(target, previous, self.config.slowdown_ema_alpha)
            )

        if abs(self._slowdown_filtered - decision.slowdown_factor) < 1e-6:
            return decision

        return SafetyDecision(
            stop_required=False,
            slowdown_factor=self._slowdown_filtered,
            status=decision.status,
            reason=decision.reason,
            front_distance_m=decision.front_distance_m,
            side_distance_m=decision.side_distance_m,
            stale_sensors=decision.stale_sensors,
            invalid_sensors=decision.invalid_sensors,
            source=decision.source,
        )


__all__ = [
    "SafetyPolicyConfig",
    "SafetyPolicyUpdate",
    "SafetyPolicy",
    "fusion_config_for_state",
]
