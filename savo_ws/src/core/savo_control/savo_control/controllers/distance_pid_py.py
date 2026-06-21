#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Python fallback distance PID controller."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Optional
import math

from savo_control.models import DistanceApproachConfig
from savo_control.utils import apply_deadband, clamp_abs, within_tolerance

from .pid_py import Pid, PidConfig, PidResult


@dataclass
class DistanceControllerConfig:
    pid: PidConfig = field(default_factory=PidConfig)

    distance_tolerance_m: float = 0.03
    output_deadband_m_s: float = 0.0
    min_effective_vx_m_s: float = 0.0
    max_vx_m_s: float = 0.0

    zero_output_within_tolerance: bool = True
    reset_pid_on_target_change: bool = True
    target_change_reset_threshold_m: float = 0.01

    min_dt_sec: float = 1e-6
    max_dt_sec: float = 1.0

    direction_mode: str = "bidirectional"

    @classmethod
    def from_distance_approach_config(
        cls,
        cfg: DistanceApproachConfig,
    ) -> "DistanceControllerConfig":
        safe = cfg.sanitized()

        pid_cfg = PidConfig(
            kp=safe.kp,
            ki=safe.ki,
            kd=safe.kd,
        )
        pid_cfg.set_output_symmetric(max(safe.max_forward_vx, safe.max_reverse_vx))

        return cls(
            pid=pid_cfg,
            distance_tolerance_m=safe.tolerance_m,
            output_deadband_m_s=0.0,
            min_effective_vx_m_s=safe.min_vx_when_active,
            max_vx_m_s=max(safe.max_forward_vx, safe.max_reverse_vx),
            zero_output_within_tolerance=True,
            direction_mode="bidirectional" if safe.allow_reverse else "approach_from_far_only",
        )


@dataclass
class DistanceControllerResult:
    vx_cmd_m_s: float = 0.0

    target_distance_m: float = 0.0
    current_distance_m: float = 0.0
    error_distance_m: float = 0.0

    has_target: bool = False
    within_tolerance: bool = False

    valid: bool = False
    dt_valid: bool = False
    distance_valid: bool = False

    pid: PidResult = field(default_factory=PidResult)
    direction_limited: bool = False


class DistancePid:
    """Distance controller using Robot Savo distance-approach sign convention."""

    VALID_DIRECTION_MODES = {
        "bidirectional",
        "approach_from_far_only",
        "back_away_only",
    }

    def __init__(self, cfg: Optional[DistanceControllerConfig] = None) -> None:
        self._config = cfg if cfg is not None else DistanceControllerConfig()
        self._normalize_config()

        self._pid = Pid(self._config.pid)
        self._pid.set_config(self._config.pid)

        self._has_target = False
        self._target_distance_m = 0.0

    def set_config(self, cfg: DistanceControllerConfig) -> None:
        self._config = cfg
        self._normalize_config()
        self._pid.set_config(self._config.pid)

    @property
    def config(self) -> DistanceControllerConfig:
        return self._config

    def pid(self) -> Pid:
        return self._pid

    def has_target(self) -> bool:
        return self._has_target

    def target_distance_m(self) -> float:
        return self._target_distance_m

    def set_target_distance(self, target_distance_m: float) -> None:
        if not isfinite(target_distance_m):
            return

        new_target = float(target_distance_m)

        if self._has_target and self._config.reset_pid_on_target_change:
            delta = abs(new_target - self._target_distance_m)
            if delta >= abs(self._config.target_change_reset_threshold_m):
                self._pid.reset()

        self._target_distance_m = new_target
        self._has_target = True

    def clear_target(self) -> None:
        self._has_target = False
        self._target_distance_m = 0.0
        self._pid.reset()

    def reset(self) -> None:
        self._pid.reset()

    def is_within_tolerance(self, current_distance_m: float) -> bool:
        if not self._has_target or not isfinite(current_distance_m):
            return False

        return within_tolerance(
            float(current_distance_m),
            self._target_distance_m,
            self._config.distance_tolerance_m,
        )

    def update(self, current_distance_m: float, dt_sec: float) -> DistanceControllerResult:
        return self._update_impl(
            current_distance_m=current_distance_m,
            target_distance_m=self._target_distance_m,
            has_target=self._has_target,
            dt_sec=dt_sec,
        )

    def update_to_target(
        self,
        current_distance_m: float,
        target_distance_m: float,
        dt_sec: float,
    ) -> DistanceControllerResult:
        return self._update_impl(
            current_distance_m=current_distance_m,
            target_distance_m=target_distance_m,
            has_target=isfinite(target_distance_m),
            dt_sec=dt_sec,
        )

    def _update_impl(
        self,
        *,
        current_distance_m: float,
        target_distance_m: float,
        has_target: bool,
        dt_sec: float,
    ) -> DistanceControllerResult:
        out = DistanceControllerResult()
        out.current_distance_m = (
            float(current_distance_m) if isfinite(current_distance_m) else 0.0
        )
        out.dt_valid = self._valid_dt(dt_sec)
        out.distance_valid = isfinite(current_distance_m)
        out.has_target = has_target

        if not out.distance_valid or not has_target:
            return out

        out.target_distance_m = float(target_distance_m)
        out.error_distance_m = out.current_distance_m - out.target_distance_m
        out.within_tolerance = within_tolerance(
            out.current_distance_m,
            out.target_distance_m,
            self._config.distance_tolerance_m,
        )

        out.pid = self._pid.update(out.error_distance_m, dt_sec)

        if not out.pid.valid:
            return out

        vx_cmd = out.pid.output

        if self._config.max_vx_m_s > 0.0 and isfinite(self._config.max_vx_m_s):
            vx_cmd = clamp_abs(vx_cmd, self._config.max_vx_m_s)

        vx_cmd, out.direction_limited = self._apply_direction_policy(
            vx_cmd,
            out.error_distance_m,
        )

        if (
            self._config.min_effective_vx_m_s > 0.0
            and isfinite(self._config.min_effective_vx_m_s)
            and abs(vx_cmd) > 0.0
            and abs(vx_cmd) < abs(self._config.min_effective_vx_m_s)
        ):
            vx_cmd = math.copysign(abs(self._config.min_effective_vx_m_s), vx_cmd)

        vx_cmd = apply_deadband(vx_cmd, self._config.output_deadband_m_s)

        if self._config.zero_output_within_tolerance and out.within_tolerance:
            vx_cmd = 0.0

        out.vx_cmd_m_s = vx_cmd
        out.valid = out.dt_valid and out.distance_valid and out.pid.valid
        return out

    def _valid_dt(self, dt_sec: float) -> bool:
        return (
            isfinite(dt_sec)
            and self._config.min_dt_sec <= dt_sec <= self._config.max_dt_sec
        )

    def _normalize_config(self) -> None:
        c = self._config

        if not isfinite(c.distance_tolerance_m) or c.distance_tolerance_m < 0.0:
            c.distance_tolerance_m = 0.03

        if not isfinite(c.output_deadband_m_s) or c.output_deadband_m_s < 0.0:
            c.output_deadband_m_s = 0.0

        if not isfinite(c.min_effective_vx_m_s) or c.min_effective_vx_m_s < 0.0:
            c.min_effective_vx_m_s = 0.0

        if not isfinite(c.max_vx_m_s):
            c.max_vx_m_s = 0.0
        elif c.max_vx_m_s < 0.0:
            c.max_vx_m_s = abs(c.max_vx_m_s)

        if (
            not isfinite(c.target_change_reset_threshold_m)
            or c.target_change_reset_threshold_m < 0.0
        ):
            c.target_change_reset_threshold_m = 0.01

        if not isfinite(c.min_dt_sec) or c.min_dt_sec <= 0.0:
            c.min_dt_sec = 1e-6

        if not isfinite(c.max_dt_sec) or c.max_dt_sec < c.min_dt_sec:
            c.max_dt_sec = max(1.0, c.min_dt_sec)

        if c.direction_mode not in self.VALID_DIRECTION_MODES:
            c.direction_mode = "bidirectional"

    def _apply_direction_policy(
        self,
        vx_cmd: float,
        error_distance_m: float,
    ) -> tuple[float, bool]:
        if not isfinite(vx_cmd) or not isfinite(error_distance_m):
            return 0.0, True

        mode = self._config.direction_mode

        if mode == "bidirectional":
            return vx_cmd, False

        if mode == "approach_from_far_only":
            if error_distance_m <= 0.0:
                return 0.0, abs(vx_cmd) > 0.0
            if vx_cmd < 0.0:
                return 0.0, True
            return vx_cmd, False

        if mode == "back_away_only":
            if error_distance_m >= 0.0:
                return 0.0, abs(vx_cmd) > 0.0
            if vx_cmd > 0.0:
                return 0.0, True
            return vx_cmd, False

        return vx_cmd, False


DistanceController = DistancePid


if __name__ == "__main__":
    pid_cfg = PidConfig(kp=0.8, ki=0.0, kd=0.02)
    pid_cfg.set_output_symmetric(0.12)
    pid_cfg.set_integral_clamp_symmetric(0.5)

    cfg = DistanceControllerConfig(
        pid=pid_cfg,
        distance_tolerance_m=0.03,
        output_deadband_m_s=0.005,
        min_effective_vx_m_s=0.02,
        max_vx_m_s=0.10,
        zero_output_within_tolerance=True,
        direction_mode="approach_from_far_only",
    )

    ctrl = DistancePid(cfg)
    ctrl.set_target_distance(0.80)

    print("=== distance_pid_py.py self-test ===")
    for i, distance_m in enumerate([1.20, 1.10, 1.00, 0.92, 0.84, 0.805, 0.800], start=1):
        result = ctrl.update(distance_m, 0.05)
        print(
            f"step={i:02d} cur={result.current_distance_m:.3f}m "
            f"target={result.target_distance_m:.3f}m "
            f"err={result.error_distance_m:+.3f}m "
            f"vx={result.vx_cmd_m_s:+.3f}m/s "
            f"tol={int(result.within_tolerance)} "
            f"valid={int(result.valid)}"
        )
