#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Python fallback heading/yaw PID controller."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Optional
import math

from savo_control.utils import (
    apply_deadband,
    clamp_abs,
    shortest_angle_error_rad,
    within_tolerance,
    wrap_angle_rad as _wrap_angle_rad,
)

from .pid_py import Pid, PidConfig, PidResult


def wrap_angle_rad(angle: float) -> float:
    if not isfinite(angle):
        return 0.0

    return _wrap_angle_rad(angle)


def shortest_angular_distance_rad(from_angle: float, to_angle: float) -> float:
    if not isfinite(from_angle) or not isfinite(to_angle):
        return 0.0

    return shortest_angle_error_rad(to_angle, from_angle)


def saturate_abs(value: float, abs_limit: float) -> float:
    if not isfinite(value):
        return 0.0

    if not isfinite(abs_limit) or abs_limit <= 0.0:
        return value

    return clamp_abs(value, abs_limit)


def copy_sign(magnitude: float, sign_source: float) -> float:
    safe_magnitude = float(magnitude) if isfinite(magnitude) else 0.0
    safe_sign = float(sign_source) if isfinite(sign_source) else 0.0

    return math.copysign(abs(safe_magnitude), safe_sign)


@dataclass
class HeadingControllerConfig:
    pid: PidConfig = field(default_factory=PidConfig)

    heading_tolerance_rad: float = 0.03
    output_deadband_rad_s: float = 0.0
    min_effective_wz_rad_s: float = 0.0
    max_wz_rad_s: float = 0.0

    reset_pid_on_hold_capture: bool = True
    reset_pid_on_target_change: bool = True
    target_change_reset_threshold_rad: float = 0.02

    min_dt_sec: float = 1e-6
    max_dt_sec: float = 1.0


@dataclass
class HeadingControllerResult:
    wz_cmd_rad_s: float = 0.0

    target_yaw_rad: float = 0.0
    current_yaw_rad: float = 0.0
    error_yaw_rad: float = 0.0

    has_target: bool = False
    within_tolerance: bool = False

    valid: bool = False
    dt_valid: bool = False
    yaw_valid: bool = False

    pid: PidResult = field(default_factory=PidResult)


class HeadingController:
    """Heading/yaw controller for Python fallback, tests, and diagnostics."""

    def __init__(self, cfg: Optional[HeadingControllerConfig] = None) -> None:
        self._config = cfg if cfg is not None else HeadingControllerConfig()
        self._normalize_config()

        self._pid = Pid(self._config.pid)
        self._pid.set_config(self._config.pid)

        self._has_target = False
        self._target_yaw_rad = 0.0

    def set_config(self, cfg: HeadingControllerConfig) -> None:
        self._config = cfg
        self._normalize_config()
        self._pid.set_config(self._config.pid)

    @property
    def config(self) -> HeadingControllerConfig:
        return self._config

    def pid(self) -> Pid:
        return self._pid

    def has_target(self) -> bool:
        return self._has_target

    def target_yaw_rad(self) -> float:
        return self._target_yaw_rad

    def set_target_yaw(self, target_yaw_rad: float) -> None:
        if not isfinite(target_yaw_rad):
            return

        new_target = wrap_angle_rad(target_yaw_rad)

        if self._has_target and self._config.reset_pid_on_target_change:
            delta = abs(
                shortest_angular_distance_rad(
                    self._target_yaw_rad,
                    new_target,
                )
            )
            if delta >= abs(self._config.target_change_reset_threshold_rad):
                self._pid.reset()

        self._target_yaw_rad = new_target
        self._has_target = True

    def capture_hold_target(self, current_yaw_rad: float) -> None:
        if not isfinite(current_yaw_rad):
            return

        self._target_yaw_rad = wrap_angle_rad(current_yaw_rad)
        self._has_target = True

        if self._config.reset_pid_on_hold_capture:
            self._pid.reset()

    def clear_target(self) -> None:
        self._has_target = False
        self._target_yaw_rad = 0.0
        self._pid.reset()

    def reset(self) -> None:
        self._pid.reset()

    def is_within_tolerance(self, current_yaw_rad: float) -> bool:
        if not self._has_target or not isfinite(current_yaw_rad):
            return False

        error = shortest_angular_distance_rad(
            wrap_angle_rad(current_yaw_rad),
            self._target_yaw_rad,
        )

        return within_tolerance(
            error,
            0.0,
            self._config.heading_tolerance_rad,
        )

    def update(self, current_yaw_rad: float, dt_sec: float) -> HeadingControllerResult:
        return self._update_impl(
            current_yaw_rad=current_yaw_rad,
            target_yaw_rad=self._target_yaw_rad,
            has_target=self._has_target,
            dt_sec=dt_sec,
        )

    def update_to_target(
        self,
        current_yaw_rad: float,
        target_yaw_rad: float,
        dt_sec: float,
    ) -> HeadingControllerResult:
        return self._update_impl(
            current_yaw_rad=current_yaw_rad,
            target_yaw_rad=target_yaw_rad,
            has_target=isfinite(target_yaw_rad),
            dt_sec=dt_sec,
        )

    def _update_impl(
        self,
        *,
        current_yaw_rad: float,
        target_yaw_rad: float,
        has_target: bool,
        dt_sec: float,
    ) -> HeadingControllerResult:
        out = HeadingControllerResult()
        out.current_yaw_rad = (
            float(current_yaw_rad) if isfinite(current_yaw_rad) else 0.0
        )
        out.dt_valid = self._valid_dt(dt_sec)
        out.yaw_valid = isfinite(current_yaw_rad)
        out.has_target = has_target

        if not out.yaw_valid or not has_target:
            return out

        current_wrapped = wrap_angle_rad(current_yaw_rad)
        target_wrapped = wrap_angle_rad(target_yaw_rad)

        out.current_yaw_rad = current_wrapped
        out.target_yaw_rad = target_wrapped
        out.error_yaw_rad = shortest_angular_distance_rad(
            current_wrapped,
            target_wrapped,
        )
        out.within_tolerance = within_tolerance(
            out.error_yaw_rad,
            0.0,
            self._config.heading_tolerance_rad,
        )

        out.pid = self._pid.update(out.error_yaw_rad, dt_sec)

        if not out.pid.valid:
            return out

        wz_cmd = out.pid.output

        if self._config.max_wz_rad_s > 0.0 and isfinite(self._config.max_wz_rad_s):
            wz_cmd = saturate_abs(wz_cmd, self._config.max_wz_rad_s)

        if (
            self._config.min_effective_wz_rad_s > 0.0
            and isfinite(self._config.min_effective_wz_rad_s)
            and abs(wz_cmd) > 0.0
            and abs(wz_cmd) < abs(self._config.min_effective_wz_rad_s)
        ):
            wz_cmd = copy_sign(self._config.min_effective_wz_rad_s, wz_cmd)

        wz_cmd = apply_deadband(wz_cmd, self._config.output_deadband_rad_s)

        if out.within_tolerance:
            wz_cmd = 0.0

        out.wz_cmd_rad_s = wz_cmd
        out.valid = out.dt_valid and out.yaw_valid and out.pid.valid
        return out

    def _valid_dt(self, dt_sec: float) -> bool:
        return (
            isfinite(dt_sec)
            and self._config.min_dt_sec <= dt_sec <= self._config.max_dt_sec
        )

    def _normalize_config(self) -> None:
        c = self._config

        if not isfinite(c.heading_tolerance_rad) or c.heading_tolerance_rad < 0.0:
            c.heading_tolerance_rad = 0.03

        if not isfinite(c.output_deadband_rad_s) or c.output_deadband_rad_s < 0.0:
            c.output_deadband_rad_s = 0.0

        if not isfinite(c.min_effective_wz_rad_s) or c.min_effective_wz_rad_s < 0.0:
            c.min_effective_wz_rad_s = 0.0

        if not isfinite(c.max_wz_rad_s):
            c.max_wz_rad_s = 0.0
        elif c.max_wz_rad_s < 0.0:
            c.max_wz_rad_s = abs(c.max_wz_rad_s)

        if (
            not isfinite(c.target_change_reset_threshold_rad)
            or c.target_change_reset_threshold_rad < 0.0
        ):
            c.target_change_reset_threshold_rad = 0.02

        if not isfinite(c.min_dt_sec) or c.min_dt_sec <= 0.0:
            c.min_dt_sec = 1e-6

        if not isfinite(c.max_dt_sec) or c.max_dt_sec < c.min_dt_sec:
            c.max_dt_sec = max(1.0, c.min_dt_sec)


HeadingPid = HeadingController


if __name__ == "__main__":
    pid_cfg = PidConfig(kp=1.5, ki=0.0, kd=0.05)
    pid_cfg.set_output_symmetric(0.6)

    cfg = HeadingControllerConfig(
        pid=pid_cfg,
        heading_tolerance_rad=0.03,
        output_deadband_rad_s=0.01,
        min_effective_wz_rad_s=0.05,
        max_wz_rad_s=0.5,
        min_dt_sec=1e-4,
        max_dt_sec=0.5,
    )

    ctrl = HeadingController(cfg)
    ctrl.set_target_yaw(math.radians(90.0))

    print("=== heading_pid_py.py self-test ===")
    for i, yaw in enumerate(
        [
            math.radians(0),
            math.radians(25),
            math.radians(55),
            math.radians(80),
            math.radians(89),
            math.radians(90),
        ],
        start=1,
    ):
        result = ctrl.update(yaw, 0.05)
        print(
            f"step={i:02d} yaw={math.degrees(result.current_yaw_rad):6.1f}deg "
            f"target={math.degrees(result.target_yaw_rad):6.1f}deg "
            f"err={math.degrees(result.error_yaw_rad):+6.2f}deg "
            f"wz={result.wz_cmd_rad_s:+.3f}rad/s "
            f"tol={int(result.within_tolerance)} "
            f"valid={int(result.valid)}"
        )
