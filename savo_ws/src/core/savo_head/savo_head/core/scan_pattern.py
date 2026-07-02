# -*- coding: utf-8 -*-

"""Reusable staged scan helpers for Robot Savo head fallback tools."""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Iterable, Optional

import yaml

from savo_head.models.pantilt_command import PanTiltCommand
from savo_head.models.scan_status import (
    SCAN_STATE_DONE,
    SCAN_STATE_ERROR,
    SCAN_STATE_IDLE,
    SCAN_STATE_PAUSED,
    SCAN_STATE_RUNNING,
    ScanProfile,
    ScanStatus,
    advance_scan_step,
    pause_scan,
    resume_scan,
    start_scan,
    stop_scan,
)


SCAN_PARAM_PREFIX = "semantic_scan_"


@dataclass(frozen=True)
class ScanStepResult:
    status: ScanStatus
    command: PanTiltCommand

    def to_dict(self) -> dict:
        return {
            "status": self.status.to_dict(),
            "command": self.command.to_dict(),
        }


@dataclass(frozen=True)
class ScanRuntime:
    profile: ScanProfile = ScanProfile()
    status: ScanStatus = ScanStatus()
    last_command: Optional[PanTiltCommand] = None

    def normalized(self) -> "ScanRuntime":
        profile = self.profile.normalized()
        return ScanRuntime(
            profile=profile,
            status=self.status.normalized(profile),
            last_command=self.last_command,
        )

    def running(self) -> bool:
        return self.status.state == SCAN_STATE_RUNNING

    def paused(self) -> bool:
        return self.status.state == SCAN_STATE_PAUSED

    def stopped(self) -> bool:
        return self.status.state in (
            SCAN_STATE_IDLE,
            SCAN_STATE_DONE,
            SCAN_STATE_ERROR,
        )

    def start(self, stamp_s: float = 0.0) -> "ScanRuntime":
        profile = self.profile.normalized()
        return replace(
            self,
            profile=profile,
            status=start_scan(profile, stamp_s=stamp_s),
            last_command=None,
        )

    def pause(self, stamp_s: float = 0.0) -> "ScanRuntime":
        return replace(
            self,
            status=pause_scan(self.status, stamp_s=stamp_s),
        )

    def resume(self, stamp_s: float = 0.0) -> "ScanRuntime":
        return replace(
            self,
            status=resume_scan(self.status, stamp_s=stamp_s),
        )

    def stop(self, stamp_s: float = 0.0, error: str = "") -> "ScanRuntime":
        return replace(
            self,
            status=stop_scan(self.status, stamp_s=stamp_s, error=error),
            last_command=None,
        )

    def step(self, stamp_s: float = 0.0) -> tuple["ScanRuntime", ScanStepResult]:
        profile = self.profile.normalized()
        status, command = advance_scan_step(profile, self.status, stamp_s=stamp_s)

        runtime = replace(
            self,
            profile=profile,
            status=status,
            last_command=command,
        )

        return runtime, ScanStepResult(status=status, command=command)


def _tuple_ints(value: Any, fallback: Iterable[int]) -> tuple[int, ...]:
    if value is None:
        return tuple(int(v) for v in fallback)

    if isinstance(value, str):
        items = [item.strip() for item in value.split(",") if item.strip()]
        return tuple(int(item) for item in items)

    return tuple(int(item) for item in value)


def profile_from_params(params: dict[str, Any]) -> ScanProfile:
    return ScanProfile(
        name=str(params.get("default_scan_profile", "semantic_scan")),
        enabled=bool(params.get("semantic_scan_enabled", True)),
        mode=str(params.get("semantic_scan_mode", "staged")),
        pan_min_deg=int(params.get("semantic_scan_pan_min_deg", 0)),
        pan_center_deg=int(params.get("semantic_scan_pan_center_deg", 72)),
        pan_max_deg=int(params.get("semantic_scan_pan_max_deg", 170)),
        tilt_min_deg=int(params.get("semantic_scan_tilt_min_deg", 45)),
        tilt_max_deg=int(params.get("semantic_scan_tilt_max_deg", 130)),
        pan_step_deg=int(params.get("semantic_scan_pan_step_deg", 2)),
        tilt_step_deg=int(params.get("semantic_scan_tilt_step_deg", 2)),
        step_delay_s=float(params.get("semantic_scan_step_delay_s", 0.12)),
        start_pan_deg=int(params.get("semantic_scan_start_pan_deg", 0)),
        start_tilt_deg=int(params.get("semantic_scan_start_tilt_deg", 45)),
        pan_targets_deg=_tuple_ints(
            params.get("semantic_scan_pan_targets_deg"),
            fallback=(72, 170, 72, 0),
        ),
        tilt_sweep_pan_targets_deg=_tuple_ints(
            params.get("semantic_scan_tilt_sweep_pan_targets_deg"),
            fallback=(72,),
        ),
        hold_at_pan_target_s=float(params.get("semantic_scan_hold_at_pan_target_s", 0.10)),
        hold_after_tilt_sweep_s=float(
            params.get("semantic_scan_hold_after_tilt_sweep_s", 0.15)
        ),
        pause_on_manual_command=bool(
            params.get("semantic_scan_pause_on_manual_command", True)
        ),
        resume_after_manual_s=float(params.get("semantic_scan_resume_after_manual_s", 0.0)),
        center_on_stop=bool(params.get("semantic_scan_center_on_stop", True)),
    ).normalized()


def profile_to_params(profile: ScanProfile) -> dict[str, Any]:
    item = profile.normalized()
    return {
        "default_scan_profile": item.name,
        "semantic_scan_enabled": item.enabled,
        "semantic_scan_mode": item.mode,
        "semantic_scan_pan_min_deg": item.pan_min_deg,
        "semantic_scan_pan_center_deg": item.pan_center_deg,
        "semantic_scan_pan_max_deg": item.pan_max_deg,
        "semantic_scan_tilt_min_deg": item.tilt_min_deg,
        "semantic_scan_tilt_max_deg": item.tilt_max_deg,
        "semantic_scan_pan_step_deg": item.pan_step_deg,
        "semantic_scan_tilt_step_deg": item.tilt_step_deg,
        "semantic_scan_step_delay_s": item.step_delay_s,
        "semantic_scan_start_pan_deg": item.start_pan_deg,
        "semantic_scan_start_tilt_deg": item.start_tilt_deg,
        "semantic_scan_pan_targets_deg": list(item.pan_targets_deg),
        "semantic_scan_tilt_sweep_pan_targets_deg": list(item.tilt_sweep_pan_targets_deg),
        "semantic_scan_hold_at_pan_target_s": item.hold_at_pan_target_s,
        "semantic_scan_hold_after_tilt_sweep_s": item.hold_after_tilt_sweep_s,
        "semantic_scan_pause_on_manual_command": item.pause_on_manual_command,
        "semantic_scan_resume_after_manual_s": item.resume_after_manual_s,
        "semantic_scan_center_on_stop": item.center_on_stop,
    }


def load_scan_profile_yaml(path: str | Path, node_name: str = "savo_head") -> ScanProfile:
    data = yaml.safe_load(Path(path).read_text()) or {}
    params = data.get(node_name, {}).get("ros__parameters", {})
    return profile_from_params(params)


def validate_scan_profile(profile: ScanProfile) -> list[str]:
    return profile.validation_errors()


def make_scan_runtime(profile: ScanProfile | None = None) -> ScanRuntime:
    item = (profile or ScanProfile()).normalized()
    return ScanRuntime(profile=item, status=ScanStatus().normalized(item))


def preview_scan(
    profile: ScanProfile | None = None,
    *,
    steps: int = 40,
    start_stamp_s: float = 0.0,
    dt_s: float | None = None,
) -> list[ScanStepResult]:
    item = (profile or ScanProfile()).normalized()
    runtime = make_scan_runtime(item).start(stamp_s=start_stamp_s)
    out: list[ScanStepResult] = []

    step_dt = item.step_delay_s if dt_s is None else float(dt_s)
    stamp = float(start_stamp_s)

    for _ in range(max(0, int(steps))):
        stamp += step_dt
        runtime, result = runtime.step(stamp_s=stamp)
        out.append(result)

    return out


def pan_targets_from_preview(results: Iterable[ScanStepResult]) -> tuple[int, ...]:
    targets: list[int] = []

    for result in results:
        pan = int(result.status.current_pan_target_deg)
        if not targets or targets[-1] != pan:
            targets.append(pan)

    return tuple(targets)


__all__ = [
    "SCAN_PARAM_PREFIX",
    "ScanStepResult",
    "ScanRuntime",
    "profile_from_params",
    "profile_to_params",
    "load_scan_profile_yaml",
    "validate_scan_profile",
    "make_scan_runtime",
    "preview_scan",
    "pan_targets_from_preview",
]
