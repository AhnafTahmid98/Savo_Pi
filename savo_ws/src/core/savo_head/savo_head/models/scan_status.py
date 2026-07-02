# -*- coding: utf-8 -*-

"""Staged pan-tilt scan models for Python fallback tools and tests."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Final, Tuple

from savo_head.constants import (
    PAN_CENTER_DEG_DEFAULT,
    PAN_MAX_DEG_DEFAULT,
    PAN_MIN_DEG_DEFAULT,
    SCAN_PAN_STEP_DEG_DEFAULT,
    SCAN_PAN_TARGETS_DEFAULT,
    SCAN_STEP_DELAY_S_DEFAULT,
    SCAN_TILT_STEP_DEG_DEFAULT,
    SCAN_TILT_SWEEP_PAN_TARGETS,
    TILT_MAX_DEG_DEFAULT,
    TILT_MIN_DEG_DEFAULT,
)
from savo_head.models.pantilt_command import PanTiltCommand, absolute_command, hold_command


SCAN_MODE_STAGED: Final[str] = "staged"

SCAN_STATE_IDLE: Final[str] = "idle"
SCAN_STATE_RUNNING: Final[str] = "running"
SCAN_STATE_PAUSED: Final[str] = "paused"
SCAN_STATE_DONE: Final[str] = "done"
SCAN_STATE_ERROR: Final[str] = "error"

SCAN_PHASE_PAN: Final[str] = "pan"
SCAN_PHASE_TILT_SWEEP: Final[str] = "tilt_sweep"

TILT_DIRECTION_UP: Final[int] = 1
TILT_DIRECTION_DOWN: Final[int] = -1

VALID_SCAN_STATES: Final[tuple[str, ...]] = (
    SCAN_STATE_IDLE,
    SCAN_STATE_RUNNING,
    SCAN_STATE_PAUSED,
    SCAN_STATE_DONE,
    SCAN_STATE_ERROR,
)

VALID_SCAN_PHASES: Final[tuple[str, ...]] = (
    SCAN_PHASE_PAN,
    SCAN_PHASE_TILT_SWEEP,
)


@dataclass(frozen=True)
class ScanProfile:
    name: str = "semantic_scan"
    enabled: bool = True
    mode: str = SCAN_MODE_STAGED

    pan_min_deg: int = PAN_MIN_DEG_DEFAULT
    pan_center_deg: int = PAN_CENTER_DEG_DEFAULT
    pan_max_deg: int = PAN_MAX_DEG_DEFAULT

    tilt_min_deg: int = TILT_MIN_DEG_DEFAULT
    tilt_max_deg: int = TILT_MAX_DEG_DEFAULT

    pan_step_deg: int = SCAN_PAN_STEP_DEG_DEFAULT
    tilt_step_deg: int = SCAN_TILT_STEP_DEG_DEFAULT
    step_delay_s: float = SCAN_STEP_DELAY_S_DEFAULT

    start_pan_deg: int = PAN_MIN_DEG_DEFAULT
    start_tilt_deg: int = TILT_MIN_DEG_DEFAULT

    pan_targets_deg: Tuple[int, ...] = SCAN_PAN_TARGETS_DEFAULT
    tilt_sweep_pan_targets_deg: Tuple[int, ...] = SCAN_TILT_SWEEP_PAN_TARGETS

    hold_at_pan_target_s: float = 0.10
    hold_after_tilt_sweep_s: float = 0.15

    pause_on_manual_command: bool = True
    resume_after_manual_s: float = 0.0
    center_on_stop: bool = True

    def normalized(self) -> "ScanProfile":
        pan_min = int(min(self.pan_min_deg, self.pan_max_deg))
        pan_max = int(max(self.pan_min_deg, self.pan_max_deg))
        tilt_min = int(min(self.tilt_min_deg, self.tilt_max_deg))
        tilt_max = int(max(self.tilt_min_deg, self.tilt_max_deg))

        def clamp_pan(value: int) -> int:
            return max(pan_min, min(pan_max, int(value)))

        def clamp_tilt(value: int) -> int:
            return max(tilt_min, min(tilt_max, int(value)))

        targets = tuple(clamp_pan(v) for v in self.pan_targets_deg)
        sweep_targets = tuple(clamp_pan(v) for v in self.tilt_sweep_pan_targets_deg)

        if not targets:
            targets = (clamp_pan(self.pan_center_deg),)

        return replace(
            self,
            mode=self.mode if self.mode == SCAN_MODE_STAGED else SCAN_MODE_STAGED,
            pan_min_deg=pan_min,
            pan_center_deg=clamp_pan(self.pan_center_deg),
            pan_max_deg=pan_max,
            tilt_min_deg=tilt_min,
            tilt_max_deg=tilt_max,
            pan_step_deg=max(1, int(self.pan_step_deg)),
            tilt_step_deg=max(1, int(self.tilt_step_deg)),
            step_delay_s=max(0.01, float(self.step_delay_s)),
            start_pan_deg=clamp_pan(self.start_pan_deg),
            start_tilt_deg=clamp_tilt(self.start_tilt_deg),
            pan_targets_deg=targets,
            tilt_sweep_pan_targets_deg=sweep_targets,
            hold_at_pan_target_s=max(0.0, float(self.hold_at_pan_target_s)),
            hold_after_tilt_sweep_s=max(0.0, float(self.hold_after_tilt_sweep_s)),
            resume_after_manual_s=max(0.0, float(self.resume_after_manual_s)),
        )

    def validation_errors(self) -> list[str]:
        errors: list[str] = []

        if self.mode != SCAN_MODE_STAGED:
            errors.append(f"unsupported scan mode: {self.mode!r}")

        if not self.pan_targets_deg:
            errors.append("pan_targets_deg must not be empty")

        if self.pan_min_deg >= self.pan_max_deg:
            errors.append("pan_min_deg must be lower than pan_max_deg")

        if self.tilt_min_deg >= self.tilt_max_deg:
            errors.append("tilt_min_deg must be lower than tilt_max_deg")

        if self.pan_step_deg <= 0:
            errors.append("pan_step_deg must be positive")

        if self.tilt_step_deg <= 0:
            errors.append("tilt_step_deg must be positive")

        if self.step_delay_s <= 0.0:
            errors.append("step_delay_s must be positive")

        return errors

    def is_valid(self) -> bool:
        return not self.validation_errors()

    def needs_tilt_sweep_at(self, pan_deg: int) -> bool:
        return int(pan_deg) in set(int(v) for v in self.tilt_sweep_pan_targets_deg)


@dataclass(frozen=True)
class ScanStatus:
    state: str = SCAN_STATE_IDLE
    phase: str = SCAN_PHASE_PAN

    pan_deg: int = PAN_MIN_DEG_DEFAULT
    tilt_deg: int = TILT_MIN_DEG_DEFAULT

    pan_target_index: int = 0
    current_pan_target_deg: int = PAN_CENTER_DEG_DEFAULT
    tilt_direction: int = TILT_DIRECTION_UP

    cycle_count: int = 0
    step_count: int = 0

    stamp_s: float = 0.0
    last_transition_s: float = 0.0
    error: str = ""

    def normalized(self, profile: ScanProfile | None = None) -> "ScanStatus":
        p = (profile or ScanProfile()).normalized()
        targets = p.pan_targets_deg
        index = int(self.pan_target_index) % len(targets)

        pan = max(p.pan_min_deg, min(p.pan_max_deg, int(self.pan_deg)))
        tilt = max(p.tilt_min_deg, min(p.tilt_max_deg, int(self.tilt_deg)))

        direction = (
            TILT_DIRECTION_DOWN
            if int(self.tilt_direction) == TILT_DIRECTION_DOWN
            else TILT_DIRECTION_UP
        )

        return replace(
            self,
            state=self.state if self.state in VALID_SCAN_STATES else SCAN_STATE_IDLE,
            phase=self.phase if self.phase in VALID_SCAN_PHASES else SCAN_PHASE_PAN,
            pan_deg=pan,
            tilt_deg=tilt,
            pan_target_index=index,
            current_pan_target_deg=targets[index],
            tilt_direction=direction,
            cycle_count=max(0, int(self.cycle_count)),
            step_count=max(0, int(self.step_count)),
            stamp_s=float(self.stamp_s),
            last_transition_s=float(self.last_transition_s),
            error=str(self.error),
        )

    def running(self) -> bool:
        return self.state == SCAN_STATE_RUNNING

    def paused(self) -> bool:
        return self.state == SCAN_STATE_PAUSED

    def current_target(self, profile: ScanProfile | None = None) -> int:
        p = (profile or ScanProfile()).normalized()
        return p.pan_targets_deg[int(self.pan_target_index) % len(p.pan_targets_deg)]

    def to_dict(self) -> dict:
        return {
            "state": self.state,
            "phase": self.phase,
            "pan_deg": int(self.pan_deg),
            "tilt_deg": int(self.tilt_deg),
            "pan_target_index": int(self.pan_target_index),
            "current_pan_target_deg": int(self.current_pan_target_deg),
            "tilt_direction": int(self.tilt_direction),
            "cycle_count": int(self.cycle_count),
            "step_count": int(self.step_count),
            "stamp_s": float(self.stamp_s),
            "last_transition_s": float(self.last_transition_s),
            "error": self.error,
        }


def move_toward(current: int, target: int, step: int) -> int:
    current_i = int(current)
    target_i = int(target)
    step_i = max(1, int(step))

    if current_i < target_i:
        return min(current_i + step_i, target_i)
    if current_i > target_i:
        return max(current_i - step_i, target_i)
    return current_i


def start_scan(profile: ScanProfile | None = None, stamp_s: float = 0.0) -> ScanStatus:
    p = (profile or ScanProfile()).normalized()
    return ScanStatus(
        state=SCAN_STATE_RUNNING,
        phase=SCAN_PHASE_PAN,
        pan_deg=p.start_pan_deg,
        tilt_deg=p.start_tilt_deg,
        pan_target_index=0,
        current_pan_target_deg=p.pan_targets_deg[0],
        tilt_direction=TILT_DIRECTION_UP,
        stamp_s=float(stamp_s),
        last_transition_s=float(stamp_s),
    )


def pause_scan(status: ScanStatus, stamp_s: float = 0.0) -> ScanStatus:
    return replace(
        status.normalized(),
        state=SCAN_STATE_PAUSED,
        stamp_s=float(stamp_s),
        last_transition_s=float(stamp_s),
    )


def resume_scan(status: ScanStatus, stamp_s: float = 0.0) -> ScanStatus:
    return replace(
        status.normalized(),
        state=SCAN_STATE_RUNNING,
        stamp_s=float(stamp_s),
        last_transition_s=float(stamp_s),
    )


def stop_scan(status: ScanStatus, stamp_s: float = 0.0, error: str = "") -> ScanStatus:
    return replace(
        status.normalized(),
        state=SCAN_STATE_ERROR if error else SCAN_STATE_DONE,
        stamp_s=float(stamp_s),
        last_transition_s=float(stamp_s),
        error=str(error),
    )


def next_pan_target_index(profile: ScanProfile, index: int) -> tuple[int, int]:
    p = profile.normalized()
    next_index = (int(index) + 1) % len(p.pan_targets_deg)
    wrapped = 1 if next_index == 0 else 0
    return next_index, wrapped


def advance_scan_step(
    profile: ScanProfile,
    status: ScanStatus,
    stamp_s: float = 0.0,
) -> tuple[ScanStatus, PanTiltCommand]:
    p = profile.normalized()
    s = status.normalized(p)

    if not p.enabled:
        return stop_scan(s, stamp_s, "scan_disabled"), hold_command(source="scan", stamp_s=stamp_s)

    if s.state in (SCAN_STATE_IDLE, SCAN_STATE_DONE):
        started = start_scan(p, stamp_s)
        return started, absolute_command(
            pan_deg=started.pan_deg,
            tilt_deg=started.tilt_deg,
            source="scan",
            stamp_s=stamp_s,
            reason="scan_start",
        )

    if s.state == SCAN_STATE_PAUSED:
        return s, hold_command(source="scan", stamp_s=stamp_s, reason="scan_paused")

    if s.state == SCAN_STATE_ERROR:
        return s, hold_command(source="scan", stamp_s=stamp_s, reason="scan_error")

    target = s.current_target(p)

    if s.phase == SCAN_PHASE_PAN:
        new_pan = move_toward(s.pan_deg, target, p.pan_step_deg)
        new_status = replace(
            s,
            pan_deg=new_pan,
            current_pan_target_deg=target,
            step_count=s.step_count + 1,
            stamp_s=float(stamp_s),
        )

        if new_pan == target:
            if p.needs_tilt_sweep_at(target):
                new_status = replace(
                    new_status,
                    phase=SCAN_PHASE_TILT_SWEEP,
                    tilt_deg=p.tilt_min_deg,
                    tilt_direction=TILT_DIRECTION_UP,
                    last_transition_s=float(stamp_s),
                )
                return new_status, absolute_command(
                    pan_deg=new_pan,
                    tilt_deg=p.tilt_min_deg,
                    source="scan",
                    stamp_s=stamp_s,
                    reason="scan_tilt_start",
                )

            next_index, wrapped = next_pan_target_index(p, s.pan_target_index)
            new_status = replace(
                new_status,
                pan_target_index=next_index,
                current_pan_target_deg=p.pan_targets_deg[next_index],
                cycle_count=s.cycle_count + wrapped,
                last_transition_s=float(stamp_s),
            )

        return new_status, absolute_command(
            pan_deg=new_status.pan_deg,
            tilt_deg=new_status.tilt_deg,
            source="scan",
            stamp_s=stamp_s,
            reason="scan_pan",
        )

    next_tilt = s.tilt_deg + (s.tilt_direction * p.tilt_step_deg)
    direction = s.tilt_direction
    phase = SCAN_PHASE_TILT_SWEEP
    next_index = s.pan_target_index
    cycle_count = s.cycle_count

    if next_tilt >= p.tilt_max_deg:
        next_tilt = p.tilt_max_deg
        direction = TILT_DIRECTION_DOWN

    elif next_tilt <= p.tilt_min_deg:
        next_tilt = p.tilt_min_deg
        phase = SCAN_PHASE_PAN
        next_index, wrapped = next_pan_target_index(p, s.pan_target_index)
        cycle_count += wrapped

    new_status = replace(
        s,
        phase=phase,
        tilt_deg=next_tilt,
        tilt_direction=direction,
        pan_target_index=next_index,
        current_pan_target_deg=p.pan_targets_deg[next_index],
        cycle_count=cycle_count,
        step_count=s.step_count + 1,
        stamp_s=float(stamp_s),
        last_transition_s=float(stamp_s) if phase == SCAN_PHASE_PAN else s.last_transition_s,
    )

    return new_status, absolute_command(
        pan_deg=new_status.pan_deg,
        tilt_deg=new_status.tilt_deg,
        source="scan",
        stamp_s=stamp_s,
        reason="scan_tilt",
    )


__all__ = [
    "SCAN_MODE_STAGED",
    "SCAN_STATE_IDLE",
    "SCAN_STATE_RUNNING",
    "SCAN_STATE_PAUSED",
    "SCAN_STATE_DONE",
    "SCAN_STATE_ERROR",
    "SCAN_PHASE_PAN",
    "SCAN_PHASE_TILT_SWEEP",
    "TILT_DIRECTION_UP",
    "TILT_DIRECTION_DOWN",
    "VALID_SCAN_STATES",
    "VALID_SCAN_PHASES",
    "ScanProfile",
    "ScanStatus",
    "move_toward",
    "start_scan",
    "pause_scan",
    "resume_scan",
    "stop_scan",
    "next_pan_target_index",
    "advance_scan_step",
]
