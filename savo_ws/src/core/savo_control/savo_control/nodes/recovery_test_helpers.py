# -*- coding: utf-8 -*-

"""Pure helpers for recovery_test_manager_node.py."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping

from savo_control.models import TwistCommand
from savo_control.utils import clamp, finite_or_zero


RECOVERY_TEST_SOURCE = "recovery_test"


class RecoveryTestState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    FINISHED = "FINISHED"
    UNKNOWN_TEST = "UNKNOWN_TEST"
    SAFETY_STOP = "SAFETY_STOP"
    TIMEOUT = "TIMEOUT"
    DISABLED = "DISABLED"


@dataclass(frozen=True)
class RecoveryTestLimits:
    max_backup_vx: float = 0.10
    max_turn_wz: float = 0.35
    max_duration_s: float = 8.0

    def sanitized(self) -> "RecoveryTestLimits":
        return RecoveryTestLimits(
            max_backup_vx=max(0.0, abs(finite_or_zero(self.max_backup_vx))),
            max_turn_wz=max(0.0, abs(finite_or_zero(self.max_turn_wz))),
            max_duration_s=max(0.1, finite_or_zero(self.max_duration_s)),
        )


@dataclass(frozen=True)
class RecoveryTestStage:
    name: str
    duration_s: float
    command: TwistCommand
    request_recovery: bool = False

    def sanitized(self, limits: RecoveryTestLimits) -> "RecoveryTestStage":
        safe = limits.sanitized()

        return RecoveryTestStage(
            name=str(self.name),
            duration_s=max(0.0, finite_or_zero(self.duration_s)),
            command=self.command.sanitized().clamp(
                max_vx=safe.max_backup_vx,
                max_vy=safe.max_backup_vx,
                max_wz=safe.max_turn_wz,
            ),
            request_recovery=bool(self.request_recovery),
        )


@dataclass(frozen=True)
class RecoveryTestProfile:
    name: str
    stages: tuple[RecoveryTestStage, ...]

    def sanitized(self, limits: RecoveryTestLimits) -> "RecoveryTestProfile":
        return RecoveryTestProfile(
            name=str(self.name),
            stages=tuple(stage.sanitized(limits) for stage in self.stages),
        )

    def empty(self) -> bool:
        return not self.stages

    def total_duration_s(self) -> float:
        return sum(stage.duration_s for stage in self.stages)


@dataclass(frozen=True)
class ActiveRecoveryTest:
    profile: RecoveryTestProfile
    start_s: float
    stage_start_s: float
    stage_index: int = 0

    def elapsed_s(self, *, now_s: float) -> float:
        return max(0.0, finite_or_zero(now_s) - self.start_s)

    def stage_elapsed_s(self, *, now_s: float) -> float:
        return max(0.0, finite_or_zero(now_s) - self.stage_start_s)


@dataclass(frozen=True)
class RecoveryTestStepResult:
    state: RecoveryTestState
    command: TwistCommand
    request_recovery: bool
    active_test: ActiveRecoveryTest | None
    finished: bool = False
    reason: str = ""


def normalize_test_name(name: object) -> str:
    return str(name or "").strip()


def stop_command(*, stamp_sec: float = 0.0) -> TwistCommand:
    return TwistCommand.zero(source=RECOVERY_TEST_SOURCE, stamp_sec=stamp_sec)


def command_from_values(
    *,
    vx: float = 0.0,
    vy: float = 0.0,
    wz: float = 0.0,
    limits: RecoveryTestLimits,
    stamp_sec: float = 0.0,
) -> TwistCommand:
    safe = limits.sanitized()

    return TwistCommand(
        vx=finite_or_zero(vx),
        vy=finite_or_zero(vy),
        wz=finite_or_zero(wz),
        source=RECOVERY_TEST_SOURCE,
        stamp_sec=stamp_sec,
    ).clamp(
        max_vx=safe.max_backup_vx,
        max_vy=safe.max_backup_vx,
        max_wz=safe.max_turn_wz,
    )


def request_only_profile(name: str = "request_only") -> RecoveryTestProfile:
    return RecoveryTestProfile(
        name=normalize_test_name(name),
        stages=(
            RecoveryTestStage(
                name="request",
                duration_s=0.5,
                command=stop_command(),
                request_recovery=True,
            ),
        ),
    )


def backup_profile(
    *,
    name: str,
    duration_s: float,
    vx: float,
    limits: RecoveryTestLimits,
) -> RecoveryTestProfile:
    return RecoveryTestProfile(
        name=normalize_test_name(name),
        stages=(
            RecoveryTestStage(
                name="backup",
                duration_s=max(0.0, finite_or_zero(duration_s)),
                command=command_from_values(vx=vx, limits=limits),
                request_recovery=True,
            ),
        ),
    ).sanitized(limits)


def rotate_profile(
    *,
    name: str,
    duration_s: float,
    wz: float,
    limits: RecoveryTestLimits,
) -> RecoveryTestProfile:
    return RecoveryTestProfile(
        name=normalize_test_name(name),
        stages=(
            RecoveryTestStage(
                name="rotate",
                duration_s=max(0.0, finite_or_zero(duration_s)),
                command=command_from_values(wz=wz, limits=limits),
                request_recovery=True,
            ),
        ),
    ).sanitized(limits)


def backup_then_rotate_profile(
    *,
    name: str,
    backup_s: float,
    rotate_s: float,
    vx: float,
    wz: float,
    limits: RecoveryTestLimits,
) -> RecoveryTestProfile:
    return RecoveryTestProfile(
        name=normalize_test_name(name),
        stages=(
            RecoveryTestStage(
                name="backup",
                duration_s=max(0.0, finite_or_zero(backup_s)),
                command=command_from_values(vx=vx, limits=limits),
                request_recovery=True,
            ),
            RecoveryTestStage(
                name="rotate",
                duration_s=max(0.0, finite_or_zero(rotate_s)),
                command=command_from_values(wz=wz, limits=limits),
                request_recovery=True,
            ),
        ),
    ).sanitized(limits)


def default_recovery_tests(
    limits: RecoveryTestLimits | None = None,
) -> dict[str, RecoveryTestProfile]:
    safe = (limits or RecoveryTestLimits()).sanitized()

    profiles = [
        request_only_profile("request_only"),
        backup_profile(
            name="backup_short",
            duration_s=1.0,
            vx=-0.06,
            limits=safe,
        ),
        backup_profile(
            name="backup_long",
            duration_s=2.0,
            vx=-0.06,
            limits=safe,
        ),
        rotate_profile(
            name="rotate_left",
            duration_s=1.2,
            wz=0.25,
            limits=safe,
        ),
        rotate_profile(
            name="rotate_right",
            duration_s=1.2,
            wz=-0.25,
            limits=safe,
        ),
        backup_then_rotate_profile(
            name="backup_then_left",
            backup_s=1.0,
            rotate_s=1.0,
            vx=-0.06,
            wz=0.25,
            limits=safe,
        ),
        backup_then_rotate_profile(
            name="backup_then_right",
            backup_s=1.0,
            rotate_s=1.0,
            vx=-0.06,
            wz=-0.25,
            limits=safe,
        ),
    ]

    return {profile.name: profile for profile in profiles}


def profile_from_mapping(
    name: str,
    data: Mapping[str, object],
    *,
    limits: RecoveryTestLimits,
) -> RecoveryTestProfile:
    test_type = str(data.get("type", "backup"))

    if test_type == "request_only":
        return request_only_profile(name).sanitized(limits)

    if test_type == "rotate":
        return rotate_profile(
            name=name,
            duration_s=float(data.get("duration_s", 1.0)),
            wz=float(data.get("wz", 0.25)),
            limits=limits,
        )

    if test_type == "backup_then_rotate":
        return backup_then_rotate_profile(
            name=name,
            backup_s=float(data.get("backup_s", 1.0)),
            rotate_s=float(data.get("rotate_s", 1.0)),
            vx=float(data.get("vx", -0.06)),
            wz=float(data.get("wz", 0.25)),
            limits=limits,
        )

    return backup_profile(
        name=name,
        duration_s=float(data.get("duration_s", 1.0)),
        vx=float(data.get("vx", -0.06)),
        limits=limits,
    )


def start_active_test(
    profile: RecoveryTestProfile,
    *,
    now_s: float,
) -> ActiveRecoveryTest:
    now = finite_or_zero(now_s)
    return ActiveRecoveryTest(
        profile=profile,
        start_s=now,
        stage_start_s=now,
        stage_index=0,
    )


def step_active_test(
    active: ActiveRecoveryTest,
    *,
    now_s: float,
    max_duration_s: float,
) -> RecoveryTestStepResult:
    profile = active.profile

    if profile.empty():
        return RecoveryTestStepResult(
            state=RecoveryTestState.FINISHED,
            command=stop_command(stamp_sec=now_s),
            request_recovery=False,
            active_test=None,
            finished=True,
            reason="empty_profile",
        )

    if active.elapsed_s(now_s=now_s) > max(0.0, finite_or_zero(max_duration_s)):
        return RecoveryTestStepResult(
            state=RecoveryTestState.TIMEOUT,
            command=stop_command(stamp_sec=now_s),
            request_recovery=False,
            active_test=None,
            finished=True,
            reason="timeout",
        )

    stage = profile.stages[min(active.stage_index, len(profile.stages) - 1)]

    if active.stage_elapsed_s(now_s=now_s) >= stage.duration_s:
        next_index = active.stage_index + 1

        if next_index >= len(profile.stages):
            return RecoveryTestStepResult(
                state=RecoveryTestState.FINISHED,
                command=stop_command(stamp_sec=now_s),
                request_recovery=False,
                active_test=None,
                finished=True,
                reason="finished",
            )

        next_active = ActiveRecoveryTest(
            profile=profile,
            start_s=active.start_s,
            stage_start_s=finite_or_zero(now_s),
            stage_index=next_index,
        )
        next_stage = profile.stages[next_index]
        return RecoveryTestStepResult(
            state=RecoveryTestState.RUNNING,
            command=next_stage.command.sanitized(),
            request_recovery=next_stage.request_recovery,
            active_test=next_active,
            reason=next_stage.name,
        )

    return RecoveryTestStepResult(
        state=RecoveryTestState.RUNNING,
        command=stage.command.sanitized(),
        request_recovery=stage.request_recovery,
        active_test=active,
        reason=stage.name,
    )


def status_text(
    *,
    state: RecoveryTestState,
    active_name: str = "",
    reason: str = "",
    enabled: bool = True,
    safety_stop: bool = False,
    request_recovery: bool = False,
    command: TwistCommand | None = None,
) -> str:
    cmd = (command or stop_command()).sanitized()

    return (
        f"state={state.value}; "
        f"enabled={str(enabled).lower()}; "
        f"active_test={active_name or 'none'}; "
        f"reason={reason}; "
        f"safety_stop={str(safety_stop).lower()}; "
        f"request_recovery={str(request_recovery).lower()}; "
        f"vx={cmd.vx:.3f}; vy={cmd.vy:.3f}; wz={cmd.wz:.3f}"
    )


__all__ = [
    "RECOVERY_TEST_SOURCE",
    "ActiveRecoveryTest",
    "RecoveryTestLimits",
    "RecoveryTestProfile",
    "RecoveryTestStage",
    "RecoveryTestState",
    "RecoveryTestStepResult",
    "backup_profile",
    "backup_then_rotate_profile",
    "command_from_values",
    "default_recovery_tests",
    "normalize_test_name",
    "profile_from_mapping",
    "request_only_profile",
    "rotate_profile",
    "start_active_test",
    "status_text",
    "step_active_test",
    "stop_command",
]
