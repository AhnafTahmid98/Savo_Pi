# -*- coding: utf-8 -*-

"""Pure helpers for auto_test_manager_node.py."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping

from savo_control.models import TwistCommand
from savo_control.utils import clamp, finite_or_zero


AUTO_TEST_SOURCE = "auto_test"


class AutoTestState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    FINISHED = "FINISHED"
    UNKNOWN_TEST = "UNKNOWN_TEST"
    SAFETY_STOP = "SAFETY_STOP"
    TIMEOUT = "TIMEOUT"
    DISABLED = "DISABLED"


@dataclass(frozen=True)
class AutoTestLimits:
    max_vx: float = 0.18
    max_vy: float = 0.18
    max_wz: float = 0.45
    low_speed_scale: float = 0.50

    def sanitized(self) -> "AutoTestLimits":
        return AutoTestLimits(
            max_vx=max(0.0, abs(finite_or_zero(self.max_vx))),
            max_vy=max(0.0, abs(finite_or_zero(self.max_vy))),
            max_wz=max(0.0, abs(finite_or_zero(self.max_wz))),
            low_speed_scale=clamp(finite_or_zero(self.low_speed_scale), 0.0, 1.0),
        )


@dataclass(frozen=True)
class TestStage:
    name: str
    duration_s: float
    command: TwistCommand

    def sanitized(self, limits: AutoTestLimits) -> "TestStage":
        safe_limits = limits.sanitized()
        duration = max(0.0, finite_or_zero(self.duration_s))

        return TestStage(
            name=str(self.name),
            duration_s=duration,
            command=self.command.sanitized().clamp(
                max_vx=safe_limits.max_vx,
                max_vy=safe_limits.max_vy,
                max_wz=safe_limits.max_wz,
            ),
        )


@dataclass(frozen=True)
class AutoTestProfile:
    name: str
    test_type: str
    stages: tuple[TestStage, ...]
    pulses: int = 0
    active_s: float = 0.0
    rest_s: float = 0.0

    def sanitized(self, limits: AutoTestLimits) -> "AutoTestProfile":
        safe_type = str(self.test_type or "constant_twist")
        safe_stages = tuple(stage.sanitized(limits) for stage in self.stages)

        return AutoTestProfile(
            name=str(self.name),
            test_type=safe_type,
            stages=safe_stages,
            pulses=max(0, int(self.pulses)),
            active_s=max(0.0, finite_or_zero(self.active_s)),
            rest_s=max(0.0, finite_or_zero(self.rest_s)),
        )

    def total_duration_s(self) -> float:
        if self.test_type == "pulse_twist":
            return float(self.pulses) * (self.active_s + self.rest_s)

        return sum(stage.duration_s for stage in self.stages)

    def empty(self) -> bool:
        return not self.stages


@dataclass(frozen=True)
class ActiveAutoTest:
    profile: AutoTestProfile
    start_s: float
    stage_index: int = 0
    pulses_done: int = 0
    pulse_active: bool = True
    last_pulse_switch_s: float = 0.0

    def elapsed_s(self, *, now_s: float) -> float:
        return max(0.0, finite_or_zero(now_s) - self.start_s)


@dataclass(frozen=True)
class AutoTestStepResult:
    state: AutoTestState
    command: TwistCommand
    active_test: ActiveAutoTest | None
    finished: bool = False
    reason: str = ""


def normalize_test_name(name: object) -> str:
    return str(name or "").strip()


def stop_command(*, stamp_sec: float = 0.0) -> TwistCommand:
    return TwistCommand.zero(source=AUTO_TEST_SOURCE, stamp_sec=stamp_sec)


def command_from_values(
    *,
    vx: float = 0.0,
    vy: float = 0.0,
    wz: float = 0.0,
    limits: AutoTestLimits,
    stamp_sec: float = 0.0,
) -> TwistCommand:
    safe = limits.sanitized()

    return TwistCommand(
        vx=finite_or_zero(vx),
        vy=finite_or_zero(vy),
        wz=finite_or_zero(wz),
        source=AUTO_TEST_SOURCE,
        stamp_sec=stamp_sec,
    ).clamp(
        max_vx=safe.max_vx,
        max_vy=safe.max_vy,
        max_wz=safe.max_wz,
    )


def constant_profile(
    *,
    name: str,
    duration_s: float,
    vx: float,
    vy: float,
    wz: float,
    limits: AutoTestLimits,
) -> AutoTestProfile:
    return AutoTestProfile(
        name=normalize_test_name(name),
        test_type="constant_twist",
        stages=(
            TestStage(
                name=normalize_test_name(name),
                duration_s=max(0.0, finite_or_zero(duration_s)),
                command=command_from_values(
                    vx=vx,
                    vy=vy,
                    wz=wz,
                    limits=limits,
                ),
            ),
        ),
    ).sanitized(limits)


def pulse_profile(
    *,
    name: str,
    pulses: int,
    active_s: float,
    rest_s: float,
    vx: float,
    vy: float,
    wz: float,
    limits: AutoTestLimits,
) -> AutoTestProfile:
    return AutoTestProfile(
        name=normalize_test_name(name),
        test_type="pulse_twist",
        stages=(
            TestStage(
                name=normalize_test_name(name),
                duration_s=max(0.0, finite_or_zero(active_s)),
                command=command_from_values(
                    vx=vx,
                    vy=vy,
                    wz=wz,
                    limits=limits,
                ),
            ),
        ),
        pulses=max(0, int(pulses)),
        active_s=max(0.0, finite_or_zero(active_s)),
        rest_s=max(0.0, finite_or_zero(rest_s)),
    ).sanitized(limits)


def default_auto_tests(limits: AutoTestLimits | None = None) -> dict[str, AutoTestProfile]:
    safe_limits = (limits or AutoTestLimits()).sanitized()

    profiles = [
        constant_profile(
            name="forward_slow",
            duration_s=2.0,
            vx=0.10,
            vy=0.0,
            wz=0.0,
            limits=safe_limits,
        ),
        constant_profile(
            name="backward_slow",
            duration_s=2.0,
            vx=-0.10,
            vy=0.0,
            wz=0.0,
            limits=safe_limits,
        ),
        constant_profile(
            name="strafe_left_slow",
            duration_s=1.5,
            vx=0.0,
            vy=0.10,
            wz=0.0,
            limits=safe_limits,
        ),
        constant_profile(
            name="strafe_right_slow",
            duration_s=1.5,
            vx=0.0,
            vy=-0.10,
            wz=0.0,
            limits=safe_limits,
        ),
        constant_profile(
            name="rotate_ccw_slow",
            duration_s=1.5,
            vx=0.0,
            vy=0.0,
            wz=0.25,
            limits=safe_limits,
        ),
        constant_profile(
            name="rotate_cw_slow",
            duration_s=1.5,
            vx=0.0,
            vy=0.0,
            wz=-0.25,
            limits=safe_limits,
        ),
        pulse_profile(
            name="forward_pulse",
            pulses=3,
            active_s=0.6,
            rest_s=0.8,
            vx=0.10,
            vy=0.0,
            wz=0.0,
            limits=safe_limits,
        ),
        pulse_profile(
            name="rotate_pulse",
            pulses=3,
            active_s=0.5,
            rest_s=0.8,
            vx=0.0,
            vy=0.0,
            wz=0.25,
            limits=safe_limits,
        ),
    ]

    return {profile.name: profile for profile in profiles}


def profile_from_mapping(
    name: str,
    data: Mapping[str, object],
    *,
    limits: AutoTestLimits,
) -> AutoTestProfile:
    test_type = str(data.get("type", "constant_twist"))

    if test_type == "pulse_twist":
        return pulse_profile(
            name=name,
            pulses=int(data.get("pulses", 1)),
            active_s=float(data.get("active_s", 0.5)),
            rest_s=float(data.get("rest_s", 0.5)),
            vx=float(data.get("vx", 0.0)),
            vy=float(data.get("vy", 0.0)),
            wz=float(data.get("wz", 0.0)),
            limits=limits,
        )

    return constant_profile(
        name=name,
        duration_s=float(data.get("duration_s", 1.0)),
        vx=float(data.get("vx", 0.0)),
        vy=float(data.get("vy", 0.0)),
        wz=float(data.get("wz", 0.0)),
        limits=limits,
    )


def start_active_test(profile: AutoTestProfile, *, now_s: float) -> ActiveAutoTest:
    return ActiveAutoTest(
        profile=profile,
        start_s=finite_or_zero(now_s),
        stage_index=0,
        pulses_done=0,
        pulse_active=True,
        last_pulse_switch_s=finite_or_zero(now_s),
    )


def step_active_test(
    active: ActiveAutoTest,
    *,
    now_s: float,
    max_duration_s: float,
) -> AutoTestStepResult:
    profile = active.profile

    if profile.empty():
        return AutoTestStepResult(
            state=AutoTestState.FINISHED,
            command=stop_command(stamp_sec=now_s),
            active_test=None,
            finished=True,
            reason="empty_profile",
        )

    if active.elapsed_s(now_s=now_s) > max(0.0, finite_or_zero(max_duration_s)):
        return AutoTestStepResult(
            state=AutoTestState.TIMEOUT,
            command=stop_command(stamp_sec=now_s),
            active_test=None,
            finished=True,
            reason="timeout",
        )

    if profile.test_type == "pulse_twist":
        return _step_pulse_test(active, now_s=now_s)

    return _step_constant_test(active, now_s=now_s)


def _step_constant_test(active: ActiveAutoTest, *, now_s: float) -> AutoTestStepResult:
    profile = active.profile
    stage = profile.stages[min(active.stage_index, len(profile.stages) - 1)]
    elapsed_in_stage = max(0.0, finite_or_zero(now_s) - active.start_s)

    if elapsed_in_stage >= stage.duration_s:
        next_index = active.stage_index + 1

        if next_index >= len(profile.stages):
            return AutoTestStepResult(
                state=AutoTestState.FINISHED,
                command=stop_command(stamp_sec=now_s),
                active_test=None,
                finished=True,
                reason="finished",
            )

        next_active = ActiveAutoTest(
            profile=profile,
            start_s=finite_or_zero(now_s),
            stage_index=next_index,
            pulses_done=active.pulses_done,
            pulse_active=active.pulse_active,
            last_pulse_switch_s=active.last_pulse_switch_s,
        )
        next_stage = profile.stages[next_index]
        return AutoTestStepResult(
            state=AutoTestState.RUNNING,
            command=next_stage.command.sanitized(),
            active_test=next_active,
            reason=next_stage.name,
        )

    return AutoTestStepResult(
        state=AutoTestState.RUNNING,
        command=stage.command.sanitized(),
        active_test=active,
        reason=stage.name,
    )


def _step_pulse_test(active: ActiveAutoTest, *, now_s: float) -> AutoTestStepResult:
    profile = active.profile
    stage = profile.stages[0]

    if profile.pulses <= 0:
        return AutoTestStepResult(
            state=AutoTestState.FINISHED,
            command=stop_command(stamp_sec=now_s),
            active_test=None,
            finished=True,
            reason="no_pulses",
        )

    current_period = profile.active_s if active.pulse_active else profile.rest_s
    elapsed_since_switch = max(0.0, finite_or_zero(now_s) - active.last_pulse_switch_s)

    if elapsed_since_switch >= current_period:
        if active.pulse_active:
            next_active = ActiveAutoTest(
                profile=profile,
                start_s=active.start_s,
                stage_index=active.stage_index,
                pulses_done=active.pulses_done + 1,
                pulse_active=False,
                last_pulse_switch_s=finite_or_zero(now_s),
            )
        else:
            next_active = ActiveAutoTest(
                profile=profile,
                start_s=active.start_s,
                stage_index=active.stage_index,
                pulses_done=active.pulses_done,
                pulse_active=True,
                last_pulse_switch_s=finite_or_zero(now_s),
            )

        if next_active.pulses_done >= profile.pulses and not next_active.pulse_active:
            return AutoTestStepResult(
                state=AutoTestState.FINISHED,
                command=stop_command(stamp_sec=now_s),
                active_test=None,
                finished=True,
                reason="finished",
            )

        command = stage.command.sanitized() if next_active.pulse_active else stop_command(
            stamp_sec=now_s
        )
        return AutoTestStepResult(
            state=AutoTestState.RUNNING,
            command=command,
            active_test=next_active,
            reason="pulse_active" if next_active.pulse_active else "pulse_rest",
        )

    command = stage.command.sanitized() if active.pulse_active else stop_command(
        stamp_sec=now_s
    )
    return AutoTestStepResult(
        state=AutoTestState.RUNNING,
        command=command,
        active_test=active,
        reason="pulse_active" if active.pulse_active else "pulse_rest",
    )


def status_text(
    *,
    state: AutoTestState,
    active_name: str = "",
    reason: str = "",
    enabled: bool = True,
    safety_stop: bool = False,
    command: TwistCommand | None = None,
) -> str:
    cmd = (command or stop_command()).sanitized()

    return (
        f"state={state.value}; "
        f"enabled={str(enabled).lower()}; "
        f"active_test={active_name or 'none'}; "
        f"reason={reason}; "
        f"safety_stop={str(safety_stop).lower()}; "
        f"vx={cmd.vx:.3f}; vy={cmd.vy:.3f}; wz={cmd.wz:.3f}"
    )


__all__ = [
    "AUTO_TEST_SOURCE",
    "ActiveAutoTest",
    "AutoTestLimits",
    "AutoTestProfile",
    "AutoTestState",
    "AutoTestStepResult",
    "TestStage",
    "command_from_values",
    "constant_profile",
    "default_auto_tests",
    "normalize_test_name",
    "profile_from_mapping",
    "pulse_profile",
    "start_active_test",
    "status_text",
    "step_active_test",
    "stop_command",
]
