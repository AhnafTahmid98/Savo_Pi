# -*- coding: utf-8 -*-

"""Pure helpers for straight_line_pid_test_node.py."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import cos, sin

from savo_control.controllers.pid_py import Pid, PidConfig
from savo_control.models import TwistCommand
from savo_control.utils import clamp, finite_or_zero, hypot2, wrap_angle_rad


STRAIGHT_LINE_SOURCE = "straight_line_pid_test"


class StraightLineState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    GOAL_REACHED = "GOAL_REACHED"
    BLOCKED = "BLOCKED"
    STALE_ODOM = "STALE_ODOM"
    DISABLED = "DISABLED"
    TIMEOUT = "TIMEOUT"


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def sanitized(self) -> "Pose2D":
        return Pose2D(
            x=finite_or_zero(self.x),
            y=finite_or_zero(self.y),
            yaw=wrap_angle_rad(finite_or_zero(self.yaw)),
        )


@dataclass(frozen=True)
class StraightLineConfig:
    target_distance_m: float = 0.80
    goal_tolerance_m: float = 0.04

    forward_vx_m_s: float = 0.10
    max_vx_m_s: float = 0.14
    max_vy_m_s: float = 0.08
    max_wz_rad_s: float = 0.35

    lateral_kp: float = 0.80
    yaw_kp: float = 1.20

    min_dt_s: float = 1.0e-4
    max_duration_s: float = 12.0

    def sanitized(self) -> "StraightLineConfig":
        max_vx = max(0.0, abs(finite_or_zero(self.max_vx_m_s)))
        max_vy = max(0.0, abs(finite_or_zero(self.max_vy_m_s)))
        max_wz = max(0.0, abs(finite_or_zero(self.max_wz_rad_s)))

        return StraightLineConfig(
            target_distance_m=max(0.0, finite_or_zero(self.target_distance_m)),
            goal_tolerance_m=max(0.0, finite_or_zero(self.goal_tolerance_m)),
            forward_vx_m_s=clamp(
                abs(finite_or_zero(self.forward_vx_m_s)),
                0.0,
                max_vx,
            ),
            max_vx_m_s=max_vx,
            max_vy_m_s=max_vy,
            max_wz_rad_s=max_wz,
            lateral_kp=finite_or_zero(self.lateral_kp),
            yaw_kp=finite_or_zero(self.yaw_kp),
            min_dt_s=(
                finite_or_zero(self.min_dt_s)
                if finite_or_zero(self.min_dt_s) > 0.0
                else 1.0e-6
            ),
            max_duration_s=max(0.1, finite_or_zero(self.max_duration_s)),
        )


@dataclass(frozen=True)
class StraightLineRun:
    start_pose: Pose2D
    target_distance_m: float
    start_s: float

    def elapsed_s(self, *, now_s: float) -> float:
        return max(0.0, finite_or_zero(now_s) - self.start_s)


@dataclass(frozen=True)
class StraightLineStep:
    state: StraightLineState
    command: TwistCommand
    run: StraightLineRun | None

    forward_progress_m: float = 0.0
    lateral_error_m: float = 0.0
    yaw_error_rad: float = 0.0
    remaining_m: float = 0.0

    finished: bool = False
    reason: str = ""


def stop_command(*, stamp_sec: float = 0.0) -> TwistCommand:
    return TwistCommand.zero(source=STRAIGHT_LINE_SOURCE, stamp_sec=stamp_sec)


def start_run(
    *,
    pose: Pose2D,
    config: StraightLineConfig,
    now_s: float,
) -> StraightLineRun:
    safe_cfg = config.sanitized()

    return StraightLineRun(
        start_pose=pose.sanitized(),
        target_distance_m=safe_cfg.target_distance_m,
        start_s=finite_or_zero(now_s),
    )


def pose_delta_in_start_frame(
    *,
    start: Pose2D,
    current: Pose2D,
) -> tuple[float, float]:
    safe_start = start.sanitized()
    safe_current = current.sanitized()

    dx = safe_current.x - safe_start.x
    dy = safe_current.y - safe_start.y

    c = cos(safe_start.yaw)
    s = sin(safe_start.yaw)

    forward = c * dx + s * dy
    lateral = -s * dx + c * dy

    return forward, lateral


def yaw_error_to_start(
    *,
    start_yaw: float,
    current_yaw: float,
) -> float:
    return wrap_angle_rad(finite_or_zero(start_yaw) - finite_or_zero(current_yaw))


def make_straight_line_command(
    *,
    forward_vx: float,
    lateral_error_m: float,
    yaw_error_rad: float,
    config: StraightLineConfig,
    stamp_sec: float = 0.0,
) -> TwistCommand:
    safe = config.sanitized()

    vy = clamp(
        -safe.lateral_kp * finite_or_zero(lateral_error_m),
        -safe.max_vy_m_s,
        safe.max_vy_m_s,
    )
    wz = clamp(
        safe.yaw_kp * finite_or_zero(yaw_error_rad),
        -safe.max_wz_rad_s,
        safe.max_wz_rad_s,
    )
    vx = clamp(
        finite_or_zero(forward_vx),
        -safe.max_vx_m_s,
        safe.max_vx_m_s,
    )

    return TwistCommand(
        vx=vx,
        vy=vy,
        wz=wz,
        source=STRAIGHT_LINE_SOURCE,
        stamp_sec=stamp_sec,
    ).sanitized()


def step_straight_line(
    *,
    run: StraightLineRun | None,
    pose: Pose2D | None,
    config: StraightLineConfig,
    now_s: float,
    enabled: bool = True,
    safety_stop: bool = False,
    odom_fresh: bool = True,
) -> StraightLineStep:
    safe_cfg = config.sanitized()

    if not enabled:
        return StraightLineStep(
            state=StraightLineState.DISABLED,
            command=stop_command(stamp_sec=now_s),
            run=None,
            finished=True,
            reason="disabled",
        )

    if safety_stop:
        return StraightLineStep(
            state=StraightLineState.BLOCKED,
            command=stop_command(stamp_sec=now_s),
            run=None,
            finished=True,
            reason="safety_stop",
        )

    if not odom_fresh or pose is None:
        return StraightLineStep(
            state=StraightLineState.STALE_ODOM,
            command=stop_command(stamp_sec=now_s),
            run=run,
            reason="odom_stale",
        )

    if run is None:
        run = start_run(pose=pose, config=safe_cfg, now_s=now_s)

    if run.elapsed_s(now_s=now_s) > safe_cfg.max_duration_s:
        return StraightLineStep(
            state=StraightLineState.TIMEOUT,
            command=stop_command(stamp_sec=now_s),
            run=None,
            finished=True,
            reason="timeout",
        )

    forward, lateral = pose_delta_in_start_frame(
        start=run.start_pose,
        current=pose,
    )
    remaining = max(0.0, run.target_distance_m - forward)
    yaw_error = yaw_error_to_start(
        start_yaw=run.start_pose.yaw,
        current_yaw=pose.yaw,
    )

    if remaining <= safe_cfg.goal_tolerance_m:
        return StraightLineStep(
            state=StraightLineState.GOAL_REACHED,
            command=stop_command(stamp_sec=now_s),
            run=None,
            forward_progress_m=forward,
            lateral_error_m=lateral,
            yaw_error_rad=yaw_error,
            remaining_m=remaining,
            finished=True,
            reason="goal_reached",
        )

    command = make_straight_line_command(
        forward_vx=safe_cfg.forward_vx_m_s,
        lateral_error_m=lateral,
        yaw_error_rad=yaw_error,
        config=safe_cfg,
        stamp_sec=now_s,
    )

    return StraightLineStep(
        state=StraightLineState.RUNNING,
        command=command,
        run=run,
        forward_progress_m=forward,
        lateral_error_m=lateral,
        yaw_error_rad=yaw_error,
        remaining_m=remaining,
        reason="tracking",
    )


def status_text(
    *,
    state: StraightLineState,
    enabled: bool = True,
    reason: str = "",
    forward_progress_m: float = 0.0,
    lateral_error_m: float = 0.0,
    yaw_error_rad: float = 0.0,
    remaining_m: float = 0.0,
    command: TwistCommand | None = None,
) -> str:
    cmd = (command or stop_command()).sanitized()

    return (
        f"state={state.value}; "
        f"enabled={str(enabled).lower()}; "
        f"reason={reason}; "
        f"progress_m={finite_or_zero(forward_progress_m):.3f}; "
        f"lateral_error_m={finite_or_zero(lateral_error_m):.3f}; "
        f"yaw_error_rad={finite_or_zero(yaw_error_rad):.3f}; "
        f"remaining_m={finite_or_zero(remaining_m):.3f}; "
        f"vx={cmd.vx:.3f}; vy={cmd.vy:.3f}; wz={cmd.wz:.3f}"
    )


def make_lateral_pid(config: StraightLineConfig) -> Pid:
    safe = config.sanitized()
    cfg = PidConfig(kp=safe.lateral_kp, ki=0.0, kd=0.0)
    cfg.set_output_symmetric(safe.max_vy_m_s)
    return Pid(cfg)


def make_yaw_pid(config: StraightLineConfig) -> Pid:
    safe = config.sanitized()
    cfg = PidConfig(kp=safe.yaw_kp, ki=0.0, kd=0.0)
    cfg.set_output_symmetric(safe.max_wz_rad_s)
    return Pid(cfg)


__all__ = [
    "STRAIGHT_LINE_SOURCE",
    "Pose2D",
    "StraightLineConfig",
    "StraightLineRun",
    "StraightLineState",
    "StraightLineStep",
    "make_lateral_pid",
    "make_straight_line_command",
    "make_yaw_pid",
    "pose_delta_in_start_frame",
    "start_run",
    "status_text",
    "step_straight_line",
    "stop_command",
    "yaw_error_to_start",
]
