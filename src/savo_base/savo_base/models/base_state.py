#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/models/base_state.py
-------------------------------------------
Professional base-state models for Robot Savo (ROS 2 Jazzy, real robot testing).

Purpose
- Represent the runtime state of the mobile base in a clean, reusable way
- Track commanded motion, applied motion, wheel duties, and safety flags
- Provide validation and helper methods for controllers, dashboards, and logging

Scope
- Pure data models + helper methods (no ROS imports, no hardware I/O)
- Can be used by:
  - savo_base motor execution node
  - dry-run board tests
  - cmd_vel shaper / mux integration
  - dashboards and telemetry serializers
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple, Optional, Any
import math
import time


# =============================================================================
# Local exceptions
# =============================================================================
class BaseStateValidationError(ValueError):
    """Raised when a base-state model contains invalid values."""
    pass


# =============================================================================
# Generic helpers
# =============================================================================
def _is_finite(x: float) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def _clamp(x: float, lo: float, hi: float) -> float:
    if lo > hi:
        raise BaseStateValidationError(f"Invalid clamp range: lo={lo} > hi={hi}")
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def _now_monotonic() -> float:
    return float(time.monotonic())


def _now_unix() -> float:
    return float(time.time())


# =============================================================================
# Motion vector models
# =============================================================================
@dataclass
class BodyCommand:
    """
    Body-frame command/state vector (holonomic mecanum base).

    Units depend on usage context:
    - normalized (dimensionless) OR
    - physical SI (m/s, m/s, rad/s)
    """
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def validate(self, *, field_name: str = "body_command") -> "BodyCommand":
        if not _is_finite(self.vx):
            raise BaseStateValidationError(f"{field_name}.vx must be finite, got {self.vx!r}")
        if not _is_finite(self.vy):
            raise BaseStateValidationError(f"{field_name}.vy must be finite, got {self.vy!r}")
        if not _is_finite(self.wz):
            raise BaseStateValidationError(f"{field_name}.wz must be finite, got {self.wz!r}")
        return self

    def zero(self) -> "BodyCommand":
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        return self

    def copy(self) -> "BodyCommand":
        return BodyCommand(vx=float(self.vx), vy=float(self.vy), wz=float(self.wz))

    def as_tuple(self) -> Tuple[float, float, float]:
        self.validate()
        return float(self.vx), float(self.vy), float(self.wz)

    def xy_speed(self) -> float:
        self.validate()
        return math.hypot(float(self.vx), float(self.vy))

    def is_zero(self, eps: float = 1e-9) -> bool:
        self.validate()
        return (
            abs(float(self.vx)) <= eps and
            abs(float(self.vy)) <= eps and
            abs(float(self.wz)) <= eps
        )

    def clamp_per_axis(self, *, vx_abs: float, vy_abs: float, wz_abs: float) -> "BodyCommand":
        self.validate()
        if vx_abs < 0 or vy_abs < 0 or wz_abs < 0:
            raise BaseStateValidationError("Axis clamp abs values must be >= 0")
        self.vx = _clamp(float(self.vx), -float(vx_abs), +float(vx_abs))
        self.vy = _clamp(float(self.vy), -float(vy_abs), +float(vy_abs))
        self.wz = _clamp(float(self.wz), -float(wz_abs), +float(wz_abs))
        return self

    def to_dict(self) -> Dict[str, float]:
        self.validate()
        return {
            "vx": float(self.vx),
            "vy": float(self.vy),
            "wz": float(self.wz),
        }


@dataclass
class WheelDutyCommand:
    """
    Signed wheel duty commands for Robot Savo wheel order (FL, RL, FR, RR).

    Values are signed "wheel intent" duties (typically clamped to [-max_abs_duty, +max_abs_duty]).
    The board driver later maps signs to channel pairs.
    """
    fl: int = 0
    rl: int = 0
    fr: int = 0
    rr: int = 0

    def validate(self, *, hard_cap_abs: int = 4095, field_name: str = "wheel_duty") -> "WheelDutyCommand":
        try:
            vals = [int(self.fl), int(self.rl), int(self.fr), int(self.rr)]
        except Exception as e:
            raise BaseStateValidationError(f"{field_name} values must be int-like") from e

        if hard_cap_abs < 0 or hard_cap_abs > 4095:
            raise BaseStateValidationError(f"hard_cap_abs must be in [0,4095], got {hard_cap_abs}")

        for name, v in zip(("fl", "rl", "fr", "rr"), vals):
            if abs(v) > hard_cap_abs:
                raise BaseStateValidationError(
                    f"{field_name}.{name}={v} exceeds hard_cap_abs={hard_cap_abs}"
                )
        return self

    def zero(self) -> "WheelDutyCommand":
        self.fl = self.rl = self.fr = self.rr = 0
        return self

    def copy(self) -> "WheelDutyCommand":
        return WheelDutyCommand(fl=int(self.fl), rl=int(self.rl), fr=int(self.fr), rr=int(self.rr))

    def as_tuple(self) -> Tuple[int, int, int, int]:
        self.validate()
        return int(self.fl), int(self.rl), int(self.fr), int(self.rr)

    def max_abs(self) -> int:
        self.validate()
        return max(abs(int(self.fl)), abs(int(self.rl)), abs(int(self.fr)), abs(int(self.rr)))

    def clamp(self, *, max_abs_duty: int, hard_cap_abs: int = 4095) -> "WheelDutyCommand":
        self.validate(hard_cap_abs=hard_cap_abs)
        if max_abs_duty < 0 or max_abs_duty > hard_cap_abs:
            raise BaseStateValidationError(
                f"max_abs_duty must be in [0,{hard_cap_abs}], got {max_abs_duty}"
            )

        def c(v: int) -> int:
            vv = int(v)
            if vv > max_abs_duty:
                return int(max_abs_duty)
            if vv < -max_abs_duty:
                return -int(max_abs_duty)
            return vv

        self.fl = c(self.fl)
        self.rl = c(self.rl)
        self.fr = c(self.fr)
        self.rr = c(self.rr)
        return self

    def to_dict(self) -> Dict[str, int]:
        self.validate()
        return {
            "fl": int(self.fl),
            "rl": int(self.rl),
            "fr": int(self.fr),
            "rr": int(self.rr),
        }


# =============================================================================
# Safety / mode / source state models
# =============================================================================
@dataclass
class SafetyState:
    """
    Safety-related runtime flags seen by the base execution layer.
    """
    enabled: bool = True
    estop_active: bool = False
    safety_stop_active: bool = False
    motor_output_enabled: bool = True

    # Optional slowdown factor from perception/control stack (0..1)
    slowdown_factor: float = 1.0

    # Human-readable cause fields (optional, for logs/dashboard)
    last_stop_reason: str = ""
    last_estop_source: str = ""

    def validate(self) -> "SafetyState":
        if not isinstance(self.enabled, bool):
            raise BaseStateValidationError("SafetyState.enabled must be bool")
        if not isinstance(self.estop_active, bool):
            raise BaseStateValidationError("SafetyState.estop_active must be bool")
        if not isinstance(self.safety_stop_active, bool):
            raise BaseStateValidationError("SafetyState.safety_stop_active must be bool")
        if not isinstance(self.motor_output_enabled, bool):
            raise BaseStateValidationError("SafetyState.motor_output_enabled must be bool")

        if not _is_finite(self.slowdown_factor):
            raise BaseStateValidationError(f"slowdown_factor must be finite, got {self.slowdown_factor!r}")
        self.slowdown_factor = _clamp(float(self.slowdown_factor), 0.0, 1.0)

        if not isinstance(self.last_stop_reason, str):
            raise BaseStateValidationError("last_stop_reason must be str")
        if not isinstance(self.last_estop_source, str):
            raise BaseStateValidationError("last_estop_source must be str")

        return self

    def motion_blocked(self) -> bool:
        self.validate()
        return bool(self.estop_active or self.safety_stop_active or (not self.motor_output_enabled))

    def effective_slowdown_factor(self) -> float:
        self.validate()
        if self.motion_blocked():
            return 0.0
        return float(self.slowdown_factor)

    def clear_stop_reasons(self) -> "SafetyState":
        self.last_stop_reason = ""
        self.last_estop_source = ""
        return self

    def to_dict(self) -> Dict[str, Any]:
        self.validate()
        return {
            "enabled": self.enabled,
            "estop_active": self.estop_active,
            "safety_stop_active": self.safety_stop_active,
            "motor_output_enabled": self.motor_output_enabled,
            "slowdown_factor": float(self.slowdown_factor),
            "motion_blocked": self.motion_blocked(),
            "last_stop_reason": self.last_stop_reason,
            "last_estop_source": self.last_estop_source,
        }


@dataclass
class BaseModeState:
    """
    Execution-mode state for the base package.

    This is intentionally generic so it can support:
    - direct teleop
    - ROS cmd_vel
    - autonomous/nav
    - voice/LLM-originated motion (via upstream control stack)
    """
    mode: str = "IDLE"                 # e.g., IDLE / MANUAL / AUTO / FOLLOW / NAV / TEST
    command_source: str = "none"       # e.g., cmd_vel_safe / keyboard / dryrun / nav2 / mux
    armed: bool = False                # base allowed to actuate
    heartbeat_ok: bool = True          # optional upstream heartbeat status

    def validate(self) -> "BaseModeState":
        if not isinstance(self.mode, str) or not self.mode.strip():
            raise BaseStateValidationError("BaseModeState.mode must be a non-empty string")
        if not isinstance(self.command_source, str) or not self.command_source.strip():
            raise BaseStateValidationError("BaseModeState.command_source must be a non-empty string")
        if not isinstance(self.armed, bool):
            raise BaseStateValidationError("BaseModeState.armed must be bool")
        if not isinstance(self.heartbeat_ok, bool):
            raise BaseStateValidationError("BaseModeState.heartbeat_ok must be bool")
        return self

    def to_dict(self) -> Dict[str, Any]:
        self.validate()
        return {
            "mode": self.mode,
            "command_source": self.command_source,
            "armed": self.armed,
            "heartbeat_ok": self.heartbeat_ok,
        }


# =============================================================================
# Main base runtime state aggregate
# =============================================================================
@dataclass
class BaseState:
    """
    Aggregated Robot Savo base runtime state.

    Includes:
    - commanded body motion (normalized + physical)
    - applied body motion after shaping/safety
    - wheel duty command actually sent (or to be sent) to board
    - safety flags / mode state
    - timing / sequence bookkeeping
    """
    robot_name: str = "Robot Savo"
    node_name: str = "savo_base"
    profile_name: str = "default"

    # --- mode/safety ---
    mode: BaseModeState = field(default_factory=BaseModeState)
    safety: SafetyState = field(default_factory=SafetyState)

    # --- command states ---
    # command requested by upstream layer (e.g. /cmd_vel_safe mapped into normalized)
    cmd_normalized_requested: BodyCommand = field(default_factory=BodyCommand)

    # command after shaping/slowdown/clamping inside savo_base
    cmd_normalized_applied: BodyCommand = field(default_factory=BodyCommand)

    # optional physical command states (if used by your node)
    cmd_physical_requested: BodyCommand = field(default_factory=BodyCommand)
    cmd_physical_applied: BodyCommand = field(default_factory=BodyCommand)

    # final wheel duties sent to motor board
    wheel_duty_requested: WheelDutyCommand = field(default_factory=WheelDutyCommand)
    wheel_duty_applied: WheelDutyCommand = field(default_factory=WheelDutyCommand)

    # --- board/runtime status ---
    board_type: str = "unknown"             # e.g. freenove_mecanum / dryrun
    board_connected: bool = False
    board_ready: bool = False
    outputs_active: bool = False            # whether non-zero motor command is currently active

    # --- lifecycle / timing ---
    sequence_id: int = 0                    # increment per applied command cycle
    last_update_monotonic_s: float = field(default_factory=_now_monotonic)
    last_update_unix_s: float = field(default_factory=_now_unix)
    last_command_monotonic_s: float = 0.0
    last_nonzero_command_monotonic_s: float = 0.0

    # --- diagnostics ---
    last_error: str = ""
    last_warning: str = ""

    def validate(self) -> "BaseState":
        if not isinstance(self.robot_name, str) or not self.robot_name.strip():
            raise BaseStateValidationError("robot_name must be a non-empty string")
        if not isinstance(self.node_name, str) or not self.node_name.strip():
            raise BaseStateValidationError("node_name must be a non-empty string")
        if not isinstance(self.profile_name, str) or not self.profile_name.strip():
            raise BaseStateValidationError("profile_name must be a non-empty string")

        self.mode.validate()
        self.safety.validate()

        self.cmd_normalized_requested.validate(field_name="cmd_normalized_requested")
        self.cmd_normalized_applied.validate(field_name="cmd_normalized_applied")
        self.cmd_physical_requested.validate(field_name="cmd_physical_requested")
        self.cmd_physical_applied.validate(field_name="cmd_physical_applied")

        self.wheel_duty_requested.validate(field_name="wheel_duty_requested")
        self.wheel_duty_applied.validate(field_name="wheel_duty_applied")

        if not isinstance(self.board_type, str) or not self.board_type.strip():
            raise BaseStateValidationError("board_type must be a non-empty string")
        if not isinstance(self.board_connected, bool):
            raise BaseStateValidationError("board_connected must be bool")
        if not isinstance(self.board_ready, bool):
            raise BaseStateValidationError("board_ready must be bool")
        if not isinstance(self.outputs_active, bool):
            raise BaseStateValidationError("outputs_active must be bool")

        if not isinstance(self.sequence_id, int) or self.sequence_id < 0:
            raise BaseStateValidationError("sequence_id must be int >= 0")

        for fname in (
            "last_update_monotonic_s",
            "last_update_unix_s",
            "last_command_monotonic_s",
            "last_nonzero_command_monotonic_s",
        ):
            v = getattr(self, fname)
            if not _is_finite(v) or float(v) < 0.0:
                raise BaseStateValidationError(f"{fname} must be finite and >= 0, got {v!r}")

        if not isinstance(self.last_error, str):
            raise BaseStateValidationError("last_error must be str")
        if not isinstance(self.last_warning, str):
            raise BaseStateValidationError("last_warning must be str")

        return self

    # -------------------------------------------------------------------------
    # Runtime helpers
    # -------------------------------------------------------------------------
    def touch(self) -> "BaseState":
        """
        Update generic timestamps (called each cycle).
        """
        self.last_update_monotonic_s = _now_monotonic()
        self.last_update_unix_s = _now_unix()
        return self

    def next_sequence(self) -> int:
        """
        Increment and return sequence id for a new control-cycle application.
        """
        self.sequence_id = int(self.sequence_id) + 1
        self.touch()
        return self.sequence_id

    def mark_command_received(self, *, nonzero: Optional[bool] = None) -> "BaseState":
        """
        Mark command receipt timestamp. If nonzero is None, infer from requested commands.
        """
        t = _now_monotonic()
        self.last_command_monotonic_s = t
        if nonzero is None:
            nonzero = (
                (not self.cmd_normalized_requested.is_zero()) or
                (self.wheel_duty_requested.max_abs() > 0)
            )
        if bool(nonzero):
            self.last_nonzero_command_monotonic_s = t
        self.touch()
        return self

    def zero_motion(self, *, keep_requested: bool = False, reason: str = "") -> "BaseState":
        """
        Force applied motion and wheel duties to zero (used for estop/safety stop/disable).
        Optionally also zero requested motion.
        """
        if not keep_requested:
            self.cmd_normalized_requested.zero()
            self.cmd_physical_requested.zero()
            self.wheel_duty_requested.zero()

        self.cmd_normalized_applied.zero()
        self.cmd_physical_applied.zero()
        self.wheel_duty_applied.zero()
        self.outputs_active = False

        if reason:
            self.safety.last_stop_reason = str(reason)

        self.touch()
        return self

    def apply_safety_to_normalized(self) -> "BaseState":
        """
        Apply current safety state to requested normalized command and write applied command.

        Rules:
        - If motion is blocked -> applied command = zero
        - Else apply slowdown factor to requested command
        """
        self.validate()
        if self.safety.motion_blocked():
            self.cmd_normalized_applied.zero()
            self.outputs_active = False
        else:
            sf = self.safety.effective_slowdown_factor()
            self.cmd_normalized_applied.vx = float(self.cmd_normalized_requested.vx) * sf
            self.cmd_normalized_applied.vy = float(self.cmd_normalized_requested.vy) * sf
            self.cmd_normalized_applied.wz = float(self.cmd_normalized_requested.wz) * sf
            self.outputs_active = not self.cmd_normalized_applied.is_zero()
        self.touch()
        return self

    def board_can_drive(self) -> bool:
        """
        True if the base is armed, board is ready, and safety allows motion output.
        """
        self.validate()
        return bool(
            self.mode.armed and
            self.mode.heartbeat_ok and
            self.board_connected and
            self.board_ready and
            (not self.safety.motion_blocked())
        )

    def command_age_s(self) -> float:
        """
        Age of last received command using monotonic clock.
        """
        self.validate()
        now = _now_monotonic()
        if self.last_command_monotonic_s <= 0.0:
            return math.inf
        return max(0.0, now - float(self.last_command_monotonic_s))

    def nonzero_command_age_s(self) -> float:
        """
        Age of last non-zero command using monotonic clock.
        """
        self.validate()
        now = _now_monotonic()
        if self.last_nonzero_command_monotonic_s <= 0.0:
            return math.inf
        return max(0.0, now - float(self.last_nonzero_command_monotonic_s))

    def set_error(self, msg: str) -> "BaseState":
        self.last_error = str(msg)
        self.touch()
        return self

    def clear_error(self) -> "BaseState":
        self.last_error = ""
        self.touch()
        return self

    def set_warning(self, msg: str) -> "BaseState":
        self.last_warning = str(msg)
        self.touch()
        return self

    def clear_warning(self) -> "BaseState":
        self.last_warning = ""
        self.touch()
        return self

    # -------------------------------------------------------------------------
    # Summaries / serialization
    # -------------------------------------------------------------------------
    def status_level(self) -> str:
        """
        Coarse status label for dashboards/logs.
        """
        self.validate()
        if self.last_error:
            return "ERROR"
        if self.safety.estop_active:
            return "ESTOP"
        if self.safety.safety_stop_active:
            return "SAFETY_STOP"
        if not self.mode.armed:
            return "DISARMED"
        if not self.board_connected or not self.board_ready:
            return "NOT_READY"
        if self.outputs_active:
            return "ACTIVE"
        return "IDLE"

    def short_summary(self) -> str:
        """
        Human-readable compact line for logs/dashboard prints.
        """
        self.validate()
        d = self.wheel_duty_applied
        c = self.cmd_normalized_applied
        return (
            f"[{self.status_level()}] mode={self.mode.mode} src={self.mode.command_source} "
            f"armed={self.mode.armed} board={self.board_type} ready={self.board_ready} "
            f"stop={self.safety.safety_stop_active} estop={self.safety.estop_active} "
            f"slow={self.safety.slowdown_factor:.2f} "
            f"cmdN=({c.vx:.2f},{c.vy:.2f},{c.wz:.2f}) "
            f"duty=({d.fl},{d.rl},{d.fr},{d.rr}) seq={self.sequence_id}"
        )

    def to_dict(self) -> Dict[str, Any]:
        self.validate()
        return {
            "robot_name": self.robot_name,
            "node_name": self.node_name,
            "profile_name": self.profile_name,
            "status_level": self.status_level(),

            "mode": self.mode.to_dict(),
            "safety": self.safety.to_dict(),

            "cmd_normalized_requested": self.cmd_normalized_requested.to_dict(),
            "cmd_normalized_applied": self.cmd_normalized_applied.to_dict(),

            "cmd_physical_requested": self.cmd_physical_requested.to_dict(),
            "cmd_physical_applied": self.cmd_physical_applied.to_dict(),

            "wheel_duty_requested": self.wheel_duty_requested.to_dict(),
            "wheel_duty_applied": self.wheel_duty_applied.to_dict(),

            "board": {
                "board_type": self.board_type,
                "board_connected": self.board_connected,
                "board_ready": self.board_ready,
                "outputs_active": self.outputs_active,
                "can_drive": self.board_can_drive(),
            },

            "timing": {
                "sequence_id": int(self.sequence_id),
                "last_update_monotonic_s": float(self.last_update_monotonic_s),
                "last_update_unix_s": float(self.last_update_unix_s),
                "last_command_monotonic_s": float(self.last_command_monotonic_s),
                "last_nonzero_command_monotonic_s": float(self.last_nonzero_command_monotonic_s),
                "command_age_s": self.command_age_s(),
                "nonzero_command_age_s": self.nonzero_command_age_s(),
            },

            "diagnostics": {
                "last_error": self.last_error,
                "last_warning": self.last_warning,
                "summary": self.short_summary(),
            },
        }


# =============================================================================
# Canonical constructors (Robot Savo)
# =============================================================================
def make_robot_savo_base_state(
    *,
    board_type: str = "freenove_mecanum",
    profile_name: str = "robot_savo_default_base_limits",
    command_source: str = "cmd_vel_safe",
    mode: str = "IDLE",
) -> BaseState:
    """
    Create a validated BaseState with Robot Savo-friendly defaults.

    Notes
    -----
    - command_source defaults to `cmd_vel_safe` because your architecture routes
      motion through safety gating before final actuation.
    - voice/LLM control remains upstream (savo_speech/savo_intent/savo_control);
      by the time motion reaches savo_base it is just another command source path.
    """
    state = BaseState(
        robot_name="Robot Savo",
        node_name="savo_base",
        profile_name=profile_name,
        mode=BaseModeState(mode=mode, command_source=command_source, armed=False, heartbeat_ok=True),
        safety=SafetyState(
            enabled=True,
            estop_active=False,
            safety_stop_active=False,
            motor_output_enabled=True,
            slowdown_factor=1.0,
        ),
        board_type=board_type,
        board_connected=False,
        board_ready=False,
        outputs_active=False,
    )
    return state.validate()


# =============================================================================
# Public exports
# =============================================================================
__all__ = [
    # exceptions
    "BaseStateValidationError",

    # primitive models
    "BodyCommand",
    "WheelDutyCommand",
    "SafetyState",
    "BaseModeState",

    # aggregate state
    "BaseState",

    # constructors
    "make_robot_savo_base_state",
]