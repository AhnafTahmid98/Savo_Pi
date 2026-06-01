#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/models/wheel_command.py
----------------------------------------------
Professional wheel-command models for Robot Savo (ROS 2 Jazzy, real robot).

Purpose
- Represent wheel-space motion commands for the Robot Savo mecanum base
- Keep command metadata (source, timing, mode, watchdog-friendly fields)
- Validate and serialize commands for execution, logging, and telemetry

Scope
- Pure Python data models (no ROS imports, no hardware I/O)
- Intended for use by:
  - savo_base command intake / execution node
  - teleop / nav / follow / LLM-text command bridge
  - motor board adapters (after scaling/clamping)
  - watchdog and diagnostics layers
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Any, Optional, Tuple
import math
import time


# =============================================================================
# Exceptions
# =============================================================================
class WheelCommandValidationError(ValueError):
    """Raised when a wheel-command model contains invalid values."""
    pass


# =============================================================================
# Helpers
# =============================================================================
def _is_finite(x: float) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def _clamp(x: float, lo: float, hi: float) -> float:
    if lo > hi:
        raise WheelCommandValidationError(f"Invalid clamp range: lo={lo} > hi={hi}")
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def _now_unix() -> float:
    return float(time.time())


def _now_mono() -> float:
    return float(time.monotonic())


def _norm4(a: float, b: float, c: float, d: float) -> float:
    return max(abs(float(a)), abs(float(b)), abs(float(c)), abs(float(d)))


# =============================================================================
# Wheel values (normalized)
# =============================================================================
@dataclass
class WheelNorm:
    """
    Normalized wheel command in wheel order: FL, RL, FR, RR.

    Range
    -----
    Each value is typically in [-1.0, +1.0].
    """
    fl: float = 0.0
    rl: float = 0.0
    fr: float = 0.0
    rr: float = 0.0

    def validate(self, *, limit_abs: float = 1.0, allow_over: bool = False) -> "WheelNorm":
        for name in ("fl", "rl", "fr", "rr"):
            v = getattr(self, name)
            if not _is_finite(v):
                raise WheelCommandValidationError(f"{name} must be finite, got {v!r}")
            fv = float(v)
            if (not allow_over) and abs(fv) > float(limit_abs) + 1e-12:
                raise WheelCommandValidationError(
                    f"{name}={fv:.6f} exceeds |{limit_abs}|"
                )
            setattr(self, name, fv)
        return self

    def copy(self) -> "WheelNorm":
        return WheelNorm(fl=float(self.fl), rl=float(self.rl), fr=float(self.fr), rr=float(self.rr))

    def zero(self) -> "WheelNorm":
        self.fl = self.rl = self.fr = self.rr = 0.0
        return self

    def max_abs(self) -> float:
        self.validate(allow_over=True)
        return _norm4(self.fl, self.rl, self.fr, self.rr)

    def any_nonzero(self, eps: float = 1e-9) -> bool:
        self.validate(allow_over=True)
        return self.max_abs() > float(eps)

    def normalized(self) -> "WheelNorm":
        """
        Return a new WheelNorm scaled so max(|wheel|) <= 1.0.
        """
        self.validate(allow_over=True)
        m = max(1.0, self.max_abs())
        return WheelNorm(
            fl=float(self.fl) / m,
            rl=float(self.rl) / m,
            fr=float(self.fr) / m,
            rr=float(self.rr) / m,
        ).validate()

    def clamp(self, limit_abs: float = 1.0) -> "WheelNorm":
        """
        Hard clamp each wheel independently to [-limit_abs, +limit_abs].
        """
        self.validate(allow_over=True)
        la = abs(float(limit_abs))
        self.fl = _clamp(float(self.fl), -la, la)
        self.rl = _clamp(float(self.rl), -la, la)
        self.fr = _clamp(float(self.fr), -la, la)
        self.rr = _clamp(float(self.rr), -la, la)
        return self.validate(limit_abs=la)

    def scale(self, gain: float) -> "WheelNorm":
        if not _is_finite(gain):
            raise WheelCommandValidationError(f"gain must be finite, got {gain!r}")
        g = float(gain)
        self.fl *= g
        self.rl *= g
        self.fr *= g
        self.rr *= g
        return self.validate(allow_over=True)

    def as_tuple(self) -> Tuple[float, float, float, float]:
        self.validate(allow_over=True)
        return float(self.fl), float(self.rl), float(self.fr), float(self.rr)

    def to_dict(self) -> Dict[str, float]:
        self.validate(allow_over=True)
        return {
            "fl": float(self.fl),
            "rl": float(self.rl),
            "fr": float(self.fr),
            "rr": float(self.rr),
        }


# =============================================================================
# Wheel values (signed duty)
# =============================================================================
@dataclass
class WheelDuty:
    """
    Signed wheel duty command in wheel order: FL, RL, FR, RR.

    Notes
    -----
    - These are wheel-space signed duties (not raw PCA9685 channels).
    - Typical range after scaling is [-max_abs_duty, +max_abs_duty], hard cap 4095.
    """
    fl: int = 0
    rl: int = 0
    fr: int = 0
    rr: int = 0

    def validate(self, *, hard_cap_abs: int = 4095) -> "WheelDuty":
        if not isinstance(hard_cap_abs, int) or hard_cap_abs < 0 or hard_cap_abs > 4095:
            raise WheelCommandValidationError(f"hard_cap_abs must be int in [0,4095], got {hard_cap_abs!r}")

        for name in ("fl", "rl", "fr", "rr"):
            v = getattr(self, name)
            try:
                iv = int(v)
            except Exception as e:
                raise WheelCommandValidationError(f"{name} must be int-like") from e
            if abs(iv) > hard_cap_abs:
                raise WheelCommandValidationError(f"{name}={iv} exceeds hard_cap_abs={hard_cap_abs}")
            setattr(self, name, iv)
        return self

    def copy(self) -> "WheelDuty":
        return WheelDuty(fl=int(self.fl), rl=int(self.rl), fr=int(self.fr), rr=int(self.rr))

    def zero(self) -> "WheelDuty":
        self.fl = self.rl = self.fr = self.rr = 0
        return self

    def max_abs(self) -> int:
        self.validate()
        return max(abs(int(self.fl)), abs(int(self.rl)), abs(int(self.fr)), abs(int(self.rr)))

    def any_nonzero(self) -> bool:
        self.validate()
        return self.max_abs() > 0

    def clamp(self, *, max_abs_duty: int, hard_cap_abs: int = 4095) -> "WheelDuty":
        self.validate(hard_cap_abs=hard_cap_abs)
        if not isinstance(max_abs_duty, int) or max_abs_duty < 0 or max_abs_duty > hard_cap_abs:
            raise WheelCommandValidationError(
                f"max_abs_duty must be int in [0,{hard_cap_abs}], got {max_abs_duty!r}"
            )

        def c(v: int) -> int:
            iv = int(v)
            if iv > max_abs_duty:
                return max_abs_duty
            if iv < -max_abs_duty:
                return -max_abs_duty
            return iv

        self.fl = c(self.fl)
        self.rl = c(self.rl)
        self.fr = c(self.fr)
        self.rr = c(self.rr)
        return self.validate(hard_cap_abs=hard_cap_abs)

    def as_tuple(self) -> Tuple[int, int, int, int]:
        self.validate()
        return int(self.fl), int(self.rl), int(self.fr), int(self.rr)

    def to_dict(self) -> Dict[str, int]:
        self.validate()
        return {
            "fl": int(self.fl),
            "rl": int(self.rl),
            "fr": int(self.fr),
            "rr": int(self.rr),
        }


# =============================================================================
# Metadata / headers
# =============================================================================
@dataclass
class WheelCommandHeader:
    """
    Metadata for a wheel command (source, mode, timing).
    """
    command_id: str = ""
    source: str = "unknown"              # e.g. teleop, nav, follow, llm_bridge
    source_detail: str = ""              # optional detail (session, node, etc.)
    mode: str = "manual"                 # e.g. manual, auto, follow, stop, test
    frame_id: str = "base_link"          # wheel command is base-local
    priority: int = 0                    # for upstream mux/audit (informational here)
    created_unix_s: float = field(default_factory=_now_unix)
    created_monotonic_s: float = field(default_factory=_now_mono)
    note: str = ""

    def validate(self) -> "WheelCommandHeader":
        for name in ("command_id", "source", "source_detail", "mode", "frame_id", "note"):
            v = getattr(self, name)
            if not isinstance(v, str):
                raise WheelCommandValidationError(f"{name} must be str")

        # command_id may be blank (caller can fill later)
        if not isinstance(self.priority, int):
            raise WheelCommandValidationError("priority must be int")

        if not _is_finite(self.created_unix_s) or float(self.created_unix_s) < 0.0:
            raise WheelCommandValidationError("created_unix_s must be finite and >= 0")
        if not _is_finite(self.created_monotonic_s) or float(self.created_monotonic_s) < 0.0:
            raise WheelCommandValidationError("created_monotonic_s must be finite and >= 0")

        self.created_unix_s = float(self.created_unix_s)
        self.created_monotonic_s = float(self.created_monotonic_s)
        return self

    def to_dict(self) -> Dict[str, Any]:
        self.validate()
        return {
            "command_id": self.command_id,
            "source": self.source,
            "source_detail": self.source_detail,
            "mode": self.mode,
            "frame_id": self.frame_id,
            "priority": int(self.priority),
            "created_unix_s": float(self.created_unix_s),
            "created_monotonic_s": float(self.created_monotonic_s),
            "note": self.note,
        }


# =============================================================================
# Main command model
# =============================================================================
@dataclass
class WheelCommand:
    """
    Robot Savo wheel-space command model.

    Design intent
    -------------
    This model carries a command through the `savo_base` pipeline:
    - normalized wheel command (from kinematics/mixer)
    - optional scaled signed duty (after scaling module)
    - command timing fields for watchdog
    - source/mode metadata for logging/telemetry/debugging

    The actual motor board driver may consume `wheel_duty` after validation/clamping.
    """
    header: WheelCommandHeader = field(default_factory=WheelCommandHeader)

    # Core wheel command (normalized)
    wheel_norm: WheelNorm = field(default_factory=WheelNorm)

    # Optional post-scaling wheel duty (set later in pipeline)
    wheel_duty: Optional[WheelDuty] = None

    # Control / safety intent
    is_stop: bool = False               # explicit stop request (not just all zeros)
    emergency_stop: bool = False        # hard stop intent (e-stop or safety stop)
    watchdog_freshness_exempt: bool = False  # usually False; True only for special cases

    # Watchdog / execution timing
    timeout_s: float = 0.30             # command freshness timeout target
    valid_until_monotonic_s: float = 0.0  # optional precomputed expiry; 0 => derive from header+timeout

    # Optional upstream command-space context (useful for audit/debug)
    vx_cmd: Optional[float] = None
    vy_cmd: Optional[float] = None
    wz_cmd: Optional[float] = None
    speed_scale: Optional[float] = None     # e.g. teleop scale 0..1 or duty scale factor normalized

    # Diagnostics / provenance
    sequence: int = 0
    applied_conventions: str = "robot_savo_default"  # from kinematics/conventions.py
    note: str = ""

    def validate(self) -> "WheelCommand":
        self.header.validate()
        self.wheel_norm.validate(allow_over=True)  # allow upstream over-range before normalize()

        if self.wheel_duty is not None:
            if not isinstance(self.wheel_duty, WheelDuty):
                raise WheelCommandValidationError("wheel_duty must be WheelDuty or None")
            self.wheel_duty.validate()

        for f in ("is_stop", "emergency_stop", "watchdog_freshness_exempt"):
            if not isinstance(getattr(self, f), bool):
                raise WheelCommandValidationError(f"{f} must be bool")

        if not _is_finite(self.timeout_s) or float(self.timeout_s) <= 0.0:
            raise WheelCommandValidationError("timeout_s must be finite and > 0")
        self.timeout_s = float(self.timeout_s)

        if not _is_finite(self.valid_until_monotonic_s) or float(self.valid_until_monotonic_s) < 0.0:
            raise WheelCommandValidationError("valid_until_monotonic_s must be finite and >= 0")
        self.valid_until_monotonic_s = float(self.valid_until_monotonic_s)

        for f in ("vx_cmd", "vy_cmd", "wz_cmd", "speed_scale"):
            v = getattr(self, f)
            if v is not None and (not _is_finite(v)):
                raise WheelCommandValidationError(f"{f} must be finite float or None")

        if not isinstance(self.sequence, int) or self.sequence < 0:
            raise WheelCommandValidationError("sequence must be int >= 0")

        if not isinstance(self.applied_conventions, str):
            raise WheelCommandValidationError("applied_conventions must be str")
        if not isinstance(self.note, str):
            raise WheelCommandValidationError("note must be str")

        # Keep stop flags consistent with wheel values
        if self.emergency_stop:
            self.is_stop = True

        return self

    # -------------------------------------------------------------------------
    # Command freshness helpers (watchdog)
    # -------------------------------------------------------------------------
    def age_s(self, now_monotonic_s: Optional[float] = None) -> float:
        self.validate()
        now = _now_mono() if now_monotonic_s is None else float(now_monotonic_s)
        if not _is_finite(now) or now < 0.0:
            raise WheelCommandValidationError(f"now_monotonic_s must be finite and >= 0, got {now!r}")
        return max(0.0, now - float(self.header.created_monotonic_s))

    def expiry_monotonic_s(self) -> float:
        self.validate()
        if self.valid_until_monotonic_s > 0.0:
            return float(self.valid_until_monotonic_s)
        return float(self.header.created_monotonic_s + self.timeout_s)

    def is_expired(self, now_monotonic_s: Optional[float] = None) -> bool:
        self.validate()
        now = _now_mono() if now_monotonic_s is None else float(now_monotonic_s)
        if not _is_finite(now) or now < 0.0:
            raise WheelCommandValidationError(f"now_monotonic_s must be finite and >= 0, got {now!r}")
        if self.watchdog_freshness_exempt:
            return False
        return bool(now >= self.expiry_monotonic_s())

    # -------------------------------------------------------------------------
    # Normalized-wheel transforms
    # -------------------------------------------------------------------------
    def normalize_wheels(self) -> "WheelCommand":
        """
        Normalize wheel_norm so max(|wheel|) <= 1.0.
        """
        self.validate()
        self.wheel_norm = self.wheel_norm.normalized()
        return self.validate()

    def clamp_wheels(self, limit_abs: float = 1.0) -> "WheelCommand":
        """
        Hard clamp normalized wheels independently.
        """
        self.validate()
        self.wheel_norm.clamp(limit_abs=float(limit_abs))
        return self.validate()

    def scale_wheels(self, gain: float) -> "WheelCommand":
        """
        Scale normalized wheels by gain (may exceed 1.0 before later normalization/clamp).
        """
        self.validate()
        self.wheel_norm.scale(float(gain))
        return self.validate()

    # -------------------------------------------------------------------------
    # Duty conversion / assignment
    # -------------------------------------------------------------------------
    def compute_wheel_duty(self, *, max_abs_duty: int, hard_cap_abs: int = 4095) -> "WheelCommand":
        """
        Convert wheel_norm to signed wheel duties using current normalized values.

        This assumes wheel_norm is already in the desired final range
        (usually normalized/clamped to [-1,1] before conversion).
        """
        self.validate()
        if not isinstance(max_abs_duty, int) or max_abs_duty < 0 or max_abs_duty > hard_cap_abs:
            raise WheelCommandValidationError(
                f"max_abs_duty must be int in [0,{hard_cap_abs}], got {max_abs_duty!r}"
            )

        wn = self.wheel_norm.copy()

        # For explicit/emergency stop, force zero
        if self.is_stop or self.emergency_stop:
            self.wheel_duty = WheelDuty(0, 0, 0, 0)
            return self.validate()

        # Convert
        self.wheel_duty = WheelDuty(
            fl=int(round(float(wn.fl) * max_abs_duty)),
            rl=int(round(float(wn.rl) * max_abs_duty)),
            fr=int(round(float(wn.fr) * max_abs_duty)),
            rr=int(round(float(wn.rr) * max_abs_duty)),
        ).clamp(max_abs_duty=max_abs_duty, hard_cap_abs=hard_cap_abs)

        return self.validate()

    def set_wheel_duty(self, duty: WheelDuty) -> "WheelCommand":
        """
        Assign externally computed wheel duty.
        """
        self.validate()
        if not isinstance(duty, WheelDuty):
            raise WheelCommandValidationError("duty must be WheelDuty")
        self.wheel_duty = duty.copy().validate()
        return self.validate()

    # -------------------------------------------------------------------------
    # Stop helpers
    # -------------------------------------------------------------------------
    def make_stop(self, *, emergency: bool = False, note: str = "") -> "WheelCommand":
        """
        Convert this command into a stop command (zero normalized wheels and duty).
        """
        self.validate()
        self.wheel_norm.zero()
        self.wheel_duty = WheelDuty(0, 0, 0, 0)
        self.is_stop = True
        self.emergency_stop = bool(emergency)
        if note:
            self.note = str(note)
        return self.validate()

    def is_zero_command(self, eps: float = 1e-9) -> bool:
        self.validate()
        if self.wheel_duty is not None:
            return (not self.wheel_duty.any_nonzero())
        return (not self.wheel_norm.any_nonzero(eps=eps))

    # -------------------------------------------------------------------------
    # Summary / serialization
    # -------------------------------------------------------------------------
    def summary(self) -> str:
        self.validate()
        wn = self.wheel_norm
        duty_str = "None"
        if self.wheel_duty is not None:
            d = self.wheel_duty
            duty_str = f"({d.fl},{d.rl},{d.fr},{d.rr})"

        return (
            f"WheelCommand[src={self.header.source} mode={self.header.mode} "
            f"seq={self.sequence} stop={self.is_stop} estop={self.emergency_stop} "
            f"norm=({wn.fl:.3f},{wn.rl:.3f},{wn.fr:.3f},{wn.rr:.3f}) "
            f"duty={duty_str} timeout={self.timeout_s:.3f}s]"
        )

    def to_dict(self) -> Dict[str, Any]:
        self.validate()

        return {
            "header": self.header.to_dict(),
            "wheel_norm": self.wheel_norm.to_dict(),
            "wheel_duty": None if self.wheel_duty is None else self.wheel_duty.to_dict(),

            "flags": {
                "is_stop": self.is_stop,
                "emergency_stop": self.emergency_stop,
                "watchdog_freshness_exempt": self.watchdog_freshness_exempt,
                "is_zero_command": self.is_zero_command(),
            },

            "timing": {
                "timeout_s": float(self.timeout_s),
                "valid_until_monotonic_s": float(self.valid_until_monotonic_s),
                "expiry_monotonic_s": float(self.expiry_monotonic_s()),
                "age_s": float(self.age_s()),
                "expired_now": bool(self.is_expired()),
            },

            "command_space_context": {
                "vx_cmd": None if self.vx_cmd is None else float(self.vx_cmd),
                "vy_cmd": None if self.vy_cmd is None else float(self.vy_cmd),
                "wz_cmd": None if self.wz_cmd is None else float(self.wz_cmd),
                "speed_scale": None if self.speed_scale is None else float(self.speed_scale),
            },

            "diagnostics": {
                "sequence": int(self.sequence),
                "applied_conventions": self.applied_conventions,
                "note": self.note,
                "summary": self.summary(),
            },
        }


# =============================================================================
# Canonical constructors for Robot Savo
# =============================================================================
def make_robot_savo_wheel_command(
    *,
    source: str = "unknown",
    mode: str = "manual",
    wheel_norm: Optional[WheelNorm] = None,
    timeout_s: float = 0.30,
    command_id: str = "",
    source_detail: str = "",
    frame_id: str = "base_link",
    priority: int = 0,
    sequence: int = 0,
    applied_conventions: str = "robot_savo_default",
    note: str = "",
) -> WheelCommand:
    """
    Create a validated wheel command for Robot Savo.

    Recommended usage
    -----------------
    - Build from kinematics mix output (normalized wheels)
    - Then scale/normalize/clamp as needed
    - Then compute `wheel_duty` using max_abs_duty from base limits
    """
    cmd = WheelCommand(
        header=WheelCommandHeader(
            command_id=str(command_id),
            source=str(source),
            source_detail=str(source_detail),
            mode=str(mode),
            frame_id=str(frame_id),
            priority=int(priority),
        ),
        wheel_norm=wheel_norm.copy() if isinstance(wheel_norm, WheelNorm) else WheelNorm(),
        timeout_s=float(timeout_s),
        sequence=int(sequence),
        applied_conventions=str(applied_conventions),
        note=str(note),
    )
    return cmd.validate()


def make_robot_savo_stop_wheel_command(
    *,
    source: str = "safety",
    mode: str = "stop",
    emergency: bool = False,
    timeout_s: float = 0.30,
    note: str = "Stop command",
) -> WheelCommand:
    """
    Create a validated Robot Savo stop command (all wheel outputs zero).
    """
    cmd = make_robot_savo_wheel_command(
        source=source,
        mode=mode,
        wheel_norm=WheelNorm(0.0, 0.0, 0.0, 0.0),
        timeout_s=timeout_s,
        note=note,
    )
    cmd.make_stop(emergency=bool(emergency), note=note)
    return cmd.validate()


# =============================================================================
# Public exports
# =============================================================================
__all__ = [
    "WheelCommandValidationError",
    "WheelNorm",
    "WheelDuty",
    "WheelCommandHeader",
    "WheelCommand",
    "make_robot_savo_wheel_command",
    "make_robot_savo_stop_wheel_command",
]