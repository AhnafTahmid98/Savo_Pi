#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/models/motor_board_status.py
---------------------------------------------------
Professional motor-board status models for Robot Savo (ROS 2 Jazzy, real robot).

Purpose
- Represent runtime health/state of the low-level motor board interface
- Track connection/readiness, output enable state, active wheel duties, and faults
- Provide safe validation + serialization for dashboards/logging/telemetry

Scope
- Pure Python data models (no ROS imports, no hardware I/O)
- Intended for use by:
  - savo_base execution node
  - Freenove/PCA9685 board drivers
  - dry-run motor board backend
  - diagnostics and dashboards
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Any, Tuple, Optional
import math
import time


# =============================================================================
# Exceptions
# =============================================================================
class MotorBoardStatusValidationError(ValueError):
    """Raised when a motor-board status model contains invalid values."""
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
        raise MotorBoardStatusValidationError(f"Invalid clamp range: lo={lo} > hi={hi}")
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def _now_mono() -> float:
    return float(time.monotonic())


def _now_unix() -> float:
    return float(time.time())


# =============================================================================
# Wheel duty state (signed wheel command domain)
# =============================================================================
@dataclass
class WheelDutyState:
    """
    Signed wheel duties for Robot Savo wheel order: FL, RL, FR, RR.

    Notes
    -----
    - These are wheel-space signed duties, not raw PCA9685 per-channel values.
    - Typical valid range is [-max_abs_duty, +max_abs_duty], hard-capped at 4095.
    """
    fl: int = 0
    rl: int = 0
    fr: int = 0
    rr: int = 0

    def validate(self, *, hard_cap_abs: int = 4095, field_name: str = "wheel_duty") -> "WheelDutyState":
        try:
            vals = [int(self.fl), int(self.rl), int(self.fr), int(self.rr)]
        except Exception as e:
            raise MotorBoardStatusValidationError(f"{field_name} values must be int-like") from e

        if not isinstance(hard_cap_abs, int) or hard_cap_abs < 0 or hard_cap_abs > 4095:
            raise MotorBoardStatusValidationError(f"hard_cap_abs must be int in [0,4095], got {hard_cap_abs!r}")

        for name, v in zip(("fl", "rl", "fr", "rr"), vals):
            if abs(v) > hard_cap_abs:
                raise MotorBoardStatusValidationError(
                    f"{field_name}.{name}={v} exceeds hard_cap_abs={hard_cap_abs}"
                )
        return self

    def zero(self) -> "WheelDutyState":
        self.fl = self.rl = self.fr = self.rr = 0
        return self

    def copy(self) -> "WheelDutyState":
        return WheelDutyState(fl=int(self.fl), rl=int(self.rl), fr=int(self.fr), rr=int(self.rr))

    def as_tuple(self) -> Tuple[int, int, int, int]:
        self.validate()
        return int(self.fl), int(self.rl), int(self.fr), int(self.rr)

    def max_abs(self) -> int:
        self.validate()
        return max(abs(int(self.fl)), abs(int(self.rl)), abs(int(self.fr)), abs(int(self.rr)))

    def any_nonzero(self) -> bool:
        self.validate()
        return self.max_abs() > 0

    def clamp(self, *, max_abs_duty: int, hard_cap_abs: int = 4095) -> "WheelDutyState":
        self.validate(hard_cap_abs=hard_cap_abs)
        if not isinstance(max_abs_duty, int) or max_abs_duty < 0 or max_abs_duty > hard_cap_abs:
            raise MotorBoardStatusValidationError(
                f"max_abs_duty must be int in [0,{hard_cap_abs}], got {max_abs_duty!r}"
            )

        def c(v: int) -> int:
            vv = int(v)
            if vv > max_abs_duty:
                return max_abs_duty
            if vv < -max_abs_duty:
                return -max_abs_duty
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
# Fault / warning state
# =============================================================================
@dataclass
class MotorBoardFaultState:
    """
    Fault and warning flags for the motor board execution layer.

    Keep this generic so it works for:
    - real Freenove/PCA9685 hardware
    - dry-run board backend
    - future drivers
    """
    has_fault: bool = False
    has_warning: bool = False

    # Common causes (driver/runtime-level)
    i2c_error: bool = False
    bus_not_found: bool = False
    device_not_found: bool = False
    write_error: bool = False
    read_error: bool = False
    overcurrent_suspected: bool = False
    undervoltage_suspected: bool = False
    watchdog_timeout: bool = False
    estop_forced_zero: bool = False

    last_fault_code: str = ""
    last_fault_message: str = ""
    last_warning_code: str = ""
    last_warning_message: str = ""

    fault_count: int = 0
    warning_count: int = 0

    def validate(self) -> "MotorBoardFaultState":
        bool_fields = [
            "has_fault", "has_warning",
            "i2c_error", "bus_not_found", "device_not_found",
            "write_error", "read_error",
            "overcurrent_suspected", "undervoltage_suspected",
            "watchdog_timeout", "estop_forced_zero",
        ]
        for f in bool_fields:
            if not isinstance(getattr(self, f), bool):
                raise MotorBoardStatusValidationError(f"{f} must be bool")

        str_fields = [
            "last_fault_code", "last_fault_message",
            "last_warning_code", "last_warning_message",
        ]
        for f in str_fields:
            if not isinstance(getattr(self, f), str):
                raise MotorBoardStatusValidationError(f"{f} must be str")

        if not isinstance(self.fault_count, int) or self.fault_count < 0:
            raise MotorBoardStatusValidationError("fault_count must be int >= 0")
        if not isinstance(self.warning_count, int) or self.warning_count < 0:
            raise MotorBoardStatusValidationError("warning_count must be int >= 0")

        # Keep aggregate flags consistent
        self.has_fault = bool(
            self.has_fault or self.i2c_error or self.bus_not_found or self.device_not_found
            or self.write_error or self.read_error or self.overcurrent_suspected
            or self.undervoltage_suspected or self.watchdog_timeout
        )
        self.has_warning = bool(
            self.has_warning or self.estop_forced_zero or self.last_warning_code or self.last_warning_message
        )
        return self

    def clear_faults(self) -> "MotorBoardFaultState":
        self.has_fault = False
        self.i2c_error = False
        self.bus_not_found = False
        self.device_not_found = False
        self.write_error = False
        self.read_error = False
        self.overcurrent_suspected = False
        self.undervoltage_suspected = False
        self.watchdog_timeout = False
        self.last_fault_code = ""
        self.last_fault_message = ""
        return self.validate()

    def clear_warnings(self) -> "MotorBoardFaultState":
        self.has_warning = False
        self.estop_forced_zero = False
        self.last_warning_code = ""
        self.last_warning_message = ""
        return self.validate()

    def set_fault(self, code: str, message: str = "") -> "MotorBoardFaultState":
        self.has_fault = True
        self.fault_count = int(self.fault_count) + 1
        self.last_fault_code = str(code)
        self.last_fault_message = str(message)
        return self.validate()

    def set_warning(self, code: str, message: str = "") -> "MotorBoardFaultState":
        self.has_warning = True
        self.warning_count = int(self.warning_count) + 1
        self.last_warning_code = str(code)
        self.last_warning_message = str(message)
        return self.validate()

    def to_dict(self) -> Dict[str, Any]:
        self.validate()
        return {
            "has_fault": self.has_fault,
            "has_warning": self.has_warning,
            "i2c_error": self.i2c_error,
            "bus_not_found": self.bus_not_found,
            "device_not_found": self.device_not_found,
            "write_error": self.write_error,
            "read_error": self.read_error,
            "overcurrent_suspected": self.overcurrent_suspected,
            "undervoltage_suspected": self.undervoltage_suspected,
            "watchdog_timeout": self.watchdog_timeout,
            "estop_forced_zero": self.estop_forced_zero,
            "last_fault_code": self.last_fault_code,
            "last_fault_message": self.last_fault_message,
            "last_warning_code": self.last_warning_code,
            "last_warning_message": self.last_warning_message,
            "fault_count": int(self.fault_count),
            "warning_count": int(self.warning_count),
        }


# =============================================================================
# PCA9685 / board config snapshot
# =============================================================================
@dataclass
class MotorBoardConfigSnapshot:
    """
    Runtime configuration snapshot of the motor board driver.

    This is useful for telemetry and debugging "what settings are active right now?"
    """
    board_type: str = "unknown"               # e.g. freenove_mecanum / dryrun
    driver_name: str = "unknown"              # e.g. pca9685_driver
    backend: str = "unknown"                  # e.g. smbus / dryrun
    i2c_bus: int = 1
    i2c_addr: int = 0x40
    pwm_freq_hz: float = 50.0
    max_abs_duty: int = 3000
    hard_cap_duty: int = 4095
    quench_ms: int = 18

    # Robot Savo locked wheel channel map
    # FL(0,1), RL(3,2), FR(6,7), RR(4,5)
    wheel_map_name: str = "robot_savo_freenove_locked_v1"

    # Optional inversion flags at board/wheel layer
    invert_fl: bool = False
    invert_rl: bool = False
    invert_fr: bool = False
    invert_rr: bool = False

    def validate(self) -> "MotorBoardConfigSnapshot":
        for f in ("board_type", "driver_name", "backend", "wheel_map_name"):
            v = getattr(self, f)
            if not isinstance(v, str) or not v.strip():
                raise MotorBoardStatusValidationError(f"{f} must be non-empty str")

        if not isinstance(self.i2c_bus, int) or self.i2c_bus < 0:
            raise MotorBoardStatusValidationError("i2c_bus must be int >= 0")
        if not isinstance(self.i2c_addr, int) or self.i2c_addr < 0x00 or self.i2c_addr > 0x7F:
            raise MotorBoardStatusValidationError("i2c_addr must be int in [0x00,0x7F]")

        if not _is_finite(self.pwm_freq_hz) or float(self.pwm_freq_hz) <= 0.0:
            raise MotorBoardStatusValidationError("pwm_freq_hz must be finite and > 0")

        if not isinstance(self.hard_cap_duty, int) or self.hard_cap_duty < 0 or self.hard_cap_duty > 4095:
            raise MotorBoardStatusValidationError("hard_cap_duty must be int in [0,4095]")
        if not isinstance(self.max_abs_duty, int) or self.max_abs_duty < 0 or self.max_abs_duty > self.hard_cap_duty:
            raise MotorBoardStatusValidationError("max_abs_duty must be int in [0, hard_cap_duty]")
        if not isinstance(self.quench_ms, int) or self.quench_ms < 0:
            raise MotorBoardStatusValidationError("quench_ms must be int >= 0")

        for f in ("invert_fl", "invert_rl", "invert_fr", "invert_rr"):
            if not isinstance(getattr(self, f), bool):
                raise MotorBoardStatusValidationError(f"{f} must be bool")

        return self

    def invert_tuple(self) -> Tuple[int, int, int, int]:
        self.validate()
        return (
            -1 if self.invert_fl else +1,
            -1 if self.invert_rl else +1,
            -1 if self.invert_fr else +1,
            -1 if self.invert_rr else +1,
        )

    def to_dict(self) -> Dict[str, Any]:
        self.validate()
        return {
            "board_type": self.board_type,
            "driver_name": self.driver_name,
            "backend": self.backend,
            "i2c_bus": int(self.i2c_bus),
            "i2c_addr": int(self.i2c_addr),
            "pwm_freq_hz": float(self.pwm_freq_hz),
            "max_abs_duty": int(self.max_abs_duty),
            "hard_cap_duty": int(self.hard_cap_duty),
            "quench_ms": int(self.quench_ms),
            "wheel_map_name": self.wheel_map_name,
            "invert_fl": self.invert_fl,
            "invert_rl": self.invert_rl,
            "invert_fr": self.invert_fr,
            "invert_rr": self.invert_rr,
        }


# =============================================================================
# Main aggregate status model
# =============================================================================
@dataclass
class MotorBoardStatus:
    """
    Aggregated motor-board runtime status for Robot Savo.

    This object is intended to be owned/updated by the base driver or base node.

    Includes
    --------
    - connection/readiness state
    - output enable state
    - active requested/applied wheel duties
    - config snapshot (bus/address/pwm/etc.)
    - fault/warning state
    - timing / watchdog / counters
    """
    robot_name: str = "Robot Savo"
    node_name: str = "savo_base"
    board_name: str = "freenove_mecanum_board"

    # Connectivity / lifecycle
    connected: bool = False
    initialized: bool = False
    ready: bool = False
    outputs_enabled: bool = False
    dryrun: bool = False

    # Current output state (wheel-space signed duty)
    duty_requested: WheelDutyState = field(default_factory=WheelDutyState)
    duty_applied: WheelDutyState = field(default_factory=WheelDutyState)

    # Runtime config snapshot
    config: MotorBoardConfigSnapshot = field(default_factory=MotorBoardConfigSnapshot)

    # Faults/warnings
    faults: MotorBoardFaultState = field(default_factory=MotorBoardFaultState)

    # Counters
    command_count: int = 0
    zero_command_count: int = 0
    nonzero_command_count: int = 0
    write_count: int = 0
    stop_count: int = 0
    estop_zero_count: int = 0

    # Timestamps
    created_unix_s: float = field(default_factory=_now_unix)
    last_update_unix_s: float = field(default_factory=_now_unix)
    last_update_monotonic_s: float = field(default_factory=_now_mono)
    last_connect_monotonic_s: float = 0.0
    last_command_monotonic_s: float = 0.0
    last_nonzero_command_monotonic_s: float = 0.0
    last_write_monotonic_s: float = 0.0
    last_stop_monotonic_s: float = 0.0

    # Diagnostics
    last_error: str = ""
    last_warning: str = ""
    note: str = ""

    def validate(self) -> "MotorBoardStatus":
        for f in ("robot_name", "node_name", "board_name"):
            v = getattr(self, f)
            if not isinstance(v, str) or not v.strip():
                raise MotorBoardStatusValidationError(f"{f} must be non-empty str")

        for f in ("connected", "initialized", "ready", "outputs_enabled", "dryrun"):
            if not isinstance(getattr(self, f), bool):
                raise MotorBoardStatusValidationError(f"{f} must be bool")

        self.duty_requested.validate(hard_cap_abs=self.config.hard_cap_duty, field_name="duty_requested")
        self.duty_applied.validate(hard_cap_abs=self.config.hard_cap_duty, field_name="duty_applied")
        self.config.validate()
        self.faults.validate()

        for f in (
            "command_count", "zero_command_count", "nonzero_command_count",
            "write_count", "stop_count", "estop_zero_count",
        ):
            v = getattr(self, f)
            if not isinstance(v, int) or v < 0:
                raise MotorBoardStatusValidationError(f"{f} must be int >= 0")

        for f in (
            "created_unix_s",
            "last_update_unix_s",
            "last_update_monotonic_s",
            "last_connect_monotonic_s",
            "last_command_monotonic_s",
            "last_nonzero_command_monotonic_s",
            "last_write_monotonic_s",
            "last_stop_monotonic_s",
        ):
            v = getattr(self, f)
            if not _is_finite(v) or float(v) < 0.0:
                raise MotorBoardStatusValidationError(f"{f} must be finite and >= 0, got {v!r}")

        for f in ("last_error", "last_warning", "note"):
            if not isinstance(getattr(self, f), str):
                raise MotorBoardStatusValidationError(f"{f} must be str")

        # Keep "ready" consistent: board should only be ready if initialized + connected + no fault
        if self.ready and (not self.connected or not self.initialized):
            self.ready = False

        # If outputs_enabled true but not ready -> downgrade outputs_enabled
        if self.outputs_enabled and not self.ready:
            self.outputs_enabled = False

        return self

    # -------------------------------------------------------------------------
    # Timing helpers
    # -------------------------------------------------------------------------
    def touch(self) -> "MotorBoardStatus":
        self.last_update_unix_s = _now_unix()
        self.last_update_monotonic_s = _now_mono()
        return self

    def connect_mark(self) -> "MotorBoardStatus":
        self.connected = True
        self.last_connect_monotonic_s = _now_mono()
        self.touch()
        return self

    def disconnect_mark(self, *, reason: str = "") -> "MotorBoardStatus":
        self.connected = False
        self.initialized = False
        self.ready = False
        self.outputs_enabled = False
        self.duty_requested.zero()
        self.duty_applied.zero()
        if reason:
            self.last_warning = str(reason)
        self.touch()
        return self

    # -------------------------------------------------------------------------
    # Command / output bookkeeping
    # -------------------------------------------------------------------------
    def record_command_requested(self, duty: WheelDutyState) -> "MotorBoardStatus":
        """
        Record requested wheel duty command (before actual board write).
        """
        self.validate()
        duty.validate(hard_cap_abs=self.config.hard_cap_duty)
        self.duty_requested = duty.copy()
        self.command_count += 1
        self.last_command_monotonic_s = _now_mono()

        if duty.any_nonzero():
            self.nonzero_command_count += 1
            self.last_nonzero_command_monotonic_s = self.last_command_monotonic_s
        else:
            self.zero_command_count += 1

        self.touch()
        return self

    def record_write_applied(self, duty: WheelDutyState) -> "MotorBoardStatus":
        """
        Record successfully applied duty command to board.
        """
        self.validate()
        duty.validate(hard_cap_abs=self.config.hard_cap_duty)
        self.duty_applied = duty.copy()
        self.write_count += 1
        self.last_write_monotonic_s = _now_mono()
        self.touch()
        return self

    def record_stop(self, *, reason: str = "") -> "MotorBoardStatus":
        """
        Record stop action (zero applied outputs).
        """
        self.stop_count += 1
        self.duty_requested.zero()
        self.duty_applied.zero()
        self.outputs_enabled = False if self.faults.has_fault else self.outputs_enabled
        self.last_stop_monotonic_s = _now_mono()
        if reason:
            self.last_warning = str(reason)
        self.touch()
        return self

    def record_estop_forced_zero(self, *, reason: str = "ESTOP forced zero") -> "MotorBoardStatus":
        """
        Record estop-forced zeroing of outputs.
        """
        self.estop_zero_count += 1
        self.faults.estop_forced_zero = True
        self.faults.set_warning(code="ESTOP_FORCED_ZERO", message=reason)
        self.duty_requested.zero()
        self.duty_applied.zero()
        self.outputs_enabled = False
        self.last_stop_monotonic_s = _now_mono()
        self.touch()
        return self

    # -------------------------------------------------------------------------
    # Fault/warning helpers
    # -------------------------------------------------------------------------
    def set_error(self, msg: str, *, code: str = "") -> "MotorBoardStatus":
        self.last_error = str(msg)
        if code or msg:
            self.faults.set_fault(code=code or "BOARD_ERROR", message=msg)
        self.ready = False
        self.outputs_enabled = False
        self.touch()
        return self

    def clear_error(self) -> "MotorBoardStatus":
        self.last_error = ""
        self.touch()
        return self

    def set_warning(self, msg: str, *, code: str = "") -> "MotorBoardStatus":
        self.last_warning = str(msg)
        if code or msg:
            self.faults.set_warning(code=code or "BOARD_WARNING", message=msg)
        self.touch()
        return self

    def clear_warning(self) -> "MotorBoardStatus":
        self.last_warning = ""
        self.touch()
        return self

    # -------------------------------------------------------------------------
    # Status / telemetry helpers
    # -------------------------------------------------------------------------
    def outputs_active(self) -> bool:
        self.validate()
        return bool(self.outputs_enabled and self.duty_applied.any_nonzero())

    def command_age_s(self) -> float:
        self.validate()
        if self.last_command_monotonic_s <= 0.0:
            return math.inf
        return max(0.0, _now_mono() - float(self.last_command_monotonic_s))

    def write_age_s(self) -> float:
        self.validate()
        if self.last_write_monotonic_s <= 0.0:
            return math.inf
        return max(0.0, _now_mono() - float(self.last_write_monotonic_s))

    def status_level(self) -> str:
        self.validate()
        if self.faults.has_fault or self.last_error:
            return "ERROR"
        if not self.connected:
            return "DISCONNECTED"
        if not self.initialized:
            return "UNINITIALIZED"
        if not self.ready:
            return "NOT_READY"
        if self.outputs_active():
            return "ACTIVE"
        return "READY"

    def short_summary(self) -> str:
        self.validate()
        d = self.duty_applied
        cfg = self.config
        return (
            f"[{self.status_level()}] board={self.board_name} type={cfg.board_type} "
            f"bus={cfg.i2c_bus} addr=0x{cfg.i2c_addr:02X} pwm={cfg.pwm_freq_hz:.1f}Hz "
            f"ready={self.ready} out_en={self.outputs_enabled} dryrun={self.dryrun} "
            f"duty=({d.fl},{d.rl},{d.fr},{d.rr}) writes={self.write_count} "
            f"faults={self.faults.fault_count} warns={self.faults.warning_count}"
        )

    def to_dict(self) -> Dict[str, Any]:
        self.validate()
        return {
            "robot_name": self.robot_name,
            "node_name": self.node_name,
            "board_name": self.board_name,
            "status_level": self.status_level(),

            "state": {
                "connected": self.connected,
                "initialized": self.initialized,
                "ready": self.ready,
                "outputs_enabled": self.outputs_enabled,
                "outputs_active": self.outputs_active(),
                "dryrun": self.dryrun,
            },

            "duty_requested": self.duty_requested.to_dict(),
            "duty_applied": self.duty_applied.to_dict(),

            "config": self.config.to_dict(),
            "faults": self.faults.to_dict(),

            "counters": {
                "command_count": int(self.command_count),
                "zero_command_count": int(self.zero_command_count),
                "nonzero_command_count": int(self.nonzero_command_count),
                "write_count": int(self.write_count),
                "stop_count": int(self.stop_count),
                "estop_zero_count": int(self.estop_zero_count),
            },

            "timing": {
                "created_unix_s": float(self.created_unix_s),
                "last_update_unix_s": float(self.last_update_unix_s),
                "last_update_monotonic_s": float(self.last_update_monotonic_s),
                "last_connect_monotonic_s": float(self.last_connect_monotonic_s),
                "last_command_monotonic_s": float(self.last_command_monotonic_s),
                "last_nonzero_command_monotonic_s": float(self.last_nonzero_command_monotonic_s),
                "last_write_monotonic_s": float(self.last_write_monotonic_s),
                "last_stop_monotonic_s": float(self.last_stop_monotonic_s),
                "command_age_s": self.command_age_s(),
                "write_age_s": self.write_age_s(),
            },

            "diagnostics": {
                "last_error": self.last_error,
                "last_warning": self.last_warning,
                "note": self.note,
                "summary": self.short_summary(),
            },
        }


# =============================================================================
# Canonical constructors (Robot Savo)
# =============================================================================
def make_robot_savo_freenove_board_status(
    *,
    i2c_bus: int = 1,
    i2c_addr: int = 0x40,
    pwm_freq_hz: float = 50.0,
    max_abs_duty: int = 3000,
    quench_ms: int = 18,
    invert_fl: bool = False,
    invert_rl: bool = False,
    invert_fr: bool = False,
    invert_rr: bool = False,
) -> MotorBoardStatus:
    """
    Create a validated MotorBoardStatus configured for Robot Savo's Freenove/PCA9685 base board.

    Locked hardware mapping (Robot Savo current baseline)
    -----------------------------------------------------
    FL(0,1), RL(3,2), FR(6,7), RR(4,5)
    PCA9685 @ 0x40, typical PWM 50Hz, max duty ~3000
    """
    status = MotorBoardStatus(
        robot_name="Robot Savo",
        node_name="savo_base",
        board_name="freenove_mecanum_board",
        connected=False,
        initialized=False,
        ready=False,
        outputs_enabled=False,
        dryrun=False,
        config=MotorBoardConfigSnapshot(
            board_type="freenove_mecanum",
            driver_name="pca9685_driver",
            backend="smbus",
            i2c_bus=int(i2c_bus),
            i2c_addr=int(i2c_addr),
            pwm_freq_hz=float(pwm_freq_hz),
            max_abs_duty=int(max_abs_duty),
            hard_cap_duty=4095,
            quench_ms=int(quench_ms),
            wheel_map_name="robot_savo_freenove_locked_v1",
            invert_fl=bool(invert_fl),
            invert_rl=bool(invert_rl),
            invert_fr=bool(invert_fr),
            invert_rr=bool(invert_rr),
        ),
    )
    return status.validate()


def make_robot_savo_dryrun_board_status(
    *,
    max_abs_duty: int = 3000,
) -> MotorBoardStatus:
    """
    Create a dry-run backend status model for development/testing without hardware.
    """
    status = MotorBoardStatus(
        robot_name="Robot Savo",
        node_name="savo_base",
        board_name="dryrun_motor_board",
        connected=True,
        initialized=True,
        ready=True,
        outputs_enabled=False,
        dryrun=True,
        config=MotorBoardConfigSnapshot(
            board_type="dryrun",
            driver_name="dryrun_motor_board",
            backend="dryrun",
            i2c_bus=1,
            i2c_addr=0x40,
            pwm_freq_hz=50.0,
            max_abs_duty=int(max_abs_duty),
            hard_cap_duty=4095,
            quench_ms=0,
            wheel_map_name="robot_savo_freenove_locked_v1",
        ),
        note="Dry-run backend: no physical motor output",
    )
    return status.validate()


# =============================================================================
# Public exports
# =============================================================================
__all__ = [
    "MotorBoardStatusValidationError",
    "WheelDutyState",
    "MotorBoardFaultState",
    "MotorBoardConfigSnapshot",
    "MotorBoardStatus",
    "make_robot_savo_freenove_board_status",
    "make_robot_savo_dryrun_board_status",
]