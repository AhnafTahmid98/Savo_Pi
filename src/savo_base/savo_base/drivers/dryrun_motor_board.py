#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/drivers/dryrun_motor_board.py
----------------------------------------------------
Professional dry-run motor board for Robot Savo (ROS 2 Jazzy / non-ROS tools).

Purpose
- Safe software-only stand-in for the real Freenove mecanum motor board driver
- Lets you test teleop, kinematics, scaling, and control logic without hardware
- Preserves the same wheel-order conventions and API style used by real board drivers

Wheel order (locked, Robot Savo)
- (FL, RL, FR, RR)

Design goals
- No hardware dependencies (no smbus / I2C)
- Same style as the real board interface (`set_motor_model`, `stop`, `close`)
- Optional command logging/history for diagnostics and unit tests
- Clamp behavior matches PCA9685 signed duty expectations (-4095..4095)

Typical use
-----------
from savo_base.drivers.dryrun_motor_board import DryRunMotorBoard

board = DryRunMotorBoard(debug=True)
board.set_motor_model(1000, 1000, 1000, 1000)
board.stop()
board.close()
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

try:
    from .board_exceptions import (
        BoardValidationError,
        WheelCommandError,
        EmergencyStopActiveError,
    )
except Exception:  # pragma: no cover
    # Minimal fallbacks for early bootstrap
    class BoardValidationError(ValueError):
        pass

    class WheelCommandError(RuntimeError):
        pass

    class EmergencyStopActiveError(RuntimeError):
        pass


# =============================================================================
# Typed records
# =============================================================================
@dataclass(frozen=True)
class DryRunWheelCommand:
    """
    Snapshot of one applied signed wheel command in locked order (FL, RL, FR, RR).
    """
    timestamp_s: float
    fl: int
    rl: int
    fr: int
    rr: int
    source: str = "set_motor_model"

    def as_tuple(self) -> Tuple[int, int, int, int]:
        return (self.fl, self.rl, self.fr, self.rr)

    def as_dict(self) -> Dict[str, int | float | str]:
        return {
            "timestamp_s": self.timestamp_s,
            "fl": self.fl,
            "rl": self.rl,
            "fr": self.fr,
            "rr": self.rr,
            "source": self.source,
        }


@dataclass
class DryRunMotorBoardState:
    """
    Mutable runtime state for the dry-run board.
    """
    is_open: bool = True
    estop_active: bool = False
    command_count: int = 0
    last_command_time_s: float = 0.0
    last_command: Tuple[int, int, int, int] = (0, 0, 0, 0)
    stop_count: int = 0


# =============================================================================
# Dry-run board implementation
# =============================================================================
class DryRunMotorBoard:
    """
    Professional software-only motor board simulator for Robot Savo.

    API compatibility target (real board wrappers)
    ---------------------------------------------
    - set_motor_model(d_fl, d_rl, d_fr, d_rr)
    - stop()
    - close()

    Optional helpers
    ----------------
    - set_emergency_stop(active)
    - reset_history()
    - get_history()
    - get_state_dict()
    """

    # Robot Savo locked wheel order
    WHEEL_ORDER = ("FL", "RL", "FR", "RR")

    def __init__(
        self,
        *,
        max_abs_duty: int = 4095,
        max_history: int = 1000,
        debug: bool = False,
        name: str = "DryRunMotorBoard",
    ) -> None:
        self.name = str(name)
        self.debug = bool(debug)

        self.max_abs_duty = self._validate_max_abs_duty(max_abs_duty)
        self.max_history = self._validate_max_history(max_history)

        self._state = DryRunMotorBoardState()
        self._history: List[DryRunWheelCommand] = []

        if self.debug:
            print(
                f"[{self.name}] init: max_abs_duty={self.max_abs_duty}, "
                f"max_history={self.max_history}"
            )

    # -------------------------------------------------------------------------
    # Validation / utility helpers
    # -------------------------------------------------------------------------
    @staticmethod
    def _validate_max_abs_duty(value: int) -> int:
        try:
            v = int(value)
        except Exception as e:
            raise BoardValidationError(f"max_abs_duty must be int, got {value!r}") from e
        if v < 0 or v > 4095:
            raise BoardValidationError(f"max_abs_duty must be in [0, 4095], got {v}")
        return v

    @staticmethod
    def _validate_max_history(value: int) -> int:
        try:
            v = int(value)
        except Exception as e:
            raise BoardValidationError(f"max_history must be int, got {value!r}") from e
        if v < 1:
            raise BoardValidationError(f"max_history must be >= 1, got {v}")
        return v

    @staticmethod
    def _to_int(name: str, value) -> int:
        try:
            return int(value)
        except Exception as e:
            raise BoardValidationError(f"{name} must be int-compatible, got {value!r}") from e

    def _clamp_signed_duty(self, value: int) -> int:
        if value > self.max_abs_duty:
            return self.max_abs_duty
        if value < -self.max_abs_duty:
            return -self.max_abs_duty
        return int(value)

    def _ensure_open(self) -> None:
        if not self._state.is_open:
            raise WheelCommandError(f"{self.name} is closed")

    def _append_history(self, cmd: DryRunWheelCommand) -> None:
        self._history.append(cmd)
        if len(self._history) > self.max_history:
            # keep latest entries only
            self._history = self._history[-self.max_history :]

    # -------------------------------------------------------------------------
    # Core API (compatible style)
    # -------------------------------------------------------------------------
    def set_motor_model(self, d_fl, d_rl, d_fr, d_rr) -> None:
        """
        Apply signed wheel duties in Robot Savo locked order (FL, RL, FR, RR).

        Values are clamped to [-max_abs_duty, +max_abs_duty].
        """
        self._ensure_open()

        if self._state.estop_active:
            raise EmergencyStopActiveError(
                f"{self.name}: command rejected because emergency stop is active"
            )

        fl = self._clamp_signed_duty(self._to_int("d_fl", d_fl))
        rl = self._clamp_signed_duty(self._to_int("d_rl", d_rl))
        fr = self._clamp_signed_duty(self._to_int("d_fr", d_fr))
        rr = self._clamp_signed_duty(self._to_int("d_rr", d_rr))

        t = time.time()

        self._state.last_command = (fl, rl, fr, rr)
        self._state.last_command_time_s = t
        self._state.command_count += 1

        cmd = DryRunWheelCommand(
            timestamp_s=t,
            fl=fl,
            rl=rl,
            fr=fr,
            rr=rr,
            source="set_motor_model",
        )
        self._append_history(cmd)

        if self.debug:
            print(
                f"[{self.name}] cmd#{self._state.command_count}: "
                f"FL={fl:>5} RL={rl:>5} FR={fr:>5} RR={rr:>5}"
            )

    def stop(self) -> None:
        """
        Stop all wheels (software simulation).
        """
        self._ensure_open()

        t = time.time()
        self._state.last_command = (0, 0, 0, 0)
        self._state.last_command_time_s = t
        self._state.command_count += 1
        self._state.stop_count += 1

        cmd = DryRunWheelCommand(
            timestamp_s=t,
            fl=0,
            rl=0,
            fr=0,
            rr=0,
            source="stop",
        )
        self._append_history(cmd)

        if self.debug:
            print(f"[{self.name}] stop()")

    def close(self) -> None:
        """
        Close the dry-run board (marks instance unusable for further commands).
        """
        if not self._state.is_open:
            return

        # best-effort stop before closing
        try:
            self.stop()
        except Exception:
            pass

        self._state.is_open = False

        if self.debug:
            print(f"[{self.name}] close()")

    # -------------------------------------------------------------------------
    # Optional professional helpers
    # -------------------------------------------------------------------------
    def set_emergency_stop(self, active: bool = True) -> None:
        """
        Enable/disable software emergency stop flag.

        When active, `set_motor_model(...)` raises `EmergencyStopActiveError`.
        """
        self._ensure_open()
        self._state.estop_active = bool(active)

        if bool(active):
            # best practice: immediately force stop record when estop activates
            try:
                self.stop()
            except Exception:
                pass

        if self.debug:
            print(f"[{self.name}] estop={'ON' if self._state.estop_active else 'OFF'}")

    def clear_emergency_stop(self) -> None:
        """
        Convenience alias to disable emergency stop.
        """
        self.set_emergency_stop(False)

    def reset_history(self) -> None:
        """
        Clear stored command history (does not alter current state).
        """
        self._history.clear()
        if self.debug:
            print(f"[{self.name}] history reset")

    def get_history(self) -> List[DryRunWheelCommand]:
        """
        Return a shallow copy of command history records.
        """
        return list(self._history)

    def get_last_command(self) -> Tuple[int, int, int, int]:
        """
        Return the last applied command tuple (FL, RL, FR, RR).
        """
        return tuple(self._state.last_command)

    def get_state_dict(self) -> Dict[str, object]:
        """
        Structured state snapshot for logs/tests/diagnostics.
        """
        return {
            "name": self.name,
            "is_open": self._state.is_open,
            "estop_active": self._state.estop_active,
            "command_count": self._state.command_count,
            "stop_count": self._state.stop_count,
            "last_command_time_s": self._state.last_command_time_s,
            "last_command": {
                "FL": self._state.last_command[0],
                "RL": self._state.last_command[1],
                "FR": self._state.last_command[2],
                "RR": self._state.last_command[3],
            },
            "max_abs_duty": self.max_abs_duty,
            "max_history": self.max_history,
            "history_len": len(self._history),
            "wheel_order": self.WHEEL_ORDER,
        }

    def summary(self) -> str:
        """
        Compact human-readable status line.
        """
        fl, rl, fr, rr = self._state.last_command
        return (
            f"{self.name}(open={self._state.is_open}, estop={self._state.estop_active}, "
            f"cmds={self._state.command_count}, last=[FL={fl}, RL={rl}, FR={fr}, RR={rr}])"
        )

    # -------------------------------------------------------------------------
    # Context manager support
    # -------------------------------------------------------------------------
    def __enter__(self) -> "DryRunMotorBoard":
        self._ensure_open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


# Backward-compatible aliases (helpful while codebase is evolving)
DryRunMecanumBoard = DryRunMotorBoard
MockMotorBoard = DryRunMotorBoard