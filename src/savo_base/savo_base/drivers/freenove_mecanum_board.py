#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/drivers/freenove_mecanum_board.py
--------------------------------------------------------
Professional Freenove/PCA9685 mecanum motor-board wrapper for Robot Savo
(ROS 2 Jazzy, real hardware testing).

Purpose
- Encode the *board-specific* motor channel map and direction semantics that were
  proven on the real Robot Savo hardware
- Provide a clean API for ROS nodes and CLI tools (teleop, smoke test, automode)
- Build on top of the generic low-level PCA9685 driver

IMPORTANT (locked real hardware behavior)
- Channel map (proven):
    FL (front-left)  : (0,1)
    RL (rear-left)   : (3,2)
    FR (front-right) : (6,7)
    RR (rear-right)  : (4,5)
- Wheel direction semantics (proven):
    d > 0  -> first channel = 0, second channel = d
    d < 0  -> second channel = 0, first channel = -d
    d = 0  -> BOTH channels = 4095   (board-proven stop behavior)
- Sign-flip quench (default 18 ms) is preserved to protect real hardware during reversals.

Design notes
- This module is hardware-focused (no ROS imports)
- Kinematics (vx, vy, wz -> wheel mixing) should remain in kinematics modules
- Safety/watchdog logic should remain in nodes/safety modules
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Tuple

from .pca9685_driver import PCA9685Config, PCA9685Driver, create_from_config


# =============================================================================
# Config dataclass
# =============================================================================
@dataclass(frozen=True)
class FreenoveMecanumBoardConfig:
    """
    Typed config for the Freenove mecanum motor board wrapper.

    Fields
    - pca9685: PCA9685 low-level driver config
    - invert_fl/rl/fr/rr: per-wheel invert flags (False=normal, True=invert)
    - quench_ms: neutral delay applied when wheel command sign flips (+ to - or - to +)
    """
    pca9685: PCA9685Config = PCA9685Config()
    invert_fl: bool = False
    invert_rl: bool = False
    invert_fr: bool = False
    invert_rr: bool = False
    quench_ms: int = 18


# =============================================================================
# Freenove mecanum board wrapper
# =============================================================================
class FreenoveMecanumBoard:
    """
    Board-specific motor wrapper for Robot Savo's Freenove + PCA9685 drivetrain.

    Public API (main)
    - set_motor_model(d_fl, d_rl, d_fr, d_rr)   # signed wheel commands [-4095..4095]
    - stop()                                    # board-proven stop behavior
    - close()                                   # stop + close I2C

    Wheel command convention
    - Positive/negative sign follows the proven physical direction semantics encoded
      in the per-wheel writers and optional invert flags.
    """

    # Locked Robot Savo channel map (wheel -> (A_channel, B_channel))
    # NOTE: "positive uses second channel" is implemented in the wheel writer methods.
    FL_PAIR: Tuple[int, int] = (0, 1)
    RL_PAIR: Tuple[int, int] = (3, 2)
    FR_PAIR: Tuple[int, int] = (6, 7)
    RR_PAIR: Tuple[int, int] = (4, 5)

    PWM_MIN = -4095
    PWM_MAX = 4095

    def __init__(
        self,
        *,
        i2c_bus: int = 1,
        addr: int = 0x40,
        pwm_freq_hz: float = 50.0,
        invert_fl: bool = False,
        invert_rl: bool = False,
        invert_fr: bool = False,
        invert_rr: bool = False,
        quench_ms: int = 18,
        debug: bool = False,
    ) -> None:
        """
        Initialize the Freenove mecanum board wrapper.

        Args:
            i2c_bus: PCA9685 I2C bus (Pi usually 1)
            addr: PCA9685 address (default 0x40)
            pwm_freq_hz: PWM frequency (Robot Savo proven baseline: 50 Hz)
            invert_fl/rl/fr/rr: per-wheel invert flags
            quench_ms: neutral delay (ms) on sign flip; 0 disables quench
            debug: enable debug logging

        Notes
        - This wrapper preserves Robot Savo's proven stop behavior for d==0 on a wheel:
          both channels = 4095
        """
        self.debug = bool(debug)

        self.pwm = create_from_config(
            PCA9685Config(
                bus=i2c_bus,
                address=addr,
                pwm_freq_hz=pwm_freq_hz,
                debug=debug,
                init_reset=True,
            )
        )

        # Per-wheel invert multipliers (+1 or -1)
        self.FL_INV = -1 if invert_fl else +1
        self.RL_INV = -1 if invert_rl else +1
        self.FR_INV = -1 if invert_fr else +1
        self.RR_INV = -1 if invert_rr else +1

        self.quench_ms = max(0, int(quench_ms))

        # Track previous sign for quench behavior: -1 / 0 / +1
        self._last_sign: Dict[str, int] = {
            "fl": 0,
            "rl": 0,
            "fr": 0,
            "rr": 0,
        }

        self._log(
            "Initialized FreenoveMecanumBoard("
            f"i2c_bus={i2c_bus}, addr=0x{addr:02X}, pwm_freq_hz={pwm_freq_hz}, "
            f"inv=({self.FL_INV},{self.RL_INV},{self.FR_INV},{self.RR_INV}), "
            f"quench_ms={self.quench_ms})"
        )

    # -------------------------------------------------------------------------
    # Logging helpers
    # -------------------------------------------------------------------------
    def _log(self, msg: str) -> None:
        if self.debug:
            print(f"[FreenoveMecanumBoard] {msg}")

    # -------------------------------------------------------------------------
    # Core wheel low-level writers (LOCKED real-hardware semantics)
    # -------------------------------------------------------------------------
    def _wheel_fl(self, d: int) -> None:
        """
        Front-left wheel writer (pair 0,1) — locked Robot Savo semantics.
        Positive -> ch0=0, ch1=d
        Negative -> ch1=0, ch0=-d
        Zero     -> ch0=4095, ch1=4095  (proven board stop behavior)
        """
        a, b = self.FL_PAIR
        if d > 0:
            self.pwm.set_motor_pwm(a, 0)
            self.pwm.set_motor_pwm(b, d)
        elif d < 0:
            self.pwm.set_motor_pwm(b, 0)
            self.pwm.set_motor_pwm(a, -d)
        else:
            self.pwm.set_motor_pwm(a, 4095)
            self.pwm.set_motor_pwm(b, 4095)

    def _wheel_rl(self, d: int) -> None:
        """
        Rear-left wheel writer (pair 3,2) — locked Robot Savo semantics.
        """
        a, b = self.RL_PAIR
        if d > 0:
            self.pwm.set_motor_pwm(a, 0)
            self.pwm.set_motor_pwm(b, d)
        elif d < 0:
            self.pwm.set_motor_pwm(b, 0)
            self.pwm.set_motor_pwm(a, -d)
        else:
            self.pwm.set_motor_pwm(a, 4095)
            self.pwm.set_motor_pwm(b, 4095)

    def _wheel_fr(self, d: int) -> None:
        """
        Front-right wheel writer (pair 6,7) — locked Robot Savo semantics.
        """
        a, b = self.FR_PAIR
        if d > 0:
            self.pwm.set_motor_pwm(a, 0)
            self.pwm.set_motor_pwm(b, d)
        elif d < 0:
            self.pwm.set_motor_pwm(b, 0)
            self.pwm.set_motor_pwm(a, -d)
        else:
            self.pwm.set_motor_pwm(a, 4095)
            self.pwm.set_motor_pwm(b, 4095)

    def _wheel_rr(self, d: int) -> None:
        """
        Rear-right wheel writer (pair 4,5) — locked Robot Savo semantics.
        """
        a, b = self.RR_PAIR
        if d > 0:
            self.pwm.set_motor_pwm(a, 0)
            self.pwm.set_motor_pwm(b, d)
        elif d < 0:
            self.pwm.set_motor_pwm(b, 0)
            self.pwm.set_motor_pwm(a, -d)
        else:
            self.pwm.set_motor_pwm(a, 4095)
            self.pwm.set_motor_pwm(b, 4095)

    # -------------------------------------------------------------------------
    # Clamp / sign / quench helpers
    # -------------------------------------------------------------------------
    @staticmethod
    def _sign(v: int) -> int:
        if v > 0:
            return +1
        if v < 0:
            return -1
        return 0

    @classmethod
    def _clamp_signed_pwm(cls, value: int) -> int:
        v = int(value)
        if v > cls.PWM_MAX:
            return cls.PWM_MAX
        if v < cls.PWM_MIN:
            return cls.PWM_MIN
        return v

    @classmethod
    def _clamp4(cls, d_fl: int, d_rl: int, d_fr: int, d_rr: int) -> Tuple[int, int, int, int]:
        return (
            cls._clamp_signed_pwm(d_fl),
            cls._clamp_signed_pwm(d_rl),
            cls._clamp_signed_pwm(d_fr),
            cls._clamp_signed_pwm(d_rr),
        )

    def _apply_quench(self, wheel_name: str, new_val: int, writer_fn) -> None:
        """
        Apply sign-flip quench before writing a new wheel command.

        Logic (locked to proven behavior)
        - If previous sign != new sign and both are non-zero:
            write neutral (0 => both channels 4095 in writer)
            sleep quench_ms
        - Then write the new value
        """
        prev_sign = self._last_sign[wheel_name]
        new_sign = self._sign(new_val)

        if self.quench_ms > 0 and prev_sign != 0 and new_sign != 0 and prev_sign != new_sign:
            self._log(
                f"Quench {wheel_name}: sign {prev_sign:+d} -> {new_sign:+d}, "
                f"{self.quench_ms} ms neutral"
            )
            writer_fn(0)
            time.sleep(self.quench_ms / 1000.0)

        writer_fn(new_val)
        self._last_sign[wheel_name] = new_sign

    # -------------------------------------------------------------------------
    # Public motor APIs
    # -------------------------------------------------------------------------
    def set_motor_model(self, d_fl: int, d_rl: int, d_fr: int, d_rr: int) -> None:
        """
        Set all four wheel commands (signed PWM counts).

        Args:
            d_fl, d_rl, d_fr, d_rr:
                Signed wheel PWM commands in range approx [-4095 .. +4095]
                (values are clamped if out of range).

        Processing order
        1) Apply per-wheel invert multipliers
        2) Clamp to signed PWM range
        3) Apply sign-flip quench per wheel
        4) Write to PCA9685 using locked wheel semantics
        """
        # Apply invert flags (proven teleop compatibility)
        d_fl_i = int(d_fl) * self.FL_INV
        d_rl_i = int(d_rl) * self.RL_INV
        d_fr_i = int(d_fr) * self.FR_INV
        d_rr_i = int(d_rr) * self.RR_INV

        # Clamp to safe signed range
        d_fl_i, d_rl_i, d_fr_i, d_rr_i = self._clamp4(d_fl_i, d_rl_i, d_fr_i, d_rr_i)

        self._log(
            f"set_motor_model raw=({d_fl},{d_rl},{d_fr},{d_rr}) "
            f"inv/clamped=({d_fl_i},{d_rl_i},{d_fr_i},{d_rr_i})"
        )

        # Apply per-wheel writes with quench
        self._apply_quench("fl", d_fl_i, self._wheel_fl)
        self._apply_quench("rl", d_rl_i, self._wheel_rl)
        self._apply_quench("fr", d_fr_i, self._wheel_fr)
        self._apply_quench("rr", d_rr_i, self._wheel_rr)

    def stop(self) -> None:
        """
        Stop all wheels using Robot Savo's board-proven stop behavior.

        This calls set_motor_model(0,0,0,0), which results in:
        - each wheel pair written as 4095/4095 via the wheel writers
        """
        self._log("stop()")
        self.set_motor_model(0, 0, 0, 0)

    def emergency_stop(self) -> None:
        """
        Alias for stop() for readability in safety/watchdog code.
        """
        self.stop()

    # -------------------------------------------------------------------------
    # Diagnostics / utility APIs
    # -------------------------------------------------------------------------
    def ping(self) -> bool:
        """
        Check underlying PCA9685 connectivity.
        """
        return self.pwm.ping()

    def get_locked_channel_map(self) -> dict:
        """
        Return the locked wheel-to-channel map for diagnostics/logging.
        """
        return {
            "FL": self.FL_PAIR,
            "RL": self.RL_PAIR,
            "FR": self.FR_PAIR,
            "RR": self.RR_PAIR,
        }

    def get_invert_multipliers(self) -> dict:
        """
        Return the active per-wheel invert multipliers.
        """
        return {
            "FL_INV": self.FL_INV,
            "RL_INV": self.RL_INV,
            "FR_INV": self.FR_INV,
            "RR_INV": self.RR_INV,
        }

    # -------------------------------------------------------------------------
    # Context manager / teardown
    # -------------------------------------------------------------------------
    def close(self) -> None:
        """
        Safe shutdown:
        - stop motors using board-proven stop behavior
        - close PCA9685 I2C handle
        """
        self._log("close() -> stop + pwm.close()")
        try:
            self.stop()
        finally:
            self.pwm.close()

    def __enter__(self) -> "FreenoveMecanumBoard":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


# =============================================================================
# Convenience factory
# =============================================================================
def create_board_from_config(cfg: FreenoveMecanumBoardConfig) -> FreenoveMecanumBoard:
    """
    Create a FreenoveMecanumBoard from a typed config object.
    """
    return FreenoveMecanumBoard(
        i2c_bus=cfg.pca9685.bus,
        addr=cfg.pca9685.address,
        pwm_freq_hz=cfg.pca9685.pwm_freq_hz,
        invert_fl=cfg.invert_fl,
        invert_rl=cfg.invert_rl,
        invert_fr=cfg.invert_fr,
        invert_rr=cfg.invert_rr,
        quench_ms=cfg.quench_ms,
        debug=cfg.pca9685.debug,
    )