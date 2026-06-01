#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/drivers/pca9685_driver.py
------------------------------------------------
Professional low-level PCA9685 I2C PWM driver for Robot Savo (ROS 2 Jazzy stack).

Purpose
- Provide a clean, reusable, hardware-focused PCA9685 driver
- Serve as the foundation for board-specific wrappers (e.g., Freenove mecanum board)
- Support real robot testing on Raspberry Pi (I2C bus + PCA9685 @ 0x40)

Important note (Robot Savo real hardware)
- This file is the *generic low-level PCA9685 driver*.
- Your proven physical board behavior (wheel channel pairs, sign semantics, quench, stop=4095/4095)
  belongs in `freenove_mecanum_board.py`, which should use this driver.
- We keep this module generic and stable so it can be reused safely.

Dependencies
- python3-smbus  (imported as `smbus`)
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import smbus


# =============================================================================
# Exceptions
# =============================================================================
class PCA9685Error(RuntimeError):
    """Base exception for PCA9685 driver errors."""


class PCA9685I2CError(PCA9685Error):
    """Raised when an I2C communication error occurs."""


class PCA9685ValueError(PCA9685Error):
    """Raised when an invalid argument is passed to the driver."""


# =============================================================================
# Config dataclass
# =============================================================================
@dataclass(frozen=True)
class PCA9685Config:
    """
    Typed config for creating a PCA9685 driver instance.

    Fields
    - bus: Raspberry Pi I2C bus number (usually 1)
    - address: PCA9685 I2C address (default 0x40 on Freenove board)
    - pwm_freq_hz: PWM frequency to apply after initialization
    - debug: enable debug logging
    - init_reset: write MODE1=0x00 on startup
    """
    bus: int = 1
    address: int = 0x40
    pwm_freq_hz: float = 50.0
    debug: bool = False
    init_reset: bool = True


# =============================================================================
# PCA9685 low-level driver
# =============================================================================
class PCA9685Driver:
    """
    Low-level PCA9685 PWM driver (I2C).

    Typical usage:
        pwm = PCA9685Driver(bus=1, address=0x40, debug=False)
        pwm.set_pwm_freq(50.0)
        pwm.set_motor_pwm(channel=0, duty=1200)
        ...
        pwm.close()

    Notes
    - Duty values are 12-bit counts: 0..4095
    - `set_motor_pwm()` uses ON=0, OFF=duty (simple motor-friendly PWM)
    - Board-specific wheel logic is intentionally NOT in this class
    """

    # -----------------------------
    # Registers (PCA9685)
    # -----------------------------
    _MODE1 = 0x00
    _MODE2 = 0x01
    _SUBADR1 = 0x02
    _SUBADR2 = 0x03
    _SUBADR3 = 0x04
    _PRESCALE = 0xFE

    _LED0_ON_L = 0x06
    _LED0_ON_H = 0x07
    _LED0_OFF_L = 0x08
    _LED0_OFF_H = 0x09

    _ALLLED_ON_L = 0xFA
    _ALLLED_ON_H = 0xFB
    _ALLLED_OFF_L = 0xFC
    _ALLLED_OFF_H = 0xFD

    # -----------------------------
    # Constants
    # -----------------------------
    _OSC_HZ = 25_000_000.0
    _PWM_STEPS = 4096
    _PWM_MIN = 0
    _PWM_MAX = 4095
    _CH_MIN = 0
    _CH_MAX = 15
    _PRESCALE_MIN = 3
    _PRESCALE_MAX = 255

    def __init__(
        self,
        bus: int = 1,
        address: int = 0x40,
        debug: bool = False,
        init_reset: bool = True,
    ) -> None:
        """
        Initialize the PCA9685 I2C driver.

        Args:
            bus: I2C bus number (Pi usually uses 1)
            address: PCA9685 I2C address (default 0x40)
            debug: print debug logs if True
            init_reset: if True, writes MODE1=0x00 at startup

        Raises:
            PCA9685ValueError: invalid bus/address
            PCA9685I2CError: cannot open I2C bus or communicate
        """
        # Validate constructor args
        if int(bus) < 0:
            raise PCA9685ValueError(f"Invalid I2C bus: {bus}")
        addr_i = int(address)
        if not (0x03 <= addr_i <= 0x77):
            raise PCA9685ValueError(f"Invalid I2C address: 0x{addr_i:02X}")

        self.busno = int(bus)
        self.address = addr_i
        self.debug = bool(debug)
        self._closed = False
        self._current_pwm_freq_hz: Optional[float] = None

        try:
            self.bus = smbus.SMBus(self.busno)
        except Exception as e:
            raise PCA9685I2CError(
                f"Failed to open I2C bus {self.busno} for PCA9685 @ 0x{self.address:02X}: {e}"
            ) from e

        # Optional wake/reset to known state
        if init_reset:
            self.write(self._MODE1, 0x00)

        self._log(
            f"Initialized bus={self.busno}, addr=0x{self.address:02X}, init_reset={init_reset}"
        )

    # -------------------------------------------------------------------------
    # Internal helpers
    # -------------------------------------------------------------------------
    def _log(self, msg: str) -> None:
        if self.debug:
            print(f"[PCA9685] {msg}")

    def _ensure_open(self) -> None:
        if self._closed:
            raise PCA9685Error("PCA9685Driver is closed")

    @staticmethod
    def _u8(v: int) -> int:
        return int(v) & 0xFF

    @classmethod
    def _validate_channel(cls, channel: int) -> int:
        ch = int(channel)
        if not (cls._CH_MIN <= ch <= cls._CH_MAX):
            raise PCA9685ValueError(
                f"Invalid channel {channel}; expected {cls._CH_MIN}..{cls._CH_MAX}"
            )
        return ch

    @classmethod
    def _validate_count(cls, value: int, name: str) -> int:
        v = int(value)
        if not (cls._PWM_MIN <= v <= cls._PWM_MAX):
            raise PCA9685ValueError(
                f"Invalid {name}={value}; expected {cls._PWM_MIN}..{cls._PWM_MAX}"
            )
        return v

    # -------------------------------------------------------------------------
    # Raw I2C register access
    # -------------------------------------------------------------------------
    def write(self, reg: int, value: int) -> None:
        """
        Write a single byte to a PCA9685 register.
        """
        self._ensure_open()
        reg_i = int(reg)
        val_i = self._u8(value)
        try:
            self.bus.write_byte_data(self.address, reg_i, val_i)
        except Exception as e:
            raise PCA9685I2CError(
                f"I2C write failed @0x{self.address:02X} reg=0x{reg_i:02X} val=0x{val_i:02X}: {e}"
            ) from e

    def read(self, reg: int) -> int:
        """
        Read a single byte from a PCA9685 register.
        """
        self._ensure_open()
        reg_i = int(reg)
        try:
            return int(self.bus.read_byte_data(self.address, reg_i))
        except Exception as e:
            raise PCA9685I2CError(
                f"I2C read failed @0x{self.address:02X} reg=0x{reg_i:02X}: {e}"
            ) from e

    # -------------------------------------------------------------------------
    # PCA9685 configuration
    # -------------------------------------------------------------------------
    def reset_mode1(self) -> None:
        """
        Reset MODE1 to 0x00 (wake/default mode).
        """
        self._ensure_open()
        self.write(self._MODE1, 0x00)
        self._log("MODE1 reset to 0x00")

    def set_pwm_freq(self, freq_hz: float) -> None:
        """
        Set PWM frequency in Hz.

        Typical Robot Savo baseline for motor board:
        - 50 Hz (proven working with your Freenove/PCA9685 setup)

        Args:
            freq_hz: desired PWM frequency in Hz (> 0)

        Raises:
            PCA9685ValueError: invalid frequency
            PCA9685I2CError: I2C communication failure
        """
        self._ensure_open()

        if not isinstance(freq_hz, (int, float)):
            raise PCA9685ValueError(f"Invalid pwm frequency type: {type(freq_hz)}")
        if float(freq_hz) <= 0.0:
            raise PCA9685ValueError(f"Invalid pwm frequency: {freq_hz}")

        # Datasheet formula:
        # prescale = round(osc_clock / (4096 * freq)) - 1
        prescale_val = (self._OSC_HZ / (self._PWM_STEPS * float(freq_hz))) - 1.0
        prescale = int(math.floor(prescale_val + 0.5))

        # Clamp to valid range
        if prescale < self._PRESCALE_MIN:
            prescale = self._PRESCALE_MIN
        if prescale > self._PRESCALE_MAX:
            prescale = self._PRESCALE_MAX

        old_mode = self.read(self._MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10  # set SLEEP bit

        self._log(
            f"set_pwm_freq({freq_hz:.3f} Hz) -> prescale={prescale}, old_mode=0x{old_mode:02X}"
        )

        # Required sequence: sleep -> set prescale -> wake -> restart
        self.write(self._MODE1, sleep_mode)
        self.write(self._PRESCALE, prescale)
        self.write(self._MODE1, old_mode)
        time.sleep(0.005)  # oscillator stabilization
        self.write(self._MODE1, old_mode | 0x80)  # RESTART bit

        self._current_pwm_freq_hz = float(freq_hz)

    @property
    def current_pwm_freq_hz(self) -> Optional[float]:
        """
        Last frequency value passed to set_pwm_freq(), if known.
        """
        return self._current_pwm_freq_hz

    # -------------------------------------------------------------------------
    # PWM output APIs
    # -------------------------------------------------------------------------
    def set_pwm(self, channel: int, on: int, off: int) -> None:
        """
        Set raw PWM ON/OFF counts for one channel.

        Args:
            channel: PCA9685 channel index (0..15)
            on: ON count  (0..4095)
            off: OFF count (0..4095)

        Raises:
            PCA9685ValueError / PCA9685I2CError
        """
        self._ensure_open()
        ch = self._validate_channel(channel)
        on_i = self._validate_count(on, "on")
        off_i = self._validate_count(off, "off")

        base = self._LED0_ON_L + (4 * ch)

        self.write(base + 0, on_i & 0xFF)
        self.write(base + 1, (on_i >> 8) & 0x0F)
        self.write(base + 2, off_i & 0xFF)
        self.write(base + 3, (off_i >> 8) & 0x0F)

    def set_motor_pwm(self, channel: int, duty: int) -> None:
        """
        Set motor-friendly PWM duty for one channel using ON=0, OFF=duty.

        Args:
            channel: PCA9685 channel index (0..15)
            duty: duty count (clamped to 0..4095)

        Notes
        - This is the main API used by higher-level motor board wrappers.
        - Board/wheel direction semantics are *not* handled here.
        """
        self._ensure_open()
        ch = self._validate_channel(channel)

        d = int(duty)
        if d < self._PWM_MIN:
            d = self._PWM_MIN
        elif d > self._PWM_MAX:
            d = self._PWM_MAX

        self.set_pwm(ch, 0, d)

    def set_all_pwm(self, on: int, off: int) -> None:
        """
        Set raw PWM ON/OFF counts for all channels.
        """
        self._ensure_open()
        on_i = self._validate_count(on, "on")
        off_i = self._validate_count(off, "off")

        self.write(self._ALLLED_ON_L, on_i & 0xFF)
        self.write(self._ALLLED_ON_H, (on_i >> 8) & 0x0F)
        self.write(self._ALLLED_OFF_L, off_i & 0xFF)
        self.write(self._ALLLED_OFF_H, (off_i >> 8) & 0x0F)

    def set_all_motor_pwm(self, duty: int) -> None:
        """
        Set the same duty (0..4095) on all channels using ON=0, OFF=duty.
        """
        self._ensure_open()
        d = int(duty)
        if d < self._PWM_MIN:
            d = self._PWM_MIN
        elif d > self._PWM_MAX:
            d = self._PWM_MAX
        self.set_all_pwm(0, d)

    def stop_all_channels(self) -> None:
        """
        Set all channels to duty=0 (PWM OFF).

        Note
        - This is a generic PCA9685-level stop (all PWM off).
        - Robot Savo wheel-pair stop behavior (both channels=4095 per wheel pair)
          should be implemented in the board wrapper.
        """
        self._log("stop_all_channels() -> all channels duty=0")
        self.set_all_pwm(0, 0)

    # -------------------------------------------------------------------------
    # Diagnostics helpers
    # -------------------------------------------------------------------------
    def ping(self) -> bool:
        """
        Basic connectivity check: tries to read MODE1.
        Returns True if read succeeds, False otherwise.
        """
        try:
            _ = self.read(self._MODE1)
            return True
        except PCA9685Error:
            return False

    def read_mode1(self) -> int:
        """Read MODE1 register."""
        return self.read(self._MODE1)

    def read_prescale(self) -> int:
        """Read PRESCALE register."""
        return self.read(self._PRESCALE)

    # -------------------------------------------------------------------------
    # Teardown / context manager
    # -------------------------------------------------------------------------
    def close(self) -> None:
        """
        Close the I2C bus handle. Safe to call multiple times.
        """
        if self._closed:
            return
        try:
            self.bus.close()
            self._log("I2C bus closed")
        except Exception as e:
            raise PCA9685I2CError(
                f"Failed to close I2C bus {self.busno} for PCA9685 @ 0x{self.address:02X}: {e}"
            ) from e
        finally:
            self._closed = True

    def __enter__(self) -> "PCA9685Driver":
        self._ensure_open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


# =============================================================================
# Convenience factory
# =============================================================================
def create_from_config(cfg: PCA9685Config) -> PCA9685Driver:
    """
    Create and initialize a PCA9685Driver from a typed config.

    Applies:
    - constructor settings
    - set_pwm_freq(cfg.pwm_freq_hz)

    Returns:
        PCA9685Driver (initialized and frequency configured)
    """
    driver = PCA9685Driver(
        bus=cfg.bus,
        address=cfg.address,
        debug=cfg.debug,
        init_reset=cfg.init_reset,
    )
    driver.set_pwm_freq(cfg.pwm_freq_hz)
    return driver