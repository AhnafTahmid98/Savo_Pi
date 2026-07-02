# -*- coding: utf-8 -*-

"""Python fallback PCA9685 driver for Robot Savo head diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

from savo_head.constants import (
    I2C_BUS_DEFAULT,
    PCA9685_ADDR_DEFAULT,
    PCA9685_PWM_FREQ_HZ_DEFAULT,
    PCA9685_PULSE_MAX_US,
    PCA9685_PULSE_MIN_US,
    PCA9685_SERVO_PERIOD_US,
    PCA9685_TICKS_PER_CYCLE,
)
from savo_head.core.calibration import clamp_pulse_us


MODE1_REG = 0x00
PRESCALE_REG = 0xFE
LED0_ON_L_REG = 0x06
LED0_ON_H_REG = 0x07
LED0_OFF_L_REG = 0x08
LED0_OFF_H_REG = 0x09

MODE1_SLEEP = 0x10
MODE1_RESTART = 0x80

OSCILLATOR_HZ = 25_000_000.0


@dataclass(frozen=True)
class Pca9685Config:
    i2c_bus: int = I2C_BUS_DEFAULT
    address: int = PCA9685_ADDR_DEFAULT
    pwm_frequency_hz: float = PCA9685_PWM_FREQ_HZ_DEFAULT
    dryrun: bool = False

    def normalized(self) -> "Pca9685Config":
        freq = float(self.pwm_frequency_hz)
        if freq <= 0.0:
            freq = PCA9685_PWM_FREQ_HZ_DEFAULT

        return Pca9685Config(
            i2c_bus=int(self.i2c_bus),
            address=int(self.address),
            pwm_frequency_hz=freq,
            dryrun=bool(self.dryrun),
        )


class Pca9685Driver:
    def __init__(self, config: Pca9685Config | None = None, bus: Optional[Any] = None):
        self.config = (config or Pca9685Config()).normalized()
        self._external_bus = bus is not None
        self._bus = bus
        self._opened = False
        self._writes: list[tuple[int, int]] = []

    @property
    def opened(self) -> bool:
        return self._opened

    @property
    def writes(self) -> tuple[tuple[int, int], ...]:
        return tuple(self._writes)

    def open(self) -> None:
        if self._opened:
            return

        if self.config.dryrun:
            self._opened = True
            return

        if self._bus is None:
            try:
                import smbus  # type: ignore
            except ImportError as exc:
                raise RuntimeError("python3-smbus is required for PCA9685 hardware access") from exc

            self._bus = smbus.SMBus(self.config.i2c_bus)

        self.write_byte(MODE1_REG, 0x00)
        self.set_pwm_frequency(self.config.pwm_frequency_hz)
        self._opened = True

    def close(self) -> None:
        if self._bus is not None and not self._external_bus:
            close = getattr(self._bus, "close", None)
            if callable(close):
                close()

        self._bus = None if not self._external_bus else self._bus
        self._opened = False

    def ensure_open(self) -> None:
        if not self._opened:
            self.open()

    def write_byte(self, register: int, value: int) -> None:
        reg = int(register) & 0xFF
        val = int(value) & 0xFF
        self._writes.append((reg, val))

        if self.config.dryrun:
            return

        if self._bus is None:
            raise RuntimeError("PCA9685 bus is not open")

        self._bus.write_byte_data(self.config.address, reg, val)

    def read_byte(self, register: int) -> int:
        reg = int(register) & 0xFF

        if self.config.dryrun:
            return 0x00

        if self._bus is None:
            raise RuntimeError("PCA9685 bus is not open")

        return int(self._bus.read_byte_data(self.config.address, reg))

    def set_pwm_frequency(self, frequency_hz: float) -> int:
        freq = float(frequency_hz)
        if freq <= 0.0:
            raise ValueError("frequency_hz must be positive")

        prescale_value = OSCILLATOR_HZ
        prescale_value /= float(PCA9685_TICKS_PER_CYCLE)
        prescale_value /= freq
        prescale_value -= 1.0
        prescale = int(prescale_value + 0.5)

        old_mode = self.read_byte(MODE1_REG)
        sleep_mode = (old_mode & 0x7F) | MODE1_SLEEP

        self.write_byte(MODE1_REG, sleep_mode)
        self.write_byte(PRESCALE_REG, prescale)
        self.write_byte(MODE1_REG, old_mode)
        self.write_byte(MODE1_REG, old_mode | MODE1_RESTART)

        return prescale

    def set_pwm(self, channel: int, on_ticks: int, off_ticks: int) -> None:
        ch = int(channel)
        if not 0 <= ch <= 15:
            raise ValueError(f"PCA9685 channel must be 0..15, got {channel!r}")

        on = max(0, min(PCA9685_TICKS_PER_CYCLE - 1, int(on_ticks)))
        off = max(0, min(PCA9685_TICKS_PER_CYCLE - 1, int(off_ticks)))

        base = LED0_ON_L_REG + 4 * ch

        self.write_byte(base, on & 0xFF)
        self.write_byte(base + 1, on >> 8)
        self.write_byte(base + 2, off & 0xFF)
        self.write_byte(base + 3, off >> 8)

    def set_servo_pulse_us(self, channel: int, pulse_us: float) -> int:
        pulse = clamp_pulse_us(float(pulse_us))
        ticks = int(pulse * float(PCA9685_TICKS_PER_CYCLE) / float(PCA9685_SERVO_PERIOD_US))
        self.set_pwm(channel, 0, ticks)
        return ticks

    def stop_channel(self, channel: int) -> None:
        self.set_pwm(channel, 0, 0)

    def stop_all(self) -> None:
        for channel in range(16):
            self.stop_channel(channel)


def pulse_us_to_ticks(pulse_us: float) -> int:
    pulse = clamp_pulse_us(float(pulse_us))
    return int(pulse * float(PCA9685_TICKS_PER_CYCLE) / float(PCA9685_SERVO_PERIOD_US))


def validate_servo_pulse_us(pulse_us: float) -> bool:
    value = float(pulse_us)
    return PCA9685_PULSE_MIN_US <= value <= PCA9685_PULSE_MAX_US


__all__ = [
    "MODE1_REG",
    "PRESCALE_REG",
    "LED0_ON_L_REG",
    "LED0_ON_H_REG",
    "LED0_OFF_L_REG",
    "LED0_OFF_H_REG",
    "Pca9685Config",
    "Pca9685Driver",
    "pulse_us_to_ticks",
    "validate_servo_pulse_us",
]
