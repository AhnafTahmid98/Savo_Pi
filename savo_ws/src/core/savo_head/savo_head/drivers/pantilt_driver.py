# -*- coding: utf-8 -*-

"""Python fallback pan-tilt driver for Robot Savo head diagnostics."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from savo_head.constants import (
    HEAD_BACKEND_DEFAULT,
    I2C_BUS_DEFAULT,
    PCA9685_ADDR_DEFAULT,
    PCA9685_PWM_FREQ_HZ_DEFAULT,
    STATUS_DRYRUN,
    STATUS_OK,
)
from savo_head.core.calibration import (
    DEFAULT_SERVO_CALIBRATION,
    HeadServoCalibration,
    ServoChannelCalibration,
    angle_to_pulse_us,
    calibration_for_axis,
)
from savo_head.drivers.pca9685_driver import Pca9685Config, Pca9685Driver
from savo_head.models.head_state import (
    MODE_CENTERING,
    MODE_IDLE,
    MODE_MANUAL,
    PanTiltLimits,
    PanTiltState,
)
from savo_head.models.pantilt_command import (
    COMMAND_STOP,
    PanTiltCommand,
    center_command,
    hold_command,
)


BACKEND_PCA9685 = "pca9685"
BACKEND_DRYRUN = "dryrun"


@dataclass(frozen=True)
class ServoOutput:
    axis: str
    logical_channel: str
    pca9685_channel: int
    angle_deg: int
    pulse_us: float
    ticks: int

    def to_dict(self) -> dict:
        return {
            "axis": self.axis,
            "logical_channel": self.logical_channel,
            "pca9685_channel": self.pca9685_channel,
            "angle_deg": self.angle_deg,
            "pulse_us": self.pulse_us,
            "ticks": self.ticks,
        }


@dataclass(frozen=True)
class PanTiltDriverConfig:
    backend: str = HEAD_BACKEND_DEFAULT
    i2c_bus: int = I2C_BUS_DEFAULT
    pca9685_address: int = PCA9685_ADDR_DEFAULT
    pwm_frequency_hz: float = PCA9685_PWM_FREQ_HZ_DEFAULT
    calibration: HeadServoCalibration = field(default_factory=lambda: DEFAULT_SERVO_CALIBRATION)
    center_on_open: bool = False
    center_on_close: bool = False

    def normalized(self) -> "PanTiltDriverConfig":
        backend = str(self.backend).strip().lower()
        if backend not in (BACKEND_PCA9685, BACKEND_DRYRUN):
            backend = BACKEND_DRYRUN

        return PanTiltDriverConfig(
            backend=backend,
            i2c_bus=int(self.i2c_bus),
            pca9685_address=int(self.pca9685_address),
            pwm_frequency_hz=float(self.pwm_frequency_hz),
            calibration=self.calibration.normalized(),
            center_on_open=bool(self.center_on_open),
            center_on_close=bool(self.center_on_close),
        )

    def dryrun(self) -> bool:
        return self.normalized().backend == BACKEND_DRYRUN


class PanTiltDriver:
    def __init__(
        self,
        config: PanTiltDriverConfig | None = None,
        pca_driver: Optional[Pca9685Driver] = None,
    ):
        self.config = (config or PanTiltDriverConfig()).normalized()
        self.calibration = self.config.calibration.normalized()

        self._pca_driver = pca_driver or Pca9685Driver(
            Pca9685Config(
                i2c_bus=self.config.i2c_bus,
                address=self.config.pca9685_address,
                pwm_frequency_hz=self.config.pwm_frequency_hz,
                dryrun=self.config.dryrun(),
            )
        )

        self._opened = False
        self._state = PanTiltState(
            pan_deg=self.calibration.pan.center_deg,
            tilt_deg=self.calibration.tilt.center_deg,
            mode=MODE_IDLE,
            status=STATUS_DRYRUN if self.config.dryrun() else STATUS_OK,
            source="init",
        )
        self._last_outputs: list[ServoOutput] = []

    @property
    def opened(self) -> bool:
        return self._opened

    @property
    def state(self) -> PanTiltState:
        return self._state

    @property
    def last_outputs(self) -> tuple[ServoOutput, ...]:
        return tuple(self._last_outputs)

    def open(self) -> None:
        if self._opened:
            return

        self._pca_driver.open()
        self._opened = True

        if self.config.center_on_open:
            self.center(source="open")

    def close(self) -> None:
        if self._opened and self.config.center_on_close:
            self.center(source="close")

        self._pca_driver.close()
        self._opened = False

    def ensure_open(self) -> None:
        if not self._opened:
            self.open()

    def limits(self) -> PanTiltLimits:
        return PanTiltLimits(
            pan_min_deg=self.calibration.pan.min_deg,
            pan_center_deg=self.calibration.pan.center_deg,
            pan_max_deg=self.calibration.pan.max_deg,
            tilt_min_deg=self.calibration.tilt.min_deg,
            tilt_center_deg=self.calibration.tilt.center_deg,
            tilt_max_deg=self.calibration.tilt.max_deg,
        )

    def _servo_output(self, axis: str, angle_deg: int) -> ServoOutput:
        item = calibration_for_axis(axis, self.calibration)
        angle = item.clamp_angle(angle_deg)
        pulse_us = angle_to_pulse_us(
            angle,
            error_deg=item.error_deg,
            direction=item.direction,
        )
        ticks = self._pca_driver.set_servo_pulse_us(item.pca9685_channel, pulse_us)

        return ServoOutput(
            axis=axis,
            logical_channel=item.logical_channel,
            pca9685_channel=item.pca9685_channel,
            angle_deg=angle,
            pulse_us=pulse_us,
            ticks=ticks,
        )

    def set_axis(
        self,
        axis: str,
        angle_deg: int,
        *,
        mode: str = MODE_MANUAL,
        source: str = "direct",
        stamp_s: float = 0.0,
    ) -> ServoOutput:
        self.ensure_open()

        axis_norm = str(axis).strip().lower()
        output = self._servo_output(axis_norm, angle_deg)

        if axis_norm == "pan":
            self._state = PanTiltState(
                pan_deg=output.angle_deg,
                tilt_deg=self._state.tilt_deg,
                mode=mode,
                status=STATUS_DRYRUN if self.config.dryrun() else STATUS_OK,
                stamp_s=float(stamp_s),
                source=source,
            )
        elif axis_norm == "tilt":
            self._state = PanTiltState(
                pan_deg=self._state.pan_deg,
                tilt_deg=output.angle_deg,
                mode=mode,
                status=STATUS_DRYRUN if self.config.dryrun() else STATUS_OK,
                stamp_s=float(stamp_s),
                source=source,
            )
        else:
            raise ValueError(f"unknown head axis: {axis!r}")

        self._last_outputs = [output]
        return output

    def set_pan_tilt(
        self,
        pan_deg: int,
        tilt_deg: int,
        *,
        mode: str = MODE_MANUAL,
        source: str = "direct",
        stamp_s: float = 0.0,
    ) -> PanTiltState:
        self.ensure_open()

        limits = self.limits()
        pan = limits.clamp_pan(pan_deg)
        tilt = limits.clamp_tilt(tilt_deg)

        pan_output = self._servo_output("pan", pan)
        tilt_output = self._servo_output("tilt", tilt)

        self._last_outputs = [pan_output, tilt_output]
        self._state = PanTiltState(
            pan_deg=pan_output.angle_deg,
            tilt_deg=tilt_output.angle_deg,
            mode=mode,
            status=STATUS_DRYRUN if self.config.dryrun() else STATUS_OK,
            stamp_s=float(stamp_s),
            source=source,
        )

        return self._state

    def center(self, *, source: str = "center", stamp_s: float = 0.0) -> PanTiltState:
        return self.set_pan_tilt(
            self.calibration.pan.center_deg,
            self.calibration.tilt.center_deg,
            mode=MODE_CENTERING,
            source=source,
            stamp_s=stamp_s,
        )

    def hold(self, *, source: str = "hold", stamp_s: float = 0.0) -> PanTiltState:
        return PanTiltState(
            pan_deg=self._state.pan_deg,
            tilt_deg=self._state.tilt_deg,
            mode=self._state.mode,
            status=self._state.status,
            stamp_s=float(stamp_s),
            source=source,
        )

    def stop(self, *, source: str = "stop", stamp_s: float = 0.0) -> PanTiltState:
        self._state = PanTiltState(
            pan_deg=self._state.pan_deg,
            tilt_deg=self._state.tilt_deg,
            mode=MODE_IDLE,
            status=self._state.status,
            stamp_s=float(stamp_s),
            source=source,
        )
        return self._state

    def apply_command(self, command: PanTiltCommand) -> PanTiltState:
        cmd = command.normalized(self.limits())

        if cmd.command_type == COMMAND_STOP:
            return self.stop(source=cmd.source, stamp_s=cmd.stamp_s)

        if cmd == hold_command(source=cmd.source, stamp_s=cmd.stamp_s, reason=cmd.reason):
            return self.hold(source=cmd.source, stamp_s=cmd.stamp_s)

        target = cmd.target_from_state(self._state, self.limits())

        return self.set_pan_tilt(
            target.pan_deg,
            target.tilt_deg,
            mode=target.mode,
            source=cmd.source,
            stamp_s=cmd.stamp_s,
        )

    def apply_center_command(self, *, stamp_s: float = 0.0) -> PanTiltState:
        return self.apply_command(center_command(stamp_s=stamp_s))

    def status_dict(self) -> dict:
        return {
            "opened": self.opened,
            "backend": self.config.backend,
            "dryrun": self.config.dryrun(),
            "state": self.state.to_dict(),
            "last_outputs": [item.to_dict() for item in self.last_outputs],
        }


def make_dryrun_pantilt_driver(center_on_open: bool = False) -> PanTiltDriver:
    return PanTiltDriver(
        PanTiltDriverConfig(
            backend=BACKEND_DRYRUN,
            center_on_open=center_on_open,
            center_on_close=False,
        )
    )


def make_pca9685_pantilt_driver(center_on_open: bool = True, center_on_close: bool = True) -> PanTiltDriver:
    return PanTiltDriver(
        PanTiltDriverConfig(
            backend=BACKEND_PCA9685,
            center_on_open=center_on_open,
            center_on_close=center_on_close,
        )
    )


__all__ = [
    "BACKEND_PCA9685",
    "BACKEND_DRYRUN",
    "ServoOutput",
    "PanTiltDriverConfig",
    "PanTiltDriver",
    "make_dryrun_pantilt_driver",
    "make_pca9685_pantilt_driver",
]
