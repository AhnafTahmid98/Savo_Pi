# -*- coding: utf-8 -*-

"""Python fallback hardware drivers for Robot Savo head diagnostics."""

from __future__ import annotations

from savo_head.drivers.pca9685_driver import (
    LED0_OFF_H_REG,
    LED0_OFF_L_REG,
    LED0_ON_H_REG,
    LED0_ON_L_REG,
    MODE1_REG,
    PRESCALE_REG,
    Pca9685Config,
    Pca9685Driver,
    pulse_us_to_ticks,
    validate_servo_pulse_us,
)
from savo_head.drivers.pantilt_driver import (
    BACKEND_DRYRUN,
    BACKEND_PCA9685,
    PanTiltDriver,
    PanTiltDriverConfig,
    ServoOutput,
    make_dryrun_pantilt_driver,
    make_pca9685_pantilt_driver,
)

__all__ = [
    # PCA9685
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
    # Pan-tilt
    "BACKEND_PCA9685",
    "BACKEND_DRYRUN",
    "ServoOutput",
    "PanTiltDriverConfig",
    "PanTiltDriver",
    "make_dryrun_pantilt_driver",
    "make_pca9685_pantilt_driver",
]
