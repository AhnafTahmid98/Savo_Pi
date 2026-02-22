#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/drivers/board_exceptions.py
--------------------------------------------------
Professional driver/board exception hierarchy for Robot Savo (ROS 2 Jazzy).

Purpose
- Provide a clear, reusable exception hierarchy across `savo_base/drivers`
- Standardize error handling for:
    * PCA9685 driver
    * Freenove mecanum board wrapper
    * future board / motor / GPIO driver modules
- Improve debugging on real hardware by attaching structured context to failures

Design notes
- No ROS dependencies
- Safe to import from any CLI tool, test script, or ROS node
- Exceptions support optional metadata for better logs / diagnostics
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional


# =============================================================================
# Base exception + structured context
# =============================================================================
@dataclass(frozen=True)
class BoardErrorContext:
    """
    Optional structured context attached to driver exceptions.

    Common fields (examples):
    - driver="PCA9685"
    - address=0x40
    - bus=1
    - channel=7
    - wheel="FR"
    - operation="set_pwm"
    - value=3000
    """
    driver: Optional[str] = None
    operation: Optional[str] = None
    bus: Optional[int] = None
    address: Optional[int] = None
    channel: Optional[int] = None
    wheel: Optional[str] = None
    register: Optional[int] = None
    value: Optional[int | float | str] = None
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert context to a compact dict, excluding None values.
        """
        out: Dict[str, Any] = {}
        if self.driver is not None:
            out["driver"] = self.driver
        if self.operation is not None:
            out["operation"] = self.operation
        if self.bus is not None:
            out["bus"] = self.bus
        if self.address is not None:
            out["address"] = self.address
        if self.channel is not None:
            out["channel"] = self.channel
        if self.wheel is not None:
            out["wheel"] = self.wheel
        if self.register is not None:
            out["register"] = self.register
        if self.value is not None:
            out["value"] = self.value
        if self.extra:
            out["extra"] = dict(self.extra)
        return out

    def format_compact(self) -> str:
        """
        Human-readable compact formatting for logs.
        """
        parts = []
        if self.driver is not None:
            parts.append(f"driver={self.driver}")
        if self.operation is not None:
            parts.append(f"op={self.operation}")
        if self.bus is not None:
            parts.append(f"bus={self.bus}")
        if self.address is not None:
            parts.append(f"addr=0x{int(self.address):02X}")
        if self.channel is not None:
            parts.append(f"ch={self.channel}")
        if self.wheel is not None:
            parts.append(f"wheel={self.wheel}")
        if self.register is not None:
            parts.append(f"reg=0x{int(self.register):02X}")
        if self.value is not None:
            parts.append(f"value={self.value}")
        if self.extra:
            parts.append(f"extra={self.extra}")
        return ", ".join(parts)


class BoardException(RuntimeError):
    """
    Base exception for all Robot Savo board/driver failures.

    Supports optional structured context and exception chaining.
    """

    def __init__(
        self,
        message: str,
        *,
        context: Optional[BoardErrorContext] = None,
        cause: Optional[BaseException] = None,
    ) -> None:
        self.message = str(message)
        self.context = context
        self.cause = cause
        super().__init__(self.__str__())

        # Keep explicit chaining where helpful
        if cause is not None:
            self.__cause__ = cause

    def __str__(self) -> str:
        if self.context is None:
            return self.message
        ctx = self.context.format_compact()
        if not ctx:
            return self.message
        return f"{self.message} [{ctx}]"

    def to_dict(self) -> Dict[str, Any]:
        """
        Structured representation suitable for logs/JSON diagnostics.
        """
        out: Dict[str, Any] = {
            "type": self.__class__.__name__,
            "message": self.message,
        }
        if self.context is not None:
            out["context"] = self.context.to_dict()
        if self.cause is not None:
            out["cause_type"] = self.cause.__class__.__name__
            out["cause_message"] = str(self.cause)
        return out


# =============================================================================
# Configuration / validation errors
# =============================================================================
class BoardConfigError(BoardException):
    """
    Invalid board/driver configuration (bus, address, mapping, limits, etc.).
    """


class BoardValidationError(BoardException):
    """
    Invalid runtime input passed to a driver method (channel, duty, frequency, etc.).
    """


class UnsupportedBoardOperationError(BoardException):
    """
    Requested operation is not supported by this board/driver implementation.
    """


# =============================================================================
# Connection / I2C / hardware communication errors
# =============================================================================
class BoardConnectionError(BoardException):
    """
    Base class for board connection failures.
    """


class I2CBusOpenError(BoardConnectionError):
    """
    Failed to open I2C/SMBus device (e.g., /dev/i2c-1 unavailable).
    """


class I2CCommunicationError(BoardConnectionError):
    """
    Low-level I2C transaction failed (read/write error, remote I/O error, etc.).
    """


class I2CDeviceNotFoundError(BoardConnectionError):
    """
    Expected I2C device address not responding on the selected bus.
    """


class DevicePermissionError(BoardConnectionError):
    """
    Permission denied when accessing hardware interface (I2C/GPIO/etc.).
    """


class DeviceBusyError(BoardConnectionError):
    """
    Hardware device/interface is busy or locked by another process.
    """


# =============================================================================
# PCA9685-specific errors
# =============================================================================
class PCA9685Error(BoardException):
    """
    Base class for PCA9685-specific failures.
    """


class PCA9685InitError(PCA9685Error):
    """
    Failed to initialize the PCA9685 controller.
    """


class PCA9685RegisterError(PCA9685Error):
    """
    PCA9685 register read/write failure.
    """


class PCA9685FrequencyError(PCA9685Error):
    """
    Invalid or failed PWM frequency configuration.
    """


class PCA9685ChannelError(PCA9685Error):
    """
    Invalid PCA9685 channel index or channel operation failure.
    """


class PCA9685DutyCycleError(PCA9685Error):
    """
    Invalid duty cycle or duty cycle write failure.
    """


# =============================================================================
# Motor board / wheel mapping errors
# =============================================================================
class MotorBoardError(BoardException):
    """
    Base class for motor-board wrapper failures (e.g., Freenove mecanum wrapper).
    """


class WheelMappingError(MotorBoardError):
    """
    Invalid or inconsistent wheel-to-channel mapping.
    """


class WheelCommandError(MotorBoardError):
    """
    Invalid wheel command or failure while applying wheel command.
    """


class MotorSafetyError(MotorBoardError):
    """
    Safety-related motor control failure (quench, emergency stop, invalid transition).
    """


class EmergencyStopActiveError(MotorSafetyError):
    """
    Motion command rejected because emergency stop is active.
    """


# =============================================================================
# Generic helpers for wrapping lower-level exceptions
# =============================================================================
def wrap_i2c_error(
    exc: BaseException,
    *,
    message: str,
    driver: str = "I2C",
    operation: str | None = None,
    bus: int | None = None,
    address: int | None = None,
    channel: int | None = None,
    register: int | None = None,
    value: int | float | str | None = None,
    wheel: str | None = None,
    extra: Optional[Dict[str, Any]] = None,
) -> I2CCommunicationError:
    """
    Convenience helper to wrap a low-level exception into an I2CCommunicationError.
    """
    ctx = BoardErrorContext(
        driver=driver,
        operation=operation,
        bus=bus,
        address=address,
        channel=channel,
        wheel=wheel,
        register=register,
        value=value,
        extra=dict(extra or {}),
    )
    return I2CCommunicationError(message, context=ctx, cause=exc)


def wrap_pca9685_error(
    exc: BaseException,
    *,
    message: str,
    operation: str | None = None,
    bus: int | None = None,
    address: int | None = None,
    channel: int | None = None,
    register: int | None = None,
    value: int | float | str | None = None,
    extra: Optional[Dict[str, Any]] = None,
) -> PCA9685Error:
    """
    Convenience helper to wrap a low-level exception into a PCA9685Error.
    """
    ctx = BoardErrorContext(
        driver="PCA9685",
        operation=operation,
        bus=bus,
        address=address,
        channel=channel,
        register=register,
        value=value,
        extra=dict(extra or {}),
    )
    return PCA9685Error(message, context=ctx, cause=exc)


def wrap_motor_board_error(
    exc: BaseException,
    *,
    message: str,
    driver: str = "MotorBoard",
    operation: str | None = None,
    wheel: str | None = None,
    channel: int | None = None,
    value: int | float | str | None = None,
    extra: Optional[Dict[str, Any]] = None,
) -> MotorBoardError:
    """
    Convenience helper to wrap a low-level exception into a MotorBoardError.
    """
    ctx = BoardErrorContext(
        driver=driver,
        operation=operation,
        wheel=wheel,
        channel=channel,
        value=value,
        extra=dict(extra or {}),
    )
    return MotorBoardError(message, context=ctx, cause=exc)