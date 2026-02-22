#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/drivers/board_factory.py
-----------------------------------------------
Professional driver/board factory for Robot Savo (ROS 2 Jazzy).

Purpose
- Create hardware driver instances from explicit arguments or config dictionaries
- Centralize board construction logic for:
    * CLI teleop tools
    * ROS2 base nodes
    * diagnostics / bringup scripts
- Keep real-robot hardware setup consistent across modules

Supported targets (current)
- PCA9685 driver (`savo_base.drivers.pca9685_driver.PCA9685Driver`)
- Freenove mecanum motor board wrapper
  (`savo_base.drivers.freenove_mecanum_board.FreenoveMecanumBoard`)

Design notes
- This module does not import ROS
- It is safe for non-ROS scripts and ROS nodes
- Lazy imports are used so import errors are raised only when a requested driver is built
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Tuple


# =============================================================================
# Exceptions (from local driver exception hierarchy)
# =============================================================================
try:
    from .board_exceptions import (
        BoardConfigError,
        BoardValidationError,
        UnsupportedBoardOperationError,
    )
except Exception:  # pragma: no cover
    # Minimal fallback for early bootstrap/debug scenarios
    class _BoardFactoryBootstrapError(RuntimeError):
        pass

    class BoardConfigError(_BoardFactoryBootstrapError):
        pass

    class BoardValidationError(_BoardFactoryBootstrapError):
        pass

    class UnsupportedBoardOperationError(_BoardFactoryBootstrapError):
        pass


# =============================================================================
# Public constants / identifiers
# =============================================================================
BOARD_TYPE_PCA9685 = "pca9685"
BOARD_TYPE_FREENOVE_MECANUM = "freenove_mecanum"

SUPPORTED_BOARD_TYPES: Tuple[str, ...] = (
    BOARD_TYPE_PCA9685,
    BOARD_TYPE_FREENOVE_MECANUM,
)


# =============================================================================
# Typed config containers
# =============================================================================
@dataclass(frozen=True)
class PCA9685FactoryConfig:
    """
    Factory config for creating a PCA9685 driver instance.
    """
    i2c_bus: int = 1
    address: int = 0x40
    debug: bool = False


@dataclass(frozen=True)
class FreenoveMecanumFactoryConfig:
    """
    Factory config for creating a Freenove mecanum board wrapper instance.

    Defaults are aligned with Robot Savo's real-hardware proven settings.
    """
    i2c_bus: int = 1
    address: int = 0x40
    pwm_freq_hz: float = 50.0

    # Robot Savo proven wheel invert defaults usually start as +1/+1/+1/+1;
    # final runtime command may still pass --invert-* flags.
    wheel_inverts: Tuple[int, int, int, int] = (+1, +1, +1, +1)

    quench_ms: int = 18
    max_duty: int = 3000
    debug: bool = False

    # Future-safe extension storage (ignored by factory unless needed)
    extra: Dict[str, Any] = field(default_factory=dict)


# =============================================================================
# Validation helpers
# =============================================================================
def _normalize_board_type(board_type: str) -> str:
    bt = str(board_type).strip().lower()
    aliases = {
        "pca": BOARD_TYPE_PCA9685,
        "pca9685_driver": BOARD_TYPE_PCA9685,
        "freenove": BOARD_TYPE_FREENOVE_MECANUM,
        "freenove_board": BOARD_TYPE_FREENOVE_MECANUM,
        "mecanum": BOARD_TYPE_FREENOVE_MECANUM,
        "mecanum_board": BOARD_TYPE_FREENOVE_MECANUM,
    }
    return aliases.get(bt, bt)


def _require_int(name: str, value: Any) -> int:
    try:
        return int(value)
    except Exception as e:
        raise BoardConfigError(f"{name} must be an integer, got {value!r}") from e


def _require_float(name: str, value: Any) -> float:
    try:
        return float(value)
    except Exception as e:
        raise BoardConfigError(f"{name} must be a number, got {value!r}") from e


def _parse_i2c_address(value: Any) -> int:
    """
    Accept int or string forms like '0x40' / '64'.
    """
    if isinstance(value, str):
        s = value.strip().lower()
        try:
            return int(s, 0)
        except Exception as e:
            raise BoardConfigError(f"Invalid I2C address string: {value!r}") from e
    return _require_int("address", value)


def _validate_i2c_bus(i2c_bus: int) -> int:
    b = _require_int("i2c_bus", i2c_bus)
    if b < 0:
        raise BoardConfigError(f"i2c_bus must be >= 0, got {b}")
    return b


def _validate_i2c_address(address: int) -> int:
    a = _require_int("address", address)
    if not (0x03 <= a <= 0x77):
        raise BoardConfigError(f"I2C address out of 7-bit range (0x03..0x77): 0x{a:02X}")
    return a


def _validate_sign_tuple4(name: str, values: Any) -> Tuple[int, int, int, int]:
    if not isinstance(values, (tuple, list)):
        raise BoardConfigError(f"{name} must be a 4-item tuple/list of +/-1, got {type(values).__name__}")

    if len(values) != 4:
        raise BoardConfigError(f"{name} must contain exactly 4 items (FL, RL, FR, RR), got {len(values)}")

    out = []
    for i, v in enumerate(values):
        iv = _require_int(f"{name}[{i}]", v)
        if iv not in (-1, +1):
            raise BoardConfigError(f"{name}[{i}] must be -1 or +1, got {iv}")
        out.append(iv)
    return (out[0], out[1], out[2], out[3])


def _validate_pwm_freq_hz(freq: float) -> float:
    f = _require_float("pwm_freq_hz", freq)
    if f <= 0.0:
        raise BoardConfigError(f"pwm_freq_hz must be > 0, got {f}")
    return f


def _validate_quench_ms(quench_ms: int) -> int:
    q = _require_int("quench_ms", quench_ms)
    if q < 0:
        raise BoardConfigError(f"quench_ms must be >= 0, got {q}")
    return q


def _validate_max_duty(max_duty: int) -> int:
    d = _require_int("max_duty", max_duty)
    if not (0 <= d <= 4095):
        raise BoardConfigError(f"max_duty must be in [0, 4095], got {d}")
    return d


# =============================================================================
# Lazy import helpers
# =============================================================================
def _import_pca9685_driver_class():
    """
    Resolve PCA9685 driver class lazily.

    Expected class name:
    - PCA9685Driver

    Fallback accepted:
    - PCA9685
    """
    try:
        from .pca9685_driver import PCA9685Driver  # type: ignore
        return PCA9685Driver
    except ImportError:
        try:
            from .pca9685_driver import PCA9685  # type: ignore
            return PCA9685
        except Exception as e:
            raise BoardConfigError(
                "Could not import PCA9685 driver class from savu_base.drivers.pca9685_driver "
                "(expected PCA9685Driver or PCA9685)"
            ) from e


def _import_freenove_mecanum_board_class():
    """
    Resolve Freenove mecanum board wrapper class lazily.

    Expected class names (accepted):
    - FreenoveMecanumBoard
    - RobotSavoBoard
    - RobotSavo
    """
    try:
        from .freenove_mecanum_board import FreenoveMecanumBoard  # type: ignore
        return FreenoveMecanumBoard
    except ImportError:
        # Backward-compatible fallbacks
        for cls_name in ("RobotSavoBoard", "RobotSavo"):
            try:
                mod = __import__(
                    "savo_base.drivers.freenove_mecanum_board",
                    fromlist=[cls_name]
                )
                return getattr(mod, cls_name)
            except Exception:
                continue

        raise BoardConfigError(
            "Could not import Freenove mecanum board class from "
            "savo_base.drivers.freenove_mecanum_board "
            "(expected FreenoveMecanumBoard / RobotSavoBoard / RobotSavo)"
        )


# =============================================================================
# Direct creation helpers (typed)
# =============================================================================
def create_pca9685_driver(cfg: PCA9685FactoryConfig):
    """
    Create and return a PCA9685 driver instance.

    The actual class is resolved lazily from `pca9685_driver.py`.
    """
    if not isinstance(cfg, PCA9685FactoryConfig):
        raise BoardValidationError("cfg must be PCA9685FactoryConfig")

    driver_cls = _import_pca9685_driver_class()

    i2c_bus = _validate_i2c_bus(cfg.i2c_bus)
    address = _validate_i2c_address(cfg.address)
    debug = bool(cfg.debug)

    # We support common constructor signatures:
    #   (bus=1, address=0x40, debug=False)
    #   (i2c_bus=1, address=0x40, debug=False)
    #   (busno=1, address=0x40, debug=False)
    ctor_attempts = [
        dict(bus=i2c_bus, address=address, debug=debug),
        dict(i2c_bus=i2c_bus, address=address, debug=debug),
        dict(busno=i2c_bus, address=address, debug=debug),
    ]

    last_error: Optional[Exception] = None
    for kwargs in ctor_attempts:
        try:
            return driver_cls(**kwargs)
        except TypeError as e:
            last_error = e
            continue
        except Exception:
            raise  # real runtime/hardware errors should bubble up

    raise BoardConfigError(
        "PCA9685 driver constructor signature mismatch. "
        "Expected one of kwargs: (bus/address/debug), (i2c_bus/address/debug), (busno/address/debug)."
    ) from last_error


def create_freenove_mecanum_board(cfg: FreenoveMecanumFactoryConfig):
    """
    Create and return a Freenove mecanum board wrapper instance.

    The actual class is resolved lazily from `freenove_mecanum_board.py`.
    """
    if not isinstance(cfg, FreenoveMecanumFactoryConfig):
        raise BoardValidationError("cfg must be FreenoveMecanumFactoryConfig")

    board_cls = _import_freenove_mecanum_board_class()

    i2c_bus = _validate_i2c_bus(cfg.i2c_bus)
    address = _validate_i2c_address(cfg.address)
    pwm_freq_hz = _validate_pwm_freq_hz(cfg.pwm_freq_hz)
    wheel_inverts = _validate_sign_tuple4("wheel_inverts", cfg.wheel_inverts)
    quench_ms = _validate_quench_ms(cfg.quench_ms)
    max_duty = _validate_max_duty(cfg.max_duty)
    debug = bool(cfg.debug)

    # Common constructor signatures expected from your evolving codebase:
    # - FreenoveMecanumBoard(i2c_bus=..., addr=..., pwm_freq=..., inv=..., quench_ms=..., max_duty=..., debug=...)
    # - FreenoveMecanumBoard(i2c_bus=..., address=..., pwm_freq_hz=..., wheel_inverts=..., ...)
    # - RobotSavo(... legacy ...)
    ctor_attempts = [
        dict(
            i2c_bus=i2c_bus,
            addr=address,
            pwm_freq=pwm_freq_hz,
            inv=wheel_inverts,
            quench_ms=quench_ms,
            max_duty=max_duty,
            debug=debug,
            **dict(cfg.extra),
        ),
        dict(
            i2c_bus=i2c_bus,
            address=address,
            pwm_freq_hz=pwm_freq_hz,
            wheel_inverts=wheel_inverts,
            quench_ms=quench_ms,
            max_duty=max_duty,
            debug=debug,
            **dict(cfg.extra),
        ),
        dict(
            bus=i2c_bus,
            address=address,
            pwm_freq=pwm_freq_hz,
            inv=wheel_inverts,
            quench_ms=quench_ms,
            max_duty=max_duty,
            debug=debug,
            **dict(cfg.extra),
        ),
    ]

    last_error: Optional[Exception] = None
    for kwargs in ctor_attempts:
        try:
            return board_cls(**kwargs)
        except TypeError as e:
            last_error = e
            continue
        except Exception:
            raise  # let actual hardware/runtime errors propagate

    raise BoardConfigError(
        "Freenove mecanum board constructor signature mismatch. "
        "Please align freenove_mecanum_board.py constructor with the factory or update ctor_attempts."
    ) from last_error


# =============================================================================
# Dict/config-based factory API (recommended for ROS params / YAML-style config)
# =============================================================================
def create_board(board_type: str, config: Optional[Mapping[str, Any]] = None):
    """
    Generic board factory.

    Args:
        board_type:
            One of:
              - 'pca9685'
              - 'freenove_mecanum'
            Aliases like 'freenove', 'mecanum', 'pca' are also accepted.
        config:
            Mapping of parameters (YAML/ROS-param style)

    Returns:
        Instantiated board/driver object

    Examples
    --------
    PCA9685:
        create_board("pca9685", {"i2c_bus": 1, "address": "0x40", "debug": False})

    Freenove mecanum:
        create_board("freenove_mecanum", {
            "i2c_bus": 1,
            "address": "0x40",
            "pwm_freq_hz": 50.0,
            "wheel_inverts": [1, 1, 1, 1],
            "quench_ms": 18,
            "max_duty": 3000,
            "debug": True,
        })
    """
    bt = _normalize_board_type(board_type)
    cfg = dict(config or {})

    if bt == BOARD_TYPE_PCA9685:
        p_cfg = PCA9685FactoryConfig(
            i2c_bus=_validate_i2c_bus(cfg.get("i2c_bus", cfg.get("bus", 1))),
            address=_validate_i2c_address(_parse_i2c_address(cfg.get("address", cfg.get("addr", 0x40)))),
            debug=bool(cfg.get("debug", False)),
        )
        return create_pca9685_driver(p_cfg)

    if bt == BOARD_TYPE_FREENOVE_MECANUM:
        extra = dict(cfg)

        # Pop recognized keys so `extra` remains future extension only
        i2c_bus_raw = extra.pop("i2c_bus", extra.pop("bus", 1))
        addr_raw = extra.pop("address", extra.pop("addr", 0x40))
        pwm_freq_raw = extra.pop("pwm_freq_hz", extra.pop("pwm_freq", 50.0))
        inv_raw = extra.pop("wheel_inverts", extra.pop("inv", (+1, +1, +1, +1)))
        quench_raw = extra.pop("quench_ms", 18)
        max_duty_raw = extra.pop("max_duty", 3000)
        debug_raw = extra.pop("debug", False)

        f_cfg = FreenoveMecanumFactoryConfig(
            i2c_bus=_validate_i2c_bus(i2c_bus_raw),
            address=_validate_i2c_address(_parse_i2c_address(addr_raw)),
            pwm_freq_hz=_validate_pwm_freq_hz(pwm_freq_raw),
            wheel_inverts=_validate_sign_tuple4("wheel_inverts", inv_raw),
            quench_ms=_validate_quench_ms(quench_raw),
            max_duty=_validate_max_duty(max_duty_raw),
            debug=bool(debug_raw),
            extra=extra,
        )
        return create_freenove_mecanum_board(f_cfg)

    raise UnsupportedBoardOperationError(
        f"Unsupported board_type={board_type!r}. "
        f"Supported types: {SUPPORTED_BOARD_TYPES}"
    )


# =============================================================================
# Convenience APIs for your Robot Savo default hardware
# =============================================================================
def create_robot_savo_default_mecanum_board(
    *,
    i2c_bus: int = 1,
    address: int = 0x40,
    pwm_freq_hz: float = 50.0,
    wheel_inverts: Tuple[int, int, int, int] = (+1, +1, +1, +1),
    quench_ms: int = 18,
    max_duty: int = 3000,
    debug: bool = False,
):
    """
    Convenience builder using Robot Savo's current proven hardware defaults.
    """
    cfg = FreenoveMecanumFactoryConfig(
        i2c_bus=i2c_bus,
        address=address,
        pwm_freq_hz=pwm_freq_hz,
        wheel_inverts=wheel_inverts,
        quench_ms=quench_ms,
        max_duty=max_duty,
        debug=debug,
    )
    return create_freenove_mecanum_board(cfg)


def describe_supported_boards() -> Dict[str, Dict[str, Any]]:
    """
    Return a structured description of supported factory targets.
    Useful for diagnostics, CLI help, or docs generation.
    """
    return {
        BOARD_TYPE_PCA9685: {
            "description": "Low-level PCA9685 PWM driver",
            "required_keys": [],
            "optional_keys": ["i2c_bus|bus", "address|addr", "debug"],
            "defaults": {
                "i2c_bus": 1,
                "address": "0x40",
                "debug": False,
            },
        },
        BOARD_TYPE_FREENOVE_MECANUM: {
            "description": "Freenove mecanum motor board wrapper for Robot Savo",
            "required_keys": [],
            "optional_keys": [
                "i2c_bus|bus",
                "address|addr",
                "pwm_freq_hz|pwm_freq",
                "wheel_inverts|inv",
                "quench_ms",
                "max_duty",
                "debug",
            ],
            "defaults": {
                "i2c_bus": 1,
                "address": "0x40",
                "pwm_freq_hz": 50.0,
                "wheel_inverts": [1, 1, 1, 1],
                "quench_ms": 18,
                "max_duty": 3000,
                "debug": False,
            },
            "robot_savo_notes": {
                "channel_map": {
                    "FL": [0, 1],
                    "RL": [3, 2],
                    "FR": [6, 7],
                    "RR": [4, 5],
                },
                "real_robot_testing": "Use proven sign/invert settings from motor_direction_test and teleop diagnostics.",
            },
        },
    }