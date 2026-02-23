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
    """Factory config for creating a PCA9685 driver instance."""
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
    wheel_inverts: Tuple[int, int, int, int] = (+1, +1, +1, +1)  # (FL, RL, FR, RR)
    quench_ms: int = 18
    max_duty: int = 3000
    debug: bool = False
    # Future-safe extension storage (ignored by current strict constructor attempts)
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
        "robot_savo_freenove_mecanum": BOARD_TYPE_FREENOVE_MECANUM,
        "robot_savo_freenove_mecanum_dryrun_motoroff": BOARD_TYPE_FREENOVE_MECANUM,  # profile safety alias
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
    """Accept int or string forms like '0x40' / '64'."""
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
        raise BoardConfigError(
            f"{name} must be a 4-item tuple/list of +/-1, got {type(values).__name__}"
        )
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


def _sign_tuple_to_invert_flags(wheel_inverts: Tuple[int, int, int, int]) -> Dict[str, bool]:
    """
    Convert (+1/-1) tuple order (FL, RL, FR, RR) into bool invert flags expected by
    current FreenoveMecanumBoard constructor.
    """
    fl, rl, fr, rr = wheel_inverts
    return {
        "invert_fl": (fl < 0),
        "invert_rl": (rl < 0),
        "invert_fr": (fr < 0),
        "invert_rr": (rr < 0),
    }


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
                "Could not import PCA9685 driver class from svo_base.drivers.pca9685_driver "
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
        for cls_name in ("RobotSavoBoard", "RobotSavo"):
            try:
                mod = __import__("savo_base.drivers.freenove_mecanum_board", fromlist=[cls_name])
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

    # Common constructor signatures in evolving codebase
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
    _ = _validate_max_duty(cfg.max_duty)  # validated for config consistency (board may ignore)
    debug = bool(cfg.debug)

    invert_flags = _sign_tuple_to_invert_flags(wheel_inverts)

    # IMPORTANT:
    # Current FreenoveMecanumBoard constructor is strict. Do NOT pass node metadata
    # / extra kwargs into the primary strict attempts.
    #
    # Typical current constructor:
    #   __init__(*, i2c_bus=1, addr=0x40, pwm_freq_hz=50.0,
    #            invert_fl=False, invert_rl=False, invert_fr=False, invert_rr=False,
    #            quench_ms=18, debug=False)
    extra = dict(cfg.extra)

    # Drop keys that are node/factory metadata or known mismatches for legacy attempts too.
    for k in (
        "backend", "board_backend", "name", "board_name", "dryrun",
        "source_name", "timing_utils", "topic_names",
        "max_duty", "wheel_inverts", "inv",
        "address", "addr", "bus",
        "pwm_freq", "pwm_freq_hz",
        "invert_fl", "invert_rl", "invert_fr", "invert_rr",
        "board_type",
    ):
        extra.pop(k, None)

    ctor_attempts = [
        # === Current locked constructor (strict; NO extra kwargs) ===
        dict(
            i2c_bus=i2c_bus,
            addr=address,
            pwm_freq_hz=pwm_freq_hz,
            quench_ms=quench_ms,
            debug=debug,
            **invert_flags,
        ),
        # alt address kw (strict; NO extra kwargs)
        dict(
            i2c_bus=i2c_bus,
            address=address,
            pwm_freq_hz=pwm_freq_hz,
            quench_ms=quench_ms,
            debug=debug,
            **invert_flags,
        ),

        # === Legacy attempts (allow leftover extra if an older class supports it) ===
        dict(
            i2c_bus=i2c_bus,
            addr=address,
            pwm_freq=pwm_freq_hz,
            inv=wheel_inverts,
            quench_ms=quench_ms,
            debug=debug,
            **extra,
        ),
        dict(
            bus=i2c_bus,
            address=address,
            pwm_freq=pwm_freq_hz,
            inv=wheel_inverts,
            quench_ms=quench_ms,
            debug=debug,
            **extra,
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

        # Remove metadata / compatibility keys that are not constructor params
        for k in (
            "backend", "board_backend", "name", "board_name", "dryrun",
            "source_name", "timing_utils", "topic_names", "board_type",
        ):
            extra.pop(k, None)

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
# Compatibility wrapper for ROS base driver (legacy/evolving call style)
# =============================================================================
def make_motor_board(**kwargs):
    """
    Backward-compatible motor board factory used by evolving ROS base nodes.

    Accepts legacy kwargs such as:
      backend, board_backend, name, board_name, dryrun, i2c_bus, pca9685_addr,
      pwm_freq_hz, quench_ms, max_duty, invert_fl, invert_rl, invert_fr, invert_rr, debug

    Returns:
      A motor board object (dryrun or Freenove mecanum)
    """
    data = dict(kwargs)

    backend = str(data.pop("backend", data.pop("board_backend", "auto"))).strip().lower()
    _name = data.pop("name", data.pop("board_name", None))  # kept for compatibility/logging only
    dryrun_flag = bool(data.pop("dryrun", False))

    # Strip node metadata that must never reach constructors
    data.pop("source_name", None)
    data.pop("timing_utils", None)
    data.pop("topic_names", None)
    data.pop("board_type", None)

    # normalize common fields
    i2c_bus = data.pop("i2c_bus", data.pop("bus", 1))
    address = data.pop("address", data.pop("addr", data.pop("pca9685_addr", 0x40)))
    pwm_freq_hz = data.pop("pwm_freq_hz", data.pop("pwm_freq", 50.0))
    quench_ms = data.pop("quench_ms", 18)
    max_duty = data.pop("max_duty", 3000)
    debug = bool(data.pop("debug", False))

    # per-wheel bool invert flags -> wheel_inverts tuple (+1/-1)
    invert_fl = bool(data.pop("invert_fl", False))
    invert_rl = bool(data.pop("invert_rl", False))
    invert_fr = bool(data.pop("invert_fr", False))
    invert_rr = bool(data.pop("invert_rr", False))
    wheel_inverts = (
        -1 if invert_fl else +1,
        -1 if invert_rl else +1,
        -1 if invert_fr else +1,
        -1 if invert_rr else +1,
    )

    wants_dryrun = dryrun_flag or (backend in ("dryrun", "mock", "sim"))

    if wants_dryrun:
        try:
            from .dryrun_motor_board import DryRunMotorBoard  # type: ignore

            ctor_attempts = [
                dict(max_duty=max_duty, debug=debug),
                dict(debug=debug),
                dict(),
            ]
            last_error: Optional[Exception] = None
            for attempt in ctor_attempts:
                try:
                    return DryRunMotorBoard(**attempt)
                except TypeError as e:
                    last_error = e
                    continue

            raise BoardConfigError("DryRunMotorBoard constructor signature mismatch.") from last_error
        except Exception:
            # fallback to hardware wrapper path if dryrun board unavailable
            pass

    # Hardware (or dryrun fallback) -> Freenove board
    config = {
        "i2c_bus": i2c_bus,
        "address": address,
        "pwm_freq_hz": pwm_freq_hz,
        "wheel_inverts": wheel_inverts,
        "quench_ms": quench_ms,
        "max_duty": max_duty,  # validated + may be ignored by current constructor
        "debug": debug,
    }

    # carry through only safe extras
    for k, v in data.items():
        if k in ("backend", "board_backend", "name", "board_name", "dryrun"):
            continue
        config[k] = v

    return create_board(BOARD_TYPE_FREENOVE_MECANUM, config)


# =============================================================================
# Convenience APIs for Robot Savo default hardware
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
    """Convenience builder using Robot Savo's current proven hardware defaults."""
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
                "real_robot_testing": (
                    "Use proven sign/invert settings from motor_direction_test "
                    "and teleop diagnostics."
                ),
            },
        },
    }


# =============================================================================
# Public exports
# =============================================================================
__all__ = [
    # constants
    "BOARD_TYPE_PCA9685",
    "BOARD_TYPE_FREENOVE_MECANUM",
    "SUPPORTED_BOARD_TYPES",

    # typed configs
    "PCA9685FactoryConfig",
    "FreenoveMecanumFactoryConfig",

    # typed builders
    "create_pca9685_driver",
    "create_freenove_mecanum_board",
    "create_robot_savo_default_mecanum_board",

    # generic / compatibility builders
    "create_board",
    "make_motor_board",

    # introspection
    "describe_supported_boards",
]