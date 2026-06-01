#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/utils/param_loader.py
--------------------------------------------
Professional ROS2 Jazzy-friendly parameter loading helpers for `savo_base`.

Why this exists
---------------
`savo_base` nodes (especially real hardware execution nodes) need:
- consistent parameter declaration
- typed parameter reads with safe defaults
- string/int/float/bool parsing helpers (including "0x40" style hex)
- compact logging of loaded parameter values
- optional grouped loading via dataclasses

This module keeps parameter handling clean and reusable across:
- base_driver_node
- base_watchdog_node
- base_state_publisher_node
- diagnostics / bringup helpers

Design goals
------------
- Safe defaults for real robot testing
- Minimal dependencies (stdlib + rclpy only when used with a Node)
- Works even if called outside ROS2 (for unit tests / dry-run tooling)

Usage examples
--------------
# 1) Simple typed reads
from savo_base.utils.param_loader import ParamLoader

pl = ParamLoader(self)  # self = ROS2 Node
pl.declare("max_duty", 3000)
max_duty = pl.get_int("max_duty", default=3000, lo=0, hi=4095)

# 2) Hex/string parsing
pl.declare("pca9685_addr", "0x40")
addr = pl.get_int("pca9685_addr", default=0x40, base_auto=True)  # -> 64

# 3) Bulk declaration
pl.declare_many({
    "loop_hz": 30.0,
    "watchdog_timeout_s": 0.30,
    "dryrun": False,
})

# 4) Structured summary logging
pl.log_loaded(component="base_driver_node")
"""

from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple, Union


# =============================================================================
# Optional ROS imports (module remains importable without ROS for tests)
# =============================================================================

try:
    from rclpy.node import Node
    _HAS_RCLPY = True
except Exception:
    Node = Any  # type: ignore
    _HAS_RCLPY = False


# =============================================================================
# Small internal logger fallback (to avoid hard dependency on utils.logging)
# =============================================================================

class _MiniLogger:
    def info(self, msg: str) -> None:
        print(msg)

    def warn(self, msg: str) -> None:
        print(msg)

    def error(self, msg: str) -> None:
        print(msg)

    def debug(self, msg: str) -> None:
        print(msg)


def _get_node_logger(node: Any) -> Any:
    try:
        if node is not None and hasattr(node, "get_logger") and callable(node.get_logger):
            return node.get_logger()
    except Exception:
        pass
    return _MiniLogger()


# =============================================================================
# Generic parsing helpers
# =============================================================================

def _clamp_num(value: Union[int, float], lo: Optional[Union[int, float]], hi: Optional[Union[int, float]]) -> Union[int, float]:
    if lo is not None and value < lo:
        value = lo
    if hi is not None and value > hi:
        value = hi
    return value


def _to_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return bool(default)
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in ("1", "true", "t", "yes", "y", "on"):
        return True
    if text in ("0", "false", "f", "no", "n", "off"):
        return False
    return bool(default)


def _to_int(value: Any, default: int = 0, *, base_auto: bool = True) -> int:
    if value is None:
        return int(default)
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float):
        return int(value)
    text = str(value).strip()
    try:
        return int(text, 0 if base_auto else 10)
    except Exception:
        try:
            return int(float(text))
        except Exception:
            return int(default)


def _to_float(value: Any, default: float = 0.0) -> float:
    if value is None:
        return float(default)
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    try:
        return float(str(value).strip())
    except Exception:
        return float(default)


def _to_str(value: Any, default: str = "") -> str:
    if value is None:
        return str(default)
    try:
        return str(value)
    except Exception:
        return str(default)


def _to_list_str(value: Any, default: Optional[List[str]] = None) -> List[str]:
    if default is None:
        default = []
    if value is None:
        return list(default)
    if isinstance(value, (list, tuple)):
        return [str(v) for v in value]
    text = str(value).strip()
    if not text:
        return list(default)
    # Comma-separated fallback
    return [p.strip() for p in text.split(",") if p.strip()]


# =============================================================================
# Structured records
# =============================================================================

@dataclass
class ParamSpec:
    """
    Describes a parameter declaration and optional typed loading constraints.
    """
    name: str
    default: Any
    kind: str = "auto"       # auto | bool | int | float | str | list_str
    lo: Optional[float] = None
    hi: Optional[float] = None
    description: str = ""
    base_auto: bool = True   # relevant for int parsing


@dataclass
class ParamRecord:
    """
    Stores final loaded parameter value and metadata for diagnostics/logging.
    """
    name: str
    declared_default: Any
    loaded_value: Any
    kind: str
    clamped: bool = False
    parse_fallback_used: bool = False
    notes: str = ""


@dataclass
class ParamLoadSummary:
    """
    Aggregate summary for dashboards / logs / tests.
    """
    component: str = "savo_base"
    records: Dict[str, ParamRecord] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "component": self.component,
            "count": len(self.records),
            "params": {
                k: {
                    "declared_default": v.declared_default,
                    "loaded_value": v.loaded_value,
                    "kind": v.kind,
                    "clamped": v.clamped,
                    "parse_fallback_used": v.parse_fallback_used,
                    "notes": v.notes,
                }
                for k, v in self.records.items()
            },
        }

    def values_dict(self) -> Dict[str, Any]:
        return {k: v.loaded_value for k, v in self.records.items()}


# =============================================================================
# ParamLoader
# =============================================================================

class ParamLoader:
    """
    Professional parameter loader wrapper for ROS2 Nodes.

    Features
    --------
    - idempotent parameter declaration
    - typed reads (bool/int/float/str/list_str)
    - bounds clamping for numeric params
    - local record of loaded values (for diagnostics/logging)
    - helper methods for common Robot Savo hardware params

    Notes
    -----
    This class is intentionally conservative:
    - It declares before reading.
    - It falls back safely if a value is malformed.
    - It logs warnings instead of throwing for non-critical issues.
    """

    def __init__(self, node: Optional[Any], *, component: str = "savo_base") -> None:
        self.node = node
        self.component = component
        self.logger = _get_node_logger(node)
        self.summary = ParamLoadSummary(component=component)

    # -------------------------------------------------------------------------
    # Core ROS helpers
    # -------------------------------------------------------------------------
    def _has_node(self) -> bool:
        return self.node is not None

    def _declare_if_needed(self, name: str, default: Any) -> None:
        """
        Declare parameter if not already declared.
        Works with ROS2 Jazzy Node APIs.
        """
        if not self._has_node():
            return

        try:
            # Preferred path
            if hasattr(self.node, "has_parameter") and callable(self.node.has_parameter):
                if self.node.has_parameter(name):
                    return
        except Exception:
            pass

        try:
            self.node.declare_parameter(name, default)
            return
        except Exception:
            # Some nodes may raise if already declared; ignore gracefully
            return

    def _read_raw(self, name: str, default: Any) -> Any:
        """
        Read raw parameter value from ROS node if available, else return default.
        """
        if not self._has_node():
            return default

        try:
            param_obj = self.node.get_parameter(name)
            # Most common path in rclpy:
            if hasattr(param_obj, "value"):
                return param_obj.value
            # Fallback path via ParameterValue
            if hasattr(param_obj, "get_parameter_value"):
                pv = param_obj.get_parameter_value()
                # Try common fields (best-effort)
                for attr in (
                    "bool_value",
                    "integer_value",
                    "double_value",
                    "string_value",
                    "string_array_value",
                    "integer_array_value",
                    "double_array_value",
                    "bool_array_value",
                ):
                    if hasattr(pv, attr):
                        val = getattr(pv, attr)
                        # Skip empty defaults if param may be different type
                        if val is not None:
                            return val
        except Exception:
            pass

        return default

    # -------------------------------------------------------------------------
    # Public declaration helpers
    # -------------------------------------------------------------------------
    def declare(self, name: str, default: Any, *, description: str = "") -> "ParamLoader":
        """
        Declare one parameter (idempotent).
        """
        self._declare_if_needed(name, default)
        return self

    def declare_many(self, params: Mapping[str, Any]) -> "ParamLoader":
        """
        Declare multiple parameters from {name: default}.
        """
        for name, default in params.items():
            self._declare_if_needed(str(name), default)
        return self

    def declare_specs(self, specs: Iterable[ParamSpec]) -> "ParamLoader":
        """
        Declare multiple parameters from ParamSpec list.
        """
        for spec in specs:
            self._declare_if_needed(spec.name, spec.default)
        return self

    # -------------------------------------------------------------------------
    # Record helper
    # -------------------------------------------------------------------------
    def _record(
        self,
        *,
        name: str,
        declared_default: Any,
        loaded_value: Any,
        kind: str,
        clamped: bool = False,
        parse_fallback_used: bool = False,
        notes: str = "",
    ) -> None:
        self.summary.records[name] = ParamRecord(
            name=name,
            declared_default=declared_default,
            loaded_value=loaded_value,
            kind=kind,
            clamped=clamped,
            parse_fallback_used=parse_fallback_used,
            notes=notes,
        )

    # -------------------------------------------------------------------------
    # Typed getters
    # -------------------------------------------------------------------------
    def get_bool(self, name: str, *, default: bool = False) -> bool:
        self._declare_if_needed(name, default)
        raw = self._read_raw(name, default)
        val = _to_bool(raw, default=default)
        fallback = not isinstance(raw, bool) and str(raw).strip().lower() not in (
            "true", "false", "1", "0", "yes", "no", "on", "off"
        )
        self._record(
            name=name,
            declared_default=default,
            loaded_value=val,
            kind="bool",
            parse_fallback_used=fallback,
        )
        return val

    def get_int(
        self,
        name: str,
        *,
        default: int = 0,
        lo: Optional[int] = None,
        hi: Optional[int] = None,
        base_auto: bool = True,
    ) -> int:
        self._declare_if_needed(name, default)
        raw = self._read_raw(name, default)
        val_before = _to_int(raw, default=default, base_auto=base_auto)
        val = int(_clamp_num(val_before, lo, hi))
        clamped = (val != val_before)
        fallback = False
        try:
            # Detect parse fallback approximately
            if isinstance(raw, str):
                stripped = raw.strip()
                # if raw string isn't parseable exactly, _to_int already fell back
                int(stripped, 0 if base_auto else 10)
        except Exception:
            fallback = True
        self._record(
            name=name,
            declared_default=default,
            loaded_value=val,
            kind="int",
            clamped=clamped,
            parse_fallback_used=fallback,
            notes=f"range=[{lo},{hi}]" if (lo is not None or hi is not None) else "",
        )
        return val

    def get_float(
        self,
        name: str,
        *,
        default: float = 0.0,
        lo: Optional[float] = None,
        hi: Optional[float] = None,
    ) -> float:
        self._declare_if_needed(name, default)
        raw = self._read_raw(name, default)
        val_before = _to_float(raw, default=default)
        val = float(_clamp_num(val_before, lo, hi))
        clamped = (val != val_before)
        fallback = False
        try:
            float(raw)
        except Exception:
            fallback = True
        self._record(
            name=name,
            declared_default=default,
            loaded_value=val,
            kind="float",
            clamped=clamped,
            parse_fallback_used=fallback,
            notes=f"range=[{lo},{hi}]" if (lo is not None or hi is not None) else "",
        )
        return val

    def get_str(self, name: str, *, default: str = "") -> str:
        self._declare_if_needed(name, default)
        raw = self._read_raw(name, default)
        val = _to_str(raw, default=default)
        self._record(
            name=name,
            declared_default=default,
            loaded_value=val,
            kind="str",
        )
        return val

    def get_list_str(self, name: str, *, default: Optional[List[str]] = None) -> List[str]:
        if default is None:
            default = []
        self._declare_if_needed(name, default)
        raw = self._read_raw(name, default)
        val = _to_list_str(raw, default=default)
        self._record(
            name=name,
            declared_default=list(default),
            loaded_value=list(val),
            kind="list_str",
        )
        return val

    # -------------------------------------------------------------------------
    # Generic getter from spec
    # -------------------------------------------------------------------------
    def get_from_spec(self, spec: ParamSpec) -> Any:
        kind = (spec.kind or "auto").strip().lower()

        if kind == "bool":
            return self.get_bool(spec.name, default=bool(spec.default))

        if kind == "int":
            lo_i = int(spec.lo) if spec.lo is not None else None
            hi_i = int(spec.hi) if spec.hi is not None else None
            return self.get_int(
                spec.name,
                default=int(spec.default),
                lo=lo_i,
                hi=hi_i,
                base_auto=bool(spec.base_auto),
            )

        if kind == "float":
            lo_f = float(spec.lo) if spec.lo is not None else None
            hi_f = float(spec.hi) if spec.hi is not None else None
            return self.get_float(
                spec.name,
                default=float(spec.default),
                lo=lo_f,
                hi=hi_f,
            )

        if kind == "str":
            return self.get_str(spec.name, default=str(spec.default))

        if kind == "list_str":
            if isinstance(spec.default, list):
                dflt = [str(x) for x in spec.default]
            else:
                dflt = _to_list_str(spec.default, default=[])
            return self.get_list_str(spec.name, default=dflt)

        # auto inference
        d = spec.default
        if isinstance(d, bool):
            return self.get_bool(spec.name, default=d)
        if isinstance(d, int) and not isinstance(d, bool):
            return self.get_int(spec.name, default=d, lo=int(spec.lo) if spec.lo is not None else None, hi=int(spec.hi) if spec.hi is not None else None)
        if isinstance(d, float):
            return self.get_float(spec.name, default=d, lo=spec.lo, hi=spec.hi)
        if isinstance(d, (list, tuple)):
            return self.get_list_str(spec.name, default=[str(x) for x in d])

        return self.get_str(spec.name, default=str(d))

    def load_specs(self, specs: Iterable[ParamSpec]) -> Dict[str, Any]:
        """
        Declare + load a set of ParamSpec entries.
        Returns {name: typed_value}
        """
        values: Dict[str, Any] = {}
        for spec in specs:
            self._declare_if_needed(spec.name, spec.default)
            values[spec.name] = self.get_from_spec(spec)
        return values

    # -------------------------------------------------------------------------
    # Convenience bundles for Robot Savo base hardware bringup
    # -------------------------------------------------------------------------
    def load_base_board_params(self) -> Dict[str, Any]:
        """
        Common hardware board params used by `savo_base` real robot nodes.

        Returns dict with keys:
        - i2c_bus
        - pca9685_addr
        - pwm_freq_hz
        - quench_ms
        - invert_fl / invert_rl / invert_fr / invert_rr
        - dryrun
        - board_backend
        - board_name
        """
        specs = [
            ParamSpec("i2c_bus", 1, kind="int", lo=0, hi=16),
            ParamSpec("pca9685_addr", "0x40", kind="int", lo=0x00, hi=0x7F, base_auto=True),
            ParamSpec("pwm_freq_hz", 50.0, kind="float", lo=1.0, hi=2000.0),
            ParamSpec("quench_ms", 18, kind="int", lo=0, hi=500),
            ParamSpec("invert_fl", False, kind="bool"),
            ParamSpec("invert_rl", False, kind="bool"),
            ParamSpec("invert_fr", False, kind="bool"),
            ParamSpec("invert_rr", False, kind="bool"),
            ParamSpec("dryrun", False, kind="bool"),
            ParamSpec("board_backend", "auto", kind="str"),
            ParamSpec("board_name", "robot_savo_freenove_mecanum", kind="str"),
        ]
        return self.load_specs(specs)

    def load_base_motion_limits(self) -> Dict[str, Any]:
        """
        Common command-space limits / scaling params for base execution.
        """
        specs = [
            ParamSpec("vx_limit", 1.0, kind="float", lo=0.0, hi=10.0),
            ParamSpec("vy_limit", 1.0, kind="float", lo=0.0, hi=10.0),
            ParamSpec("wz_limit", 1.0, kind="float", lo=0.0, hi=20.0),
            ParamSpec("max_duty", 3000, kind="int", lo=0, hi=4095),
            ParamSpec("turn_gain", 1.0, kind="float", lo=0.0, hi=10.0),
            ParamSpec("forward_sign", -1, kind="int", lo=-1, hi=1),
            ParamSpec("strafe_sign", 1, kind="int", lo=-1, hi=1),
            ParamSpec("rotate_sign", 1, kind="int", lo=-1, hi=1),
        ]
        vals = self.load_specs(specs)

        # Ensure sign params are only ±1 (not 0)
        for k in ("forward_sign", "strafe_sign", "rotate_sign"):
            if int(vals[k]) == 0:
                self.logger.warn(f"[{self.component}] Parameter '{k}' cannot be 0, forcing to +1")
                vals[k] = 1
                if k in self.summary.records:
                    self.summary.records[k].loaded_value = 1
                    self.summary.records[k].notes = (self.summary.records[k].notes + "; forced_nonzero_sign").strip("; ")
        return vals

    def load_base_runtime_timing(self) -> Dict[str, Any]:
        """
        Common loop/watchdog timing params.
        """
        specs = [
            ParamSpec("loop_hz", 30.0, kind="float", lo=1.0, hi=500.0),
            ParamSpec("watchdog_timeout_s", 0.30, kind="float", lo=0.02, hi=10.0),
            ParamSpec("publish_status_hz", 2.0, kind="float", lo=0.1, hi=100.0),
            ParamSpec("slowdown_default", 1.0, kind="float", lo=0.0, hi=1.0),
            ParamSpec("slowdown_min", 0.0, kind="float", lo=0.0, hi=1.0),
            ParamSpec("slowdown_max", 1.0, kind="float", lo=0.0, hi=1.0),
            ParamSpec("use_safety_stop", True, kind="bool"),
            ParamSpec("use_slowdown_factor", True, kind="bool"),
        ]
        vals = self.load_specs(specs)

        # Normalize slowdown range if user configured min > max
        if float(vals["slowdown_min"]) > float(vals["slowdown_max"]):
            old_min = float(vals["slowdown_min"])
            old_max = float(vals["slowdown_max"])
            vals["slowdown_min"], vals["slowdown_max"] = old_max, old_min
            self.logger.warn(
                f"[{self.component}] slowdown_min > slowdown_max; swapped values "
                f"({old_min} -> {vals['slowdown_min']}, {old_max} -> {vals['slowdown_max']})"
            )
            if "slowdown_min" in self.summary.records:
                self.summary.records["slowdown_min"].loaded_value = vals["slowdown_min"]
                self.summary.records["slowdown_min"].notes = "swapped_with_slowdown_max"
            if "slowdown_max" in self.summary.records:
                self.summary.records["slowdown_max"].loaded_value = vals["slowdown_max"]
                self.summary.records["slowdown_max"].notes = "swapped_with_slowdown_min"

        return vals

    # -------------------------------------------------------------------------
    # Logging / export helpers
    # -------------------------------------------------------------------------
    def values_dict(self) -> Dict[str, Any]:
        return self.summary.values_dict()

    def summary_dict(self) -> Dict[str, Any]:
        return self.summary.to_dict()

    def log_loaded(self, *, component: Optional[str] = None, include_values: bool = True) -> None:
        """
        Log a compact summary of loaded parameters.
        """
        comp = component or self.component
        count = len(self.summary.records)
        self.logger.info(f"[{comp}] Loaded {count} parameters")

        if not include_values:
            return

        for name in sorted(self.summary.records.keys()):
            rec = self.summary.records[name]
            suffix_parts = []
            if rec.clamped:
                suffix_parts.append("clamped")
            if rec.parse_fallback_used:
                suffix_parts.append("fallback_parse")
            if rec.notes:
                suffix_parts.append(rec.notes)
            suffix = f" ({', '.join(suffix_parts)})" if suffix_parts else ""
            self.logger.info(f"[{comp}]   {name} = {rec.loaded_value!r} [{rec.kind}]{suffix}")


# =============================================================================
# Functional helpers (for teams that prefer functions over classes)
# =============================================================================

def declare_params(node: Any, params: Mapping[str, Any]) -> None:
    """
    Functional helper: declare many parameters idempotently.
    """
    pl = ParamLoader(node)
    pl.declare_many(params)


def load_typed_params(node: Any, specs: Iterable[ParamSpec], *, component: str = "savo_base") -> Dict[str, Any]:
    """
    Functional helper: declare + load typed params from ParamSpec list.
    """
    pl = ParamLoader(node, component=component)
    return pl.load_specs(specs)


# =============================================================================
# Public exports
# =============================================================================

__all__ = [
    "ParamSpec",
    "ParamRecord",
    "ParamLoadSummary",
    "ParamLoader",
    "declare_params",
    "load_typed_params",
]