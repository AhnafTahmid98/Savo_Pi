#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""slam_toolbox readiness check. No ROS runtime required."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from savo_mapping.utils.diagnostics import (
    DiagnosticItem,
    make_error,
    make_ok,
    make_stale,
    make_warn,
)


# =============================================================================
# slam_toolbox result
# =============================================================================
@dataclass(frozen=True)
class SlamToolboxResult:
    ok: bool
    active: bool = False
    stale: bool = True

    node_name: str = "slam_toolbox"
    mode: str = "mapping"

    msg_count: int = 0
    map_msg_count: int = 0
    rate_hz: float = 0.0
    age_s: Optional[float] = None

    has_map: bool = False
    has_scan: bool = False
    has_odom: bool = False
    has_tf: bool = False

    message: str = "Waiting for slam_toolbox."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "active": self.active,
            "stale": self.stale,
            "node_name": self.node_name,
            "mode": self.mode,
            "msg_count": self.msg_count,
            "map_msg_count": self.map_msg_count,
            "rate_hz": self.rate_hz,
            "age_s": self.age_s,
            "has_map": self.has_map,
            "has_scan": self.has_scan,
            "has_odom": self.has_odom,
            "has_tf": self.has_tf,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Check logic
# =============================================================================
def evaluate_slam_toolbox_ready(
    active: bool,
    msg_count: int = 0,
    map_msg_count: int = 0,
    rate_hz: float = 0.0,
    age_s: Optional[float] = None,
    has_map: bool = False,
    has_scan: bool = False,
    has_odom: bool = False,
    has_tf: bool = False,
    node_name: str = "slam_toolbox",
    mode: str = "mapping",
    stale_timeout_s: float = 2.0,
    require_map: bool = False,
    extra: Optional[Dict[str, Any]] = None,
) -> SlamToolboxResult:
    count = max(0, int(msg_count))
    map_count = max(0, int(map_msg_count))
    rate = max(0.0, float(rate_hz))
    timeout = max(0.0, float(stale_timeout_s))

    stale = True if age_s is None else float(age_s) > timeout

    failures: list[str] = []

    if not active:
        failures.append("not_active")

    if count <= 0:
        failures.append("no_status_messages")

    if stale:
        failures.append("stale")

    if not has_scan:
        failures.append("missing_scan")

    if not has_odom:
        failures.append("missing_odom")

    if not has_tf:
        failures.append("missing_tf")

    if require_map and not has_map:
        failures.append("missing_map")

    if require_map and map_count <= 0:
        failures.append("no_map_messages")

    ok = not failures

    if ok:
        message = "slam_toolbox ready."
    else:
        message = f"slam_toolbox not ready: {', '.join(failures)}."

    return SlamToolboxResult(
        ok=ok,
        active=bool(active),
        stale=stale,
        node_name=str(node_name),
        mode=str(mode),
        msg_count=count,
        map_msg_count=map_count,
        rate_hz=rate,
        age_s=age_s,
        has_map=bool(has_map),
        has_scan=bool(has_scan),
        has_odom=bool(has_odom),
        has_tf=bool(has_tf),
        message=message,
        extra={
            "failures": failures,
            "stale_timeout_s": timeout,
            "require_map": require_map,
            **dict(extra or {}),
        },
    )


def evaluate_slam_toolbox_from_readiness(
    scan_ok: bool,
    odom_ok: bool,
    tf_ok: bool,
    map_ok: bool = False,
    active: bool = False,
    msg_count: int = 0,
    map_msg_count: int = 0,
    age_s: Optional[float] = None,
    require_map: bool = False,
) -> SlamToolboxResult:
    return evaluate_slam_toolbox_ready(
        active=active,
        msg_count=msg_count,
        map_msg_count=map_msg_count,
        age_s=age_s,
        has_map=map_ok,
        has_scan=scan_ok,
        has_odom=odom_ok,
        has_tf=tf_ok,
        require_map=require_map,
    )


def slam_toolbox_result_to_diagnostic(
    result: SlamToolboxResult,
    required: bool = False,
) -> DiagnosticItem:
    values = result.to_dict()

    if result.ok:
        return make_ok(
            "slam_toolbox",
            result.message,
            required=required,
            values=values,
        )

    if result.stale and result.active:
        return make_stale(
            "slam_toolbox",
            result.message,
            required=required,
            values=values,
        )

    if not result.active:
        return make_error(
            "slam_toolbox",
            result.message,
            required=required,
            values=values,
        )

    return make_warn(
        "slam_toolbox",
        result.message,
        required=required,
        values=values,
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    result = evaluate_slam_toolbox_ready(
        active=True,
        msg_count=10,
        map_msg_count=3,
        rate_hz=1.0,
        age_s=0.2,
        has_map=True,
        has_scan=True,
        has_odom=True,
        has_tf=True,
        require_map=False,
    )

    print(result.to_json(indent=2))
    print(slam_toolbox_result_to_diagnostic(result).to_dict())


if __name__ == "__main__":
    main()


__all__ = [
    "SlamToolboxResult",
    "evaluate_slam_toolbox_ready",
    "evaluate_slam_toolbox_from_readiness",
    "slam_toolbox_result_to_diagnostic",
    "main",
]