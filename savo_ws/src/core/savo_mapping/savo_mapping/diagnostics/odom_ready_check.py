#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Odometry readiness check. No ROS runtime required."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from savo_mapping.constants import (
    DEFAULT_ODOM_STALE_TIMEOUT_S,
    FRAME_ODOM,
    TOPIC_ODOM,
)
from savo_mapping.utils.diagnostics import (
    DiagnosticItem,
    make_error,
    make_ok,
    make_stale,
    make_warn,
)


# =============================================================================
# Odometry readiness result
# =============================================================================
@dataclass(frozen=True)
class OdomReadyResult:
    ok: bool
    topic: str = TOPIC_ODOM
    frame_id: str = FRAME_ODOM
    child_frame_id: str = "base_link"
    stale: bool = True

    msg_count: int = 0
    rate_hz: float = 0.0
    age_s: Optional[float] = None

    x: Optional[float] = None
    y: Optional[float] = None
    yaw_known: bool = False

    vx: Optional[float] = None
    vy: Optional[float] = None
    wz: Optional[float] = None

    message: str = "Waiting for filtered odometry."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "topic": self.topic,
            "frame_id": self.frame_id,
            "child_frame_id": self.child_frame_id,
            "stale": self.stale,
            "msg_count": self.msg_count,
            "rate_hz": self.rate_hz,
            "age_s": self.age_s,
            "x": self.x,
            "y": self.y,
            "yaw_known": self.yaw_known,
            "vx": self.vx,
            "vy": self.vy,
            "wz": self.wz,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Check logic
# =============================================================================
def evaluate_odom_ready(
    msg_count: int,
    rate_hz: float,
    age_s: Optional[float],
    frame_id: str = FRAME_ODOM,
    child_frame_id: str = "base_link",
    topic: str = TOPIC_ODOM,
    min_rate_hz: float = 5.0,
    stale_timeout_s: float = DEFAULT_ODOM_STALE_TIMEOUT_S,
    x: Optional[float] = None,
    y: Optional[float] = None,
    yaw_known: bool = False,
    vx: Optional[float] = None,
    vy: Optional[float] = None,
    wz: Optional[float] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> OdomReadyResult:
    count = max(0, int(msg_count))
    rate = max(0.0, float(rate_hz))
    timeout = max(0.0, float(stale_timeout_s))

    stale = True if age_s is None else float(age_s) > timeout

    failures: list[str] = []

    if count <= 0:
        failures.append("no_messages")

    if stale:
        failures.append("stale")

    if rate < min_rate_hz:
        failures.append("rate_low")

    if not str(frame_id).strip():
        failures.append("missing_frame_id")

    if not str(child_frame_id).strip():
        failures.append("missing_child_frame_id")

    ok = not failures

    if ok:
        message = "Filtered odometry ready."
    else:
        message = f"Filtered odometry not ready: {', '.join(failures)}."

    return OdomReadyResult(
        ok=ok,
        topic=str(topic),
        frame_id=str(frame_id),
        child_frame_id=str(child_frame_id),
        stale=stale,
        msg_count=count,
        rate_hz=rate,
        age_s=age_s,
        x=x,
        y=y,
        yaw_known=bool(yaw_known),
        vx=vx,
        vy=vy,
        wz=wz,
        message=message,
        extra={
            "failures": failures,
            "min_rate_hz": min_rate_hz,
            "stale_timeout_s": timeout,
            **dict(extra or {}),
        },
    )


def evaluate_odom_msg(
    msg: Any,
    msg_count: int,
    rate_hz: float,
    age_s: Optional[float],
    topic: str = TOPIC_ODOM,
    min_rate_hz: float = 5.0,
    stale_timeout_s: float = DEFAULT_ODOM_STALE_TIMEOUT_S,
) -> OdomReadyResult:
    return evaluate_odom_ready(
        msg_count=msg_count,
        rate_hz=rate_hz,
        age_s=age_s,
        frame_id=str(msg.header.frame_id),
        child_frame_id=str(msg.child_frame_id),
        topic=topic,
        min_rate_hz=min_rate_hz,
        stale_timeout_s=stale_timeout_s,
        x=float(msg.pose.pose.position.x),
        y=float(msg.pose.pose.position.y),
        yaw_known=True,
        vx=float(msg.twist.twist.linear.x),
        vy=float(msg.twist.twist.linear.y),
        wz=float(msg.twist.twist.angular.z),
    )


def odom_result_to_diagnostic(
    result: OdomReadyResult,
    required: bool = True,
) -> DiagnosticItem:
    values = result.to_dict()

    if result.ok:
        return make_ok(
            "odom",
            result.message,
            required=required,
            values=values,
        )

    if result.stale:
        return make_stale(
            "odom",
            result.message,
            required=required,
            values=values,
        )

    if result.msg_count <= 0:
        return make_error(
            "odom",
            result.message,
            required=required,
            values=values,
        )

    return make_warn(
        "odom",
        result.message,
        required=required,
        values=values,
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    result = evaluate_odom_ready(
        msg_count=50,
        rate_hz=30.0,
        age_s=0.02,
        frame_id="odom",
        child_frame_id="base_link",
        x=1.2,
        y=0.8,
        yaw_known=True,
        vx=0.0,
        vy=0.0,
        wz=0.0,
    )

    print(result.to_json(indent=2))
    print(odom_result_to_diagnostic(result).to_dict())


if __name__ == "__main__":
    main()


__all__ = [
    "OdomReadyResult",
    "evaluate_odom_ready",
    "evaluate_odom_msg",
    "odom_result_to_diagnostic",
    "main",
]