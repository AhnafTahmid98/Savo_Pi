#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Nav2 mapping-mode readiness check. No ROS runtime required."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from savo_mapping.utils.diagnostics import (
    DiagnosticItem,
    make_disabled,
    make_error,
    make_ok,
    make_stale,
    make_warn,
)


# =============================================================================
# Nav2 mapping result
# =============================================================================
@dataclass(frozen=True)
class Nav2MappingResult:
    ok: bool
    enabled: bool = False
    stale: bool = True

    lifecycle_active: bool = False
    planner_active: bool = False
    controller_active: bool = False
    bt_navigator_active: bool = False
    costmaps_active: bool = False

    action_available: bool = False
    action_name: str = "/navigate_to_pose"

    msg_count: int = 0
    age_s: Optional[float] = None
    stale_timeout_s: float = 2.0

    message: str = "Nav2 mapping disabled."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "enabled": self.enabled,
            "stale": self.stale,
            "lifecycle_active": self.lifecycle_active,
            "planner_active": self.planner_active,
            "controller_active": self.controller_active,
            "bt_navigator_active": self.bt_navigator_active,
            "costmaps_active": self.costmaps_active,
            "action_available": self.action_available,
            "action_name": self.action_name,
            "msg_count": self.msg_count,
            "age_s": self.age_s,
            "stale_timeout_s": self.stale_timeout_s,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Check logic
# =============================================================================
def evaluate_nav2_mapping_ready(
    enabled: bool,
    lifecycle_active: bool = False,
    planner_active: bool = False,
    controller_active: bool = False,
    bt_navigator_active: bool = False,
    costmaps_active: bool = False,
    action_available: bool = False,
    msg_count: int = 0,
    age_s: Optional[float] = None,
    stale_timeout_s: float = 2.0,
    action_name: str = "/navigate_to_pose",
    extra: Optional[Dict[str, Any]] = None,
) -> Nav2MappingResult:
    if not enabled:
        return Nav2MappingResult(
            ok=False,
            enabled=False,
            stale=True,
            action_name=str(action_name),
            message="Nav2 mapping disabled.",
            extra=dict(extra or {}),
        )

    count = max(0, int(msg_count))
    timeout = max(0.0, float(stale_timeout_s))
    stale = True if age_s is None else float(age_s) > timeout

    failures: list[str] = []

    if stale:
        failures.append("stale")

    if not lifecycle_active:
        failures.append("lifecycle_not_active")

    if not planner_active:
        failures.append("planner_not_active")

    if not controller_active:
        failures.append("controller_not_active")

    if not bt_navigator_active:
        failures.append("bt_navigator_not_active")

    if not costmaps_active:
        failures.append("costmaps_not_active")

    if not action_available:
        failures.append("navigate_to_pose_unavailable")

    ok = not failures

    if ok:
        message = "Nav2 mapping stack ready."
    else:
        message = f"Nav2 mapping stack not ready: {', '.join(failures)}."

    return Nav2MappingResult(
        ok=ok,
        enabled=True,
        stale=stale,
        lifecycle_active=bool(lifecycle_active),
        planner_active=bool(planner_active),
        controller_active=bool(controller_active),
        bt_navigator_active=bool(bt_navigator_active),
        costmaps_active=bool(costmaps_active),
        action_available=bool(action_available),
        action_name=str(action_name),
        msg_count=count,
        age_s=age_s,
        stale_timeout_s=timeout,
        message=message,
        extra={
            "failures": failures,
            **dict(extra or {}),
        },
    )


def evaluate_nav2_mapping_from_nodes(
    enabled: bool,
    active_nodes: set[str],
    action_available: bool,
    msg_count: int = 0,
    age_s: Optional[float] = None,
    stale_timeout_s: float = 2.0,
) -> Nav2MappingResult:
    nodes = set(active_nodes)

    return evaluate_nav2_mapping_ready(
        enabled=enabled,
        lifecycle_active="lifecycle_manager_navigation" in nodes
        or "lifecycle_manager" in nodes,
        planner_active="planner_server" in nodes,
        controller_active="controller_server" in nodes,
        bt_navigator_active="bt_navigator" in nodes,
        costmaps_active=(
            "global_costmap" in nodes
            and "local_costmap" in nodes
        ),
        action_available=action_available,
        msg_count=msg_count,
        age_s=age_s,
        stale_timeout_s=stale_timeout_s,
    )


def nav2_mapping_result_to_diagnostic(
    result: Nav2MappingResult,
    required: bool = False,
) -> DiagnosticItem:
    values = result.to_dict()

    if not result.enabled:
        return make_disabled(
            "nav2_mapping",
            result.message,
            required=required,
            values=values,
        )

    if result.ok:
        return make_ok(
            "nav2_mapping",
            result.message,
            required=required,
            values=values,
        )

    if result.stale:
        return make_stale(
            "nav2_mapping",
            result.message,
            required=required,
            values=values,
        )

    if required:
        return make_error(
            "nav2_mapping",
            result.message,
            required=required,
            values=values,
        )

    return make_warn(
        "nav2_mapping",
        result.message,
        required=required,
        values=values,
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    disabled = evaluate_nav2_mapping_ready(enabled=False)

    ready = evaluate_nav2_mapping_ready(
        enabled=True,
        lifecycle_active=True,
        planner_active=True,
        controller_active=True,
        bt_navigator_active=True,
        costmaps_active=True,
        action_available=True,
        msg_count=5,
        age_s=0.1,
    )

    print(disabled.to_json(indent=2))
    print(ready.to_json(indent=2))
    print(nav2_mapping_result_to_diagnostic(ready).to_dict())


if __name__ == "__main__":
    main()


__all__ = [
    "Nav2MappingResult",
    "evaluate_nav2_mapping_ready",
    "evaluate_nav2_mapping_from_nodes",
    "nav2_mapping_result_to_diagnostic",
    "main",
]