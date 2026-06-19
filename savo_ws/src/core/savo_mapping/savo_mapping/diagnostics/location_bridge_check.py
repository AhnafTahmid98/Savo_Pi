#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""savo_location bridge readiness check. No ROS runtime required."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from savo_mapping.models.location_bridge_status import (
    LocationBridgeStatus,
    make_location_bridge_status,
)
from savo_mapping.utils.diagnostics import (
    DiagnosticItem,
    make_disabled,
    make_error,
    make_ok,
    make_stale,
    make_warn,
)


# =============================================================================
# Location bridge result
# =============================================================================
@dataclass(frozen=True)
class LocationBridgeReadyResult:
    ok: bool
    enabled: bool = False
    stale: bool = True

    target_package: str = "savo_location"
    target_topic: str = "/savo_location/landmarks"
    status_topic: str = "/savo_mapping/location_bridge_status"

    package_available: bool = False
    topic_available: bool = False
    service_available: bool = False

    saved_count: int = 0
    confirmed_count: int = 0
    failed_count: int = 0

    msg_count: int = 0
    age_s: Optional[float] = None
    stale_timeout_s: float = 2.0

    status: LocationBridgeStatus = field(default_factory=LocationBridgeStatus)

    message: str = "Location bridge disabled."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "enabled": self.enabled,
            "stale": self.stale,
            "target_package": self.target_package,
            "target_topic": self.target_topic,
            "status_topic": self.status_topic,
            "package_available": self.package_available,
            "topic_available": self.topic_available,
            "service_available": self.service_available,
            "saved_count": self.saved_count,
            "confirmed_count": self.confirmed_count,
            "failed_count": self.failed_count,
            "msg_count": self.msg_count,
            "age_s": self.age_s,
            "stale_timeout_s": self.stale_timeout_s,
            "status": self.status.to_dict(),
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Check logic
# =============================================================================
def evaluate_location_bridge_ready(
    enabled: bool,
    package_available: bool = False,
    topic_available: bool = False,
    service_available: bool = False,
    saved_count: int = 0,
    confirmed_count: int = 0,
    failed_count: int = 0,
    msg_count: int = 0,
    age_s: Optional[float] = None,
    stale_timeout_s: float = 2.0,
    target_package: str = "savo_location",
    target_topic: str = "/savo_location/landmarks",
    status_topic: str = "/savo_mapping/location_bridge_status",
    extra: Optional[Dict[str, Any]] = None,
) -> LocationBridgeReadyResult:
    if not enabled:
        status = make_location_bridge_status(enabled=False)

        return LocationBridgeReadyResult(
            ok=False,
            enabled=False,
            stale=True,
            target_package=str(target_package),
            target_topic=str(target_topic),
            status_topic=str(status_topic),
            status=status,
            message="Location bridge disabled.",
            extra=dict(extra or {}),
        )

    messages = max(0, int(msg_count))
    saved = max(0, int(saved_count))
    confirmed = max(0, int(confirmed_count))
    failed = max(0, int(failed_count))
    timeout = max(0.0, float(stale_timeout_s))

    stale = True if age_s is None else float(age_s) > timeout

    failures: list[str] = []

    if stale:
        failures.append("stale")

    if not package_available:
        failures.append("package_not_available")

    if not topic_available and not service_available:
        failures.append("no_bridge_interface")

    if failed > 0:
        failures.append("has_failed_saves")

    ok = not failures

    if ok:
        message = "Location bridge ready."
    else:
        message = f"Location bridge not ready: {', '.join(failures)}."

    status = make_location_bridge_status(
        enabled=True,
        state="ready" if ok else "waiting_for_location_package",
        ok=ok,
        saved_count=saved,
        confirmed_count=confirmed,
        failed_count=failed,
        message=message,
        extra={
            "package_available": package_available,
            "topic_available": topic_available,
            "service_available": service_available,
            "msg_count": messages,
            "age_s": age_s,
            "stale_timeout_s": timeout,
            **dict(extra or {}),
        },
    )

    return LocationBridgeReadyResult(
        ok=ok,
        enabled=True,
        stale=stale,
        target_package=str(target_package),
        target_topic=str(target_topic),
        status_topic=str(status_topic),
        package_available=bool(package_available),
        topic_available=bool(topic_available),
        service_available=bool(service_available),
        saved_count=saved,
        confirmed_count=confirmed,
        failed_count=failed,
        msg_count=messages,
        age_s=age_s,
        stale_timeout_s=timeout,
        status=status,
        message=message,
        extra={
            "failures": failures,
            **dict(extra or {}),
        },
    )


def evaluate_location_bridge_from_status(
    status: LocationBridgeStatus,
    package_available: bool,
    topic_available: bool,
    service_available: bool,
    msg_count: int = 0,
    age_s: Optional[float] = None,
    stale_timeout_s: float = 2.0,
) -> LocationBridgeReadyResult:
    return evaluate_location_bridge_ready(
        enabled=status.enabled,
        package_available=package_available,
        topic_available=topic_available,
        service_available=service_available,
        saved_count=status.saved_count,
        confirmed_count=status.confirmed_count,
        failed_count=status.failed_count,
        msg_count=msg_count,
        age_s=age_s,
        stale_timeout_s=stale_timeout_s,
        target_package=status.target_package,
        target_topic=status.target_topic,
        status_topic=status.status_topic,
        extra={
            "bridge_state": status.state,
            "bridge_ok": status.ok,
        },
    )


def location_bridge_result_to_diagnostic(
    result: LocationBridgeReadyResult,
    required: bool = False,
) -> DiagnosticItem:
    values = result.to_dict()

    if not result.enabled:
        return make_disabled(
            "location_bridge",
            result.message,
            required=required,
            values=values,
        )

    if result.ok:
        return make_ok(
            "location_bridge",
            result.message,
            required=required,
            values=values,
        )

    if result.stale:
        return make_stale(
            "location_bridge",
            result.message,
            required=required,
            values=values,
        )

    if required:
        return make_error(
            "location_bridge",
            result.message,
            required=required,
            values=values,
        )

    return make_warn(
        "location_bridge",
        result.message,
        required=required,
        values=values,
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    disabled = evaluate_location_bridge_ready(enabled=False)

    ready = evaluate_location_bridge_ready(
        enabled=True,
        package_available=True,
        topic_available=True,
        service_available=False,
        saved_count=2,
        confirmed_count=1,
        failed_count=0,
        msg_count=5,
        age_s=0.1,
    )

    print(disabled.to_json(indent=2))
    print(ready.to_json(indent=2))
    print(location_bridge_result_to_diagnostic(ready).to_dict())


if __name__ == "__main__":
    main()


__all__ = [
    "LocationBridgeReadyResult",
    "evaluate_location_bridge_ready",
    "evaluate_location_bridge_from_status",
    "location_bridge_result_to_diagnostic",
    "main",
]