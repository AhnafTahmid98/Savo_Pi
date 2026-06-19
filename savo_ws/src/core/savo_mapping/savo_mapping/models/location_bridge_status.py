#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Location bridge status model. No ROS imports."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional


# =============================================================================
# Bridge states
# =============================================================================
class LocationBridgeState(str, Enum):
    DISABLED = "disabled"
    IDLE = "idle"
    WAITING_FOR_LOCATION_PACKAGE = "waiting_for_location_package"
    READY = "ready"
    SAVING_CANDIDATE = "saving_candidate"
    CONFIRMING_LOCATION = "confirming_location"
    SAVED = "saved"
    FAILED = "failed"

    @classmethod
    def values(cls) -> tuple[str, ...]:
        return tuple(state.value for state in cls)

    @classmethod
    def from_string(cls, value: str) -> "LocationBridgeState":
        normalized = normalize_location_bridge_state(value)

        for state in cls:
            if state.value == normalized:
                return state

        raise ValueError(f"Unsupported location bridge state: {value!r}")


# =============================================================================
# Helpers
# =============================================================================
def normalize_location_bridge_state(value: str) -> str:
    state = str(value).strip().lower().replace("-", "_").replace(" ", "_")

    if not state:
        raise ValueError("Location bridge state cannot be empty.")

    return state


def require_valid_location_bridge_state(value: str) -> str:
    return LocationBridgeState.from_string(value).value


def is_valid_location_bridge_state(value: str) -> bool:
    try:
        LocationBridgeState.from_string(value)
        return True
    except ValueError:
        return False


# =============================================================================
# Bridge result
# =============================================================================
@dataclass(frozen=True)
class LocationBridgeResult:
    landmark_key: str
    label: str
    saved: bool = False
    confirmed: bool = False
    location_id: Optional[str] = None
    message: str = ""

    def to_dict(self) -> dict:
        return {
            "landmark_key": self.landmark_key,
            "label": self.label,
            "saved": self.saved,
            "confirmed": self.confirmed,
            "location_id": self.location_id,
            "message": self.message,
        }


# =============================================================================
# Bridge status
# =============================================================================
@dataclass(frozen=True)
class LocationBridgeStatus:
    enabled: bool = False
    ok: bool = False
    state: str = LocationBridgeState.DISABLED.value

    target_package: str = "savo_location"
    target_topic: str = "/savo_location/landmarks"
    status_topic: str = "/savo_mapping/location_bridge_status"

    saved_count: int = 0
    confirmed_count: int = 0
    failed_count: int = 0

    last_result: Optional[LocationBridgeResult] = None

    message: str = "Location bridge disabled."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        require_valid_location_bridge_state(self.state)

    def to_dict(self) -> dict:
        return {
            "enabled": self.enabled,
            "ok": self.ok,
            "state": self.state,
            "target_package": self.target_package,
            "target_topic": self.target_topic,
            "status_topic": self.status_topic,
            "saved_count": self.saved_count,
            "confirmed_count": self.confirmed_count,
            "failed_count": self.failed_count,
            "last_result": (
                self.last_result.to_dict()
                if self.last_result is not None
                else None
            ),
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Builders
# =============================================================================
def make_location_bridge_disabled_status() -> LocationBridgeStatus:
    return LocationBridgeStatus(
        enabled=False,
        ok=False,
        state=LocationBridgeState.DISABLED.value,
        message="Location bridge disabled.",
    )


def make_location_bridge_status(
    enabled: bool,
    state: str = LocationBridgeState.IDLE.value,
    ok: bool = False,
    saved_count: int = 0,
    confirmed_count: int = 0,
    failed_count: int = 0,
    last_result: Optional[LocationBridgeResult] = None,
    message: str = "",
    extra: Optional[Dict[str, Any]] = None,
) -> LocationBridgeStatus:
    if not enabled:
        return make_location_bridge_disabled_status()

    valid_state = require_valid_location_bridge_state(state)

    if not message:
        message = f"Location bridge state: {valid_state}."

    return LocationBridgeStatus(
        enabled=True,
        ok=bool(ok),
        state=valid_state,
        saved_count=max(0, int(saved_count)),
        confirmed_count=max(0, int(confirmed_count)),
        failed_count=max(0, int(failed_count)),
        last_result=last_result,
        message=message,
        extra=dict(extra or {}),
    )


def make_location_saved_status(
    landmark_key: str,
    label: str,
    location_id: Optional[str] = None,
    saved_count: int = 1,
    confirmed_count: int = 0,
) -> LocationBridgeStatus:
    result = LocationBridgeResult(
        landmark_key=str(landmark_key),
        label=str(label),
        saved=True,
        confirmed=False,
        location_id=location_id,
        message="Location candidate saved.",
    )

    return make_location_bridge_status(
        enabled=True,
        state=LocationBridgeState.SAVED.value,
        ok=True,
        saved_count=saved_count,
        confirmed_count=confirmed_count,
        last_result=result,
        message="Location candidate saved to savo_location.",
    )


def make_location_confirmed_status(
    landmark_key: str,
    label: str,
    location_id: Optional[str] = None,
    saved_count: int = 1,
    confirmed_count: int = 1,
) -> LocationBridgeStatus:
    result = LocationBridgeResult(
        landmark_key=str(landmark_key),
        label=str(label),
        saved=True,
        confirmed=True,
        location_id=location_id,
        message="Location confirmed.",
    )

    return make_location_bridge_status(
        enabled=True,
        state=LocationBridgeState.CONFIRMING_LOCATION.value,
        ok=True,
        saved_count=saved_count,
        confirmed_count=confirmed_count,
        last_result=result,
        message="Location confirmed by semantic landmark.",
    )


def location_bridge_status_from_dict(data: Dict[str, Any]) -> LocationBridgeStatus:
    result_data = data.get("last_result")
    result = None

    if isinstance(result_data, dict):
        result = LocationBridgeResult(
            landmark_key=str(result_data.get("landmark_key", "")),
            label=str(result_data.get("label", "")),
            saved=bool(result_data.get("saved", False)),
            confirmed=bool(result_data.get("confirmed", False)),
            location_id=result_data.get("location_id"),
            message=str(result_data.get("message", "")),
        )

    return LocationBridgeStatus(
        enabled=bool(data.get("enabled", False)),
        ok=bool(data.get("ok", False)),
        state=require_valid_location_bridge_state(
            str(data.get("state", LocationBridgeState.DISABLED.value))
        ),
        target_package=str(data.get("target_package", "savo_location")),
        target_topic=str(data.get("target_topic", "/savo_location/landmarks")),
        status_topic=str(data.get("status_topic", "/savo_mapping/location_bridge_status")),
        saved_count=max(0, int(data.get("saved_count", 0))),
        confirmed_count=max(0, int(data.get("confirmed_count", 0))),
        failed_count=max(0, int(data.get("failed_count", 0))),
        last_result=result,
        message=str(data.get("message", "")),
        timestamp_s=float(data.get("timestamp_s", time.time())),
        extra=dict(data.get("extra", {})),
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    print(make_location_bridge_disabled_status().to_json(indent=2))

    print(
        make_location_saved_status(
            landmark_key="a201",
            label="A201",
            location_id="loc_a201",
        ).to_json(indent=2)
    )


if __name__ == "__main__":
    main()


__all__ = [
    "LocationBridgeState",
    "LocationBridgeResult",
    "LocationBridgeStatus",
    "normalize_location_bridge_state",
    "require_valid_location_bridge_state",
    "is_valid_location_bridge_state",
    "make_location_bridge_disabled_status",
    "make_location_bridge_status",
    "make_location_saved_status",
    "make_location_confirmed_status",
    "location_bridge_status_from_dict",
    "main",
]