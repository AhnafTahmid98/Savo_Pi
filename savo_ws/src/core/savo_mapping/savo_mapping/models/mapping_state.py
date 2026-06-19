#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Mapping status model. No ROS imports."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from savo_mapping.models.mapping_mode import MappingMode, require_valid_mapping_mode
from savo_mapping.models.readiness_state import (
    MappingReadinessState,
    build_default_not_ready_state,
)


# =============================================================================
# Mapping status
# =============================================================================
@dataclass(frozen=True)
class MappingStatus:
    mode: str = MappingMode.IDLE.value
    ready: bool = False
    degraded: bool = False
    active: bool = False
    message: str = "Mapping is idle."
    timestamp_s: float = field(default_factory=time.time)
    readiness: MappingReadinessState = field(default_factory=build_default_not_ready_state)
    map_name: Optional[str] = None
    session_id: Optional[str] = None
    extra: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        require_valid_mapping_mode(self.mode)

    def to_dict(self) -> dict:
        return {
            "mode": self.mode,
            "ready": self.ready,
            "degraded": self.degraded,
            "active": self.active,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "map_name": self.map_name,
            "session_id": self.session_id,
            "readiness": self.readiness.to_dict(),
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)

    def with_message(self, message: str) -> "MappingStatus":
        return MappingStatus(
            mode=self.mode,
            ready=self.ready,
            degraded=self.degraded,
            active=self.active,
            message=message,
            timestamp_s=time.time(),
            readiness=self.readiness,
            map_name=self.map_name,
            session_id=self.session_id,
            extra=dict(self.extra),
        )

    def with_readiness(self, readiness: MappingReadinessState) -> "MappingStatus":
        return MappingStatus(
            mode=self.mode,
            ready=readiness.ready,
            degraded=readiness.degraded,
            active=self.active,
            message=self.message,
            timestamp_s=time.time(),
            readiness=readiness,
            map_name=self.map_name,
            session_id=self.session_id,
            extra=dict(self.extra),
        )


# =============================================================================
# Builders
# =============================================================================
def make_mapping_status(
    mode: str = MappingMode.IDLE.value,
    readiness: Optional[MappingReadinessState] = None,
    active: bool = False,
    message: str = "",
    map_name: Optional[str] = None,
    session_id: Optional[str] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> MappingStatus:
    state = readiness if readiness is not None else build_default_not_ready_state()

    if not message:
        message = "Mapping ready." if state.ready else "Mapping not ready."

    return MappingStatus(
        mode=require_valid_mapping_mode(mode),
        ready=state.ready,
        degraded=state.degraded,
        active=active,
        message=message,
        readiness=state,
        map_name=map_name,
        session_id=session_id,
        extra=dict(extra or {}),
    )


def make_idle_status(message: str = "Mapping is idle.") -> MappingStatus:
    return make_mapping_status(
        mode=MappingMode.IDLE.value,
        active=False,
        message=message,
    )


def make_manual_mapping_status(
    readiness: MappingReadinessState,
    map_name: Optional[str] = None,
    session_id: Optional[str] = None,
) -> MappingStatus:
    return make_mapping_status(
        mode=MappingMode.MANUAL_MAPPING.value,
        readiness=readiness,
        active=True,
        message="Manual mapping is active." if readiness.ready else "Manual mapping is not ready.",
        map_name=map_name,
        session_id=session_id,
    )


def make_autonomous_mapping_status(
    readiness: MappingReadinessState,
    map_name: Optional[str] = None,
    session_id: Optional[str] = None,
) -> MappingStatus:
    return make_mapping_status(
        mode=MappingMode.AUTONOMOUS_MAPPING.value,
        readiness=readiness,
        active=True,
        message=(
            "Autonomous mapping is active."
            if readiness.ready
            else "Autonomous mapping is not ready."
        ),
        map_name=map_name,
        session_id=session_id,
    )


def mapping_status_from_dict(data: Dict[str, Any]) -> MappingStatus:
    return MappingStatus(
        mode=require_valid_mapping_mode(str(data.get("mode", MappingMode.IDLE.value))),
        ready=bool(data.get("ready", False)),
        degraded=bool(data.get("degraded", False)),
        active=bool(data.get("active", False)),
        message=str(data.get("message", "")),
        timestamp_s=float(data.get("timestamp_s", time.time())),
        map_name=data.get("map_name"),
        session_id=data.get("session_id"),
        extra=dict(data.get("extra", {})),
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    status = make_idle_status()
    print(status.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "MappingStatus",
    "make_mapping_status",
    "make_idle_status",
    "make_manual_mapping_status",
    "make_autonomous_mapping_status",
    "mapping_status_from_dict",
    "main",
]