#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""AprilTag readiness check. No ROS runtime required."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from savo_mapping.constants import TOPIC_APRILTAG_DETECTIONS
from savo_mapping.models.apriltag_status import (
    AprilTagObservation,
    AprilTagStatus,
    make_apriltag_status,
)
from savo_mapping.utils.diagnostics import (
    DiagnosticItem,
    make_disabled,
    make_ok,
    make_stale,
    make_warn,
)


# =============================================================================
# AprilTag readiness result
# =============================================================================
@dataclass(frozen=True)
class AprilTagReadyResult:
    ok: bool
    enabled: bool = False
    stale: bool = True

    topic: str = TOPIC_APRILTAG_DETECTIONS
    msg_count: int = 0
    detection_count: int = 0
    unique_tag_count: int = 0

    last_tag_id: Optional[int] = None
    last_label: Optional[str] = None
    age_s: Optional[float] = None
    stale_timeout_s: float = 1.0

    observations: tuple[AprilTagObservation, ...] = field(default_factory=tuple)
    status: AprilTagStatus = field(default_factory=AprilTagStatus)

    message: str = "AprilTag detection disabled."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "enabled": self.enabled,
            "stale": self.stale,
            "topic": self.topic,
            "msg_count": self.msg_count,
            "detection_count": self.detection_count,
            "unique_tag_count": self.unique_tag_count,
            "last_tag_id": self.last_tag_id,
            "last_label": self.last_label,
            "age_s": self.age_s,
            "stale_timeout_s": self.stale_timeout_s,
            "observations": [
                observation.to_dict()
                for observation in self.observations
            ],
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
def evaluate_apriltag_ready(
    enabled: bool,
    msg_count: int = 0,
    detection_count: int = 0,
    unique_tag_count: int = 0,
    last_tag_id: Optional[int] = None,
    last_label: Optional[str] = None,
    age_s: Optional[float] = None,
    stale_timeout_s: float = 1.0,
    topic: str = TOPIC_APRILTAG_DETECTIONS,
    observations: Optional[list[AprilTagObservation]] = None,
    min_detections: int = 1,
    require_known_label: bool = False,
    extra: Optional[Dict[str, Any]] = None,
) -> AprilTagReadyResult:
    if not enabled:
        status = make_apriltag_status(enabled=False, topic=topic)

        return AprilTagReadyResult(
            ok=False,
            enabled=False,
            stale=True,
            topic=str(topic),
            status=status,
            message="AprilTag detection disabled.",
            extra=dict(extra or {}),
        )

    messages = max(0, int(msg_count))
    detections = max(0, int(detection_count))
    unique_tags = max(0, int(unique_tag_count))
    timeout = max(0.0, float(stale_timeout_s))

    stale = True if age_s is None else float(age_s) > timeout

    failures: list[str] = []

    if messages <= 0:
        failures.append("no_messages")

    if stale:
        failures.append("stale")

    if detections < min_detections:
        failures.append("not_enough_detections")

    if unique_tags <= 0:
        failures.append("no_unique_tags")

    if require_known_label and not last_label:
        failures.append("missing_known_label")

    ok = not failures

    if ok:
        message = "AprilTag detection ready."
    else:
        message = f"AprilTag detection not ready: {', '.join(failures)}."

    status = make_apriltag_status(
        enabled=True,
        msg_count=messages,
        detection_count=detections,
        unique_tag_count=unique_tags,
        last_tag_id=last_tag_id,
        last_label=last_label,
        age_s=age_s,
        stale_timeout_s=timeout,
        topic=topic,
        observations=observations,
        extra={
            "min_detections": min_detections,
            "require_known_label": require_known_label,
            **dict(extra or {}),
        },
    )

    return AprilTagReadyResult(
        ok=ok,
        enabled=True,
        stale=stale,
        topic=str(topic),
        msg_count=messages,
        detection_count=detections,
        unique_tag_count=unique_tags,
        last_tag_id=last_tag_id,
        last_label=last_label,
        age_s=age_s,
        stale_timeout_s=timeout,
        observations=tuple(observations or ()),
        status=status,
        message=message,
        extra={
            "failures": failures,
            "min_detections": min_detections,
            "require_known_label": require_known_label,
            **dict(extra or {}),
        },
    )


def evaluate_apriltag_observations(
    observations: list[AprilTagObservation],
    enabled: bool,
    msg_count: int,
    age_s: Optional[float],
    stale_timeout_s: float = 1.0,
    topic: str = TOPIC_APRILTAG_DETECTIONS,
    require_known_label: bool = False,
) -> AprilTagReadyResult:
    tag_ids = {
        observation.tag_id
        for observation in observations
        if observation.tag_id >= 0
    }

    last = observations[-1] if observations else None

    return evaluate_apriltag_ready(
        enabled=enabled,
        msg_count=msg_count,
        detection_count=len(observations),
        unique_tag_count=len(tag_ids),
        last_tag_id=last.tag_id if last else None,
        last_label=last.label if last else None,
        age_s=age_s,
        stale_timeout_s=stale_timeout_s,
        topic=topic,
        observations=observations,
        require_known_label=require_known_label,
    )


def apriltag_result_to_diagnostic(
    result: AprilTagReadyResult,
    required: bool = False,
) -> DiagnosticItem:
    values = result.to_dict()

    if not result.enabled:
        return make_disabled(
            "apriltag",
            result.message,
            required=required,
            values=values,
        )

    if result.ok:
        return make_ok(
            "apriltag",
            result.message,
            required=required,
            values=values,
        )

    if result.stale:
        return make_stale(
            "apriltag",
            result.message,
            required=required,
            values=values,
        )

    return make_warn(
        "apriltag",
        result.message,
        required=required,
        values=values,
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    disabled = evaluate_apriltag_ready(enabled=False)

    observation = AprilTagObservation(
        tag_id=21,
        family="tag36h11",
        frame_id="camera_color_optical_frame",
        x=4.25,
        y=8.70,
        yaw=1.57,
        confidence=0.92,
        label="A201",
    )

    ready = evaluate_apriltag_observations(
        observations=[observation],
        enabled=True,
        msg_count=3,
        age_s=0.05,
        require_known_label=True,
    )

    print(disabled.to_json(indent=2))
    print(ready.to_json(indent=2))
    print(apriltag_result_to_diagnostic(ready).to_dict())


if __name__ == "__main__":
    main()


__all__ = [
    "AprilTagReadyResult",
    "evaluate_apriltag_ready",
    "evaluate_apriltag_observations",
    "apriltag_result_to_diagnostic",
    "main",
]