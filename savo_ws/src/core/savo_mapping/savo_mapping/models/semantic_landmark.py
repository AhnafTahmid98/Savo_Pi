#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Semantic landmark model for mapping-time location records. No ROS imports."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional


# =============================================================================
# Landmark state
# =============================================================================
class LandmarkState(str, Enum):
    CANDIDATE = "candidate"
    CONFIRMED = "confirmed"
    REJECTED = "rejected"
    ARCHIVED = "archived"

    @classmethod
    def values(cls) -> tuple[str, ...]:
        return tuple(state.value for state in cls)

    @classmethod
    def from_string(cls, value: str) -> "LandmarkState":
        normalized = normalize_landmark_state(value)

        for state in cls:
            if state.value == normalized:
                return state

        raise ValueError(f"Unsupported landmark state: {value!r}")


# =============================================================================
# Helpers
# =============================================================================
def normalize_landmark_state(value: str) -> str:
    state = str(value).strip().lower().replace("-", "_").replace(" ", "_")

    if not state:
        raise ValueError("Landmark state cannot be empty.")

    return state


def require_valid_landmark_state(value: str) -> str:
    return LandmarkState.from_string(value).value


def sanitize_landmark_label(label: str) -> str:
    value = str(label).strip()

    if not value:
        raise ValueError("Landmark label cannot be empty.")

    return " ".join(value.split())


def make_landmark_key(label: str) -> str:
    value = sanitize_landmark_label(label).lower()
    clean = []

    for char in value:
        if char.isalnum():
            clean.append(char)
        elif char in ("-", "_", " "):
            clean.append("_")

    key = "".join(clean)

    while "__" in key:
        key = key.replace("__", "_")

    key = key.strip("_")

    if not key:
        raise ValueError("Landmark key cannot be empty.")

    return key


# =============================================================================
# Landmark pose
# =============================================================================
@dataclass(frozen=True)
class LandmarkPose:
    x: float
    y: float
    yaw: float = 0.0
    z: float = 0.0
    frame_id: str = "map"

    def to_dict(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw": self.yaw,
            "frame_id": self.frame_id,
        }


# =============================================================================
# Semantic landmark
# =============================================================================
@dataclass(frozen=True)
class SemanticLandmark:
    label: str
    pose: LandmarkPose

    key: str = ""
    state: str = LandmarkState.CANDIDATE.value
    source: str = "mapping"
    tag_id: Optional[int] = None
    confidence: float = 0.0

    map_name: Optional[str] = None
    session_id: Optional[str] = None

    created_at_s: float = field(default_factory=time.time)
    updated_at_s: float = field(default_factory=time.time)

    notes: str = ""
    extra: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        sanitize_landmark_label(self.label)
        require_valid_landmark_state(self.state)

        if not self.key:
            object.__setattr__(self, "key", make_landmark_key(self.label))

    @property
    def confirmed(self) -> bool:
        return self.state == LandmarkState.CONFIRMED.value

    def to_dict(self) -> dict:
        return {
            "key": self.key,
            "label": self.label,
            "state": self.state,
            "source": self.source,
            "tag_id": self.tag_id,
            "confidence": self.confidence,
            "pose": self.pose.to_dict(),
            "map_name": self.map_name,
            "session_id": self.session_id,
            "created_at_s": self.created_at_s,
            "updated_at_s": self.updated_at_s,
            "confirmed": self.confirmed,
            "notes": self.notes,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)

    def with_state(self, state: str, notes: str = "") -> "SemanticLandmark":
        return SemanticLandmark(
            key=self.key,
            label=self.label,
            pose=self.pose,
            state=require_valid_landmark_state(state),
            source=self.source,
            tag_id=self.tag_id,
            confidence=self.confidence,
            map_name=self.map_name,
            session_id=self.session_id,
            created_at_s=self.created_at_s,
            updated_at_s=time.time(),
            notes=notes or self.notes,
            extra=dict(self.extra),
        )


# =============================================================================
# Builders
# =============================================================================
def make_semantic_landmark(
    label: str,
    x: float,
    y: float,
    yaw: float = 0.0,
    z: float = 0.0,
    frame_id: str = "map",
    state: str = LandmarkState.CANDIDATE.value,
    source: str = "mapping",
    tag_id: Optional[int] = None,
    confidence: float = 0.0,
    map_name: Optional[str] = None,
    session_id: Optional[str] = None,
    notes: str = "",
    extra: Optional[Dict[str, Any]] = None,
) -> SemanticLandmark:
    return SemanticLandmark(
        label=sanitize_landmark_label(label),
        pose=LandmarkPose(
            x=float(x),
            y=float(y),
            z=float(z),
            yaw=float(yaw),
            frame_id=str(frame_id).strip() or "map",
        ),
        state=require_valid_landmark_state(state),
        source=str(source).strip() or "mapping",
        tag_id=tag_id,
        confidence=max(0.0, min(1.0, float(confidence))),
        map_name=map_name,
        session_id=session_id,
        notes=str(notes),
        extra=dict(extra or {}),
    )


def semantic_landmark_from_dict(data: Dict[str, Any]) -> SemanticLandmark:
    pose_data = dict(data.get("pose", {}))

    pose = LandmarkPose(
        x=float(pose_data.get("x", 0.0)),
        y=float(pose_data.get("y", 0.0)),
        z=float(pose_data.get("z", 0.0)),
        yaw=float(pose_data.get("yaw", 0.0)),
        frame_id=str(pose_data.get("frame_id", "map")),
    )

    label = sanitize_landmark_label(str(data.get("label", "unknown")))

    return SemanticLandmark(
        key=str(data.get("key") or make_landmark_key(label)),
        label=label,
        pose=pose,
        state=require_valid_landmark_state(
            str(data.get("state", LandmarkState.CANDIDATE.value))
        ),
        source=str(data.get("source", "mapping")),
        tag_id=data.get("tag_id"),
        confidence=max(0.0, min(1.0, float(data.get("confidence", 0.0)))),
        map_name=data.get("map_name"),
        session_id=data.get("session_id"),
        created_at_s=float(data.get("created_at_s", time.time())),
        updated_at_s=float(data.get("updated_at_s", time.time())),
        notes=str(data.get("notes", "")),
        extra=dict(data.get("extra", {})),
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    landmark = make_semantic_landmark(
        label="A201",
        x=4.25,
        y=8.70,
        yaw=1.57,
        tag_id=21,
        confidence=0.92,
        map_name="savonia_campus_heart",
        notes="Detected from AprilTag during mapping.",
    )

    print(landmark.to_json(indent=2))
    print(landmark.with_state(LandmarkState.CONFIRMED.value).to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "LandmarkState",
    "LandmarkPose",
    "SemanticLandmark",
    "normalize_landmark_state",
    "require_valid_landmark_state",
    "sanitize_landmark_label",
    "make_landmark_key",
    "make_semantic_landmark",
    "semantic_landmark_from_dict",
    "main",
]