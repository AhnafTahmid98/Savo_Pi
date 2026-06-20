#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""AprilTag mapping helpers for Robot Savo semantic mapping."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Optional, Sequence

from savo_mapping.semantic.location_candidate import (
    LocationCandidate,
    LocationPose,
    make_apriltag_location_candidate,
    make_location_key,
)


@dataclass(frozen=True)
class AprilTagLabel:
    tag_id: int
    label: str
    area_type: str = "apriltag_location"
    priority: int = 0
    notes: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.tag_id < 0:
            raise ValueError("AprilTag id must be non-negative.")

        if not self.label.strip():
            raise ValueError("AprilTag label cannot be empty.")

    @property
    def key(self) -> str:
        return make_location_key(self.label)

    def to_dict(self) -> dict[str, Any]:
        return {
            "tag_id": self.tag_id,
            "key": self.key,
            "label": self.label,
            "area_type": self.area_type,
            "priority": self.priority,
            "notes": self.notes,
            "metadata": dict(self.metadata),
        }


@dataclass(frozen=True)
class AprilTagObservation:
    tag_id: int
    pose: LocationPose
    confidence: float = 1.0
    family: str = "tag36h11"
    size_m: Optional[float] = None
    timestamp_s: float = field(default_factory=time.time)
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.tag_id < 0:
            raise ValueError("AprilTag id must be non-negative.")

        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("AprilTag confidence must be between 0 and 1.")

    def to_dict(self) -> dict[str, Any]:
        return {
            "tag_id": self.tag_id,
            "pose": self.pose.to_dict(),
            "confidence": self.confidence,
            "family": self.family,
            "size_m": self.size_m,
            "timestamp_s": self.timestamp_s,
            "metadata": dict(self.metadata),
        }


@dataclass(frozen=True)
class AprilTagMappingResult:
    ok: bool
    observation: AprilTagObservation
    label: Optional[AprilTagLabel] = None
    candidate: Optional[LocationCandidate] = None
    message: str = "AprilTag mapping not run."
    timestamp_s: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "observation": self.observation.to_dict(),
            "label": self.label.to_dict() if self.label is not None else None,
            "candidate": (
                self.candidate.to_dict()
                if self.candidate is not None
                else None
            ),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


@dataclass(frozen=True)
class AprilTagMapper:
    labels: tuple[AprilTagLabel, ...] = ()
    min_confidence: float = 0.30
    allow_unknown_tags: bool = False
    unknown_label_prefix: str = "Tag"

    def __post_init__(self) -> None:
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be between 0 and 1.")

        _ensure_unique_tag_ids(self.labels)

    def find_label(self, tag_id: int) -> Optional[AprilTagLabel]:
        for label in self.labels:
            if label.tag_id == int(tag_id):
                return label

        return None

    def with_label(self, label: AprilTagLabel) -> "AprilTagMapper":
        labels = list(self.labels)
        replaced = False

        for index, item in enumerate(labels):
            if item.tag_id == label.tag_id:
                labels[index] = label
                replaced = True
                break

        if not replaced:
            labels.append(label)

        return replace(self, labels=tuple(labels))

    def map_observation(
        self,
        observation: AprilTagObservation,
    ) -> AprilTagMappingResult:
        if observation.confidence < self.min_confidence:
            return AprilTagMappingResult(
                ok=False,
                observation=observation,
                message="AprilTag confidence too low.",
            )

        label = self.find_label(observation.tag_id)

        if label is None and not self.allow_unknown_tags:
            return AprilTagMappingResult(
                ok=False,
                observation=observation,
                message=f"AprilTag id {observation.tag_id} is not registered.",
            )

        if label is None:
            label = AprilTagLabel(
                tag_id=observation.tag_id,
                label=f"{self.unknown_label_prefix} {observation.tag_id}",
                metadata={"auto_generated": True},
            )

        candidate = make_apriltag_location_candidate(
            tag_id=observation.tag_id,
            label=label.label,
            x=observation.pose.x,
            y=observation.pose.y,
            yaw=observation.pose.yaw,
            frame_id=observation.pose.frame_id,
            confidence=observation.confidence,
            area_type=label.area_type,
            notes=label.notes,
        )

        candidate = replace(
            candidate,
            metadata={
                **candidate.metadata,
                **label.metadata,
                "family": observation.family,
                "size_m": observation.size_m,
                "observation_timestamp_s": observation.timestamp_s,
                "priority": label.priority,
                "observation": observation.metadata,
            },
        )

        return AprilTagMappingResult(
            ok=True,
            observation=observation,
            label=label,
            candidate=candidate,
            message="AprilTag mapped to location candidate.",
        )

    def map_observations(
        self,
        observations: Sequence[AprilTagObservation],
    ) -> tuple[AprilTagMappingResult, ...]:
        return tuple(self.map_observation(item) for item in observations)

    def to_dict(self) -> dict[str, Any]:
        return {
            "min_confidence": self.min_confidence,
            "allow_unknown_tags": self.allow_unknown_tags,
            "unknown_label_prefix": self.unknown_label_prefix,
            "labels": [label.to_dict() for label in self.labels],
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


def make_apriltag_observation(
    *,
    tag_id: int,
    x: float,
    y: float,
    yaw: float = 0.0,
    frame_id: str = "map",
    confidence: float = 1.0,
    family: str = "tag36h11",
    size_m: Optional[float] = None,
    metadata: Optional[dict[str, Any]] = None,
) -> AprilTagObservation:
    return AprilTagObservation(
        tag_id=int(tag_id),
        pose=LocationPose(
            x=float(x),
            y=float(y),
            yaw=float(yaw),
            frame_id=str(frame_id),
        ),
        confidence=float(confidence),
        family=str(family),
        size_m=size_m,
        metadata=dict(metadata or {}),
    )


def apriltag_label_from_dict(data: dict[str, Any]) -> AprilTagLabel:
    return AprilTagLabel(
        tag_id=int(data["tag_id"]),
        label=str(data["label"]),
        area_type=str(data.get("area_type", "apriltag_location")),
        priority=int(data.get("priority", 0)),
        notes=str(data.get("notes", "")),
        metadata=dict(data.get("metadata", {})),
    )


def apriltag_observation_from_dict(data: dict[str, Any]) -> AprilTagObservation:
    pose_data = data.get("pose", {})

    return AprilTagObservation(
        tag_id=int(data["tag_id"]),
        pose=LocationPose(
            x=float(pose_data.get("x", 0.0)),
            y=float(pose_data.get("y", 0.0)),
            yaw=float(pose_data.get("yaw", 0.0)),
            frame_id=str(pose_data.get("frame_id", "map")),
        ),
        confidence=float(data.get("confidence", 1.0)),
        family=str(data.get("family", "tag36h11")),
        size_m=None if data.get("size_m") is None else float(data["size_m"]),
        timestamp_s=float(data.get("timestamp_s", time.time())),
        metadata=dict(data.get("metadata", {})),
    )


def apriltag_mapper_from_dict(data: dict[str, Any]) -> AprilTagMapper:
    return AprilTagMapper(
        labels=tuple(
            apriltag_label_from_dict(item)
            for item in data.get("labels", ())
        ),
        min_confidence=float(data.get("min_confidence", 0.30)),
        allow_unknown_tags=bool(data.get("allow_unknown_tags", False)),
        unknown_label_prefix=str(data.get("unknown_label_prefix", "Tag")),
    )


def load_apriltag_mapper(path: str | Path) -> AprilTagMapper:
    mapper_path = Path(path)

    if not mapper_path.exists():
        return AprilTagMapper()

    data = json.loads(mapper_path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise ValueError("AprilTag mapper JSON root must be an object.")

    return apriltag_mapper_from_dict(data)


def save_apriltag_mapper(
    path: str | Path,
    mapper: AprilTagMapper,
    *,
    indent: int = 2,
) -> Path:
    mapper_path = Path(path)
    mapper_path.parent.mkdir(parents=True, exist_ok=True)
    mapper_path.write_text(mapper.to_json(indent=indent) + "\n", encoding="utf-8")

    return mapper_path


def _ensure_unique_tag_ids(labels: Sequence[AprilTagLabel]) -> None:
    seen: set[int] = set()

    for label in labels:
        if label.tag_id in seen:
            raise ValueError(f"Duplicate AprilTag id: {label.tag_id}")

        seen.add(label.tag_id)


def main() -> None:
    mapper = AprilTagMapper(
        labels=(
            AprilTagLabel(
                tag_id=21,
                label="A201",
                area_type="room_entrance",
                priority=3,
            ),
        )
    )

    observation = make_apriltag_observation(
        tag_id=21,
        x=4.2,
        y=8.7,
        confidence=0.95,
    )

    result = mapper.map_observation(observation)

    print(result.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "AprilTagLabel",
    "AprilTagMapper",
    "AprilTagMappingResult",
    "AprilTagObservation",
    "apriltag_label_from_dict",
    "apriltag_mapper_from_dict",
    "apriltag_observation_from_dict",
    "load_apriltag_mapper",
    "main",
    "make_apriltag_observation",
    "save_apriltag_mapper",
]