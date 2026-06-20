#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""AprilTag observation models for Robot Savo semantic mapping."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional, Sequence

from savo_mapping.semantic.location_candidate import LocationPose


DEFAULT_TAG_FAMILY = "tag36h11"


@dataclass(frozen=True)
class AprilTagObservation:
    tag_id: int
    pose: LocationPose
    confidence: float = 1.0
    family: str = DEFAULT_TAG_FAMILY
    size_m: Optional[float] = None
    timestamp_s: float = field(default_factory=time.time)
    source_frame_id: str = ""
    camera_frame_id: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.tag_id < 0:
            raise ValueError("AprilTag id must be non-negative.")

        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("AprilTag confidence must be between 0 and 1.")

        if self.size_m is not None and self.size_m <= 0.0:
            raise ValueError("AprilTag size_m must be positive or None.")

        if not self.family.strip():
            raise ValueError("AprilTag family cannot be empty.")

    @property
    def map_frame_id(self) -> str:
        return self.pose.frame_id

    def fresh(self, *, now_s: Optional[float] = None, max_age_s: float = 2.0) -> bool:
        current_s = time.time() if now_s is None else float(now_s)
        return current_s - self.timestamp_s <= max_age_s

    def to_dict(self) -> dict[str, Any]:
        return {
            "tag_id": self.tag_id,
            "pose": self.pose.to_dict(),
            "confidence": self.confidence,
            "family": self.family,
            "size_m": self.size_m,
            "timestamp_s": self.timestamp_s,
            "source_frame_id": self.source_frame_id,
            "camera_frame_id": self.camera_frame_id,
            "metadata": dict(self.metadata),
            "map_frame_id": self.map_frame_id,
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


@dataclass(frozen=True)
class AprilTagObservationBatch:
    observations: tuple[AprilTagObservation, ...] = ()
    source: str = "apriltag_detector"
    timestamp_s: float = field(default_factory=time.time)
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def count(self) -> int:
        return len(self.observations)

    @property
    def empty(self) -> bool:
        return not self.observations

    def filter_by_confidence(
        self,
        *,
        min_confidence: float,
    ) -> "AprilTagObservationBatch":
        return AprilTagObservationBatch(
            observations=tuple(
                observation
                for observation in self.observations
                if observation.confidence >= min_confidence
            ),
            source=self.source,
            timestamp_s=self.timestamp_s,
            metadata=dict(self.metadata),
        )

    def latest_by_tag_id(self) -> dict[int, AprilTagObservation]:
        latest: dict[int, AprilTagObservation] = {}

        for observation in self.observations:
            current = latest.get(observation.tag_id)

            if current is None or observation.timestamp_s >= current.timestamp_s:
                latest[observation.tag_id] = observation

        return latest

    def to_dict(self) -> dict[str, Any]:
        return {
            "source": self.source,
            "timestamp_s": self.timestamp_s,
            "count": self.count,
            "observations": [
                observation.to_dict()
                for observation in self.observations
            ],
            "metadata": dict(self.metadata),
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
    family: str = DEFAULT_TAG_FAMILY,
    size_m: Optional[float] = None,
    source_frame_id: str = "",
    camera_frame_id: str = "",
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
        source_frame_id=str(source_frame_id),
        camera_frame_id=str(camera_frame_id),
        metadata=dict(metadata or {}),
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
        family=str(data.get("family", DEFAULT_TAG_FAMILY)),
        size_m=None if data.get("size_m") is None else float(data["size_m"]),
        timestamp_s=float(data.get("timestamp_s", time.time())),
        source_frame_id=str(data.get("source_frame_id", "")),
        camera_frame_id=str(data.get("camera_frame_id", "")),
        metadata=dict(data.get("metadata", {})),
    )


def apriltag_observations_from_dicts(
    items: Sequence[dict[str, Any]],
) -> tuple[AprilTagObservation, ...]:
    return tuple(apriltag_observation_from_dict(item) for item in items)


def apriltag_observation_batch_from_dict(
    data: dict[str, Any],
) -> AprilTagObservationBatch:
    return AprilTagObservationBatch(
        observations=apriltag_observations_from_dicts(
            data.get("observations", ())
        ),
        source=str(data.get("source", "apriltag_detector")),
        timestamp_s=float(data.get("timestamp_s", time.time())),
        metadata=dict(data.get("metadata", {})),
    )


def save_apriltag_observation_batch(
    path: str | Path,
    batch: AprilTagObservationBatch,
    *,
    indent: int = 2,
) -> Path:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(batch.to_json(indent=indent) + "\n", encoding="utf-8")

    return output_path


def load_apriltag_observation_batch(path: str | Path) -> AprilTagObservationBatch:
    input_path = Path(path)

    if not input_path.exists():
        return AprilTagObservationBatch()

    data = json.loads(input_path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise ValueError("AprilTag observation batch JSON root must be an object.")

    return apriltag_observation_batch_from_dict(data)


def latest_observations_by_tag_id(
    observations: Sequence[AprilTagObservation],
) -> dict[int, AprilTagObservation]:
    return AprilTagObservationBatch(tuple(observations)).latest_by_tag_id()


def main() -> None:
    observation = make_apriltag_observation(
        tag_id=21,
        x=4.2,
        y=8.7,
        confidence=0.95,
        source_frame_id="camera_color_optical_frame",
        camera_frame_id="camera_link",
    )

    batch = AprilTagObservationBatch(observations=(observation,))

    print(observation.to_json(indent=2))
    print(batch.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "DEFAULT_TAG_FAMILY",
    "AprilTagObservation",
    "AprilTagObservationBatch",
    "apriltag_observation_batch_from_dict",
    "apriltag_observation_from_dict",
    "apriltag_observations_from_dicts",
    "latest_observations_by_tag_id",
    "load_apriltag_observation_batch",
    "main",
    "make_apriltag_observation",
    "save_apriltag_observation_batch",
]