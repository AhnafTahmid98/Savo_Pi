#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Semantic landmark store for Robot Savo mapping."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Optional, Sequence

from savo_mapping.semantic.location_candidate import (
    CANDIDATE_STATUS_CONFIRMED,
    CANDIDATE_STATUS_REJECTED,
    LocationCandidate,
    LocationPose,
    find_candidate_by_key,
    location_candidate_from_dict,
    make_location_key,
    upsert_location_candidate,
)


STORE_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class SemanticLandmark:
    key: str
    label: str
    pose: LocationPose
    area_type: str = "place"
    source: str = "human"
    confidence: float = 1.0
    tag_id: Optional[int] = None
    confirmed_at_s: float = field(default_factory=time.time)
    confirmed_by: str = "operator"
    notes: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.key.strip():
            raise ValueError("Semantic landmark key cannot be empty.")

        if not self.label.strip():
            raise ValueError("Semantic landmark label cannot be empty.")

        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("Semantic landmark confidence must be between 0 and 1.")

    def to_dict(self) -> dict[str, Any]:
        return {
            "key": self.key,
            "label": self.label,
            "pose": self.pose.to_dict(),
            "area_type": self.area_type,
            "source": self.source,
            "confidence": self.confidence,
            "tag_id": self.tag_id,
            "confirmed_at_s": self.confirmed_at_s,
            "confirmed_by": self.confirmed_by,
            "notes": self.notes,
            "metadata": dict(self.metadata),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


@dataclass(frozen=True)
class SemanticLandmarkStore:
    map_name: str = ""
    schema_version: int = STORE_SCHEMA_VERSION
    landmarks: tuple[SemanticLandmark, ...] = ()
    candidates: tuple[LocationCandidate, ...] = ()
    updated_at_s: float = field(default_factory=time.time)

    def __post_init__(self) -> None:
        _ensure_unique_keys(self.landmarks, "landmark")
        _ensure_unique_keys(self.candidates, "candidate")

    @property
    def landmark_count(self) -> int:
        return len(self.landmarks)

    @property
    def candidate_count(self) -> int:
        return len(self.candidates)

    @property
    def pending_candidate_count(self) -> int:
        return sum(1 for candidate in self.candidates if candidate.pending)

    @property
    def confirmed_candidate_count(self) -> int:
        return sum(1 for candidate in self.candidates if candidate.confirmed)

    @property
    def rejected_candidate_count(self) -> int:
        return sum(1 for candidate in self.candidates if candidate.rejected)

    def find_landmark(self, key: str) -> Optional[SemanticLandmark]:
        clean_key = make_location_key(key)

        for landmark in self.landmarks:
            if landmark.key == clean_key:
                return landmark

        return None

    def find_candidate(self, key: str) -> Optional[LocationCandidate]:
        return find_candidate_by_key(self.candidates, key)

    def upsert_candidate(
        self,
        candidate: LocationCandidate,
    ) -> "SemanticLandmarkStore":
        return replace(
            self,
            candidates=upsert_location_candidate(self.candidates, candidate),
            updated_at_s=time.time(),
        )

    def upsert_landmark(
        self,
        landmark: SemanticLandmark,
    ) -> "SemanticLandmarkStore":
        landmarks = _upsert_landmark(self.landmarks, landmark)

        return replace(
            self,
            landmarks=landmarks,
            updated_at_s=time.time(),
        )

    def confirm_candidate(
        self,
        key: str,
        *,
        confirmed_by: str = "operator",
    ) -> "SemanticLandmarkStore":
        candidate = self.find_candidate(key)

        if candidate is None:
            raise KeyError(f"Location candidate not found: {key}")

        confirmed = candidate.confirm(confirmed_by=confirmed_by)
        landmark = semantic_landmark_from_candidate(confirmed)

        updated = self.upsert_candidate(confirmed)
        return updated.upsert_landmark(landmark)

    def reject_candidate(
        self,
        key: str,
        *,
        reason: str,
    ) -> "SemanticLandmarkStore":
        candidate = self.find_candidate(key)

        if candidate is None:
            raise KeyError(f"Location candidate not found: {key}")

        rejected = candidate.reject(reason)

        return self.upsert_candidate(rejected)

    def remove_candidate(self, key: str) -> "SemanticLandmarkStore":
        clean_key = make_location_key(key)

        return replace(
            self,
            candidates=tuple(
                candidate
                for candidate in self.candidates
                if candidate.key != clean_key
            ),
            updated_at_s=time.time(),
        )

    def remove_landmark(self, key: str) -> "SemanticLandmarkStore":
        clean_key = make_location_key(key)

        return replace(
            self,
            landmarks=tuple(
                landmark
                for landmark in self.landmarks
                if landmark.key != clean_key
            ),
            updated_at_s=time.time(),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "map_name": self.map_name,
            "updated_at_s": self.updated_at_s,
            "landmark_count": self.landmark_count,
            "candidate_count": self.candidate_count,
            "pending_candidate_count": self.pending_candidate_count,
            "confirmed_candidate_count": self.confirmed_candidate_count,
            "rejected_candidate_count": self.rejected_candidate_count,
            "landmarks": [
                landmark.to_dict()
                for landmark in self.landmarks
            ],
            "candidates": [
                candidate.to_dict()
                for candidate in self.candidates
            ],
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


def semantic_landmark_from_candidate(
    candidate: LocationCandidate,
) -> SemanticLandmark:
    if not candidate.confirmed:
        raise ValueError("Location candidate must be confirmed first.")

    return SemanticLandmark(
        key=candidate.key,
        label=candidate.label,
        pose=candidate.pose,
        area_type=candidate.area_type,
        source=candidate.source,
        confidence=candidate.confidence,
        tag_id=candidate.tag_id,
        confirmed_at_s=candidate.updated_at_s,
        confirmed_by=candidate.confirmed_by or "operator",
        notes=candidate.notes,
        metadata=dict(candidate.metadata),
    )


def semantic_landmark_from_dict(data: dict[str, Any]) -> SemanticLandmark:
    pose_data = data.get("pose", {})

    return SemanticLandmark(
        key=str(data.get("key") or make_location_key(data["label"])),
        label=str(data["label"]),
        pose=LocationPose(
            x=float(pose_data.get("x", 0.0)),
            y=float(pose_data.get("y", 0.0)),
            yaw=float(pose_data.get("yaw", 0.0)),
            frame_id=str(pose_data.get("frame_id", "map")),
        ),
        area_type=str(data.get("area_type", "place")),
        source=str(data.get("source", "human")),
        confidence=float(data.get("confidence", 1.0)),
        tag_id=None if data.get("tag_id") is None else int(data["tag_id"]),
        confirmed_at_s=float(data.get("confirmed_at_s", time.time())),
        confirmed_by=str(data.get("confirmed_by", "operator")),
        notes=str(data.get("notes", "")),
        metadata=dict(data.get("metadata", {})),
    )


def semantic_landmark_store_from_dict(
    data: dict[str, Any],
) -> SemanticLandmarkStore:
    return SemanticLandmarkStore(
        schema_version=int(data.get("schema_version", STORE_SCHEMA_VERSION)),
        map_name=str(data.get("map_name", "")),
        updated_at_s=float(data.get("updated_at_s", time.time())),
        landmarks=tuple(
            semantic_landmark_from_dict(item)
            for item in data.get("landmarks", ())
        ),
        candidates=tuple(
            location_candidate_from_dict(item)
            for item in data.get("candidates", ())
        ),
    )


def load_semantic_landmark_store(path: str | Path) -> SemanticLandmarkStore:
    store_path = Path(path)

    if not store_path.exists():
        return SemanticLandmarkStore()

    data = json.loads(store_path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise ValueError("Semantic landmark store JSON root must be an object.")

    return semantic_landmark_store_from_dict(data)


def save_semantic_landmark_store(
    path: str | Path,
    store: SemanticLandmarkStore,
    *,
    indent: int = 2,
) -> Path:
    store_path = Path(path)
    store_path.parent.mkdir(parents=True, exist_ok=True)
    store_path.write_text(store.to_json(indent=indent) + "\n", encoding="utf-8")

    return store_path


def filter_landmarks_by_source(
    landmarks: Sequence[SemanticLandmark],
    source: str,
) -> tuple[SemanticLandmark, ...]:
    return tuple(
        landmark
        for landmark in landmarks
        if landmark.source == source
    )


def filter_landmarks_by_area_type(
    landmarks: Sequence[SemanticLandmark],
    area_type: str,
) -> tuple[SemanticLandmark, ...]:
    return tuple(
        landmark
        for landmark in landmarks
        if landmark.area_type == area_type
    )


def _upsert_landmark(
    landmarks: Sequence[SemanticLandmark],
    landmark: SemanticLandmark,
) -> tuple[SemanticLandmark, ...]:
    updated: list[SemanticLandmark] = []
    replaced_item = False

    for item in landmarks:
        if item.key == landmark.key:
            updated.append(landmark)
            replaced_item = True
        else:
            updated.append(item)

    if not replaced_item:
        updated.append(landmark)

    return tuple(updated)


def _ensure_unique_keys(items: Sequence[Any], label: str) -> None:
    seen: set[str] = set()

    for item in items:
        key = str(item.key)

        if key in seen:
            raise ValueError(f"Duplicate {label} key: {key}")

        seen.add(key)


def main() -> None:
    from savo_mapping.semantic.location_candidate import (
        make_apriltag_location_candidate,
    )

    candidate = make_apriltag_location_candidate(
        tag_id=21,
        label="A201",
        x=4.2,
        y=8.7,
        confidence=0.95,
    )

    store = SemanticLandmarkStore(map_name="savonia_campus_heart")
    store = store.upsert_candidate(candidate)
    store = store.confirm_candidate("a201", confirmed_by="operator")

    print(store.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "CANDIDATE_STATUS_CONFIRMED",
    "CANDIDATE_STATUS_REJECTED",
    "STORE_SCHEMA_VERSION",
    "SemanticLandmark",
    "SemanticLandmarkStore",
    "filter_landmarks_by_area_type",
    "filter_landmarks_by_source",
    "load_semantic_landmark_store",
    "main",
    "save_semantic_landmark_store",
    "semantic_landmark_from_candidate",
    "semantic_landmark_from_dict",
    "semantic_landmark_store_from_dict",
]