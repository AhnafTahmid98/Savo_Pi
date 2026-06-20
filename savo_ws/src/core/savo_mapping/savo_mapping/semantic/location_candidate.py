#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Location candidate models for Robot Savo semantic mapping."""

from __future__ import annotations

import json
import re
import time
from dataclasses import dataclass, field, replace
from typing import Any, Optional, Sequence


CANDIDATE_STATUS_PENDING = "pending"
CANDIDATE_STATUS_CONFIRMED = "confirmed"
CANDIDATE_STATUS_REJECTED = "rejected"

SOURCE_APRILTAG = "apriltag"
SOURCE_HUMAN = "human"
SOURCE_OCR = "ocr"
SOURCE_VLM = "vlm"
SOURCE_IMPORT = "import"

VALID_CANDIDATE_STATUSES = (
    CANDIDATE_STATUS_PENDING,
    CANDIDATE_STATUS_CONFIRMED,
    CANDIDATE_STATUS_REJECTED,
)

VALID_LOCATION_SOURCES = (
    SOURCE_APRILTAG,
    SOURCE_HUMAN,
    SOURCE_OCR,
    SOURCE_VLM,
    SOURCE_IMPORT,
)


@dataclass(frozen=True)
class LocationPose:
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"

    def __post_init__(self) -> None:
        if not self.frame_id.strip():
            raise ValueError("Location pose frame_id cannot be empty.")

    def to_dict(self) -> dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "frame_id": self.frame_id,
        }


@dataclass(frozen=True)
class LocationCandidate:
    key: str
    label: str
    pose: LocationPose
    source: str = SOURCE_HUMAN
    status: str = CANDIDATE_STATUS_PENDING
    confidence: float = 1.0
    tag_id: Optional[int] = None
    area_type: str = "place"
    created_at_s: float = field(default_factory=time.time)
    updated_at_s: float = field(default_factory=time.time)
    confirmed_by: str = ""
    rejected_reason: str = ""
    notes: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.key.strip():
            raise ValueError("Location candidate key cannot be empty.")

        if not self.label.strip():
            raise ValueError("Location candidate label cannot be empty.")

        if self.status not in VALID_CANDIDATE_STATUSES:
            raise ValueError(f"Invalid location candidate status: {self.status}")

        if self.source not in VALID_LOCATION_SOURCES:
            raise ValueError(f"Invalid location candidate source: {self.source}")

        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("Location candidate confidence must be between 0 and 1.")

    @property
    def pending(self) -> bool:
        return self.status == CANDIDATE_STATUS_PENDING

    @property
    def confirmed(self) -> bool:
        return self.status == CANDIDATE_STATUS_CONFIRMED

    @property
    def rejected(self) -> bool:
        return self.status == CANDIDATE_STATUS_REJECTED

    def confirm(self, *, confirmed_by: str = "operator") -> "LocationCandidate":
        return replace(
            self,
            status=CANDIDATE_STATUS_CONFIRMED,
            confirmed_by=str(confirmed_by),
            rejected_reason="",
            updated_at_s=time.time(),
        )

    def reject(self, reason: str) -> "LocationCandidate":
        return replace(
            self,
            status=CANDIDATE_STATUS_REJECTED,
            rejected_reason=str(reason),
            updated_at_s=time.time(),
        )

    def with_label(self, label: str) -> "LocationCandidate":
        return replace(
            self,
            key=make_location_key(label),
            label=str(label).strip(),
            updated_at_s=time.time(),
        )

    def with_pose(self, pose: LocationPose) -> "LocationCandidate":
        return replace(
            self,
            pose=pose,
            updated_at_s=time.time(),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "key": self.key,
            "label": self.label,
            "pose": self.pose.to_dict(),
            "source": self.source,
            "status": self.status,
            "confidence": self.confidence,
            "tag_id": self.tag_id,
            "area_type": self.area_type,
            "created_at_s": self.created_at_s,
            "updated_at_s": self.updated_at_s,
            "confirmed_by": self.confirmed_by,
            "rejected_reason": self.rejected_reason,
            "notes": self.notes,
            "metadata": dict(self.metadata),
            "pending": self.pending,
            "confirmed": self.confirmed,
            "rejected": self.rejected,
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


def make_location_key(label: str) -> str:
    text = str(label).strip().lower()
    text = re.sub(r"[^a-z0-9]+", "_", text)
    text = re.sub(r"_+", "_", text).strip("_")

    if not text:
        raise ValueError("Location label cannot produce an empty key.")

    return text


def make_location_candidate(
    *,
    label: str,
    x: float,
    y: float,
    yaw: float = 0.0,
    frame_id: str = "map",
    source: str = SOURCE_HUMAN,
    confidence: float = 1.0,
    tag_id: Optional[int] = None,
    area_type: str = "place",
    notes: str = "",
    metadata: Optional[dict[str, Any]] = None,
) -> LocationCandidate:
    return LocationCandidate(
        key=make_location_key(label),
        label=str(label).strip(),
        pose=LocationPose(
            x=float(x),
            y=float(y),
            yaw=float(yaw),
            frame_id=str(frame_id),
        ),
        source=str(source),
        confidence=float(confidence),
        tag_id=tag_id,
        area_type=str(area_type),
        notes=str(notes),
        metadata=dict(metadata or {}),
    )


def make_apriltag_location_candidate(
    *,
    tag_id: int,
    label: str,
    x: float,
    y: float,
    yaw: float = 0.0,
    frame_id: str = "map",
    confidence: float = 1.0,
    area_type: str = "apriltag_location",
    notes: str = "",
) -> LocationCandidate:
    return make_location_candidate(
        label=label,
        x=x,
        y=y,
        yaw=yaw,
        frame_id=frame_id,
        source=SOURCE_APRILTAG,
        confidence=confidence,
        tag_id=int(tag_id),
        area_type=area_type,
        notes=notes,
        metadata={
            "source": SOURCE_APRILTAG,
            "tag_id": int(tag_id),
        },
    )


def location_candidate_from_dict(data: dict[str, Any]) -> LocationCandidate:
    pose_data = data.get("pose", {})

    return LocationCandidate(
        key=str(data.get("key") or make_location_key(data["label"])),
        label=str(data["label"]),
        pose=LocationPose(
            x=float(pose_data.get("x", 0.0)),
            y=float(pose_data.get("y", 0.0)),
            yaw=float(pose_data.get("yaw", 0.0)),
            frame_id=str(pose_data.get("frame_id", "map")),
        ),
        source=str(data.get("source", SOURCE_HUMAN)),
        status=str(data.get("status", CANDIDATE_STATUS_PENDING)),
        confidence=float(data.get("confidence", 1.0)),
        tag_id=None if data.get("tag_id") is None else int(data["tag_id"]),
        area_type=str(data.get("area_type", "place")),
        created_at_s=float(data.get("created_at_s", time.time())),
        updated_at_s=float(data.get("updated_at_s", time.time())),
        confirmed_by=str(data.get("confirmed_by", "")),
        rejected_reason=str(data.get("rejected_reason", "")),
        notes=str(data.get("notes", "")),
        metadata=dict(data.get("metadata", {})),
    )


def location_candidates_from_dicts(
    items: Sequence[dict[str, Any]],
) -> tuple[LocationCandidate, ...]:
    return tuple(location_candidate_from_dict(item) for item in items)


def filter_pending_candidates(
    candidates: Sequence[LocationCandidate],
) -> tuple[LocationCandidate, ...]:
    return tuple(candidate for candidate in candidates if candidate.pending)


def filter_confirmed_candidates(
    candidates: Sequence[LocationCandidate],
) -> tuple[LocationCandidate, ...]:
    return tuple(candidate for candidate in candidates if candidate.confirmed)


def filter_rejected_candidates(
    candidates: Sequence[LocationCandidate],
) -> tuple[LocationCandidate, ...]:
    return tuple(candidate for candidate in candidates if candidate.rejected)


def find_candidate_by_key(
    candidates: Sequence[LocationCandidate],
    key: str,
) -> Optional[LocationCandidate]:
    clean_key = make_location_key(key)

    for candidate in candidates:
        if candidate.key == clean_key:
            return candidate

    return None


def upsert_location_candidate(
    candidates: Sequence[LocationCandidate],
    candidate: LocationCandidate,
) -> tuple[LocationCandidate, ...]:
    updated: list[LocationCandidate] = []
    replaced = False

    for item in candidates:
        if item.key == candidate.key:
            updated.append(candidate)
            replaced = True
        else:
            updated.append(item)

    if not replaced:
        updated.append(candidate)

    return tuple(updated)


def main() -> None:
    candidate = make_apriltag_location_candidate(
        tag_id=21,
        label="A201",
        x=4.2,
        y=8.7,
        confidence=0.95,
    )

    confirmed = candidate.confirm(confirmed_by="operator")

    print(candidate.to_json(indent=2))
    print(confirmed.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "CANDIDATE_STATUS_CONFIRMED",
    "CANDIDATE_STATUS_PENDING",
    "CANDIDATE_STATUS_REJECTED",
    "SOURCE_APRILTAG",
    "SOURCE_HUMAN",
    "SOURCE_IMPORT",
    "SOURCE_OCR",
    "SOURCE_VLM",
    "VALID_CANDIDATE_STATUSES",
    "VALID_LOCATION_SOURCES",
    "LocationCandidate",
    "LocationPose",
    "filter_confirmed_candidates",
    "filter_pending_candidates",
    "filter_rejected_candidates",
    "find_candidate_by_key",
    "location_candidate_from_dict",
    "location_candidates_from_dicts",
    "make_apriltag_location_candidate",
    "make_location_candidate",
    "make_location_key",
    "main",
    "upsert_location_candidate",
]