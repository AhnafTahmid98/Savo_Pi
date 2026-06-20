#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Final location record models for Robot Savo semantic mapping."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Optional, Sequence

from savo_mapping.exploration.area_explorer import AreaRegion
from savo_mapping.exploration.semantic_explorer import SemanticTarget
from savo_mapping.models.exploration_status import ExplorationGoal
from savo_mapping.semantic.location_candidate import LocationPose, make_location_key
from savo_mapping.semantic.semantic_landmark_store import SemanticLandmark


LOCATION_RECORD_SCHEMA_VERSION = 1

LOCATION_RECORD_SOURCE_SEMANTIC = "semantic_landmark"
LOCATION_RECORD_SOURCE_APRILTAG = "apriltag"
LOCATION_RECORD_SOURCE_HUMAN = "human"
LOCATION_RECORD_SOURCE_IMPORT = "import"
LOCATION_RECORD_SOURCE_BRIDGE = "bridge"

VALID_LOCATION_RECORD_SOURCES = (
    LOCATION_RECORD_SOURCE_SEMANTIC,
    LOCATION_RECORD_SOURCE_APRILTAG,
    LOCATION_RECORD_SOURCE_HUMAN,
    LOCATION_RECORD_SOURCE_IMPORT,
    LOCATION_RECORD_SOURCE_BRIDGE,
)


@dataclass(frozen=True)
class LocationRecord:
    key: str
    label: str
    pose: LocationPose
    area_type: str = "place"
    source: str = LOCATION_RECORD_SOURCE_SEMANTIC
    confidence: float = 1.0
    enabled: bool = True
    aliases: tuple[str, ...] = ()
    tag_id: Optional[int] = None
    priority: int = 0
    notes: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)
    created_at_s: float = field(default_factory=time.time)
    updated_at_s: float = field(default_factory=time.time)

    def __post_init__(self) -> None:
        if not self.key.strip():
            raise ValueError("Location record key cannot be empty.")

        if not self.label.strip():
            raise ValueError("Location record label cannot be empty.")

        if self.source not in VALID_LOCATION_RECORD_SOURCES:
            raise ValueError(f"Invalid location record source: {self.source}")

        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("Location record confidence must be between 0 and 1.")

    def with_enabled(self, enabled: bool = True) -> "LocationRecord":
        return replace(
            self,
            enabled=bool(enabled),
            updated_at_s=time.time(),
        )

    def with_aliases(self, aliases: Sequence[str]) -> "LocationRecord":
        return replace(
            self,
            aliases=tuple(_clean_alias(alias) for alias in aliases if _clean_alias(alias)),
            updated_at_s=time.time(),
        )

    def to_exploration_goal(self) -> ExplorationGoal:
        return ExplorationGoal(
            x=self.pose.x,
            y=self.pose.y,
            yaw=self.pose.yaw,
            frame_id=self.pose.frame_id,
            score=float(self.priority) + self.confidence,
            reason=f"location_record:{self.key}",
        )

    def to_area_region(self) -> AreaRegion:
        return AreaRegion(
            key=self.key,
            label=self.label,
            center_x=self.pose.x,
            center_y=self.pose.y,
            yaw=self.pose.yaw,
            frame_id=self.pose.frame_id,
            area_type=self.area_type,
            priority=self.priority,
            visited=False,
            confirmed=True,
            metadata={
                **self.metadata,
                "source": self.source,
                "tag_id": self.tag_id,
            },
        )

    def to_semantic_target(self) -> SemanticTarget:
        return SemanticTarget(
            key=self.key,
            label=self.label,
            x=self.pose.x,
            y=self.pose.y,
            yaw=self.pose.yaw,
            frame_id=self.pose.frame_id,
            target_type=self.area_type,
            source=self.source,
            tag_id=self.tag_id,
            confidence=self.confidence,
            priority=self.priority,
            confirmed=True,
            visited=False,
            needs_review=False,
            metadata=dict(self.metadata),
        )

    def to_known_location_dict(self) -> dict[str, Any]:
        return {
            "key": self.key,
            "label": self.label,
            "aliases": list(self.aliases),
            "pose": self.pose.to_dict(),
            "area_type": self.area_type,
            "source": self.source,
            "confidence": self.confidence,
            "enabled": self.enabled,
            "tag_id": self.tag_id,
            "priority": self.priority,
            "notes": self.notes,
            "metadata": dict(self.metadata),
            "created_at_s": self.created_at_s,
            "updated_at_s": self.updated_at_s,
        }

    def to_dict(self) -> dict[str, Any]:
        return self.to_known_location_dict()

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


@dataclass(frozen=True)
class LocationRecordSet:
    map_name: str = ""
    schema_version: int = LOCATION_RECORD_SCHEMA_VERSION
    records: tuple[LocationRecord, ...] = ()
    updated_at_s: float = field(default_factory=time.time)

    def __post_init__(self) -> None:
        _ensure_unique_keys(self.records)

    @property
    def count(self) -> int:
        return len(self.records)

    @property
    def enabled_count(self) -> int:
        return sum(1 for record in self.records if record.enabled)

    def enabled_records(self) -> tuple[LocationRecord, ...]:
        return tuple(record for record in self.records if record.enabled)

    def find(self, key_or_label: str) -> Optional[LocationRecord]:
        clean_key = make_location_key(key_or_label)

        for record in self.records:
            if record.key == clean_key:
                return record

            alias_keys = {make_location_key(alias) for alias in record.aliases}

            if clean_key in alias_keys:
                return record

        return None

    def upsert(self, record: LocationRecord) -> "LocationRecordSet":
        updated: list[LocationRecord] = []
        replaced_item = False

        for item in self.records:
            if item.key == record.key:
                updated.append(record)
                replaced_item = True
            else:
                updated.append(item)

        if not replaced_item:
            updated.append(record)

        return replace(
            self,
            records=tuple(sorted(updated, key=lambda item: item.key)),
            updated_at_s=time.time(),
        )

    def remove(self, key_or_label: str) -> "LocationRecordSet":
        clean_key = make_location_key(key_or_label)

        return replace(
            self,
            records=tuple(
                record for record in self.records if record.key != clean_key
            ),
            updated_at_s=time.time(),
        )

    def to_known_locations_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "map_name": self.map_name,
            "updated_at_s": self.updated_at_s,
            "location_count": self.enabled_count,
            "locations": {
                record.key: record.to_known_location_dict()
                for record in self.enabled_records()
            },
        }

    def to_area_regions(self) -> tuple[AreaRegion, ...]:
        return tuple(record.to_area_region() for record in self.enabled_records())

    def to_semantic_targets(self) -> tuple[SemanticTarget, ...]:
        return tuple(record.to_semantic_target() for record in self.enabled_records())

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "map_name": self.map_name,
            "updated_at_s": self.updated_at_s,
            "count": self.count,
            "enabled_count": self.enabled_count,
            "records": [record.to_dict() for record in self.records],
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


def location_record_from_landmark(
    landmark: SemanticLandmark,
    *,
    aliases: Sequence[str] = (),
    enabled: bool = True,
    priority: int = 0,
) -> LocationRecord:
    return LocationRecord(
        key=landmark.key,
        label=landmark.label,
        pose=landmark.pose,
        area_type=landmark.area_type,
        source=_record_source_from_landmark_source(landmark.source),
        confidence=landmark.confidence,
        enabled=enabled,
        aliases=tuple(_clean_alias(alias) for alias in aliases if _clean_alias(alias)),
        tag_id=landmark.tag_id,
        priority=priority,
        notes=landmark.notes,
        metadata={
            **landmark.metadata,
            "confirmed_at_s": landmark.confirmed_at_s,
            "confirmed_by": landmark.confirmed_by,
        },
    )


def location_record_from_dict(data: dict[str, Any]) -> LocationRecord:
    pose_data = data.get("pose", {})

    return LocationRecord(
        key=str(data.get("key") or make_location_key(data["label"])),
        label=str(data["label"]),
        pose=LocationPose(
            x=float(pose_data.get("x", 0.0)),
            y=float(pose_data.get("y", 0.0)),
            yaw=float(pose_data.get("yaw", 0.0)),
            frame_id=str(pose_data.get("frame_id", "map")),
        ),
        area_type=str(data.get("area_type", "place")),
        source=str(data.get("source", LOCATION_RECORD_SOURCE_SEMANTIC)),
        confidence=float(data.get("confidence", 1.0)),
        enabled=bool(data.get("enabled", True)),
        aliases=tuple(str(alias) for alias in data.get("aliases", ())),
        tag_id=None if data.get("tag_id") is None else int(data["tag_id"]),
        priority=int(data.get("priority", 0)),
        notes=str(data.get("notes", "")),
        metadata=dict(data.get("metadata", {})),
        created_at_s=float(data.get("created_at_s", time.time())),
        updated_at_s=float(data.get("updated_at_s", time.time())),
    )


def location_record_set_from_dict(data: dict[str, Any]) -> LocationRecordSet:
    records_data = data.get("records")

    if records_data is None and isinstance(data.get("locations"), dict):
        records_data = data["locations"].values()

    return LocationRecordSet(
        map_name=str(data.get("map_name", "")),
        schema_version=int(data.get("schema_version", LOCATION_RECORD_SCHEMA_VERSION)),
        updated_at_s=float(data.get("updated_at_s", time.time())),
        records=tuple(
            location_record_from_dict(item)
            for item in records_data or ()
        ),
    )


def location_records_from_landmarks(
    landmarks: Sequence[SemanticLandmark],
    *,
    aliases_by_key: Optional[dict[str, Sequence[str]]] = None,
    priority_by_key: Optional[dict[str, int]] = None,
) -> tuple[LocationRecord, ...]:
    aliases_by_key = aliases_by_key or {}
    priority_by_key = priority_by_key or {}

    return tuple(
        location_record_from_landmark(
            landmark,
            aliases=aliases_by_key.get(landmark.key, ()),
            priority=priority_by_key.get(landmark.key, 0),
        )
        for landmark in landmarks
    )


def location_record_set_from_landmarks(
    landmarks: Sequence[SemanticLandmark],
    *,
    map_name: str = "",
    aliases_by_key: Optional[dict[str, Sequence[str]]] = None,
    priority_by_key: Optional[dict[str, int]] = None,
) -> LocationRecordSet:
    return LocationRecordSet(
        map_name=map_name,
        records=location_records_from_landmarks(
            landmarks,
            aliases_by_key=aliases_by_key,
            priority_by_key=priority_by_key,
        ),
    )


def load_location_record_set(path: str | Path) -> LocationRecordSet:
    input_path = Path(path)

    if not input_path.exists():
        return LocationRecordSet()

    data = json.loads(input_path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise ValueError("Location record JSON root must be an object.")

    return location_record_set_from_dict(data)


def save_location_record_set(
    path: str | Path,
    records: LocationRecordSet,
    *,
    indent: int = 2,
) -> Path:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(records.to_json(indent=indent) + "\n", encoding="utf-8")

    return output_path


def save_known_locations_from_records(
    path: str | Path,
    records: LocationRecordSet,
    *,
    indent: int = 2,
) -> Path:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(records.to_known_locations_dict(), indent=indent, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )

    return output_path


def _record_source_from_landmark_source(source: str) -> str:
    if source == LOCATION_RECORD_SOURCE_APRILTAG:
        return LOCATION_RECORD_SOURCE_APRILTAG

    if source == LOCATION_RECORD_SOURCE_HUMAN:
        return LOCATION_RECORD_SOURCE_HUMAN

    if source == LOCATION_RECORD_SOURCE_IMPORT:
        return LOCATION_RECORD_SOURCE_IMPORT

    return LOCATION_RECORD_SOURCE_SEMANTIC


def _clean_alias(alias: str) -> str:
    return str(alias).strip()


def _ensure_unique_keys(records: Sequence[LocationRecord]) -> None:
    seen: set[str] = set()

    for record in records:
        if record.key in seen:
            raise ValueError(f"Duplicate location record key: {record.key}")

        seen.add(record.key)


def main() -> None:
    from savo_mapping.semantic.location_candidate import (
        make_apriltag_location_candidate,
    )
    from savo_mapping.semantic.semantic_landmark_store import SemanticLandmarkStore

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

    records = location_record_set_from_landmarks(
        store.landmarks,
        map_name=store.map_name,
        aliases_by_key={"a201": ("A201 Entrance", "Room A201")},
        priority_by_key={"a201": 3},
    )

    print(records.to_json(indent=2))
    print(json.dumps(records.to_known_locations_dict(), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()


__all__ = [
    "LOCATION_RECORD_SCHEMA_VERSION",
    "LOCATION_RECORD_SOURCE_APRILTAG",
    "LOCATION_RECORD_SOURCE_BRIDGE",
    "LOCATION_RECORD_SOURCE_HUMAN",
    "LOCATION_RECORD_SOURCE_IMPORT",
    "LOCATION_RECORD_SOURCE_SEMANTIC",
    "VALID_LOCATION_RECORD_SOURCES",
    "LocationRecord",
    "LocationRecordSet",
    "load_location_record_set",
    "location_record_from_dict",
    "location_record_from_landmark",
    "location_record_set_from_dict",
    "location_record_set_from_landmarks",
    "location_records_from_landmarks",
    "main",
    "save_known_locations_from_records",
    "save_location_record_set",
]