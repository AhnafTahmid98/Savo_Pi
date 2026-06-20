#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Bridge semantic landmarks into navigation-ready locations."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional, Sequence

from savo_mapping.exploration.area_explorer import AreaRegion
from savo_mapping.exploration.semantic_explorer import SemanticTarget
from savo_mapping.semantic.location_candidate import LocationPose, make_location_key
from savo_mapping.semantic.semantic_landmark_store import (
    SemanticLandmark,
    SemanticLandmarkStore,
    load_semantic_landmark_store,
)


BRIDGE_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class BridgeLocation:
    key: str
    label: str
    pose: LocationPose
    area_type: str = "place"
    source: str = "semantic_landmark"
    confidence: float = 1.0
    enabled: bool = True
    aliases: tuple[str, ...] = ()
    tag_id: Optional[int] = None
    metadata: dict[str, Any] = field(default_factory=dict)
    updated_at_s: float = field(default_factory=time.time)

    def __post_init__(self) -> None:
        if not self.key.strip():
            raise ValueError("Bridge location key cannot be empty.")

        if not self.label.strip():
            raise ValueError("Bridge location label cannot be empty.")

        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("Bridge location confidence must be between 0 and 1.")

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
            "metadata": dict(self.metadata),
            "updated_at_s": self.updated_at_s,
        }

    def to_area_region(self, *, priority: int = 0) -> AreaRegion:
        return AreaRegion(
            key=self.key,
            label=self.label,
            center_x=self.pose.x,
            center_y=self.pose.y,
            yaw=self.pose.yaw,
            frame_id=self.pose.frame_id,
            area_type=self.area_type,
            priority=priority,
            visited=False,
            confirmed=True,
            metadata={
                **self.metadata,
                "source": self.source,
                "tag_id": self.tag_id,
            },
        )

    def to_semantic_target(self, *, priority: int = 0) -> SemanticTarget:
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
            priority=priority,
            confirmed=True,
            visited=False,
            needs_review=False,
            metadata=dict(self.metadata),
        )

    def to_dict(self) -> dict[str, Any]:
        return self.to_known_location_dict()

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


@dataclass(frozen=True)
class LocationBridgeResult:
    ok: bool
    locations: tuple[BridgeLocation, ...] = ()
    schema_version: int = BRIDGE_SCHEMA_VERSION
    map_name: str = ""
    message: str = "Location bridge not run."
    timestamp_s: float = field(default_factory=time.time)

    @property
    def location_count(self) -> int:
        return len(self.locations)

    def to_known_locations_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "map_name": self.map_name,
            "updated_at_s": self.timestamp_s,
            "location_count": self.location_count,
            "locations": {
                location.key: location.to_known_location_dict()
                for location in self.locations
                if location.enabled
            },
        }

    def to_area_regions(self) -> tuple[AreaRegion, ...]:
        return tuple(
            location.to_area_region(priority=_priority_from_metadata(location))
            for location in self.locations
            if location.enabled
        )

    def to_semantic_targets(self) -> tuple[SemanticTarget, ...]:
        return tuple(
            location.to_semantic_target(priority=_priority_from_metadata(location))
            for location in self.locations
            if location.enabled
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "schema_version": self.schema_version,
            "map_name": self.map_name,
            "location_count": self.location_count,
            "locations": [location.to_dict() for location in self.locations],
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


def bridge_location_from_landmark(
    landmark: SemanticLandmark,
    *,
    aliases: Sequence[str] = (),
    enabled: bool = True,
) -> BridgeLocation:
    return BridgeLocation(
        key=landmark.key,
        label=landmark.label,
        pose=landmark.pose,
        area_type=landmark.area_type,
        source=landmark.source,
        confidence=landmark.confidence,
        enabled=enabled,
        aliases=tuple(_clean_alias(alias) for alias in aliases if _clean_alias(alias)),
        tag_id=landmark.tag_id,
        metadata={
            **landmark.metadata,
            "confirmed_at_s": landmark.confirmed_at_s,
            "confirmed_by": landmark.confirmed_by,
            "notes": landmark.notes,
        },
        updated_at_s=time.time(),
    )


def bridge_locations_from_store(
    store: SemanticLandmarkStore,
    *,
    aliases_by_key: Optional[dict[str, Sequence[str]]] = None,
) -> tuple[BridgeLocation, ...]:
    aliases_by_key = aliases_by_key or {}

    locations = [
        bridge_location_from_landmark(
            landmark,
            aliases=aliases_by_key.get(landmark.key, ()),
        )
        for landmark in store.landmarks
    ]

    return tuple(sorted(locations, key=lambda item: item.key))


def bridge_store_to_locations(
    store: SemanticLandmarkStore,
    *,
    aliases_by_key: Optional[dict[str, Sequence[str]]] = None,
) -> LocationBridgeResult:
    locations = bridge_locations_from_store(
        store,
        aliases_by_key=aliases_by_key,
    )

    return LocationBridgeResult(
        ok=True,
        locations=locations,
        map_name=store.map_name,
        message=f"Bridged {len(locations)} semantic location(s).",
    )


def known_locations_from_store(
    store: SemanticLandmarkStore,
    *,
    aliases_by_key: Optional[dict[str, Sequence[str]]] = None,
) -> dict[str, Any]:
    return bridge_store_to_locations(
        store,
        aliases_by_key=aliases_by_key,
    ).to_known_locations_dict()


def save_known_locations(
    path: str | Path,
    data: dict[str, Any],
    *,
    indent: int = 2,
) -> Path:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(data, indent=indent, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    return output_path


def load_known_locations(path: str | Path) -> dict[str, Any]:
    input_path = Path(path)

    if not input_path.exists():
        return {
            "schema_version": BRIDGE_SCHEMA_VERSION,
            "map_name": "",
            "updated_at_s": time.time(),
            "location_count": 0,
            "locations": {},
        }

    data = json.loads(input_path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise ValueError("Known locations JSON root must be an object.")

    return data


def bridge_locations_from_known_locations(
    data: dict[str, Any],
) -> tuple[BridgeLocation, ...]:
    locations_data = data.get("locations", {})

    if not isinstance(locations_data, dict):
        raise ValueError("Known locations field must be an object.")

    return tuple(
        bridge_location_from_dict(item)
        for item in locations_data.values()
    )


def bridge_location_from_dict(data: dict[str, Any]) -> BridgeLocation:
    pose_data = data.get("pose", {})

    return BridgeLocation(
        key=str(data.get("key") or make_location_key(data["label"])),
        label=str(data["label"]),
        pose=LocationPose(
            x=float(pose_data.get("x", 0.0)),
            y=float(pose_data.get("y", 0.0)),
            yaw=float(pose_data.get("yaw", 0.0)),
            frame_id=str(pose_data.get("frame_id", "map")),
        ),
        area_type=str(data.get("area_type", "place")),
        source=str(data.get("source", "semantic_landmark")),
        confidence=float(data.get("confidence", 1.0)),
        enabled=bool(data.get("enabled", True)),
        aliases=tuple(str(alias) for alias in data.get("aliases", ())),
        tag_id=None if data.get("tag_id") is None else int(data["tag_id"]),
        metadata=dict(data.get("metadata", {})),
        updated_at_s=float(data.get("updated_at_s", time.time())),
    )


def bridge_store_file_to_known_locations(
    *,
    store_path: str | Path,
    output_path: str | Path,
    aliases_by_key: Optional[dict[str, Sequence[str]]] = None,
) -> LocationBridgeResult:
    store = load_semantic_landmark_store(store_path)
    result = bridge_store_to_locations(store, aliases_by_key=aliases_by_key)
    save_known_locations(output_path, result.to_known_locations_dict())

    return result


def find_bridge_location(
    locations: Sequence[BridgeLocation],
    key_or_label: str,
) -> Optional[BridgeLocation]:
    clean_key = make_location_key(key_or_label)

    for location in locations:
        if location.key == clean_key:
            return location

        alias_keys = {make_location_key(alias) for alias in location.aliases}

        if clean_key in alias_keys:
            return location

    return None


def _clean_alias(alias: str) -> str:
    return str(alias).strip()


def _priority_from_metadata(location: BridgeLocation) -> int:
    try:
        return int(location.metadata.get("priority", 0))
    except (TypeError, ValueError):
        return 0


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

    result = bridge_store_to_locations(
        store,
        aliases_by_key={"a201": ("A201 Entrance", "Room A201")},
    )

    print(result.to_json(indent=2))
    print(json.dumps(result.to_known_locations_dict(), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()


__all__ = [
    "BRIDGE_SCHEMA_VERSION",
    "BridgeLocation",
    "LocationBridgeResult",
    "bridge_location_from_dict",
    "bridge_location_from_landmark",
    "bridge_locations_from_known_locations",
    "bridge_locations_from_store",
    "bridge_store_file_to_known_locations",
    "bridge_store_to_locations",
    "find_bridge_location",
    "known_locations_from_store",
    "load_known_locations",
    "main",
    "save_known_locations",
]