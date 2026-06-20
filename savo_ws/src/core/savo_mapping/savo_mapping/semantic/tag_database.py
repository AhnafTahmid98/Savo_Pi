#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""AprilTag database helpers for Robot Savo semantic mapping."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Optional, Sequence

from savo_mapping.semantic.location_candidate import make_location_key


TAG_DATABASE_SCHEMA_VERSION = 1
DEFAULT_TAG_FAMILY = "tag36h11"


@dataclass(frozen=True)
class TagRecord:
    tag_id: int
    label: str
    area_type: str = "apriltag_location"
    family: str = DEFAULT_TAG_FAMILY
    size_m: Optional[float] = None
    priority: int = 0
    enabled: bool = True
    confirmed: bool = True
    aliases: tuple[str, ...] = ()
    notes: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)
    created_at_s: float = field(default_factory=time.time)
    updated_at_s: float = field(default_factory=time.time)

    def __post_init__(self) -> None:
        if self.tag_id < 0:
            raise ValueError("Tag id must be non-negative.")

        if not self.label.strip():
            raise ValueError("Tag label cannot be empty.")

        if not self.family.strip():
            raise ValueError("Tag family cannot be empty.")

        if self.size_m is not None and self.size_m <= 0.0:
            raise ValueError("Tag size_m must be positive or None.")

    @property
    def key(self) -> str:
        return make_location_key(self.label)

    def with_enabled(self, enabled: bool = True) -> "TagRecord":
        return replace(
            self,
            enabled=bool(enabled),
            updated_at_s=time.time(),
        )

    def with_label(self, label: str) -> "TagRecord":
        return replace(
            self,
            label=str(label).strip(),
            updated_at_s=time.time(),
        )

    def to_apriltag_label(self):
        from savo_mapping.semantic.apriltag_mapper import AprilTagLabel

        return AprilTagLabel(
            tag_id=self.tag_id,
            label=self.label,
            area_type=self.area_type,
            priority=self.priority,
            notes=self.notes,
            metadata={
                **self.metadata,
                "family": self.family,
                "size_m": self.size_m,
                "aliases": list(self.aliases),
                "confirmed": self.confirmed,
            },
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "tag_id": self.tag_id,
            "key": self.key,
            "label": self.label,
            "area_type": self.area_type,
            "family": self.family,
            "size_m": self.size_m,
            "priority": self.priority,
            "enabled": self.enabled,
            "confirmed": self.confirmed,
            "aliases": list(self.aliases),
            "notes": self.notes,
            "metadata": dict(self.metadata),
            "created_at_s": self.created_at_s,
            "updated_at_s": self.updated_at_s,
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


@dataclass(frozen=True)
class TagDatabase:
    map_name: str = ""
    schema_version: int = TAG_DATABASE_SCHEMA_VERSION
    records: tuple[TagRecord, ...] = ()
    updated_at_s: float = field(default_factory=time.time)

    def __post_init__(self) -> None:
        _ensure_unique_tag_ids(self.records)

    @property
    def count(self) -> int:
        return len(self.records)

    @property
    def enabled_count(self) -> int:
        return sum(1 for record in self.records if record.enabled)

    def find_by_id(self, tag_id: int) -> Optional[TagRecord]:
        for record in self.records:
            if record.tag_id == int(tag_id):
                return record

        return None

    def find_by_key_or_label(self, key_or_label: str) -> Optional[TagRecord]:
        clean_key = make_location_key(key_or_label)

        for record in self.records:
            if record.key == clean_key:
                return record

            alias_keys = {make_location_key(alias) for alias in record.aliases}

            if clean_key in alias_keys:
                return record

        return None

    def upsert(self, record: TagRecord) -> "TagDatabase":
        records: list[TagRecord] = []
        replaced_item = False

        for item in self.records:
            if item.tag_id == record.tag_id:
                records.append(record)
                replaced_item = True
            else:
                records.append(item)

        if not replaced_item:
            records.append(record)

        return replace(
            self,
            records=tuple(sorted(records, key=lambda item: item.tag_id)),
            updated_at_s=time.time(),
        )

    def remove(self, tag_id: int) -> "TagDatabase":
        return replace(
            self,
            records=tuple(
                record
                for record in self.records
                if record.tag_id != int(tag_id)
            ),
            updated_at_s=time.time(),
        )

    def set_enabled(self, tag_id: int, enabled: bool) -> "TagDatabase":
        record = self.find_by_id(tag_id)

        if record is None:
            raise KeyError(f"Tag id not found: {tag_id}")

        return self.upsert(record.with_enabled(enabled))

    def enabled_records(self) -> tuple[TagRecord, ...]:
        return tuple(record for record in self.records if record.enabled)

    def to_apriltag_labels(self):
        return tuple(
            record.to_apriltag_label()
            for record in self.enabled_records()
        )

    def to_apriltag_mapper(self, *, min_confidence: float = 0.30):
        from savo_mapping.semantic.apriltag_mapper import AprilTagMapper

        return AprilTagMapper(
            labels=self.to_apriltag_labels(),
            min_confidence=min_confidence,
            allow_unknown_tags=False,
        )

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


def make_tag_record(
    *,
    tag_id: int,
    label: str,
    area_type: str = "apriltag_location",
    family: str = DEFAULT_TAG_FAMILY,
    size_m: Optional[float] = None,
    priority: int = 0,
    enabled: bool = True,
    confirmed: bool = True,
    aliases: Sequence[str] = (),
    notes: str = "",
    metadata: Optional[dict[str, Any]] = None,
) -> TagRecord:
    return TagRecord(
        tag_id=int(tag_id),
        label=str(label).strip(),
        area_type=str(area_type),
        family=str(family),
        size_m=size_m,
        priority=int(priority),
        enabled=bool(enabled),
        confirmed=bool(confirmed),
        aliases=tuple(str(alias).strip() for alias in aliases if str(alias).strip()),
        notes=str(notes),
        metadata=dict(metadata or {}),
    )


def tag_record_from_dict(data: dict[str, Any]) -> TagRecord:
    return TagRecord(
        tag_id=int(data["tag_id"]),
        label=str(data["label"]),
        area_type=str(data.get("area_type", "apriltag_location")),
        family=str(data.get("family", DEFAULT_TAG_FAMILY)),
        size_m=None if data.get("size_m") is None else float(data["size_m"]),
        priority=int(data.get("priority", 0)),
        enabled=bool(data.get("enabled", True)),
        confirmed=bool(data.get("confirmed", True)),
        aliases=tuple(str(alias) for alias in data.get("aliases", ())),
        notes=str(data.get("notes", "")),
        metadata=dict(data.get("metadata", {})),
        created_at_s=float(data.get("created_at_s", time.time())),
        updated_at_s=float(data.get("updated_at_s", time.time())),
    )


def tag_database_from_dict(data: dict[str, Any]) -> TagDatabase:
    return TagDatabase(
        map_name=str(data.get("map_name", "")),
        schema_version=int(data.get("schema_version", TAG_DATABASE_SCHEMA_VERSION)),
        updated_at_s=float(data.get("updated_at_s", time.time())),
        records=tuple(
            tag_record_from_dict(item)
            for item in data.get("records", ())
        ),
    )


def load_tag_database(path: str | Path) -> TagDatabase:
    database_path = Path(path)

    if not database_path.exists():
        return TagDatabase()

    data = json.loads(database_path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise ValueError("Tag database JSON root must be an object.")

    return tag_database_from_dict(data)


def save_tag_database(
    path: str | Path,
    database: TagDatabase,
    *,
    indent: int = 2,
) -> Path:
    database_path = Path(path)
    database_path.parent.mkdir(parents=True, exist_ok=True)
    database_path.write_text(
        database.to_json(indent=indent) + "\n",
        encoding="utf-8",
    )

    return database_path


def _ensure_unique_tag_ids(records: Sequence[TagRecord]) -> None:
    seen: set[int] = set()

    for record in records:
        if record.tag_id in seen:
            raise ValueError(f"Duplicate tag id: {record.tag_id}")

        seen.add(record.tag_id)


def main() -> None:
    database = TagDatabase(map_name="savonia_campus_heart")
    database = database.upsert(
        make_tag_record(
            tag_id=21,
            label="A201",
            area_type="room_entrance",
            priority=3,
            aliases=("A201 Entrance", "Room A201"),
        )
    )
    database = database.upsert(
        make_tag_record(
            tag_id=7,
            label="Info Desk",
            area_type="service",
            priority=5,
        )
    )

    print(database.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "DEFAULT_TAG_FAMILY",
    "TAG_DATABASE_SCHEMA_VERSION",
    "TagDatabase",
    "TagRecord",
    "load_tag_database",
    "main",
    "make_tag_record",
    "save_tag_database",
    "tag_database_from_dict",
    "tag_record_from_dict",
]