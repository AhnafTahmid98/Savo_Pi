#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Map metadata model. No ROS imports."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional


# =============================================================================
# Map origin
# =============================================================================
@dataclass(frozen=True)
class MapOrigin:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def to_dict(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
        }


# =============================================================================
# Map metadata
# =============================================================================
@dataclass(frozen=True)
class MapMetadata:
    name: str
    frame_id: str = "map"

    width_cells: int = 0
    height_cells: int = 0
    resolution_m: float = 0.0
    origin: MapOrigin = field(default_factory=MapOrigin)

    image_file: Optional[str] = None
    yaml_file: Optional[str] = None
    session_id: Optional[str] = None

    created_at_s: float = field(default_factory=time.time)
    updated_at_s: float = field(default_factory=time.time)

    extra: Dict[str, Any] = field(default_factory=dict)

    @property
    def width_m(self) -> float:
        return max(0, self.width_cells) * max(0.0, self.resolution_m)

    @property
    def height_m(self) -> float:
        return max(0, self.height_cells) * max(0.0, self.resolution_m)

    @property
    def valid(self) -> bool:
        return (
            bool(self.name.strip())
            and self.width_cells > 0
            and self.height_cells > 0
            and self.resolution_m > 0.0
        )

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "frame_id": self.frame_id,
            "width_cells": self.width_cells,
            "height_cells": self.height_cells,
            "resolution_m": self.resolution_m,
            "width_m": self.width_m,
            "height_m": self.height_m,
            "origin": self.origin.to_dict(),
            "image_file": self.image_file,
            "yaml_file": self.yaml_file,
            "session_id": self.session_id,
            "created_at_s": self.created_at_s,
            "updated_at_s": self.updated_at_s,
            "valid": self.valid,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Helpers
# =============================================================================
def sanitize_map_name(name: str) -> str:
    value = str(name).strip().lower()

    if not value:
        raise ValueError("Map name cannot be empty.")

    clean = []

    for char in value:
        if char.isalnum():
            clean.append(char)
        elif char in ("-", "_", " "):
            clean.append("_")

    result = "".join(clean)

    while "__" in result:
        result = result.replace("__", "_")

    result = result.strip("_")

    if not result:
        raise ValueError("Map name does not contain usable characters.")

    return result


def make_map_metadata(
    name: str,
    width_cells: int = 0,
    height_cells: int = 0,
    resolution_m: float = 0.0,
    frame_id: str = "map",
    origin: Optional[MapOrigin] = None,
    image_file: Optional[str] = None,
    yaml_file: Optional[str] = None,
    session_id: Optional[str] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> MapMetadata:
    now = time.time()

    return MapMetadata(
        name=sanitize_map_name(name),
        frame_id=str(frame_id).strip() or "map",
        width_cells=max(0, int(width_cells)),
        height_cells=max(0, int(height_cells)),
        resolution_m=max(0.0, float(resolution_m)),
        origin=origin or MapOrigin(),
        image_file=image_file,
        yaml_file=yaml_file,
        session_id=session_id,
        created_at_s=now,
        updated_at_s=now,
        extra=dict(extra or {}),
    )


def map_metadata_from_dict(data: Dict[str, Any]) -> MapMetadata:
    origin_data = dict(data.get("origin", {}))

    origin = MapOrigin(
        x=float(origin_data.get("x", 0.0)),
        y=float(origin_data.get("y", 0.0)),
        yaw=float(origin_data.get("yaw", 0.0)),
    )

    created_at = float(data.get("created_at_s", time.time()))
    updated_at = float(data.get("updated_at_s", created_at))

    return MapMetadata(
        name=sanitize_map_name(str(data.get("name", "unnamed_map"))),
        frame_id=str(data.get("frame_id", "map")),
        width_cells=max(0, int(data.get("width_cells", 0))),
        height_cells=max(0, int(data.get("height_cells", 0))),
        resolution_m=max(0.0, float(data.get("resolution_m", 0.0))),
        origin=origin,
        image_file=data.get("image_file"),
        yaml_file=data.get("yaml_file"),
        session_id=data.get("session_id"),
        created_at_s=created_at,
        updated_at_s=updated_at,
        extra=dict(data.get("extra", {})),
    )


def make_saved_map_paths(
    map_name: str,
    maps_dir: str | Path = "maps/saved",
    image_ext: str = ".pgm",
) -> dict:
    clean_name = sanitize_map_name(map_name)
    base_dir = Path(maps_dir)

    ext = image_ext if str(image_ext).startswith(".") else f".{image_ext}"

    return {
        "name": clean_name,
        "base_dir": str(base_dir),
        "yaml_file": str(base_dir / f"{clean_name}.yaml"),
        "image_file": str(base_dir / f"{clean_name}{ext}"),
        "metadata_file": str(base_dir / f"{clean_name}.metadata.json"),
    }


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    meta = make_map_metadata(
        name="Savonia Campus Heart",
        width_cells=400,
        height_cells=250,
        resolution_m=0.05,
        image_file="maps/saved/savonia_campus_heart.pgm",
        yaml_file="maps/saved/savonia_campus_heart.yaml",
    )

    print(meta.to_json(indent=2))
    print(make_saved_map_paths(meta.name))


if __name__ == "__main__":
    main()


__all__ = [
    "MapOrigin",
    "MapMetadata",
    "sanitize_map_name",
    "make_map_metadata",
    "map_metadata_from_dict",
    "make_saved_map_paths",
    "main",
]
