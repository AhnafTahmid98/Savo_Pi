#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Saved map file checks. No ROS imports."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

from savo_mapping.models.map_metadata import make_saved_map_paths, sanitize_map_name
from savo_mapping.utils.diagnostics import (
    DiagnosticItem,
    make_error,
    make_ok,
    make_warn,
)


# =============================================================================
# Map file result
# =============================================================================
@dataclass(frozen=True)
class MapFileResult:
    ok: bool
    map_name: str
    yaml_file: str
    image_file: str
    metadata_file: str

    yaml_exists: bool = False
    image_exists: bool = False
    metadata_exists: bool = False

    yaml_size_bytes: int = 0
    image_size_bytes: int = 0
    metadata_size_bytes: int = 0

    resolution_m: Optional[float] = None
    image_ref: Optional[str] = None
    mode: Optional[str] = None
    origin: Optional[list[float]] = None

    message: str = "Map files not checked."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "map_name": self.map_name,
            "yaml_file": self.yaml_file,
            "image_file": self.image_file,
            "metadata_file": self.metadata_file,
            "yaml_exists": self.yaml_exists,
            "image_exists": self.image_exists,
            "metadata_exists": self.metadata_exists,
            "yaml_size_bytes": self.yaml_size_bytes,
            "image_size_bytes": self.image_size_bytes,
            "metadata_size_bytes": self.metadata_size_bytes,
            "resolution_m": self.resolution_m,
            "image_ref": self.image_ref,
            "mode": self.mode,
            "origin": self.origin,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# File helpers
# =============================================================================
def _file_size(path: Path) -> int:
    if not path.exists() or not path.is_file():
        return 0

    return int(path.stat().st_size)


def _load_map_yaml(path: Path) -> Dict[str, Any]:
    import yaml

    if not path.exists() or not path.is_file():
        return {}

    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    if data is None:
        return {}

    if not isinstance(data, dict):
        raise ValueError(f"Map YAML root must be a mapping: {path}")

    return dict(data)


def _resolve_image_path(yaml_file: Path, image_ref: str) -> Path:
    image_path = Path(image_ref).expanduser()

    if image_path.is_absolute():
        return image_path

    return (yaml_file.parent / image_path).resolve()


# =============================================================================
# Check logic
# =============================================================================
def evaluate_map_files(
    map_name: str,
    maps_dir: str | Path = "maps/saved",
    image_ext: str = ".pgm",
    require_metadata: bool = False,
    extra: Optional[Dict[str, Any]] = None,
) -> MapFileResult:
    clean_name = sanitize_map_name(map_name)
    paths = make_saved_map_paths(clean_name, maps_dir=maps_dir, image_ext=image_ext)

    yaml_path = Path(paths["yaml_file"]).expanduser()
    image_path = Path(paths["image_file"]).expanduser()
    metadata_path = Path(paths["metadata_file"]).expanduser()

    yaml_exists = yaml_path.exists() and yaml_path.is_file()
    yaml_data: Dict[str, Any] = {}

    failures: list[str] = []

    if not yaml_exists:
        failures.append("yaml_missing")
    else:
        try:
            yaml_data = _load_map_yaml(yaml_path)
        except Exception as exc:
            failures.append("yaml_invalid")
            yaml_data = {"load_error": str(exc)}

    image_ref = yaml_data.get("image")

    if isinstance(image_ref, str) and image_ref.strip():
        image_path = _resolve_image_path(yaml_path, image_ref.strip())

    image_exists = image_path.exists() and image_path.is_file()
    metadata_exists = metadata_path.exists() and metadata_path.is_file()

    if not image_exists:
        failures.append("image_missing")

    if require_metadata and not metadata_exists:
        failures.append("metadata_missing")

    resolution = yaml_data.get("resolution")
    mode = yaml_data.get("mode")
    origin = yaml_data.get("origin")

    if yaml_exists and resolution is None:
        failures.append("resolution_missing")

    if yaml_exists and image_ref is None:
        failures.append("image_ref_missing")

    if yaml_exists and origin is None:
        failures.append("origin_missing")

    ok = not failures

    if ok:
        message = "Map files ready."
    else:
        message = f"Map files not ready: {', '.join(failures)}."

    return MapFileResult(
        ok=ok,
        map_name=clean_name,
        yaml_file=str(yaml_path),
        image_file=str(image_path),
        metadata_file=str(metadata_path),
        yaml_exists=yaml_exists,
        image_exists=image_exists,
        metadata_exists=metadata_exists,
        yaml_size_bytes=_file_size(yaml_path),
        image_size_bytes=_file_size(image_path),
        metadata_size_bytes=_file_size(metadata_path),
        resolution_m=float(resolution) if resolution is not None else None,
        image_ref=str(image_ref) if image_ref is not None else None,
        mode=str(mode) if mode is not None else None,
        origin=list(origin) if isinstance(origin, list) else None,
        message=message,
        extra={
            "failures": failures,
            "require_metadata": require_metadata,
            **dict(extra or {}),
        },
    )


def evaluate_map_files_from_paths(
    yaml_file: str | Path,
    image_file: Optional[str | Path] = None,
    metadata_file: Optional[str | Path] = None,
    require_metadata: bool = False,
    extra: Optional[Dict[str, Any]] = None,
) -> MapFileResult:
    yaml_path = Path(yaml_file).expanduser().resolve()
    map_name = sanitize_map_name(yaml_path.stem)

    yaml_exists = yaml_path.exists() and yaml_path.is_file()
    yaml_data: Dict[str, Any] = {}

    failures: list[str] = []

    if not yaml_exists:
        failures.append("yaml_missing")
    else:
        try:
            yaml_data = _load_map_yaml(yaml_path)
        except Exception as exc:
            failures.append("yaml_invalid")
            yaml_data = {"load_error": str(exc)}

    image_ref = yaml_data.get("image")

    if image_file is not None:
        image_path = Path(image_file).expanduser().resolve()
    elif isinstance(image_ref, str) and image_ref.strip():
        image_path = _resolve_image_path(yaml_path, image_ref.strip())
    else:
        image_path = yaml_path.with_suffix(".pgm")

    if metadata_file is not None:
        metadata_path = Path(metadata_file).expanduser().resolve()
    else:
        metadata_path = yaml_path.with_suffix(".metadata.json")

    image_exists = image_path.exists() and image_path.is_file()
    metadata_exists = metadata_path.exists() and metadata_path.is_file()

    if not image_exists:
        failures.append("image_missing")

    if require_metadata and not metadata_exists:
        failures.append("metadata_missing")

    resolution = yaml_data.get("resolution")
    mode = yaml_data.get("mode")
    origin = yaml_data.get("origin")

    if yaml_exists and resolution is None:
        failures.append("resolution_missing")

    if yaml_exists and image_ref is None:
        failures.append("image_ref_missing")

    if yaml_exists and origin is None:
        failures.append("origin_missing")

    ok = not failures

    if ok:
        message = "Map files ready."
    else:
        message = f"Map files not ready: {', '.join(failures)}."

    return MapFileResult(
        ok=ok,
        map_name=map_name,
        yaml_file=str(yaml_path),
        image_file=str(image_path),
        metadata_file=str(metadata_path),
        yaml_exists=yaml_exists,
        image_exists=image_exists,
        metadata_exists=metadata_exists,
        yaml_size_bytes=_file_size(yaml_path),
        image_size_bytes=_file_size(image_path),
        metadata_size_bytes=_file_size(metadata_path),
        resolution_m=float(resolution) if resolution is not None else None,
        image_ref=str(image_ref) if image_ref is not None else None,
        mode=str(mode) if mode is not None else None,
        origin=list(origin) if isinstance(origin, list) else None,
        message=message,
        extra={
            "failures": failures,
            "require_metadata": require_metadata,
            **dict(extra or {}),
        },
    )


def map_file_result_to_diagnostic(
    result: MapFileResult,
    required: bool = False,
) -> DiagnosticItem:
    values = result.to_dict()

    if result.ok:
        return make_ok(
            "map_files",
            result.message,
            required=required,
            values=values,
        )

    if required:
        return make_error(
            "map_files",
            result.message,
            required=required,
            values=values,
        )

    return make_warn(
        "map_files",
        result.message,
        required=required,
        values=values,
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    result = evaluate_map_files(
        map_name="Savonia Campus Heart",
        maps_dir="maps/saved",
        require_metadata=False,
    )

    print(result.to_json(indent=2))
    print(map_file_result_to_diagnostic(result).to_dict())


if __name__ == "__main__":
    main()


__all__ = [
    "MapFileResult",
    "evaluate_map_files",
    "evaluate_map_files_from_paths",
    "map_file_result_to_diagnostic",
    "main",
]