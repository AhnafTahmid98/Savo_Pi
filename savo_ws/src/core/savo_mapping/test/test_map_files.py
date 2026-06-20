#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unit tests for Robot Savo saved-map file helpers."""

from __future__ import annotations

import json
from pathlib import Path
from tempfile import TemporaryDirectory

import pytest

from savo_mapping.diagnostics.map_file_check import (
    evaluate_map_files,
    evaluate_map_files_from_paths,
    map_file_result_to_diagnostic,
)
from savo_mapping.models.map_metadata import (
    MapOrigin,
    make_map_metadata,
    make_saved_map_paths,
    map_metadata_from_dict,
    sanitize_map_name,
)


# =============================================================================
# Helpers
# =============================================================================
def write_test_map(
    directory: Path,
    name: str = "test_map",
    image_name: str | None = None,
    resolution: float = 0.05,
    origin: str = "[0.0, 0.0, 0.0]",
) -> tuple[Path, Path]:
    image = image_name or f"{name}.pgm"
    yaml_file = directory / f"{name}.yaml"
    image_file = directory / image

    yaml_file.write_text(
        f"image: {image}\n"
        f"resolution: {resolution}\n"
        f"origin: {origin}\n"
        "mode: trinary\n",
        encoding="utf-8",
    )

    image_file.write_text(
        "P2\n"
        "1 1\n"
        "255\n"
        "0\n",
        encoding="utf-8",
    )

    return yaml_file, image_file


def write_metadata_file(path: Path, map_name: str = "test_map") -> Path:
    metadata = make_map_metadata(
        name=map_name,
        width_cells=100,
        height_cells=80,
        resolution_m=0.05,
        frame_id="map",
        origin=MapOrigin(x=0.0, y=0.0, yaw=0.0),
        image_file=f"{map_name}.pgm",
        yaml_file=f"{map_name}.yaml",
    )

    path.write_text(
        metadata.to_json(indent=2) + "\n",
        encoding="utf-8",
    )

    return path


# =============================================================================
# Map name and path helpers
# =============================================================================
def test_sanitize_map_name_for_saved_files() -> None:
    assert sanitize_map_name("Savonia Campus Heart") == "savonia_campus_heart"
    assert sanitize_map_name(" A201 / Robot Lab!! ") == "a201_robot_lab"


def test_make_saved_map_paths_default_extension() -> None:
    paths = make_saved_map_paths(
        "Savonia Campus Heart",
        maps_dir="maps/saved",
    )

    assert paths["name"] == "savonia_campus_heart"
    assert paths["base_dir"] == "maps/saved"
    assert paths["yaml_file"] == "maps/saved/savonia_campus_heart.yaml"
    assert paths["image_file"] == "maps/saved/savonia_campus_heart.pgm"
    assert paths["metadata_file"] == "maps/saved/savonia_campus_heart.metadata.json"


def test_make_saved_map_paths_custom_extension() -> None:
    paths = make_saved_map_paths(
        "Savonia Campus Heart",
        maps_dir="maps/saved",
        image_ext=".png",
    )

    assert paths["image_file"] == "maps/saved/savonia_campus_heart.png"


# =============================================================================
# Map metadata
# =============================================================================
def test_make_map_metadata_valid_map() -> None:
    metadata = make_map_metadata(
        name="Savonia Campus Heart",
        width_cells=400,
        height_cells=250,
        resolution_m=0.05,
        frame_id="map",
        origin=MapOrigin(x=-1.0, y=-2.0, yaw=0.0),
        image_file="maps/saved/savonia_campus_heart.pgm",
        yaml_file="maps/saved/savonia_campus_heart.yaml",
        session_id="session_001",
    )

    assert metadata.name == "savonia_campus_heart"
    assert metadata.valid is True
    assert metadata.width_m == pytest.approx(20.0)
    assert metadata.height_m == pytest.approx(12.5)
    assert metadata.origin.x == pytest.approx(-1.0)
    assert metadata.origin.y == pytest.approx(-2.0)
    assert metadata.session_id == "session_001"


def test_map_metadata_roundtrip_from_dict() -> None:
    metadata = make_map_metadata(
        name="Robot Lab",
        width_cells=200,
        height_cells=100,
        resolution_m=0.05,
        image_file="robot_lab.pgm",
        yaml_file="robot_lab.yaml",
    )

    loaded = map_metadata_from_dict(metadata.to_dict())

    assert loaded.name == "robot_lab"
    assert loaded.valid is True
    assert loaded.width_cells == 200
    assert loaded.height_cells == 100
    assert loaded.resolution_m == pytest.approx(0.05)
    assert loaded.image_file == "robot_lab.pgm"
    assert loaded.yaml_file == "robot_lab.yaml"


def test_map_metadata_json_is_valid() -> None:
    metadata = make_map_metadata(
        name="Info Desk",
        width_cells=100,
        height_cells=80,
        resolution_m=0.05,
    )

    data = json.loads(metadata.to_json())

    assert data["name"] == "info_desk"
    assert data["valid"] is True
    assert data["width_cells"] == 100
    assert data["height_cells"] == 80


# =============================================================================
# evaluate_map_files_from_paths
# =============================================================================
def test_evaluate_map_files_from_paths_valid_map() -> None:
    with TemporaryDirectory() as tmp:
        base = Path(tmp)
        yaml_file, image_file = write_test_map(base, "test_map")

        result = evaluate_map_files_from_paths(yaml_file)

        assert result.ok is True
        assert result.map_name == "test_map"
        assert result.yaml_exists is True
        assert result.image_exists is True
        assert result.metadata_exists is False
        assert result.yaml_size_bytes > 0
        assert result.image_size_bytes > 0
        assert result.resolution_m == pytest.approx(0.05)
        assert result.image_ref == "test_map.pgm"
        assert result.origin == [0.0, 0.0, 0.0]
        assert result.mode == "trinary"


def test_evaluate_map_files_from_paths_valid_map_with_metadata_required() -> None:
    with TemporaryDirectory() as tmp:
        base = Path(tmp)
        yaml_file, _ = write_test_map(base, "test_map")
        metadata_file = write_metadata_file(base / "test_map.metadata.json", "test_map")

        result = evaluate_map_files_from_paths(
            yaml_file=yaml_file,
            metadata_file=metadata_file,
            require_metadata=True,
        )

        assert result.ok is True
        assert result.metadata_exists is True
        assert result.metadata_size_bytes > 0


def test_evaluate_map_files_from_paths_missing_metadata_fails_when_required() -> None:
    with TemporaryDirectory() as tmp:
        base = Path(tmp)
        yaml_file, _ = write_test_map(base, "test_map")

        result = evaluate_map_files_from_paths(
            yaml_file=yaml_file,
            require_metadata=True,
        )

        diagnostic = map_file_result_to_diagnostic(result, required=True)

        assert result.ok is False
        assert "metadata_missing" in result.extra["failures"]
        assert diagnostic.level == "error"


def test_evaluate_map_files_from_paths_missing_yaml_fails() -> None:
    with TemporaryDirectory() as tmp:
        yaml_file = Path(tmp) / "missing_map.yaml"

        result = evaluate_map_files_from_paths(yaml_file)
        diagnostic = map_file_result_to_diagnostic(result, required=True)

        assert result.ok is False
        assert result.yaml_exists is False
        assert result.image_exists is False
        assert "yaml_missing" in result.extra["failures"]
        assert "image_missing" in result.extra["failures"]
        assert diagnostic.level == "error"


def test_evaluate_map_files_from_paths_missing_image_fails() -> None:
    with TemporaryDirectory() as tmp:
        base = Path(tmp)
        yaml_file = base / "test_map.yaml"

        yaml_file.write_text(
            "image: missing_map.pgm\n"
            "resolution: 0.05\n"
            "origin: [0.0, 0.0, 0.0]\n"
            "mode: trinary\n",
            encoding="utf-8",
        )

        result = evaluate_map_files_from_paths(yaml_file)

        assert result.ok is False
        assert result.yaml_exists is True
        assert result.image_exists is False
        assert "image_missing" in result.extra["failures"]


def test_evaluate_map_files_from_paths_missing_resolution_fails() -> None:
    with TemporaryDirectory() as tmp:
        base = Path(tmp)
        yaml_file = base / "test_map.yaml"
        image_file = base / "test_map.pgm"

        yaml_file.write_text(
            "image: test_map.pgm\n"
            "origin: [0.0, 0.0, 0.0]\n"
            "mode: trinary\n",
            encoding="utf-8",
        )
        image_file.write_text("P2\n1 1\n255\n0\n", encoding="utf-8")

        result = evaluate_map_files_from_paths(yaml_file)

        assert result.ok is False
        assert "resolution_missing" in result.extra["failures"]


def test_evaluate_map_files_from_paths_missing_image_ref_fails() -> None:
    with TemporaryDirectory() as tmp:
        base = Path(tmp)
        yaml_file = base / "test_map.yaml"
        image_file = base / "test_map.pgm"

        yaml_file.write_text(
            "resolution: 0.05\n"
            "origin: [0.0, 0.0, 0.0]\n"
            "mode: trinary\n",
            encoding="utf-8",
        )
        image_file.write_text("P2\n1 1\n255\n0\n", encoding="utf-8")

        result = evaluate_map_files_from_paths(
            yaml_file=yaml_file,
            image_file=image_file,
        )

        assert result.ok is False
        assert result.image_exists is True
        assert "image_ref_missing" in result.extra["failures"]


def test_evaluate_map_files_from_paths_missing_origin_fails() -> None:
    with TemporaryDirectory() as tmp:
        base = Path(tmp)
        yaml_file = base / "test_map.yaml"
        image_file = base / "test_map.pgm"

        yaml_file.write_text(
            "image: test_map.pgm\n"
            "resolution: 0.05\n"
            "mode: trinary\n",
            encoding="utf-8",
        )
        image_file.write_text("P2\n1 1\n255\n0\n", encoding="utf-8")

        result = evaluate_map_files_from_paths(yaml_file)

        assert result.ok is False
        assert "origin_missing" in result.extra["failures"]


def test_evaluate_map_files_from_paths_absolute_image_reference() -> None:
    with TemporaryDirectory() as tmp:
        base = Path(tmp)
        image_file = base / "absolute_image.pgm"
        yaml_file = base / "test_map.yaml"

        image_file.write_text("P2\n1 1\n255\n0\n", encoding="utf-8")
        yaml_file.write_text(
            f"image: {image_file}\n"
            "resolution: 0.05\n"
            "origin: [0.0, 0.0, 0.0]\n"
            "mode: trinary\n",
            encoding="utf-8",
        )

        result = evaluate_map_files_from_paths(yaml_file)

        assert result.ok is True
        assert Path(result.image_file) == image_file.resolve()


def test_evaluate_map_files_from_paths_invalid_yaml_root_fails() -> None:
    with TemporaryDirectory() as tmp:
        base = Path(tmp)
        yaml_file = base / "bad_map.yaml"

        yaml_file.write_text(
            "- not\n"
            "- mapping\n",
            encoding="utf-8",
        )

        result = evaluate_map_files_from_paths(yaml_file)

        assert result.ok is False
        assert "yaml_invalid" in result.extra["failures"]


# =============================================================================
# evaluate_map_files by map name
# =============================================================================
def test_evaluate_map_files_by_map_name_valid() -> None:
    with TemporaryDirectory() as tmp:
        maps_dir = Path(tmp)
        write_test_map(maps_dir, "savonia_campus_heart")

        result = evaluate_map_files(
            map_name="Savonia Campus Heart",
            maps_dir=maps_dir,
        )

        assert result.ok is True
        assert result.map_name == "savonia_campus_heart"
        assert result.yaml_exists is True
        assert result.image_exists is True


def test_evaluate_map_files_by_map_name_missing() -> None:
    with TemporaryDirectory() as tmp:
        result = evaluate_map_files(
            map_name="Missing Map",
            maps_dir=tmp,
        )

        assert result.ok is False
        assert result.map_name == "missing_map"
        assert "yaml_missing" in result.extra["failures"]
        assert "image_missing" in result.extra["failures"]


def test_evaluate_map_files_by_map_name_with_png_extension() -> None:
    with TemporaryDirectory() as tmp:
        maps_dir = Path(tmp)
        yaml_file = maps_dir / "test_map.yaml"
        image_file = maps_dir / "test_map.png"

        yaml_file.write_text(
            "image: test_map.png\n"
            "resolution: 0.05\n"
            "origin: [0.0, 0.0, 0.0]\n"
            "mode: trinary\n",
            encoding="utf-8",
        )
        image_file.write_text("fake png placeholder\n", encoding="utf-8")

        result = evaluate_map_files(
            map_name="test_map",
            maps_dir=maps_dir,
            image_ext=".png",
        )

        assert result.ok is True
        assert result.image_file.endswith("test_map.png")


# =============================================================================
# Diagnostics
# =============================================================================
def test_map_file_result_to_diagnostic_ok() -> None:
    with TemporaryDirectory() as tmp:
        yaml_file, _ = write_test_map(Path(tmp), "test_map")

        result = evaluate_map_files_from_paths(yaml_file)
        diagnostic = map_file_result_to_diagnostic(result, required=True)

        assert diagnostic.name == "map_files"
        assert diagnostic.level == "ok"
        assert diagnostic.required is True
        assert diagnostic.ok is True


def test_map_file_result_to_diagnostic_warn_when_optional() -> None:
    with TemporaryDirectory() as tmp:
        result = evaluate_map_files(
            map_name="missing_map",
            maps_dir=tmp,
        )

        diagnostic = map_file_result_to_diagnostic(result, required=False)

        assert diagnostic.name == "map_files"
        assert diagnostic.level == "warn"
        assert diagnostic.required is False
        assert diagnostic.warning is True


def test_map_file_result_to_diagnostic_error_when_required() -> None:
    with TemporaryDirectory() as tmp:
        result = evaluate_map_files(
            map_name="missing_map",
            maps_dir=tmp,
        )

        diagnostic = map_file_result_to_diagnostic(result, required=True)

        assert diagnostic.name == "map_files"
        assert diagnostic.level == "error"
        assert diagnostic.required is True
        assert diagnostic.failed is True