#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CLI smoke tests for Robot Savo mapping."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


# =============================================================================
# Paths
# =============================================================================
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = PACKAGE_ROOT / "scripts"


# =============================================================================
# Helpers
# =============================================================================
def run_cli(
    script_name: str,
    *args: str,
    expected_returncode: int = 0,
) -> subprocess.CompletedProcess[str]:
    script_path = SCRIPTS_DIR / script_name

    assert script_path.exists(), f"Missing CLI script: {script_path}"

    result = subprocess.run(
        [sys.executable, str(script_path), *args],
        cwd=PACKAGE_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == expected_returncode, (
        f"{script_name} returned {result.returncode}, "
        f"expected {expected_returncode}\n"
        f"STDOUT:\n{result.stdout}\n"
        f"STDERR:\n{result.stderr}"
    )

    return result


# =============================================================================
# Readiness / smoke tools
# =============================================================================
def test_mapping_readiness_cli_good() -> None:
    result = run_cli("mapping_readiness_cli.py", "--good")

    assert "savo_mapping_readiness:manual_mapping" in result.stdout
    assert "ready=true" in result.stdout
    assert "mode=manual_mapping" in result.stdout


def test_mapping_readiness_cli_bad_returns_2() -> None:
    result = run_cli(
        "mapping_readiness_cli.py",
        "--bad",
        expected_returncode=2,
    )

    assert "ready=false" in result.stdout
    assert "Mapping readiness failed." in result.stdout


def test_mapping_readiness_cli_autonomous_good_json() -> None:
    result = run_cli(
        "mapping_readiness_cli.py",
        "--mode",
        "autonomous_mapping",
        "--good",
        "--enable-map",
        "--enable-slam",
        "--enable-nav2",
        "--style",
        "json",
    )

    data = json.loads(result.stdout)

    assert data["ok"] is True
    assert data["name"] == "savo_mapping_readiness:autonomous_mapping"


def test_mapping_smoke_test_cli() -> None:
    result = run_cli("mapping_smoke_test_cli.py")

    assert "savo_mapping_smoke_test" in result.stdout
    assert "smoke_test_ok=true" in result.stdout


def test_mapping_smoke_test_cli_with_future_modules() -> None:
    result = run_cli("mapping_smoke_test_cli.py", "--include-future")

    assert "future_diagnostics" in result.stdout
    assert "smoke_test_ok=true" in result.stdout


# =============================================================================
# Params / metadata / quality tools
# =============================================================================
def test_dump_effective_params_cli() -> None:
    result = run_cli("dump_effective_params.py", "--section", "mapping_supervisor")

    assert "mapping_supervisor:" in result.stdout
    assert "scan_topic: /scan" in result.stdout
    assert "odom_topic: /odometry/filtered" in result.stdout


def test_dump_effective_params_cli_json() -> None:
    result = run_cli(
        "dump_effective_params.py",
        "--section",
        "mapping_supervisor",
        "--json",
    )

    data = json.loads(result.stdout)

    assert data["mapping_supervisor"]["scan_topic"] == "/scan"
    assert data["mapping_supervisor"]["odom_topic"] == "/odometry/filtered"


def test_map_quality_cli_good() -> None:
    result = run_cli("map_quality_cli.py")

    assert "Robot Savo map quality" in result.stdout
    assert "ok: true" in result.stdout
    assert "level: ok" in result.stdout


def test_map_quality_cli_bad_returns_2() -> None:
    result = run_cli(
        "map_quality_cli.py",
        "--width",
        "10",
        "--height",
        "10",
        "--resolution",
        "0.05",
        "--free-cells",
        "20",
        "--occupied-cells",
        "2",
        "--unknown-cells",
        "78",
        "--required",
        expected_returncode=2,
    )

    assert "ok: false" in result.stdout
    assert "level: error" in result.stdout


def test_map_metadata_cli() -> None:
    result = run_cli("map_metadata_cli.py")

    assert "Robot Savo map metadata" in result.stdout
    assert "name: savonia_campus_heart" in result.stdout
    assert "valid: True" in result.stdout


def test_map_metadata_cli_paths_only() -> None:
    result = run_cli("map_metadata_cli.py", "--paths-only")

    assert "Robot Savo map paths" in result.stdout
    assert "savonia_campus_heart.yaml" in result.stdout
    assert "savonia_campus_heart.pgm" in result.stdout
    assert "savonia_campus_heart.metadata.json" in result.stdout


def test_map_metadata_cli_write_json(tmp_path: Path) -> None:
    output_file = tmp_path / "map.metadata.json"

    result = run_cli(
        "map_metadata_cli.py",
        "--write-json",
        str(output_file),
        "--json",
    )

    assert output_file.exists()

    stdout_data = json.loads(result.stdout.split("\n", 1)[1])
    file_data = json.loads(output_file.read_text(encoding="utf-8"))

    assert stdout_data["name"] == "savonia_campus_heart"
    assert file_data["name"] == "savonia_campus_heart"


# =============================================================================
# Map load/save tools
# =============================================================================
def test_load_map_check_cli_missing_map_returns_2() -> None:
    result = run_cli(
        "load_map_check_cli.py",
        "--map-name",
        "Definitely Missing Test Map",
        expected_returncode=2,
    )

    assert "ok: false" in result.stdout
    assert "level: error" in result.stdout
    assert "yaml_missing" in result.stdout


def test_load_map_check_cli_valid_temp_map(tmp_path: Path) -> None:
    yaml_file = tmp_path / "test_map.yaml"
    image_file = tmp_path / "test_map.pgm"

    yaml_file.write_text(
        "image: test_map.pgm\n"
        "resolution: 0.05\n"
        "origin: [0.0, 0.0, 0.0]\n"
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

    result = run_cli(
        "load_map_check_cli.py",
        "--yaml-file",
        str(yaml_file),
    )

    assert "ok: true" in result.stdout
    assert "level: ok" in result.stdout
    assert "Map files ready." in result.stdout


def test_save_map_cli_prepares_paths() -> None:
    result = run_cli("save_map_cli.py")

    assert "Robot Savo save-map preparation" in result.stdout
    assert "map_name: savonia_campus_heart" in result.stdout
    assert "savonia_campus_heart.yaml" in result.stdout


def test_save_map_cli_write_metadata(tmp_path: Path) -> None:
    maps_dir = tmp_path / "maps"

    result = run_cli(
        "save_map_cli.py",
        "--maps-dir",
        str(maps_dir),
        "--width",
        "400",
        "--height",
        "250",
        "--resolution",
        "0.05",
        "--write-metadata",
    )

    metadata_file = maps_dir / "savonia_campus_heart.metadata.json"

    assert metadata_file.exists()
    assert "metadata_written:" in result.stdout

    data = json.loads(metadata_file.read_text(encoding="utf-8"))

    assert data["name"] == "savonia_campus_heart"
    assert data["valid"] is True


# =============================================================================
# Mapping helper tools
# =============================================================================
def test_manual_mapping_cli() -> None:
    result = run_cli("manual_mapping_cli.py", "--with-teleop", "--dry-status")

    assert "Robot Savo manual mapping session" in result.stdout
    assert "mode: manual_mapping" in result.stdout
    assert "use_rviz:=false" in result.stdout
    assert "teleop_letters_cli.py" in result.stdout
    assert "Simulated manual mapping status" in result.stdout


def test_autonomous_mapping_cli() -> None:
    result = run_cli(
        "autonomous_mapping_cli.py",
        "--with-pointcloud",
        "--dry-status",
        "--dry-goal",
    )

    assert "Robot Savo autonomous mapping session" in result.stdout
    assert "mode: autonomous_mapping" in result.stdout
    assert "use_rviz:=false" in result.stdout
    assert "pointcloud_enabled: True" in result.stdout
    assert "Simulated exploration status" in result.stdout


# =============================================================================
# Future optional tools
# =============================================================================
def test_pointcloud_echo_cli_dry() -> None:
    result = run_cli("pointcloud_echo_cli.py")

    assert "Robot Savo pointcloud dry check" in result.stdout
    assert "ok: true" in result.stdout
    assert "level: ok" in result.stdout


def test_voxel_costmap_check_cli_default_disabled() -> None:
    result = run_cli("voxel_costmap_check_cli.py")

    assert "Robot Savo voxel costmap check" in result.stdout
    assert "enabled: false" in result.stdout
    assert "level: disabled" in result.stdout


def test_voxel_costmap_check_cli_good() -> None:
    result = run_cli("voxel_costmap_check_cli.py", "--good")

    assert "ok: true" in result.stdout
    assert "level: ok" in result.stdout
    assert "Voxel layer ready." in result.stdout


def test_voxel_costmap_check_cli_bad_returns_2() -> None:
    result = run_cli(
        "voxel_costmap_check_cli.py",
        "--bad",
        expected_returncode=2,
    )

    assert "ok: false" in result.stdout
    assert "level: stale" in result.stdout


def test_apriltag_check_cli_default_disabled() -> None:
    result = run_cli("apriltag_check_cli.py")

    assert "Robot Savo AprilTag check" in result.stdout
    assert "enabled: false" in result.stdout
    assert "level: disabled" in result.stdout


def test_apriltag_check_cli_good() -> None:
    result = run_cli(
        "apriltag_check_cli.py",
        "--good",
        "--require-known-label",
    )

    assert "ok: true" in result.stdout
    assert "level: ok" in result.stdout
    assert "last_tag_id: 21" in result.stdout
    assert "location_key: a201" in result.stdout


def test_apriltag_check_cli_bad_returns_2() -> None:
    result = run_cli(
        "apriltag_check_cli.py",
        "--bad",
        "--require-known-label",
        expected_returncode=2,
    )

    assert "ok: false" in result.stdout
    assert "level: stale" in result.stdout


def test_semantic_landmark_cli_default() -> None:
    result = run_cli("semantic_landmark_cli.py")

    assert "Robot Savo semantic landmark" in result.stdout
    assert "key: info_desk" in result.stdout
    assert "state: candidate" in result.stdout
    assert "confirmed: false" in result.stdout


def test_semantic_landmark_cli_confirmed() -> None:
    result = run_cli(
        "semantic_landmark_cli.py",
        "--label",
        "A201",
        "--tag-id",
        "21",
        "--x",
        "4.25",
        "--y",
        "8.70",
        "--yaw",
        "1.57",
        "--confidence",
        "0.92",
        "--confirm",
    )

    assert "key: a201" in result.stdout
    assert "state: confirmed" in result.stdout
    assert "confirmed: true" in result.stdout
    assert "tag_id: 21" in result.stdout


def test_location_bridge_check_cli_default_disabled() -> None:
    result = run_cli("location_bridge_check_cli.py")

    assert "Robot Savo location bridge check" in result.stdout
    assert "enabled: false" in result.stdout
    assert "level: disabled" in result.stdout


def test_location_bridge_check_cli_good() -> None:
    result = run_cli("location_bridge_check_cli.py", "--good")

    assert "ok: true" in result.stdout
    assert "level: ok" in result.stdout
    assert "Location bridge ready." in result.stdout


def test_location_bridge_check_cli_bad_returns_2() -> None:
    result = run_cli(
        "location_bridge_check_cli.py",
        "--bad",
        expected_returncode=2,
    )

    assert "ok: false" in result.stdout
    assert "level: stale" in result.stdout


# =============================================================================
# Cleanup / operator tools
# =============================================================================
def test_clean_maps_cli_no_selection() -> None:
    result = run_cli("clean_maps_cli.py")

    assert "Nothing selected" in result.stdout


def test_clean_maps_cli_dry_run_all() -> None:
    result = run_cli("clean_maps_cli.py", "--all")

    assert "Robot Savo map cleanup" in result.stdout
    assert "dry_run: true" in result.stdout


def test_operator_notes_cli_manual() -> None:
    result = run_cli("operator_notes_cli.py", "--mode", "manual")

    assert "Robot Savo manual mapping notes" in result.stdout
    assert "Do not run RViz on the Pi." in result.stdout
    assert "use_rviz:=false" in result.stdout


def test_operator_notes_cli_rviz_macbook() -> None:
    result = run_cli(
        "operator_notes_cli.py",
        "--mode",
        "rviz",
        "--operator-machine",
        "macbook",
    )

    assert "Robot Savo RViz notes" in result.stdout
    assert "Run RViz on MacBook Air" in result.stdout
    assert "not on the Raspberry Pi" in result.stdout