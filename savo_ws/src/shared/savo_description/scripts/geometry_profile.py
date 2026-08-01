#!/usr/bin/env python3
"""Strict Robot SAVO geometry profile validation and deterministic derivation."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml


MOUNTS = (
    "lidar",
    "imu",
    "realsense_d435",
    "tof_left",
    "tof_right",
    "ultrasonic_front",
    "display",
    "respeaker",
    "pantilt_mount",
)

REQUIRED_FRAMES = (
    "map",
    "odom",
    "base_footprint",
    "base_link",
    "lidar",
    "imu",
    "camera",
    "tof_left",
    "tof_right",
    "ultrasonic_front",
    "pantilt_mount",
    "pantilt_pan",
    "pantilt_tilt",
    "pi_camera",
    "pi_camera_optical",
)


class GeometryProfileError(ValueError):
    """Raised when a safety-critical geometry contract is invalid."""


def _mapping(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise GeometryProfileError(f"{field} must be a mapping")
    return value


def _finite(value: Any, field: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise GeometryProfileError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise GeometryProfileError(f"{field} must be finite")
    if positive and result <= 0.0:
        raise GeometryProfileError(f"{field} must be positive")
    return result


def _vector(value: Any, field: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise GeometryProfileError(f"{field} must contain exactly three values")
    return tuple(_finite(item, f"{field}[{index}]") for index, item in enumerate(value))


def load_profile(path: str | Path) -> dict[str, Any]:
    profile_path = Path(path)
    data = yaml.safe_load(profile_path.read_text())
    return _mapping(data, str(profile_path))


def validate_profile(
    profile: dict[str, Any],
    *,
    require_locked: bool = False,
    allow_provisional: bool = False,
) -> None:
    metadata = _mapping(profile.get("metadata"), "metadata")
    required_metadata = (
        "profile_id",
        "schema_version",
        "robot_name",
        "units",
        "measurement_state",
        "measured_by",
        "measurement_date",
        "source",
        "geometry_revision",
        "notes",
    )
    missing = [key for key in required_metadata if key not in metadata]
    if missing:
        raise GeometryProfileError(f"metadata missing: {', '.join(missing)}")
    if metadata["schema_version"] != 1:
        raise GeometryProfileError("unsupported geometry schema_version")
    if metadata["measurement_state"] not in {"provisional", "locked"}:
        raise GeometryProfileError("measurement_state must be provisional or locked")
    if require_locked and metadata["measurement_state"] != "locked" and not allow_provisional:
        raise GeometryProfileError(
            "production requires locked geometry; use the explicit controlled-test override"
        )
    if metadata["measurement_state"] == "locked":
        if not str(metadata["measured_by"]).strip() or not str(metadata["measurement_date"]).strip():
            raise GeometryProfileError("locked geometry requires measured_by and measurement_date")

    convention = _mapping(profile.get("coordinate_convention"), "coordinate_convention")
    if convention != {"x": "forward", "y": "left", "z": "up"}:
        raise GeometryProfileError("coordinate convention must be +X forward, +Y left, +Z up")

    chassis = _mapping(profile.get("chassis"), "chassis")
    for key in (
        "length_m",
        "width_m",
        "height_m",
        "base_footprint_to_base_link_z_m",
        "mass_kg",
        "deck_thickness_m",
        "deck_spacing_m",
        "deck_mass_each_kg",
    ):
        _finite(chassis.get(key), f"chassis.{key}", positive=True)

    wheels = _mapping(profile.get("wheels"), "wheels")
    for key in ("radius_m", "width_m", "mass_each_kg"):
        _finite(wheels.get(key), f"wheels.{key}", positive=True)
    front_x = _finite(wheels.get("front_x_m"), "wheels.front_x_m")
    rear_x = _finite(wheels.get("rear_x_m"), "wheels.rear_x_m")
    left_y = _finite(wheels.get("left_y_m"), "wheels.left_y_m")
    right_y = _finite(wheels.get("right_y_m"), "wheels.right_y_m")
    _finite(wheels.get("z_m"), "wheels.z_m")
    if not front_x > 0.0 or not rear_x < 0.0 or not math.isclose(front_x, -rear_x):
        raise GeometryProfileError("front/rear wheel X offsets must be symmetric")
    if not left_y > 0.0 or not right_y < 0.0 or not math.isclose(left_y, -right_y):
        raise GeometryProfileError("left/right wheel Y offsets must be symmetric")
    if abs(front_x) > chassis["length_m"] / 2.0 or abs(left_y) > chassis["width_m"] / 2.0:
        raise GeometryProfileError("wheel centers must remain inside the chassis extents")

    frames = _mapping(profile.get("frames"), "frames")
    missing_frames = [key for key in REQUIRED_FRAMES if not str(frames.get(key, "")).strip()]
    if missing_frames:
        raise GeometryProfileError(f"required frames missing: {', '.join(missing_frames)}")
    values = [str(value) for value in frames.values()]
    if len(values) != len(set(values)):
        raise GeometryProfileError("frame names must be unique")

    mounts = _mapping(profile.get("mounts"), "mounts")
    edges: list[tuple[str, str]] = [(frames["base_footprint"], frames["base_link"])]
    for name in MOUNTS:
        mount = _mapping(mounts.get(name), f"mounts.{name}")
        _vector(mount.get("xyz_m"), f"mounts.{name}.xyz_m")
        rpy = _vector(mount.get("rpy_rad"), f"mounts.{name}.rpy_rad")
        if any(abs(angle) > math.pi * 2.0 for angle in rpy):
            raise GeometryProfileError(f"mounts.{name}.rpy_rad is outside normalized range")
        if mount.get("frame") != frames["camera" if name == "realsense_d435" else name]:
            raise GeometryProfileError(f"mounts.{name}.frame does not match frames")
        edges.append((str(mount["parent"]), str(mount["frame"])))

    if not math.isclose(mounts["tof_left"]["xyz_m"][1], -mounts["tof_right"]["xyz_m"][1]):
        raise GeometryProfileError("ToF left/right Y origins must be symmetric")

    head = _mapping(profile.get("head"), "head")
    for key in ("pan", "tilt", "pi_camera", "pi_camera_optical"):
        item = _mapping(head.get(key), f"head.{key}")
        _vector(item.get("xyz_m"), f"head.{key}.xyz_m")
        _vector(item.get("rpy_rad"), f"head.{key}.rpy_rad")
        edges.append((str(item["parent"]), str(item["frame"])))
    optical = head["pi_camera_optical"]["rpy_rad"]
    expected_optical = [-1.57079632679, 0.0, -1.57079632679]
    if any(not math.isclose(float(a), b, abs_tol=1e-9) for a, b in zip(optical, expected_optical)):
        raise GeometryProfileError("Pi camera optical-frame rotation is invalid")

    children: dict[str, str] = {}
    for parent, child in edges:
        if child in children:
            raise GeometryProfileError(f"duplicate TF child authority: {child}")
        children[child] = parent
    for child in children:
        visited: set[str] = set()
        current = child
        while current in children:
            if current in visited:
                raise GeometryProfileError(f"TF cycle contains {current}")
            visited.add(current)
            current = children[current]

    navigation = _mapping(profile.get("navigation"), "navigation")
    if navigation.get("robot_base_frame") != frames["base_footprint"]:
        raise GeometryProfileError("Nav2 robot base frame must be base_footprint")
    _finite(navigation.get("footprint_padding_m"), "navigation.footprint_padding_m")


def canonical_digest(profile: dict[str, Any]) -> str:
    encoded = json.dumps(profile, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def footprint(profile: dict[str, Any]) -> list[list[float]]:
    chassis = profile["chassis"]
    half_length = float(chassis["length_m"]) / 2.0
    half_width = float(chassis["width_m"]) / 2.0
    return [
        [half_length, half_width],
        [half_length, -half_width],
        [-half_length, -half_width],
        [-half_length, half_width],
    ]
