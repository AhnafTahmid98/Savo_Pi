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
    # Wheel centers may legitimately sit outboard of a plate edge. Validate
    # ordering and symmetry above; collision envelopes are handled separately.

    if chassis.get("base_link_convention") != "wheel_axle_plane_from_configured_wheel_radius":
        raise GeometryProfileError("base_link must use the reviewed wheel axle-plane convention")
    if not math.isclose(
        float(chassis["base_footprint_to_base_link_z_m"]),
        float(wheels["radius_m"]),
        abs_tol=1e-12,
    ):
        raise GeometryProfileError("base_link axle-plane Z must equal configured wheel radius")
    if not math.isclose(float(wheels["z_m"]), 0.0, abs_tol=1e-12):
        raise GeometryProfileError("wheel centers must lie on the axle-plane base_link")

    plate_ground = _mapping(chassis.get("plate_ground_z_m"), "chassis.plate_ground_z_m")
    modeled_plate_ground = _mapping(
        chassis.get("modeled_plate_center_ground_z_m"),
        "chassis.modeled_plate_center_ground_z_m",
    )
    for layer in ("base", "first", "second"):
        reported_z = _finite(
            plate_ground.get(layer), f"chassis.plate_ground_z_m.{layer}"
        )
        modeled_z = _finite(
            modeled_plate_ground.get(layer),
            f"chassis.modeled_plate_center_ground_z_m.{layer}",
        )
        if abs(modeled_z - reported_z) > float(chassis["deck_thickness_m"]) / 2.0:
            raise GeometryProfileError(
                f"modeled plate center for {layer} exceeds the datum ambiguity"
            )
    ambiguity = _finite(
        chassis.get("plate_z_ambiguity_m"),
        "chassis.plate_z_ambiguity_m",
        positive=True,
    )
    if not math.isclose(ambiguity, float(chassis["deck_thickness_m"]) / 2.0):
        raise GeometryProfileError("plate Z ambiguity must equal half the plate thickness")
    if chassis.get("plate_z_datum") != "unresolved_surface_or_center_plane":
        raise GeometryProfileError("plate Z datum must remain explicitly unresolved")

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
    if not math.isclose(
        float(mounts["tof_left"]["rpy_rad"][2]), math.pi / 2.0, abs_tol=1e-10
    ):
        raise GeometryProfileError("left ToF local +X must face robot +Y")
    if not math.isclose(
        float(mounts["tof_right"]["rpy_rad"][2]), -math.pi / 2.0, abs_tol=1e-10
    ):
        raise GeometryProfileError("right ToF local +X must face robot -Y")

    head = _mapping(profile.get("head"), "head")
    for key in ("pan", "tilt", "pi_camera", "pi_camera_optical"):
        item = _mapping(head.get(key), f"head.{key}")
        _vector(item.get("xyz_m"), f"head.{key}.xyz_m")
        _vector(item.get("rpy_rad"), f"head.{key}.rpy_rad")
        edges.append((str(item["parent"]), str(item["frame"])))
    if head["pan"].get("axis") != "z" or head["tilt"].get("axis") != "y":
        raise GeometryProfileError("head pan/tilt axes must be robot Z/Y")
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
    if navigation.get("footprint_model") != "measured_plate_envelope_only":
        raise GeometryProfileError("generated footprint must be labeled as plate-only")

    realsense = _mapping(
        profile.get("realsense_internal_frames"), "realsense_internal_frames"
    )
    if realsense.get("authority") != "robot_state_publisher":
        raise GeometryProfileError("robot_state_publisher must own RealSense fixed TF")
    if realsense.get("driver_publish_tf") is not False:
        raise GeometryProfileError("RealSense driver TF must remain disabled")

    remaining = profile.get("calibration_remaining")
    if not isinstance(remaining, list):
        raise GeometryProfileError("calibration_remaining must be a list")
    if metadata["measurement_state"] == "provisional" and not remaining:
        raise GeometryProfileError("provisional profile must list calibration blockers")
    if metadata["measurement_state"] == "locked" and remaining:
        raise GeometryProfileError("locked profile cannot retain calibration blockers")


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
