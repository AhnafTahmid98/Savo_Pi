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


def _matrix3(value: Any, field: str) -> tuple[tuple[float, float, float], ...]:
    if not isinstance(value, list) or len(value) != 3:
        raise GeometryProfileError(f"{field} must contain exactly three rows")
    return tuple(_vector(row, f"{field}[{index}]") for index, row in enumerate(value))


def _transpose(matrix: tuple[tuple[float, float, float], ...]):
    return tuple(tuple(matrix[row][column] for row in range(3)) for column in range(3))


def _matmul(left, right):
    return tuple(
        tuple(
            sum(left[row][item] * right[item][column] for item in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def _matvec(matrix, vector):
    return tuple(
        sum(matrix[row][item] * vector[item] for item in range(3)) for row in range(3)
    )


def _rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


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
    if (
        require_locked
        and metadata["measurement_state"] != "locked"
        and not allow_provisional
    ):
        raise GeometryProfileError(
            "production requires locked geometry; use the explicit controlled-test override"
        )
    if metadata["measurement_state"] == "locked":
        if (
            not str(metadata["measured_by"]).strip()
            or not str(metadata["measurement_date"]).strip()
        ):
            raise GeometryProfileError(
                "locked geometry requires measured_by and measurement_date"
            )

    convention = _mapping(profile.get("coordinate_convention"), "coordinate_convention")
    if convention != {"x": "forward", "y": "left", "z": "up"}:
        raise GeometryProfileError(
            "coordinate convention must be +X forward, +Y left, +Z up"
        )

    chassis = _mapping(profile.get("chassis"), "chassis")
    for key in (
        "length_m",
        "width_m",
        "height_m",
        "third_plate_length_m",
        "third_plate_width_m",
        "third_plate_height_m",
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

    if (
        chassis.get("base_link_convention")
        != "wheel_axle_plane_from_configured_wheel_radius"
    ):
        raise GeometryProfileError(
            "base_link must use the reviewed wheel axle-plane convention"
        )
    if not math.isclose(
        float(chassis["base_footprint_to_base_link_z_m"]),
        float(wheels["radius_m"]),
        abs_tol=1e-12,
    ):
        raise GeometryProfileError(
            "base_link axle-plane Z must equal configured wheel radius"
        )
    if not math.isclose(float(wheels["z_m"]), 0.0, abs_tol=1e-12):
        raise GeometryProfileError("wheel centers must lie on the axle-plane base_link")

    plate_bottom = _mapping(
        chassis.get("plate_bottom_surface_ground_z_m"),
        "chassis.plate_bottom_surface_ground_z_m",
    )
    plate_center = _mapping(
        chassis.get("plate_center_ground_z_m"),
        "chassis.plate_center_ground_z_m",
    )
    half_thickness = float(chassis["deck_thickness_m"]) / 2.0
    for layer in ("base", "first", "second", "third"):
        bottom_z = _finite(
            plate_bottom.get(layer),
            f"chassis.plate_bottom_surface_ground_z_m.{layer}",
        )
        center_z = _finite(
            plate_center.get(layer), f"chassis.plate_center_ground_z_m.{layer}"
        )
        if not math.isclose(center_z, bottom_z + half_thickness, abs_tol=1e-12):
            raise GeometryProfileError(
                f"plate center for {layer} must be bottom surface plus half thickness"
            )
    if chassis.get("plate_z_datum") != "measured_bottom_surface":
        raise GeometryProfileError("plate Z datum must be the measured bottom surface")
    spacing = _mapping(
        chassis.get("derived_plate_center_spacing_m"),
        "chassis.derived_plate_center_spacing_m",
    )
    for name, lower, upper in (
        ("base_to_first", "base", "first"),
        ("first_to_second", "first", "second"),
        ("second_to_third", "second", "third"),
    ):
        if not math.isclose(
            _finite(
                spacing.get(name), f"chassis.derived_plate_center_spacing_m.{name}"
            ),
            float(plate_center[upper]) - float(plate_center[lower]),
            abs_tol=1e-12,
        ):
            raise GeometryProfileError(f"plate center spacing {name} is inconsistent")

    frames = _mapping(profile.get("frames"), "frames")
    missing_frames = [
        key for key in REQUIRED_FRAMES if not str(frames.get(key, "")).strip()
    ]
    if missing_frames:
        raise GeometryProfileError(
            f"required frames missing: {', '.join(missing_frames)}"
        )
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
            raise GeometryProfileError(
                f"mounts.{name}.rpy_rad is outside normalized range"
            )
        if mount.get("frame") != frames["camera" if name == "realsense_d435" else name]:
            raise GeometryProfileError(f"mounts.{name}.frame does not match frames")
        edges.append((str(mount["parent"]), str(mount["frame"])))

    if not math.isclose(
        mounts["tof_left"]["xyz_m"][1], -mounts["tof_right"]["xyz_m"][1]
    ):
        raise GeometryProfileError("ToF left/right Y origins must be symmetric")
    if not math.isclose(
        float(mounts["tof_left"]["rpy_rad"][2]), math.pi / 2.0, abs_tol=1e-10
    ):
        raise GeometryProfileError("left ToF local +X must face robot +Y")
    if not math.isclose(
        float(mounts["tof_right"]["rpy_rad"][2]), -math.pi / 2.0, abs_tol=1e-10
    ):
        raise GeometryProfileError("right ToF local +X must face robot -Y")
    if mounts["lidar"]["rpy_rad"] != [0.0, 0.0, -3.089891]:
        raise GeometryProfileError("LiDAR scan-zero yaw must remain hardware validated")
    if mounts["imu"]["rpy_rad"] != [0.0, 0.0, 0.0]:
        raise GeometryProfileError("IMU orientation must remain hardware validated")

    head = _mapping(profile.get("head"), "head")
    for key in ("pan", "tilt", "pi_camera", "pi_camera_optical"):
        item = _mapping(head.get(key), f"head.{key}")
        _vector(item.get("xyz_m"), f"head.{key}.xyz_m")
        _vector(item.get("rpy_rad"), f"head.{key}.rpy_rad")
        edges.append((str(item["parent"]), str(item["frame"])))
    if head["pan"].get("axis") != "z" or head["tilt"].get("axis") != "y":
        raise GeometryProfileError("head pan/tilt axes must be robot Z/Y")
    if head.get("dynamic_authority") != "savo_head/head_tf_node":
        raise GeometryProfileError("validated head TF authority changed")
    optical = head["pi_camera_optical"]["rpy_rad"]
    expected_optical = [-1.57079632679, 0.0, -1.57079632679]
    if any(
        not math.isclose(float(a), b, abs_tol=1e-9)
        for a, b in zip(optical, expected_optical)
    ):
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
    if navigation.get("footprint_model") != (
        "complete_physical_survey_with_conservative_raw_margin"
    ):
        raise GeometryProfileError("production footprint must use the physical survey")
    padding = _finite(
        navigation.get("footprint_padding_m"),
        "navigation.footprint_padding_m",
    )
    if not math.isclose(padding, 0.020, abs_tol=1e-12):
        raise GeometryProfileError("Nav2 footprint padding must remain 0.020 m")

    physical_envelope = _mapping(
        navigation.get("physical_fixed_body_xy_envelope_m"),
        "navigation.physical_fixed_body_xy_envelope_m",
    )
    if physical_envelope.get("survey_scope") != (
        "fully_assembled_including_wheels_and_permanently_mounted_fixed_hardware"
    ):
        raise GeometryProfileError("physical envelope survey scope is incomplete")
    for field in ("length", "width"):
        value = _finite(physical_envelope.get(field), f"physical_envelope.{field}")
        if not math.isclose(value, 0.280, abs_tol=1e-12):
            raise GeometryProfileError(
                "physical fixed-body envelope must be 0.280 m square"
            )

    raw_envelope = _mapping(
        navigation.get("production_raw_xy_envelope_m"),
        "navigation.production_raw_xy_envelope_m",
    )
    for field in ("length", "width"):
        value = _finite(raw_envelope.get(field), f"production_raw.{field}")
        if not math.isclose(value, 0.290, abs_tol=1e-12):
            raise GeometryProfileError("production raw envelope must be 0.290 m square")
    expected_raw = {
        "min_x": -0.145,
        "max_x": 0.145,
        "min_y": -0.145,
        "max_y": 0.145,
    }
    for field, expected in expected_raw.items():
        value = _finite(raw_envelope.get(field), f"production_raw.{field}")
        if not math.isclose(value, expected, abs_tol=1e-12):
            raise GeometryProfileError("production raw extents must be +/-0.145 m")

    padded_envelope = _mapping(
        navigation.get("production_padded_xy_envelope_m"),
        "navigation.production_padded_xy_envelope_m",
    )
    expected_padded = {
        "min_x": expected_raw["min_x"] - padding,
        "max_x": expected_raw["max_x"] + padding,
        "min_y": expected_raw["min_y"] - padding,
        "max_y": expected_raw["max_y"] + padding,
    }
    for field, expected in expected_padded.items():
        value = _finite(padded_envelope.get(field), f"production_padded.{field}")
        if not math.isclose(value, expected, abs_tol=1e-12):
            raise GeometryProfileError("production padded extents apply padding incorrectly")

    legacy_envelope = _mapping(
        navigation.get("legacy_modeled_collision_xy_envelope_m"),
        "navigation.legacy_modeled_collision_xy_envelope_m",
    )
    legacy_max_x = _finite(
        legacy_envelope.get("max_x"), "legacy_modeled_collision.max_x"
    )
    if not math.isclose(legacy_max_x, 0.1975, abs_tol=1e-12):
        raise GeometryProfileError("legacy display discrepancy must remain documented")
    if navigation.get("collision_consistency") != (
        "physical_survey_authoritative_legacy_display_primitive_is_nonproduction_model_fidelity"
    ):
        raise GeometryProfileError("physical survey must override legacy model fidelity")

    realsense = _mapping(
        profile.get("realsense_internal_frames"), "realsense_internal_frames"
    )
    if realsense.get("authority") != "robot_state_publisher":
        raise GeometryProfileError("robot_state_publisher must own RealSense fixed TF")
    if realsense.get("driver_publish_tf") is not False:
        raise GeometryProfileError("RealSense driver TF must remain disabled")
    if realsense.get("extrinsics_state") != "factory_calibrated_from_physical_device":
        raise GeometryProfileError(
            "RealSense internal extrinsics must be factory calibrated"
        )
    if str(realsense.get("device_serial")) != "801212070967":
        raise GeometryProfileError(
            "RealSense calibration serial is not the validated D435"
        )
    if str(realsense.get("firmware_version")) != "5.16.0.1":
        raise GeometryProfileError(
            "RealSense calibration firmware provenance is invalid"
        )

    depth_to_color = _mapping(
        realsense.get("depth_to_color_optical"),
        "realsense_internal_frames.depth_to_color_optical",
    )
    color_to_depth = _mapping(
        realsense.get("color_to_depth_optical"),
        "realsense_internal_frames.color_to_depth_optical",
    )
    r_dc = _matrix3(depth_to_color.get("rotation"), "depth_to_color_optical.rotation")
    t_dc = _vector(
        depth_to_color.get("translation_m"),
        "depth_to_color_optical.translation_m",
    )
    r_cd = _matrix3(color_to_depth.get("rotation"), "color_to_depth_optical.rotation")
    t_cd = _vector(
        color_to_depth.get("translation_m"),
        "color_to_depth_optical.translation_m",
    )
    identity = _matmul(r_dc, r_cd)
    inverse_translation = tuple(a + b for a, b in zip(_matvec(r_dc, t_cd), t_dc))
    for row in range(3):
        for column in range(3):
            expected = 1.0 if row == column else 0.0
            if not math.isclose(identity[row][column], expected, abs_tol=2e-6):
                raise GeometryProfileError(
                    "RealSense optical rotations are not inverses"
                )
    if any(abs(value) > 2e-8 for value in inverse_translation):
        raise GeometryProfileError("RealSense optical translations are not inverses")

    reference = _mapping(
        realsense.get("urdf_reference"),
        "realsense_internal_frames.urdf_reference",
    )
    color_xyz = _vector(
        reference.get("camera_color_frame_xyz_m"),
        "urdf_reference.camera_color_frame_xyz_m",
    )
    color_rpy = _vector(
        reference.get("camera_color_frame_rpy_rad"),
        "urdf_reference.camera_color_frame_rpy_rad",
    )
    optical_rpy = _vector(
        realsense.get("body_to_optical_rpy_rad"),
        "realsense_internal_frames.body_to_optical_rpy_rad",
    )
    body_from_optical = _rpy_matrix(optical_rpy)
    expected_body_rotation = _matmul(
        _matmul(body_from_optical, r_cd), _transpose(body_from_optical)
    )
    expected_body_translation = _matvec(body_from_optical, t_cd)
    actual_body_rotation = _rpy_matrix(color_rpy)
    if any(
        abs(actual - expected) > 1e-6
        for actual_row, expected_row in zip(
            actual_body_rotation, expected_body_rotation
        )
        for actual, expected in zip(actual_row, expected_row)
    ):
        raise GeometryProfileError(
            "RealSense URDF body rotation does not match optical calibration"
        )
    if any(abs(a - b) > 1e-12 for a, b in zip(color_xyz, expected_body_translation)):
        raise GeometryProfileError(
            "RealSense URDF body translation does not match optical calibration"
        )

    remaining = profile.get("calibration_remaining")
    if not isinstance(remaining, list):
        raise GeometryProfileError("calibration_remaining must be a list")
    if metadata["measurement_state"] == "provisional" and not remaining:
        raise GeometryProfileError("provisional profile must list calibration blockers")
    if metadata["measurement_state"] == "locked" and remaining:
        raise GeometryProfileError("locked profile cannot retain calibration blockers")
    if metadata["measurement_state"] == "provisional" and remaining != [
        "physically_measured_complete_fixed_body_xy_envelope"
    ]:
        raise GeometryProfileError(
            "only the complete fixed-body XY envelope may block lock"
        )

    fidelity = profile.get("model_fidelity_todos")
    if not isinstance(fidelity, list) or not fidelity:
        raise GeometryProfileError(
            "model_fidelity_todos must list non-blocking estimates"
        )
    if set(fidelity).intersection(remaining):
        raise GeometryProfileError("model-fidelity TODOs cannot block geometry lock")

    closed = _mapping(profile.get("closed_calibrations"), "closed_calibrations")
    expected_closed = {
        "lidar_scan_zero_yaw": "hardware_validated",
        "imu_orientation": "hardware_validated",
        "head_signs_and_runtime_tf": "hardware_validated",
        "plate_z_datum": "owner_resolved_bottom_surface",
        "d435_internal_extrinsics": "factory_calibrated_serial_801212070967",
        "fixed_body_xy_envelope": "complete_assembled_robot_survey",
    }
    if closed != expected_closed:
        raise GeometryProfileError("closed hardware calibrations changed")


def canonical_digest(profile: dict[str, Any]) -> str:
    encoded = json.dumps(profile, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def footprint(profile: dict[str, Any]) -> list[list[float]]:
    envelope = profile["navigation"]["production_raw_xy_envelope_m"]
    half_length = float(envelope["length"]) / 2.0
    half_width = float(envelope["width"]) / 2.0
    return [
        [half_length, half_width],
        [half_length, -half_width],
        [-half_length, -half_width],
        [-half_length, half_width],
    ]
