"""Validate measured Robot Savo geometry mirrors and Xacro defaults."""

from pathlib import Path
import re

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"
URDF_DIR = PACKAGE_ROOT / "urdf"


def _load_yaml(relative_path: str) -> dict:
    with (CONFIG_DIR / relative_path).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    assert isinstance(data, dict)
    return data


def _macro_default(path: Path, name: str) -> float:
    text = path.read_text(encoding="utf-8")
    match = re.search(rf"\b{re.escape(name)}:=(-?[0-9]+(?:\.[0-9]+)?)", text)
    assert match is not None, f"missing Xacro default: {name}"
    return float(match.group(1))


def test_measurement_metadata_stays_provisional() -> None:
    profile = _load_yaml("profiles/robot_savo_core_v1.yaml")
    assert profile["metadata"]["geometry_revision"] == 3
    assert profile["metadata"]["measurement_state"] == "provisional"
    assert profile["chassis"]["plate_z_datum"] == (
        "unresolved_surface_or_center_plane"
    )
    assert profile["chassis"]["plate_z_ambiguity_m"] == pytest.approx(0.002)


def test_plate_dimensions_and_axle_frame_match_core_xacro() -> None:
    profile = _load_yaml("profiles/robot_savo_core_v1.yaml")
    mirror = _load_yaml("robot_dimensions.yaml")
    core = URDF_DIR / "robot_savo_core.xacro"
    chassis = profile["chassis"]

    assert chassis["length_m"] == mirror["base"]["length_m"] == pytest.approx(0.2796)
    assert chassis["width_m"] == mirror["base"]["width_m"] == pytest.approx(0.2100)
    assert chassis["height_m"] == mirror["base"]["height_m"] == pytest.approx(0.0040)
    assert chassis["plate_count"] == mirror["plates"]["count"] == 4
    assert chassis["third_plate_height_m"] == mirror["plates"][
        "third_thickness_m"
    ] == pytest.approx(0.0040)
    assert chassis["third_plate_length_m"] == mirror["plates"][
        "third_length_m"
    ] == pytest.approx(0.1420)
    assert chassis["third_plate_width_m"] == mirror["plates"][
        "third_width_m"
    ] == pytest.approx(0.1440)
    assert chassis["base_mesh_envelope_height_m"] == mirror["plates"][
        "base_mesh_envelope_height_m"
    ] == pytest.approx(0.0280)
    assert chassis["base_mesh_includes_motor_mounts"] is True
    assert mirror["plates"]["base_mesh_includes_motor_mounts"] is True
    assert chassis["layer_contents"] == mirror["layer_contents"]
    assert chassis["base_footprint_to_base_link_z_m"] == pytest.approx(0.0325)
    assert _macro_default(core, "base_length") == pytest.approx(0.2796)
    assert _macro_default(core, "base_width") == pytest.approx(0.2100)
    assert _macro_default(core, "base_link_z") == pytest.approx(0.0325)
    assert _macro_default(core, "third_plate_length") == pytest.approx(0.1420)
    assert _macro_default(core, "third_plate_width") == pytest.approx(0.1440)
    assert _macro_default(core, "third_plate_height") == pytest.approx(0.0040)
    assert _macro_default(core, "base_plate_z") == pytest.approx(-0.0175)
    assert _macro_default(core, "first_plate_z") == pytest.approx(0.0475)
    assert _macro_default(core, "second_plate_z") == pytest.approx(0.1635)
    assert _macro_default(core, "third_plate_z") == pytest.approx(0.2475)


def test_all_authoritative_plate_ground_heights_are_modeled() -> None:
    profile = _load_yaml("profiles/robot_savo_core_v1.yaml")["chassis"]
    mirror = _load_yaml("robot_dimensions.yaml")["plates"]

    expected_ground_z = {
        "base": 0.015,
        "first": 0.080,
        "second": 0.196,
        "third": 0.280,
    }
    expected_spacing = {
        "base_to_first": 0.065,
        "first_to_second": 0.116,
        "second_to_third": 0.084,
    }
    assert profile["plate_ground_z_m"] == pytest.approx(expected_ground_z)
    assert profile["modeled_plate_center_ground_z_m"] == pytest.approx(
        expected_ground_z
    )
    assert mirror["reported_ground_z_m"] == pytest.approx(expected_ground_z)
    assert mirror["modeled_center_ground_z_m"] == pytest.approx(expected_ground_z)
    assert profile["derived_plate_center_spacing_m"] == pytest.approx(
        expected_spacing
    )
    assert mirror["derived_center_spacing_m"] == pytest.approx(expected_spacing)
    assert profile["unmeasured_plate_ground_layers"] == []
    assert mirror["unmeasured_ground_z_layers"] == []


def test_third_lidar_plate_is_wired_through_main_xacro() -> None:
    core = (URDF_DIR / "robot_savo_core.xacro").read_text(encoding="utf-8")
    robot = (URDF_DIR / "robot_savo.urdf.xacro").read_text(encoding="utf-8")

    assert 'name="lidar_deck_link"' in core
    assert 'xyz="0 0 ${third_plate_z}"' in core
    assert (
        'size_xyz="${third_plate_length} ${third_plate_width} '
        '${third_plate_height}"' in core
    )
    for argument in (
        "third_plate_length",
        "third_plate_width",
        "third_plate_height",
        "third_plate_z",
    ):
        assert f'{argument}="$(arg {argument})"' in robot


def test_wheel_centers_match_profile_mirror_and_xacro() -> None:
    profile = _load_yaml("profiles/robot_savo_core_v1.yaml")["wheels"]
    mirror = _load_yaml("wheel_geometry.yaml")["wheels"]
    wheels = URDF_DIR / "robot_savo_wheels.xacro"

    expected = {
        "front_x_m": 0.080,
        "rear_x_m": -0.080,
        "left_y_m": 0.108,
        "right_y_m": -0.108,
    }
    for key, value in expected.items():
        assert profile[key] == mirror[key] == pytest.approx(value)
    assert _macro_default(wheels, "wheel_front_x") == pytest.approx(0.080)
    assert _macro_default(wheels, "wheel_rear_x") == pytest.approx(-0.080)
    assert _macro_default(wheels, "wheel_left_y") == pytest.approx(0.108)
    assert _macro_default(wheels, "wheel_right_y") == pytest.approx(-0.108)
    assert mirror["wheelbase_m"] == pytest.approx(0.160)
    assert mirror["track_m"] == pytest.approx(0.216)
    assert mirror["kinematic_k_m"] == pytest.approx(0.188)


def test_sensor_mount_mirror_matches_profile_values() -> None:
    profile = _load_yaml("profiles/robot_savo_core_v1.yaml")["mounts"]
    mirror = _load_yaml("sensor_mounts.yaml")["mounts"]
    names = {
        "lidar": "lidar",
        "imu": "imu",
        "realsense_d435": "camera",
        "tof_left": "tof_left",
        "tof_right": "tof_right",
        "ultrasonic_front": "ultrasonic_front",
        "pantilt_mount": "pantilt_mount",
    }
    for profile_name, mirror_name in names.items():
        assert mirror[mirror_name]["parent"] == profile[profile_name]["parent"]
        assert mirror[mirror_name]["frame"] == profile[profile_name]["frame"]
        assert mirror[mirror_name]["xyz_m"] == pytest.approx(
            profile[profile_name]["xyz_m"]
        )
        assert mirror[mirror_name]["rpy_rad"] == pytest.approx(
            profile[profile_name]["rpy_rad"]
        )


def test_lidar_frame_configs_are_consistent() -> None:
    frames = _load_yaml("frame_names.yaml")
    mounts = _load_yaml("sensor_mounts.yaml")
    costmap = _load_yaml("costmap_frames.yaml")
    assert frames["frames"]["sensors"]["lidar"] == "laser_frame"
    assert mounts["mounts"]["lidar"]["frame"] == "laser_frame"
    assert costmap["lidar"]["frame"] == "laser_frame"
