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
    assert profile["metadata"]["geometry_revision"] == 2
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
    assert chassis["base_footprint_to_base_link_z_m"] == pytest.approx(0.0325)
    assert _macro_default(core, "base_length") == pytest.approx(0.2796)
    assert _macro_default(core, "base_width") == pytest.approx(0.2100)
    assert _macro_default(core, "base_link_z") == pytest.approx(0.0325)
    assert _macro_default(core, "base_plate_z") == pytest.approx(-0.0185)
    assert _macro_default(core, "first_plate_z") == pytest.approx(0.0475)
    assert _macro_default(core, "second_plate_z") == pytest.approx(0.1675)


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
