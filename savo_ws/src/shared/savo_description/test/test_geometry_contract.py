"""Validate the current geometry baseline against the Xacro defaults."""

from pathlib import Path
import re

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"
URDF_DIR = PACKAGE_ROOT / "urdf"


def _load_yaml(filename: str) -> dict:
    with (CONFIG_DIR / filename).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    assert isinstance(data, dict)
    return data


def _macro_default(path: Path, name: str) -> float:
    text = path.read_text()
    match = re.search(rf"\b{re.escape(name)}:=(-?[0-9]+(?:\.[0-9]+)?)", text)
    assert match is not None, f"missing Xacro default: {name}"
    return float(match.group(1))


def test_baseline_metadata_requires_final_measurement() -> None:
    """Do not accidentally present the current modeled values as final."""
    for filename in (
        "robot_dimensions.yaml",
        "wheel_geometry.yaml",
        "sensor_mounts.yaml",
    ):
        data = _load_yaml(filename)
        assert data["metadata"]["profile"] == "robot_savo_core_v1"
        assert data["metadata"]["source"] == "current_xacro_baseline"
        assert data["metadata"]["final_physical_measurement_required"] is True


def test_robot_dimensions_match_core_xacro_defaults() -> None:
    """Keep the documented baseline synchronized with the core Xacro."""
    data = _load_yaml("robot_dimensions.yaml")
    core = URDF_DIR / "robot_savo_core.xacro"

    expected = {
        "length_m": "base_length",
        "width_m": "base_width",
        "height_m": "base_height",
        "base_link_z_m": "base_link_z",
        "mass_kg": "base_mass",
    }
    for yaml_key, xacro_key in expected.items():
        assert data["base"][yaml_key] == _macro_default(core, xacro_key)

    assert data["decks"]["thickness_m"] == _macro_default(core, "deck_thickness")
    assert data["decks"]["spacing_m"] == _macro_default(core, "deck_spacing")
    assert data["decks"]["mass_each_kg"] == _macro_default(core, "deck_mass")


def test_wheel_geometry_matches_wheel_xacro_defaults() -> None:
    """Keep the documented wheel geometry synchronized with Xacro."""
    data = _load_yaml("wheel_geometry.yaml")["wheels"]
    wheels = URDF_DIR / "robot_savo_wheels.xacro"

    profile = _load_yaml("profiles/robot_savo_core_v1.yaml")["wheels"]
    assert data["radius_m"] == profile["radius_m"] == _macro_default(wheels, "wheel_radius")
    assert data["width_m"] == profile["width_m"] == _macro_default(wheels, "wheel_width")
    assert data["x_offset_m"] == profile["front_x_m"] == _macro_default(wheels, "wheel_front_x")
    assert data["y_offset_m"] == profile["left_y_m"] == _macro_default(wheels, "wheel_left_y")
    assert data["z_offset_m"] == profile["z_m"] == _macro_default(wheels, "wheel_z")
    assert data["mass_each_kg"] == profile["mass_each_kg"] == _macro_default(wheels, "wheel_mass")


def test_lidar_mount_uses_locked_frame_and_xacro_height() -> None:
    """Lock the LiDAR frame contract to the modeled mount."""
    lidar = _load_yaml("sensor_mounts.yaml")["mounts"]["lidar"]
    sensors = URDF_DIR / "robot_savo_sensors.xacro"

    assert lidar["frame"] == "laser_frame"
    assert lidar["parent"] == "base_link"
    profile_lidar = _load_yaml("profiles/robot_savo_core_v1.yaml")["mounts"]["lidar"]
    assert lidar == profile_lidar
    assert "lidar_xyz:='0 0 0.205'" in sensors.read_text()
    assert lidar["rpy_rad"] == [0.0, 0.0, 0.0]


def test_lidar_frame_configs_are_consistent() -> None:
    """Keep description, mount, and costmap frame names identical."""
    frames = _load_yaml("frame_names.yaml")
    mounts = _load_yaml("sensor_mounts.yaml")
    costmap = _load_yaml("costmap_frames.yaml")

    locked = "laser_frame"
    assert frames["frames"]["sensors"]["lidar"] == locked
    assert mounts["mounts"]["lidar"]["frame"] == locked
    assert costmap["lidar"]["frame"] == locked
