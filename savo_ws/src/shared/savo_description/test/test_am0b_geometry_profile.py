import importlib.util
import sys
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "config/profiles/robot_savo_core_v1.yaml"
MODULE_PATH = ROOT / "scripts/geometry_profile.py"
SPEC = importlib.util.spec_from_file_location("geometry_profile", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_profile_schema_and_provisional_lock_gate():
    profile = MODULE.load_profile(PROFILE)
    MODULE.validate_profile(profile)
    assert profile["metadata"]["measurement_state"] == "provisional"
    with pytest.raises(MODULE.GeometryProfileError, match="requires locked geometry"):
        MODULE.validate_profile(profile, require_locked=True)
    MODULE.validate_profile(profile, require_locked=True, allow_provisional=True)
    assert "full_fixed_collision_envelope" in profile["calibration_remaining"]


def test_profile_has_required_tf_chain_without_duplicates_or_cycles():
    profile = MODULE.load_profile(PROFILE)
    frames = profile["frames"]
    assert profile["mounts"]["pantilt_mount"] == {
        "parent": "base_link",
        "frame": "pantilt_mount_link",
        "xyz_m": [0.115, 0.0, 0.2115],
        "rpy_rad": [0.0, 0.0, 0.0],
    }
    assert [
        frames["pantilt_mount"],
        frames["pantilt_pan"],
        frames["pantilt_tilt"],
        frames["pi_camera"],
        frames["pi_camera_optical"],
    ] == [
        "pantilt_mount_link",
        "pantilt_pan_link",
        "pantilt_tilt_link",
        "pi_camera_link",
        "pi_camera_optical_frame",
    ]


def test_nav2_footprint_is_derived_from_chassis_and_padding_is_separate():
    profile = MODULE.load_profile(PROFILE)
    generated = yaml.safe_load(
        (ROOT / "config/generated/nav2_footprint.yaml").read_text()
    )
    assert MODULE.footprint(profile) == [
        [0.1398, 0.105],
        [0.1398, -0.105],
        [-0.1398, -0.105],
        [-0.1398, 0.105],
    ]
    assert generated["metadata"]["envelope_semantics"] == (
        "measured_plate_only_not_complete_robot_collision_envelope"
    )
    assert generated["footprint_padding"] == profile["navigation"]["footprint_padding_m"]
    assert generated["metadata"]["geometry_sha256"] == MODULE.canonical_digest(profile)


def test_xacro_and_tf_authority_boundaries_are_explicit():
    robot = (ROOT / "urdf/robot_savo.urdf.xacro").read_text()
    sensors = (ROOT / "urdf/robot_savo_sensors.xacro").read_text()
    authority = (ROOT / "docs/tf_authority.md").read_text()
    assert "wheel_front_x" in robot
    assert "pantilt_mount_link" in sensors
    assert "pantilt_pan_link" not in sensors
    assert "savo_head/head_tf_node" in authority
    assert "publish_tf: false" in authority


def test_realsense_driver_does_not_duplicate_description_tf():
    profiles = (
        "realsense_nav_profile.yaml",
        "realsense_minimal.yaml",
        "realsense_d435_camera.yaml",
        "realsense_vo_profile.yaml",
        "realsense_pointcloud_camera.yaml",
    )
    config_root = ROOT.parents[1] / "edge/savo_realsense/config"
    for name in profiles:
        data = yaml.safe_load((config_root / name).read_text())
        node = next(iter(data.values()))
        assert node["ros__parameters"]["publish_tf"] is False
