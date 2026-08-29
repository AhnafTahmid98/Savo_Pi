import copy
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


def test_profile_schema_and_locked_gate():
    profile = MODULE.load_profile(PROFILE)
    MODULE.validate_profile(profile)
    MODULE.validate_profile(profile, require_locked=True)
    assert profile["metadata"]["measurement_state"] == "locked"
    assert profile["calibration_remaining"] == []
    assert profile["model_fidelity_todos"]


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


def test_lidar_scan_zero_yaw_is_calibrated_once_in_description_chain():
    profile = MODULE.load_profile(PROFILE)
    assert profile["mounts"]["lidar"] == {
        "parent": "base_link",
        "frame": "laser_frame",
        "xyz_m": [0.0, 0.0, 0.2975],
        "rpy_rad": [0.0, 0.0, -3.089891],
    }
    assert profile["mounts"]["imu"]["rpy_rad"] == [0.0, 0.0, 0.0]

    mirror = yaml.safe_load((ROOT / "config/sensor_mounts.yaml").read_text())
    assert mirror["mounts"]["lidar"]["rpy_rad"] == [0.0, 0.0, -3.089891]

    robot = (ROOT / "urdf/robot_savo.urdf.xacro").read_text()
    sensors = (ROOT / "urdf/robot_savo_sensors.xacro").read_text()
    launch = (ROOT / "launch/description.launch.py").read_text()
    assert '<xacro:arg name="lidar_rpy" default="0 0 -3.089891"/>' in robot
    assert "lidar_rpy:='0 0 -3.089891'" in sensors
    assert '"lidar_rpy": mounts["lidar"]["rpy_rad"]' in launch


def test_nav2_footprint_is_derived_from_physical_survey_and_padding_is_separate():
    profile = MODULE.load_profile(PROFILE)
    generated = yaml.safe_load(
        (ROOT / "config/generated/nav2_footprint.yaml").read_text()
    )
    assert MODULE.footprint(profile) == [
        [0.145, 0.145],
        [0.145, -0.145],
        [-0.145, -0.145],
        [-0.145, 0.145],
    ]
    assert generated["metadata"]["envelope_semantics"] == (
        "conservative_raw_production_envelope_from_complete_physical_survey"
    )
    assert (
        generated["footprint_padding"] == profile["navigation"]["footprint_padding_m"]
    )
    assert generated["metadata"]["geometry_sha256"] == MODULE.canonical_digest(profile)


def test_physical_survey_overrides_legacy_display_without_double_padding():
    profile = MODULE.load_profile(PROFILE)
    navigation = profile["navigation"]
    physical = navigation["physical_fixed_body_xy_envelope_m"]
    raw = navigation["production_raw_xy_envelope_m"]
    padded = navigation["production_padded_xy_envelope_m"]
    legacy = navigation["legacy_modeled_collision_xy_envelope_m"]

    assert physical["length"] == pytest.approx(0.280)
    assert physical["width"] == pytest.approx(0.280)
    assert raw == {
        "length": 0.290,
        "width": 0.290,
        "min_x": -0.145,
        "max_x": 0.145,
        "min_y": -0.145,
        "max_y": 0.145,
    }
    assert padded == {
        "min_x": -0.165,
        "max_x": 0.165,
        "min_y": -0.165,
        "max_y": 0.165,
    }
    assert legacy == {
        "source": "current_urdf_primitives_including_legacy_provisional_display",
        "min_x": -0.1398,
        "max_x": 0.1975,
        "min_y": -0.123,
        "max_y": 0.123,
    }
    assert raw["max_x"] == pytest.approx(0.145)
    assert raw["max_x"] + navigation["footprint_padding_m"] == pytest.approx(
        padded["max_x"]
    )
    assert legacy["max_x"] > raw["max_x"]
    assert navigation["collision_consistency"] == (
        "physical_survey_authoritative_legacy_display_primitive_is_nonproduction_model_fidelity"
    )


@pytest.mark.parametrize(
    ("section", "field", "invalid_value", "message"),
    (
        (
            "physical_fixed_body_xy_envelope_m",
            "length",
            0.281,
            "physical fixed-body envelope",
        ),
        (
            "production_raw_xy_envelope_m",
            "width",
            0.300,
            "production raw envelope",
        ),
        (
            "production_raw_xy_envelope_m",
            "max_x",
            0.165,
            "production raw extents",
        ),
        (
            "production_padded_xy_envelope_m",
            "max_y",
            0.185,
            "padding incorrectly",
        ),
    ),
)
def test_locked_validator_rejects_envelope_drift(
    section, field, invalid_value, message
):
    profile = copy.deepcopy(MODULE.load_profile(PROFILE))
    profile["navigation"][section][field] = invalid_value
    with pytest.raises(MODULE.GeometryProfileError, match=message):
        MODULE.validate_profile(profile, require_locked=True)


def test_d435_factory_extrinsics_and_urdf_body_conversion_are_validated():
    profile = MODULE.load_profile(PROFILE)
    realsense = profile["realsense_internal_frames"]
    assert realsense["device_serial"] == "801212070967"
    assert realsense["firmware_version"] == "5.16.0.1"
    assert realsense["authority"] == "robot_state_publisher"
    assert realsense["driver_publish_tf"] is False
    assert realsense["extrinsics_state"] == ("factory_calibrated_from_physical_device")
    depth_to_color = realsense["depth_to_color_optical"]
    color_to_depth = realsense["color_to_depth_optical"]
    r_dc = tuple(tuple(row) for row in depth_to_color["rotation"])
    r_cd = tuple(tuple(row) for row in color_to_depth["rotation"])
    t_dc = tuple(depth_to_color["translation_m"])
    t_cd = tuple(color_to_depth["translation_m"])
    product = MODULE._matmul(r_dc, r_cd)
    inverse_translation = tuple(
        rotated + translated
        for rotated, translated in zip(MODULE._matvec(r_dc, t_cd), t_dc)
    )
    for row in range(3):
        for column in range(3):
            assert product[row][column] == pytest.approx(
                1.0 if row == column else 0.0, abs=2e-6
            )
    assert inverse_translation == pytest.approx([0.0, 0.0, 0.0], abs=2e-8)

    reference = realsense["urdf_reference"]
    assert reference["camera_depth_frame_pose"] == "coincident_with_camera_link"
    assert reference["camera_color_frame_xyz_m"] == pytest.approx(
        [-0.000287127564661, 0.014867129735649, 0.000134048212203],
        abs=1e-15,
    )
    assert reference["camera_color_frame_rpy_rad"] == pytest.approx(
        [-0.007168423593435, 0.000448528015039, 0.010917321307075],
        abs=1e-15,
    )

    camera_macro = (ROOT / "urdf/macros/camera_macro.xacro").read_text()
    sensors = (ROOT / "urdf/robot_savo_sensors.xacro").read_text()
    launch = (ROOT / "launch/description.launch.py").read_text()
    assert '<origin xyz="${color_xyz}" rpy="${color_rpy}"/>' in camera_macro
    assert "camera_color_xyz" in sensors
    assert '"camera_color_xyz": realsense["camera_color_frame_xyz_m"]' in launch


def test_xacro_and_tf_authority_boundaries_are_explicit():
    robot = (ROOT / "urdf/robot_savo.urdf.xacro").read_text()
    sensors = (ROOT / "urdf/robot_savo_sensors.xacro").read_text()
    authority = (ROOT / "docs/tf_authority.md").read_text()
    assert "wheel_front_x" in robot
    assert "pantilt_mount_link" in sensors
    assert "pantilt_pan_link" not in sensors
    assert "savo_head/head_tf_node" in authority
    assert "publish_tf: false" in authority


def test_rsp_entry_point_requires_locked_physical_geometry_by_default():
    rsp_launch = (ROOT / "launch/rsp.launch.py").read_text()
    assert '"require_locked_geometry"' in rsp_launch
    assert 'default_value="true"' in rsp_launch
    assert '"allow_provisional_geometry"' in rsp_launch
    assert 'default_value="false"' in rsp_launch
    assert '"require_locked_geometry": LaunchConfiguration(' in rsp_launch
    assert '"allow_provisional_geometry": LaunchConfiguration(' in rsp_launch


def test_realsense_driver_does_not_duplicate_description_tf():
    profiles = (
        "realsense_nav_profile.yaml",
        "realsense_minimal.yaml",
        "realsense_d435_camera.yaml",
        "realsense_vo_driver.yaml",
        "realsense_pointcloud_camera.yaml",
    )
    config_root = ROOT.parents[1] / "edge/savo_realsense/config"
    for name in profiles:
        data = yaml.safe_load((config_root / name).read_text())
        node = next(iter(data.values()))
        assert node["ros__parameters"]["publish_tf"] is False
