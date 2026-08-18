"""Cross-package regression contract for measured Robot Savo geometry."""

from pathlib import Path

import pytest
import yaml


DESCRIPTION_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = DESCRIPTION_ROOT.parents[1]
PROFILE_PATH = DESCRIPTION_ROOT / "config/profiles/robot_savo_core_v1.yaml"


def _yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert isinstance(data, dict), path
    return data


def _params(path: Path, node: str) -> dict:
    return _yaml(path)[node]["ros__parameters"]


def test_measured_wheel_geometry_and_derived_kinematics() -> None:
    profile = _yaml(PROFILE_PATH)
    wheels = profile["wheels"]
    assert [wheels["front_x_m"], wheels["left_y_m"]] == pytest.approx([0.080, 0.108])
    assert [wheels["front_x_m"], wheels["right_y_m"]] == pytest.approx([0.080, -0.108])
    assert [wheels["rear_x_m"], wheels["left_y_m"]] == pytest.approx([-0.080, 0.108])
    assert [wheels["rear_x_m"], wheels["right_y_m"]] == pytest.approx([-0.080, -0.108])
    wheelbase = wheels["front_x_m"] - wheels["rear_x_m"]
    track = wheels["left_y_m"] - wheels["right_y_m"]
    assert wheelbase == pytest.approx(0.160)
    assert track == pytest.approx(0.216)
    assert (wheelbase + track) / 2.0 == pytest.approx(0.188)
    assert wheels["radius_m"] == pytest.approx(0.0325)


def test_base_link_conversion_reconstructs_ground_measurements() -> None:
    profile = _yaml(PROFILE_PATH)
    base_z = profile["chassis"]["base_footprint_to_base_link_z_m"]
    expected_ground = {
        "imu": [0.0, -0.0465, 0.0150],
        "lidar": [0.0, 0.0, 0.3300],
        "realsense_d435": [0.130, 0.0, 0.2250],
        "tof_left": [0.0, 0.106, 0.0250],
        "tof_right": [0.0, -0.106, 0.0250],
        "ultrasonic_front": [0.137, 0.0, 0.0560],
        "pantilt_mount": [0.115, 0.0, 0.2440],
    }
    for name, ground_xyz in expected_ground.items():
        base_xyz = profile["mounts"][name]["xyz_m"]
        reconstructed = [base_xyz[0], base_xyz[1], base_xyz[2] + base_z]
        assert reconstructed == pytest.approx(ground_xyz, abs=1e-12)


def test_base_localization_and_description_kinematics_agree() -> None:
    base = _params(
        SRC_ROOT / "core/savo_base/config/mecanum_kinematics.yaml",
        "/base_driver_node",
    )
    localization_paths = [
        SRC_ROOT / "core/savo_localization/config/encoders.yaml",
        SRC_ROOT / "core/savo_localization/config/wheel_odom.yaml",
        SRC_ROOT / "core/savo_localization/config/profiles/bench_encoders_4wheel.yaml",
        SRC_ROOT / "core/savo_localization/config/profiles/wheel_odom_4enc.yaml",
    ]
    assert base["wheelbase_m"] == pytest.approx(0.160)
    assert base["track_width_m"] == pytest.approx(0.216)
    assert base["kinematic_k_m"] == pytest.approx(0.188)
    for path in localization_paths:
        params = _params(path, "wheel_odom_node")
        assert params["wheel_diameter_m"] == pytest.approx(0.065)
        assert params["wheelbase_m"] == pytest.approx(0.160)
        assert params["track_m"] == pytest.approx(0.216)

    constants = (
        SRC_ROOT / "core/savo_localization/savo_localization/constants.py"
    ).read_text(encoding="utf-8")
    cpp_geometry = (
        SRC_ROOT / "core/savo_localization/include/savo_localization/mecanum_odom.hpp"
    ).read_text(encoding="utf-8")
    cpp_node = (
        SRC_ROOT
        / "core/savo_localization/include/savo_localization/wheel_odom_node.hpp"
    ).read_text(encoding="utf-8")
    assert "DEFAULT_WHEELBASE_M = 0.160" in constants
    assert "DEFAULT_TRACK_M = 0.216" in constants
    assert "double wheelbase_m{0.160};" in cpp_geometry
    assert "double track_m{0.216};" in cpp_geometry
    assert "double wheelbase_m_{0.160};" in cpp_node
    assert "double track_m_{0.216};" in cpp_node

    diagnostic = (SRC_ROOT.parents[1] / "tools/diag/motion/encoders_test.py").read_text(
        encoding="utf-8"
    )
    assert 'default=0.160, help="Front-rear wheel spacing' in diagnostic
    assert 'default=0.216, help="Left-right wheel spacing' in diagnostic
    assert "(wheelbase_m + track_m) / 2.0" in diagnostic


def test_side_tof_axes_and_front_ultrasonic_axis() -> None:
    mounts = _yaml(PROFILE_PATH)["mounts"]
    assert mounts["tof_left"]["rpy_rad"][2] == pytest.approx(1.57079632679)
    assert mounts["tof_right"]["rpy_rad"][2] == pytest.approx(-1.57079632679)
    assert mounts["ultrasonic_front"]["rpy_rad"] == pytest.approx([0.0, 0.0, 0.0])

    safety = _params(
        SRC_ROOT / "shared/savo_perception/config/core/range_safety.yaml",
        "safety_stop_node",
    )
    assert safety["tof_left_topic"].endswith("/left_m")
    assert safety["tof_right_topic"].endswith("/right_m")
    assert safety["ultrasonic_front_topic"].endswith("/front_ultrasonic_m")

    snapshot = (
        SRC_ROOT
        / "shared/savo_perception/include/savo_perception/range_sample.hpp"
    ).read_text(encoding="utf-8")
    front_section = snapshot.split("front_candidates(", 1)[1].split(
        "side_candidates(", 1
    )[0]
    side_section = snapshot.split("side_candidates(", 1)[1].split(
        "min_front_m(", 1
    )[0]
    assert "ultrasonic_front" in front_section
    assert "tof_left" not in front_section
    assert "tof_right" not in front_section
    assert "tof_left" in side_section
    assert "tof_right" in side_section


def test_head_profile_and_runtime_yaml_agree_and_remain_fail_closed() -> None:
    profile = _yaml(PROFILE_PATH)
    params = _params(SRC_ROOT / "core/savo_head/config/head_frames.yaml", "head_tf_node")
    head = profile["head"]
    assert params["base_frame"] == head["pan"]["parent"]
    assert params["pan_frame"] == head["pan"]["frame"]
    assert params["tilt_frame"] == head["tilt"]["frame"]
    assert params["camera_frame"] == head["pi_camera"]["frame"]
    assert params["camera_optical_frame"] == head["pi_camera_optical"]["frame"]
    assert params["base_to_pan_xyz_m"] == pytest.approx(head["pan"]["xyz_m"])
    assert params["pan_to_tilt_xyz_m"] == pytest.approx(head["tilt"]["xyz_m"])
    assert params["tilt_to_camera_xyz_m"] == pytest.approx(head["pi_camera"]["xyz_m"])
    assert params["camera_to_optical_xyz_m"] == pytest.approx(
        head["pi_camera_optical"]["xyz_m"]
    )
    assert params["camera_to_optical_rpy_rad"] == pytest.approx(
        head["pi_camera_optical"]["rpy_rad"]
    )
    assert params["pan_axis"] == head["pan"]["axis"] == "z"
    assert params["tilt_axis"] == head["tilt"]["axis"] == "y"
    assert params["publish_tf"] is False
    assert params["transforms_calibrated"] is False


def test_neutral_head_chain_reconstructs_measured_ground_positions() -> None:
    profile = _yaml(PROFILE_PATH)
    base_z = profile["chassis"]["base_footprint_to_base_link_z_m"]
    mount = profile["mounts"]["pantilt_mount"]["xyz_m"]
    tilt = profile["head"]["tilt"]["xyz_m"]
    camera = profile["head"]["pi_camera"]["xyz_m"]
    pan_ground = [mount[0], mount[1], mount[2] + base_z]
    tilt_ground = [pan_ground[i] + tilt[i] for i in range(3)]
    camera_ground = [tilt_ground[i] + camera[i] for i in range(3)]
    assert pan_ground == pytest.approx([0.115, 0.0, 0.244])
    assert tilt_ground == pytest.approx([0.115, 0.0, 0.290])
    assert camera_ground == pytest.approx([0.140, 0.0, 0.280])


def test_realsense_has_one_tf_authority_and_provisional_internal_extrinsics() -> None:
    config_root = SRC_ROOT / "edge/savo_realsense/config"
    for path in config_root.glob("realsense*.yaml"):
        data = _yaml(path)
        for node in data.values():
            params = node.get("ros__parameters", {})
            if "publish_tf" in params:
                assert params["publish_tf"] is False, path
    frames = _yaml(config_root / "camera_frames.yaml")
    assert frames["tf"]["publish_driver_tf"] is False
    assert frames["tf"]["internal_extrinsics_state"] == (
        "provisional_zero_translation_not_device_calibrated"
    )


def test_frame_consumers_match_description_contract() -> None:
    frames = _yaml(PROFILE_PATH)["frames"]
    localization = _yaml(SRC_ROOT / "core/savo_localization/config/frames.yaml")
    assert localization["imu_node"]["ros__parameters"]["frame_id"] == frames["imu"]
    wheel_odom = localization["wheel_odom_node"]["ros__parameters"]
    assert wheel_odom["odom_frame_id"] == frames["odom"]
    assert wheel_odom["base_frame_id"] == frames["base_footprint"]

    mapping = _yaml(SRC_ROOT / "core/savo_mapping/config/frames.yaml")["/**"][
        "ros__parameters"
    ]["frames"]
    assert mapping["map"] == frames["map"]
    assert mapping["odom"] == frames["odom"]
    assert mapping["base"] == frames["base_link"]
    assert mapping["base_footprint"] == frames["base_footprint"]
    assert mapping["lidar"] == frames["lidar"]
    assert mapping["optional"]["realsense_link"] == frames["camera"]
    assert mapping["optional"]["head_camera"] == frames["pi_camera"]

    lidar_constants = (
        SRC_ROOT / "core/savo_lidar/savo_lidar/constants.py"
    ).read_text(encoding="utf-8")
    assert f'DEFAULT_FRAME_ID: Final[str] = "{frames["lidar"]}"' in lidar_constants

    vo = _yaml(SRC_ROOT / "edge/savo_vo/config/frames.yaml")["frames"]
    assert vo["odom_frame"] == frames["odom"]
    assert vo["base_frame"] == frames["base_footprint"]
    assert vo["camera_link_frame"] == frames["camera"]
    assert vo["camera_color_frame"] == frames["camera_color_optical"]
    assert vo["camera_depth_frame"] == frames["camera_depth_optical"]


def test_production_geometry_gates_remain_fail_closed() -> None:
    mapping = _params(
        SRC_ROOT / "core/savo_mapping/config/autonomous_mapping_orchestrator.yaml",
        "autonomous_mapping_orchestrator_node",
    )["release"]
    assert mapping["require_locked_geometry"] is True
    assert mapping["allow_provisional_geometry"] is False

    for profile_name in ("production", "lidar_only", "lidar_d435_voxel"):
        profile = _yaml(
            SRC_ROOT
            / f"shared/savo_bringup/config/profiles/{profile_name}.yaml"
        )["robot_savo_bringup_profile"]
        assert profile["require_locked_geometry"] is True
        assert profile["allow_provisional_geometry"] is False

    nav_launch = (
        SRC_ROOT / "core/savo_nav/launch/production_navigation.launch.py"
    ).read_text(encoding="utf-8")
    assert "geometry_profile_not_locked" in nav_launch
    assert "release_geometry_profile_digest_mismatch" in nav_launch


def test_navigation_keeps_conservative_production_envelope() -> None:
    measured_half_width = 0.2100 / 2.0
    provisional_wheel_envelope = 0.108 + 0.030 / 2.0
    assert provisional_wheel_envelope > measured_half_width
    expected = (
        "[[0.165, 0.120], [0.165, -0.120], "
        "[-0.165, -0.120], [-0.165, 0.120]]"
    )
    for name in (
        "nav2_live_mapping.yaml",
        "nav2_saved_map.yaml",
        "nav2_saved_map_voxel.yaml",
    ):
        config = _yaml(SRC_ROOT / f"core/savo_nav/config/{name}")
        for costmap_name in ("global_costmap", "local_costmap"):
            params = config[costmap_name][costmap_name]["ros__parameters"]
            assert params["footprint"] == expected
            assert params["footprint_padding"] == pytest.approx(0.02)
