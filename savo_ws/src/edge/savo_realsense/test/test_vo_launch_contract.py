# Copyright 2026 Ahnaf Tahmid

"""Contracts for the RealSense VO driver and monitor configuration split."""

import re
from pathlib import Path

import yaml

from savo_realsense.constants import DEFAULT_POINTCLOUD_TOPIC


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read_file(path: str) -> str:
    return (PACKAGE_ROOT / path).read_text(encoding="utf-8")


def load_yaml(path: str) -> dict:
    return yaml.safe_load(read_file(path))


def test_vo_launch_separates_driver_and_monitor_configs() -> None:
    launch_text = read_file("launch/realsense_vo.launch.py")

    assert 'LaunchConfiguration("driver_config_file")' in launch_text
    assert 'LaunchConfiguration("monitor_config_file")' in launch_text
    assert '"config_file": driver_config_file' in launch_text
    assert launch_text.count("parameters=[monitor_config_file]") == 2
    assert "parameters=[driver_config_file]" not in launch_text
    assert '"realsense_vo_driver.yaml"' in launch_text
    assert '"realsense_vo_profile.yaml"' in launch_text


def test_vo_driver_config_is_flat_and_enables_required_streams() -> None:
    params = load_yaml("config/realsense_vo_driver.yaml")

    assert "realsense2_camera" not in params
    assert "/camera/camera" not in params
    assert "ros__parameters" not in params
    assert all(not isinstance(value, dict) for value in params.values())

    assert params["camera_name"] == "camera"
    assert params["camera_namespace"] == "camera"
    assert params["enable_color"] is True
    assert params["enable_depth"] is True
    assert params["depth_module.depth_profile"] == "640x480x30"
    assert params["rgb_camera.color_profile"] == "640x480x30"
    assert params["align_depth.enable"] is True
    assert params["enable_sync"] is True
    assert params["pointcloud.enable"] is True
    assert params["pointcloud.stream_filter"] == 2
    assert params["pointcloud.stream_index_filter"] == 0
    assert params["pointcloud.allow_no_texture_points"] is True
    assert params["pointcloud.ordered_pc"] is False


def test_vo_driver_disables_tf_infrared_and_motion_streams() -> None:
    params = load_yaml("config/realsense_vo_driver.yaml")

    assert params["publish_tf"] is False
    assert params["tf_publish_rate"] == 0.0
    assert params["enable_infra"] is False
    assert params["enable_infra1"] is False
    assert params["enable_infra2"] is False
    assert params["enable_gyro"] is False
    assert params["enable_accel"] is False
    assert not any(key.startswith("pointcloud__neon_") for key in params)


def test_vo_monitor_profile_requires_pointcloud() -> None:
    config = load_yaml("config/realsense_vo_profile.yaml")

    assert set(config) == {
        "camera_topic_monitor_node",
        "camera_health_node",
    }
    for node_name in config:
        params = config[node_name]["ros__parameters"]
        assert params["expected_pointcloud_hz"] == 30.0
        assert params["require_pointcloud"] is True


def test_vo_pointcloud_topic_contract_remains_raw_filter_input() -> None:
    assert DEFAULT_POINTCLOUD_TOPIC == "/camera/camera/depth/color/points"


def test_cmake_installs_vo_launch_and_driver_config_directories() -> None:
    cmake = read_file("CMakeLists.txt")

    assert (PACKAGE_ROOT / "launch" / "realsense_vo.launch.py").is_file()
    assert (PACKAGE_ROOT / "config" / "realsense_vo_driver.yaml").is_file()
    assert re.search(
        r"install\(DIRECTORY\s+launch/\s+"
        r"DESTINATION share/\$\{PROJECT_NAME\}/launch",
        cmake,
    )
    assert re.search(
        r"install\(DIRECTORY\s+config/\s+"
        r"DESTINATION share/\$\{PROJECT_NAME\}/config",
        cmake,
    )
