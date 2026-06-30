# Copyright 2026 Ahnaf Tahmid
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_yaml(name: str) -> dict:
    with (PACKAGE_ROOT / "config" / name).open("r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def test_pointcloud_camera_profile_matches_robot_savo_d435_config() -> None:
    config = load_yaml("realsense_pointcloud_camera.yaml")
    params = config["/camera/camera"]["ros__parameters"]

    assert params["enable_color"] is True
    assert params["enable_depth"] is True
    assert params["depth_module.depth_profile"] == "848x480x30"
    assert params["rgb_camera.color_profile"] == "640x480x30"
    assert params["align_depth.enable"] is True
    assert params["enable_sync"] is True


def test_main_d435_profile_has_permanent_pointcloud_enabled() -> None:
    config = load_yaml("realsense_d435_camera.yaml")
    params = config["/camera/camera"]["ros__parameters"]

    assert params["align_depth.enable"] is True
    assert params["enable_sync"] is True
    assert params["pointcloud__neon_.enable"] is True
    assert params["pointcloud__neon_.stream_filter"] == 2
    assert params["pointcloud__neon_.stream_index_filter"] == 0
    assert params["pointcloud__neon_.allow_no_texture_points"] is True
    assert params["pointcloud__neon_.ordered_pc"] is False
    assert params["pointcloud__neon_.pointcloud_qos"] == "SENSOR_DATA"


def test_pointcloud_camera_profile_enables_neon_pointcloud() -> None:
    config = load_yaml("realsense_pointcloud_camera.yaml")
    params = config["/camera/camera"]["ros__parameters"]

    assert params["pointcloud__neon_.enable"] is True
    assert params["pointcloud__neon_.stream_filter"] == 2
    assert params["pointcloud__neon_.stream_index_filter"] == 0
    assert params["pointcloud__neon_.allow_no_texture_points"] is True
    assert params["pointcloud__neon_.ordered_pc"] is False
    assert params["pointcloud__neon_.pointcloud_qos"] == "SENSOR_DATA"


def test_d435_nodes_require_pointcloud() -> None:
    config = load_yaml("realsense_d435_nodes.yaml")

    monitor_params = config["camera_topic_monitor_node"]["ros__parameters"]
    health_params = config["camera_health_node"]["ros__parameters"]

    assert monitor_params["require_pointcloud"] is True
    assert health_params["require_pointcloud"] is True
    assert monitor_params["expected_pointcloud_hz"] > 0.0
    assert health_params["expected_pointcloud_hz"] > 0.0


def test_pointcloud_nodes_require_pointcloud() -> None:
    config = load_yaml("realsense_pointcloud_nodes.yaml")

    monitor_params = config["camera_topic_monitor_node"]["ros__parameters"]
    health_params = config["camera_health_node"]["ros__parameters"]

    assert monitor_params["require_pointcloud"] is True
    assert health_params["require_pointcloud"] is True
    assert monitor_params["expected_pointcloud_hz"] > 0.0
    assert health_params["expected_pointcloud_hz"] > 0.0
