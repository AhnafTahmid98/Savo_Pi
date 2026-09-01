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
    assert params["color_qos"] == "SENSOR_DATA"
    assert params["depth_qos"] == "SENSOR_DATA"
    assert params["align_depth.enable"] is True
    assert params["enable_sync"] is True


def test_main_d435_profile_keeps_vo_streams_and_pointcloud() -> None:
    config = load_yaml("realsense_d435_camera.yaml")
    params = config["/camera/camera"]["ros__parameters"]

    assert params["enable_color"] is True
    assert params["enable_depth"] is True
    assert params["depth_module.depth_profile"] == "848x480x30"
    assert params["rgb_camera.color_profile"] == "640x480x30"
    assert params["align_depth.enable"] is True
    assert params["enable_sync"] is True
    assert params["pointcloud__neon_.enable"] is True
    assert params["pointcloud__neon_.stream_filter"] == 0
    assert params["pointcloud__neon_.stream_index_filter"] == 0
    assert params["pointcloud__neon_.allow_no_texture_points"] is True
    assert params["pointcloud__neon_.ordered_pc"] is False
    assert params["pointcloud__neon_.pointcloud_qos"] == "SENSOR_DATA"


def test_pointcloud_camera_profile_uses_direct_node_runtime_parameters() -> None:
    config = load_yaml("realsense_pointcloud_camera.yaml")
    params = config["/camera/camera"]["ros__parameters"]

    # Unlike rs_launch.py's flat config, direct RealSense ROS 4.58.1 node
    # parameters use the runtime NEON plugin names.
    assert params["pointcloud__neon_.enable"] is True
    assert params["pointcloud__neon_.stream_filter"] == 0
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
    assert monitor_params["require_aligned_depth"] is True
    assert health_params["require_aligned_depth"] is True
    assert monitor_params["expected_aligned_depth_hz"] > 0.0
    assert health_params["expected_aligned_depth_hz"] > 0.0


def test_pointcloud_nodes_require_pointcloud() -> None:
    config = load_yaml("realsense_pointcloud_nodes.yaml")

    monitor_params = config["camera_topic_monitor_node"]["ros__parameters"]
    health_params = config["camera_health_node"]["ros__parameters"]

    assert monitor_params["require_pointcloud"] is True
    assert health_params["require_pointcloud"] is True
    assert monitor_params["expected_pointcloud_hz"] > 0.0
    assert health_params["expected_pointcloud_hz"] > 0.0


def test_compatibility_pointcloud_profiles_match_canonical_production() -> None:
    assert load_yaml("realsense_pointcloud_camera.yaml") == load_yaml(
        "realsense_d435_camera.yaml"
    )
    assert load_yaml("realsense_pointcloud_nodes.yaml") == load_yaml(
        "realsense_d435_nodes.yaml"
    )


def test_native_health_uses_fresh_nonzero_pointcloud_not_expected_rate() -> None:
    implementation = (
        PACKAGE_ROOT / "src" / "camera_monitor_common.cpp"
    ).read_text(encoding="utf-8")
    ok_body = implementation.split(
        "bool StreamStatus::ok() const", maxsplit=1
    )[1].split("}", maxsplit=1)[0]

    assert "return seen && !stale && rate_hz > 0.0;" in ok_body
    assert "expected_hz" not in ok_body


def test_native_health_reports_and_gates_required_aligned_depth() -> None:
    health_implementation = (
        PACKAGE_ROOT / "src" / "camera_health_main.cpp"
    ).read_text(encoding="utf-8")
    monitor_implementation = (
        PACKAGE_ROOT / "src" / "camera_topic_monitor_main.cpp"
    ).read_text(encoding="utf-8")

    for implementation in (health_implementation, monitor_implementation):
        assert (
            "/camera/camera/aligned_depth_to_color/image_raw" in implementation
        )
        assert (
            'declare_parameter<double>("expected_aligned_depth_hz", 20.0)'
            in implementation
        )
        assert (
            'declare_parameter<bool>("require_aligned_depth", false)'
            in implementation
        )

    assert '\\"aligned_depth_ok\\"' in health_implementation
    assert '\\"require_aligned_depth\\"' in health_implementation
    assert "aligned_depth_ok && pointcloud_ok" in health_implementation


def test_minimal_profile_does_not_require_aligned_depth() -> None:
    config = load_yaml("realsense_minimal.yaml")

    for node_name in ("camera_topic_monitor_node", "camera_health_node"):
        params = config[node_name]["ros__parameters"]
        assert params["require_aligned_depth"] is False


def test_all_runtime_driver_profiles_leave_fixed_tf_to_description() -> None:
    for name in (
        "realsense_minimal.yaml",
        "realsense_d435_camera.yaml",
        "realsense_vo_driver.yaml",
        "realsense_pointcloud_camera.yaml",
        "realsense_nav_profile.yaml",
    ):
        config = load_yaml(name)
        driver_params = next(iter(config.values()))["ros__parameters"]
        assert driver_params["publish_tf"] is False, name
