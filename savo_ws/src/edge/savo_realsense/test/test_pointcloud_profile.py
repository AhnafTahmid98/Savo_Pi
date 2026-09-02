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
    assert params["pointcloud__neon_.stream_filter"] == 1
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
    assert params["pointcloud__neon_.stream_filter"] == 1
    assert params["pointcloud__neon_.stream_index_filter"] == 0
    assert params["pointcloud__neon_.allow_no_texture_points"] is True
    assert params["pointcloud__neon_.ordered_pc"] is False
    assert params["pointcloud__neon_.pointcloud_qos"] == "SENSOR_DATA"


def test_d435_health_uses_required_lightweight_pipeline_signals() -> None:
    config = load_yaml("realsense_d435_nodes.yaml")

    monitor_params = config["camera_topic_monitor_node"]["ros__parameters"]
    health_params = config["camera_health_node"]["ros__parameters"]

    assert monitor_params["require_pointcloud"] is True
    assert monitor_params["expected_pointcloud_hz"] > 0.0
    assert monitor_params["require_aligned_depth"] is True
    assert monitor_params["expected_aligned_depth_hz"] > 0.0
    assert health_params == {
        "status_hz": 2.0,
        "stale_timeout_s": 0.75,
        "depth_signal_topic": "/depth/min_front_m",
        "vo_health_topic": "/vo/health",
        "obstacle_cloud_health_topic": (
            "/savo_perception/obstacle_cloud/health"
        ),
        "require_depth_signal": True,
        "require_vo_health": True,
        "require_obstacle_cloud_health": True,
    }


def test_compatibility_nodes_use_same_lightweight_health_contract() -> None:
    config = load_yaml("realsense_pointcloud_nodes.yaml")

    monitor_params = config["camera_topic_monitor_node"]["ros__parameters"]
    health_params = config["camera_health_node"]["ros__parameters"]

    assert monitor_params["require_pointcloud"] is True
    assert monitor_params["expected_pointcloud_hz"] > 0.0
    assert health_params["require_depth_signal"] is True
    assert health_params["require_vo_health"] is True
    assert health_params["require_obstacle_cloud_health"] is True


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


def test_production_health_does_not_subscribe_to_camera_payloads() -> None:
    health_implementation = (
        PACKAGE_ROOT / "src" / "camera_health_main.cpp"
    ).read_text(encoding="utf-8")
    monitor_implementation = (
        PACKAGE_ROOT / "src" / "camera_topic_monitor_main.cpp"
    ).read_text(encoding="utf-8")

    for forbidden in (
        "sensor_msgs/msg/image.hpp",
        "sensor_msgs/msg/camera_info.hpp",
        "sensor_msgs/msg/point_cloud2.hpp",
        "create_subscription<sensor_msgs",
        "/camera/camera/color/image_raw",
        "/camera/camera/depth/image_rect_raw",
        "/camera/camera/aligned_depth_to_color/image_raw",
        "/camera/camera/depth/color/points",
    ):
        assert forbidden not in health_implementation

    assert "create_subscription<std_msgs::msg::Float32>" in health_implementation
    assert "create_subscription<std_msgs::msg::String>" in health_implementation
    assert "create_subscription<std_msgs::msg::Bool>" in health_implementation
    assert "/depth/min_front_m" in health_implementation
    assert "/vo/health" in health_implementation
    assert "/savo_perception/obstacle_cloud/health" in health_implementation
    assert "if (depth_signal_.required)" in health_implementation
    assert "if (vo_health_.required)" in health_implementation
    assert "if (obstacle_cloud_.required)" in health_implementation

    # The high-bandwidth monitor remains available only as an explicit
    # diagnostic tool and is not part of canonical production bringup.
    assert "/camera/camera/color/image_raw" in monitor_implementation
    assert "/camera/camera/depth/image_rect_raw" in monitor_implementation
    assert "/camera/camera/aligned_depth_to_color/image_raw" in monitor_implementation
    assert "/camera/camera/depth/color/points" in monitor_implementation

    assert '\\"color_ok\\"' in health_implementation
    assert '\\"color_info_ok\\"' in health_implementation
    assert '\\"depth_ok\\"' in health_implementation
    assert '\\"depth_info_ok\\"' in health_implementation
    assert '\\"aligned_depth_ok\\"' in health_implementation
    assert '\\"require_aligned_depth\\"' in health_implementation
    assert '\\"pointcloud_ok\\"' in health_implementation
    assert '\\"require_pointcloud\\"' in health_implementation


def test_native_health_state_has_fail_closed_regression_tests() -> None:
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    test_source = (
        PACKAGE_ROOT / "test" / "test_camera_health_state.cpp"
    ).read_text(encoding="utf-8")

    assert "ament_add_gtest(test_camera_health_state" in cmake
    assert "MissingRequiredDepthSignalFailsClosed" in test_source
    assert "StaleRequiredVoSignalFailsClosed" in test_source
    assert "UnhealthyRequiredObstacleCloudFailsClosed" in test_source
    assert "DisabledOptionalSignalsDoNotCreateFalseFailures" in test_source


def test_minimal_profile_does_not_require_aligned_depth() -> None:
    config = load_yaml("realsense_minimal.yaml")

    monitor_params = config["camera_topic_monitor_node"]["ros__parameters"]
    health_params = config["camera_health_node"]["ros__parameters"]

    assert monitor_params["require_aligned_depth"] is False
    assert health_params["require_depth_signal"] is False
    assert health_params["require_vo_health"] is False
    assert health_params["require_obstacle_cloud_health"] is False


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
