from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def test_live_camera_status_uses_only_lightweight_runtime_evidence():
    source = read("src/nodes/head_camera_status_node.cpp")

    for fragment in (
        "class HeadCameraStatusNode",
        "sensor_msgs::msg::CameraInfo",
        "evaluate_camera_health(",
        "camera_health_status_text(",
        "metadata_timestamp_monotonic_",
        "camera_info_is_calibrated(",
        "frame_rate_ema_alpha_",
        "count_publishers(image_topic_)",
        "create_service<std_srvs::srv::Trigger>",
        "savo_head.camera",
    ):
        assert fragment in source, f"Missing node contract: {fragment}"

    forbidden = (
        '#include "sensor_msgs/msg/image.hpp"',
        "create_subscription<sensor_msgs::msg::Image>",
        "create_subscription<sensor_msgs::msg::CompressedImage>",
        "image_data_is_valid",
        "image_sub_",
    )
    for fragment in forbidden:
        assert fragment not in source


def test_live_camera_status_topics_and_public_outputs_are_locked():
    source = read("src/nodes/head_camera_status_node.cpp")

    for fragment in (
        "kTopicCameraImageRaw",
        "kTopicCameraInfo",
        "kTopicCameraStatus",
        "kFrameCameraOptical",
        '"stream_healthy"',
        '"ready_for_pose_estimation"',
        '"stream_metadata_seen"',
        '"image_publisher_present"',
    ):
        assert fragment in source


def test_camera_health_yaml_matches_node():
    params = yaml.safe_load(read("config/camera_health.yaml"))[
        "/savo_head/head_camera_status_node"
    ]["ros__parameters"]

    assert params["image_topic"] == "/savo_head/camera/image_raw"
    assert params["camera_info_topic"] == "/savo_head/camera/camera_info"
    assert params["status_topic"] == "/savo_head/camera/status"
    assert params["health_check_service"] == "/savo_head/camera/health_check"
    assert params["expected_frame_id"] == "pi_camera_optical_frame"
    assert params["expected_encoding"] == "rgb8"
    assert params["status_publish_hz"] > 0.0


def test_cmake_builds_live_camera_status_node():
    cmake = read("CMakeLists.txt")
    assert (
        "add_savo_head_node(head_camera_status_node "
        "src/nodes/head_camera_status_node.cpp)" in cmake
    )


def test_zero_sized_uncalibrated_metadata_has_documented_config_fallback():
    core = read("src/core/camera_health.cpp")

    assert "metadata_dimensions_missing" in core
    assert "snapshot.metadata_width == 0U" in core
    assert "snapshot.metadata_height == 0U" in core
    assert "config.expected_width" in core
    assert "config.expected_height" in core
