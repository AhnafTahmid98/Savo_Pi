from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src/nodes/apriltag_detector_node.cpp"


def source_text() -> str:
    return SOURCE.read_text(encoding="utf-8")


def test_detector_uses_real_apriltag3_and_typed_ros_interfaces() -> None:
    text = source_text()

    for token in (
        "<apriltag/apriltag.h>",
        "<apriltag/apriltag_pose.h>",
        "<apriltag/tag36h11.h>",
        '"savo_msgs/msg/april_tag_observation.hpp"',
        "create_subscription<sensor_msgs::msg::Image>",
        "create_subscription<sensor_msgs::msg::CameraInfo>",
        "create_publisher<Observation>",
        "apriltag_detector_detect",
        "estimate_tag_pose",
    ):
        assert token in text

    assert "std_msgs/msg/string" not in text
    assert "nlohmann" not in text
    assert "json" not in text.lower()


def test_detector_declares_locked_parameter_names() -> None:
    text = source_text()

    for parameter in (
        '"apriltag_enabled"',
        '"apriltag_family"',
        '"tag_size_m"',
        '"pose_estimation_enabled"',
        '"image_topic"',
        '"camera_info_topic"',
        '"observation_topic"',
        '"camera_optical_frame"',
        '"allowed_tag_ids"',
        '"maximum_hamming_distance"',
        '"maximum_detection_distance_m"',
    ):
        assert parameter in text


def test_detector_preserves_camera_timestamp_and_optical_frame() -> None:
    text = source_text()

    assert "observation.header.stamp = image->header.stamp" in text
    assert "observation.header.frame_id = config_.camera_optical_frame" in text
    assert "observation.pose_valid = estimate_pose" in text
    assert "observation.pose_valid ? config_.tag_size_m : 0.0" in text


def test_detector_callback_is_bounded_to_one_pending_image() -> None:
    text = source_text()

    for token in (
        "pending_image_",
        "frames_replaced_",
        "worker_condition_",
        "worker_thread_",
        "std::move(pending_image_)",
    ):
        assert token in text


def test_detector_dependencies_and_target_are_declared() -> None:
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    for token in (
        "find_package(cv_bridge REQUIRED)",
        "find_package(OpenCV REQUIRED COMPONENTS core imgproc)",
        "pkg_check_modules(APRILTAG REQUIRED IMPORTED_TARGET apriltag)",
        "apriltag_detector_node",
        "PkgConfig::APRILTAG",
        "Threads::Threads",
    ):
        assert token in cmake

    package_root = ET.parse(ROOT / "package.xml").getroot()
    dependencies = {
        (node.text or "").strip()
        for tag in ("depend", "build_depend", "exec_depend")
        for node in package_root.findall(tag)
    }
    assert {
        "cv_bridge",
        "libapriltag-dev",
        "libopencv-dev",
        "pkg-config",
    } <= dependencies
