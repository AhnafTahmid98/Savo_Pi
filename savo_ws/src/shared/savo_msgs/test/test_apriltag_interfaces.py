from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def read_file(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def constant_value(text: str, name: str) -> str:
    pattern = (
        rf"^\s*[A-Za-z0-9_/]+\s+"
        rf"{re.escape(name)}=(\S+)\s*$"
    )

    match = re.search(pattern, text, re.MULTILINE)

    assert match is not None, f"Missing constant: {name}"
    return match.group(1)


def test_apriltag_observation_contract() -> None:
    text = read_file("msg/AprilTagObservation.msg")

    required_fields = {
        "std_msgs/Header header",
        "string detector_name",
        "string family",
        "int32 tag_id",
        "float64 tag_size_m",
        "uint64 observation_sequence",
        "uint32 image_width",
        "uint32 image_height",
        "float32[8] image_corners_xy",
        "float32 detection_quality",
        "float32 decision_margin",
        "uint8 hamming_distance",
        "bool pose_valid",
        "geometry_msgs/PoseWithCovariance pose",
        "float32 pose_error",
    }

    for field in required_fields:
        assert field in text, f"Missing field: {field}"


def test_confirmation_action_has_three_sections() -> None:
    text = read_file("action/ConfirmAprilTag.action")

    assert text.count("\n---\n") == 2


def test_confirmation_action_locks_two_duties() -> None:
    text = read_file("action/ConfirmAprilTag.action")

    assert constant_value(text, "REGISTER_LOCATION") == "1"
    assert constant_value(text, "CONFIRM_ARRIVAL") == "2"
    assert constant_value(text, "ANY_TAG_ID") == "-1"

    required_goal_fields = {
        "uint8 mode",
        "string expected_family",
        "int32 expected_tag_id",
        "string location_id",
        "string map_id",
        "uint32 map_revision",
        "builtin_interfaces/Duration timeout",
        "bool require_map_pose",
    }

    for field in required_goal_fields:
        assert field in text, f"Missing goal field: {field}"


def test_confirmation_action_result_contract() -> None:
    text = read_file("action/ConfirmAprilTag.action")

    required_result_fields = {
        "bool confirmed",
        "uint8 result_code",
        "string reason",
        "savo_msgs/AprilTagObservation final_observation",
        "bool map_pose_valid",
        "geometry_msgs/PoseStamped tag_pose_map",
        "uint32 accepted_observations",
        "uint32 rejected_observations",
        "float32 position_stddev_m",
        "float32 yaw_stddev_rad",
    }

    for field in required_result_fields:
        assert field in text, f"Missing result field: {field}"


def test_confirmation_action_terminal_results() -> None:
    text = read_file("action/ConfirmAprilTag.action")

    expected = {
        "RESULT_CONFIRMED": "0",
        "RESULT_INVALID_REQUEST": "1",
        "RESULT_NOT_FOUND": "2",
        "RESULT_WRONG_TAG": "3",
        "RESULT_UNSTABLE": "4",
        "RESULT_CAMERA_UNAVAILABLE": "5",
        "RESULT_HEAD_UNAVAILABLE": "6",
        "RESULT_ROBOT_MOVING": "7",
        "RESULT_TF_UNAVAILABLE": "8",
        "RESULT_LOCALIZATION_UNHEALTHY": "9",
        "RESULT_TIMED_OUT": "10",
        "RESULT_CANCELED": "11",
        "RESULT_INTERNAL_ERROR": "12",
    }

    actual = {
        name: constant_value(text, name)
        for name in expected
    }

    assert actual == expected


def test_cmake_generates_message_and_action() -> None:
    text = read_file("CMakeLists.txt")

    assert '"msg/AprilTagObservation.msg"' in text
    assert '"action/ConfirmAprilTag.action"' in text
    assert "rosidl_generate_interfaces(${PROJECT_NAME}" in text
    assert "ament_add_pytest_test(" in text


def test_package_metadata() -> None:
    root = ET.parse(ROOT / "package.xml").getroot()

    assert root.findtext("name") == "savo_msgs"
    assert root.findtext("version") == "0.8.0"
    assert root.findtext("member_of_group") == "rosidl_interface_packages"

    dependencies = {
        element.text
        for element in root.findall("depend")
    }

    assert {
        "builtin_interfaces",
        "std_msgs",
        "geometry_msgs",
        "nav_msgs",
    } <= dependencies

    build_dependencies = {
        element.text
        for element in root.findall("build_depend")
    }

    assert "rosidl_default_generators" in build_dependencies

    test_dependencies = {
        element.text
        for element in root.findall("test_depend")
    }

    assert {
        "ament_cmake_pytest",
        "python3-pytest",
    } <= test_dependencies
