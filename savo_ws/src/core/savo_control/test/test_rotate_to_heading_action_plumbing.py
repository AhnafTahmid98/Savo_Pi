from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]

PACKAGE_XML = ROOT / "package.xml"
CMAKE = ROOT / "CMakeLists.txt"
TOPIC_NAMES = (
    ROOT /
    "include/savo_control/topic_names.hpp"
)


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_package_declares_rclcpp_action() -> None:
    root = ET.parse(PACKAGE_XML).getroot()

    dependencies = {
        element.text.strip()
        for element in root.findall("depend")
        if element.text
    }

    assert "rclcpp_action" in dependencies
    assert "savo_msgs" in dependencies


def test_cmake_finds_rclcpp_action() -> None:
    cmake = read_text(CMAKE)

    assert (
        "find_package(rclcpp_action REQUIRED)"
        in cmake
    )


def test_rotate_node_links_rclcpp_action() -> None:
    cmake = read_text(CMAKE)

    pattern = re.compile(
        r"ament_target_dependencies\s*\(\s*"
        r"rotate_to_heading_node\s+"
        r"rclcpp_action\s*\)",
        re.MULTILINE,
    )

    assert pattern.search(cmake)


def test_action_endpoint_is_canonical() -> None:
    header = read_text(TOPIC_NAMES)

    assert (
        "inline constexpr const char * "
        "ROTATE_TO_HEADING_ACTION = "
        '"/savo_control/rotate_to_heading";'
        in header
    )


def test_action_endpoint_is_unique() -> None:
    header = read_text(TOPIC_NAMES)

    assert (
        header.count(
            '"/savo_control/rotate_to_heading"'
        )
        == 1
    )

    assert (
        header.count(
            "ROTATE_TO_HEADING_ACTION"
        )
        == 1
    )


def test_plumbing_test_is_registered() -> None:
    cmake = read_text(CMAKE)

    assert (
        "test_rotate_to_heading_action_plumbing"
        in cmake
    )

    assert (
        "test/"
        "test_rotate_to_heading_action_plumbing.py"
        in cmake
    )
