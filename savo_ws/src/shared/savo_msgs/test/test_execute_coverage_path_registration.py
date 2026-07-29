"""Validate ExecuteCoveragePath build and package registration."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
CMAKE = PACKAGE / "CMakeLists.txt"
MANIFEST = PACKAGE / "package.xml"
ACTION_RELATIVE = "action/ExecuteCoveragePath.action"


def test_action_is_registered_exactly_once() -> None:
    """Require exactly one ROSIDL registration."""
    cmake = CMAKE.read_text(encoding="utf-8")

    assert cmake.count(f'"{ACTION_RELATIVE}"') == 1
    assert "${action_files}" in cmake
    assert "rosidl_generate_interfaces" in cmake


def test_required_interface_dependencies_are_registered() -> None:
    """Require dependencies used by the action schema."""
    cmake = CMAKE.read_text(encoding="utf-8")

    assert "find_package(builtin_interfaces REQUIRED)" in cmake
    assert "find_package(nav_msgs REQUIRED)" in cmake
    assert "builtin_interfaces" in cmake
    assert "nav_msgs" in cmake


def test_package_manifest_exports_required_dependencies() -> None:
    """Require matching package dependencies and ROSIDL membership."""
    root = ET.parse(MANIFEST).getroot()

    dependencies = {
        element.text.strip()
        for element in root.findall("depend")
        if element.text
    }

    groups = {
        element.text.strip()
        for element in root.findall("member_of_group")
        if element.text
    }

    assert "builtin_interfaces" in dependencies
    assert "nav_msgs" in dependencies
    assert "rosidl_interface_packages" in groups


def test_new_contract_tests_are_registered_once() -> None:
    """Require both static tests under BUILD_TESTING."""
    cmake = CMAKE.read_text(encoding="utf-8")

    expected = (
        "test/test_execute_coverage_path_action_contract.py",
        "test/test_execute_coverage_path_registration.py",
    )

    for test_path in expected:
        assert cmake.count(test_path) == 1
