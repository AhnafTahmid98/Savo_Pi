from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def test_typed_apriltag_contract_names_and_duties() -> None:
    text = (
        ROOT
        / "include/savo_head/core/apriltag_action_contract.hpp"
    ).read_text(encoding="utf-8")

    for token in (
        "/savo_head/apriltag/observations",
        "/savo_head/apriltag/confirm",
        "kRegisterLocation = 1U",
        "kConfirmArrival = 2U",
        "IsValidDuty",
    ):
        assert token in text


def test_typed_dependencies_are_declared_without_removing_legacy_node() -> None:
    root = ET.parse(ROOT / "package.xml").getroot()
    dependencies = {
        (node.text or "").strip()
        for node in root.findall("depend")
    }

    assert {"savo_msgs", "rclcpp_action"} <= dependencies

    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "find_package(savo_msgs REQUIRED)" in cmake
    assert "find_package(rclcpp_action REQUIRED)" in cmake
    assert "apriltag_confirm_node src/nodes/apriltag_confirm_node.cpp" in cmake
