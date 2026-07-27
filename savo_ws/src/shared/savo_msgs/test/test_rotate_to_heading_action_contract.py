from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]

ACTION_PATH = ROOT / "action/RotateToHeading.action"
CMAKE_PATH = ROOT / "CMakeLists.txt"
PACKAGE_XML_PATH = ROOT / "package.xml"


EXPECTED_INTERFACE_LINES = [
    "float64 target_yaw_rad",
    "float64 max_duration_sec",
    "---",
    "bool success",
    "float64 final_yaw_rad",
    "float64 final_error_rad",
    "string reason",
    "---",
    "float64 current_yaw_rad",
    "float64 target_yaw_rad",
    "float64 error_rad",
    "float64 commanded_wz_rad_s",
    "float64 elapsed_sec",
    "bool safety_stop_active",
    "string state",
]


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def interface_lines() -> list[str]:
    result = []

    for raw_line in read_text(ACTION_PATH).splitlines():
        line = raw_line.strip()

        if not line:
            continue

        if line.startswith("#"):
            continue

        result.append(line)

    return result


def test_action_definition_has_exact_contract() -> None:
    assert interface_lines() == EXPECTED_INTERFACE_LINES


def test_action_has_three_sections() -> None:
    lines = interface_lines()

    assert lines.count("---") == 2
    assert lines.index("---") == 2
    assert lines.index("---", 3) == 7


def test_action_uses_ros_action_goal_identity() -> None:
    text = read_text(ACTION_PATH).lower()

    assert "request_id" not in text
    assert "goal_id" not in text
    assert "cancel_ack" not in text


def test_action_exposes_execution_feedback() -> None:
    lines = interface_lines()

    required = {
        "float64 current_yaw_rad",
        "float64 target_yaw_rad",
        "float64 error_rad",
        "float64 commanded_wz_rad_s",
        "float64 elapsed_sec",
        "bool safety_stop_active",
        "string state",
    }

    assert required.issubset(lines)


def test_cmake_generates_action_interface() -> None:
    cmake = read_text(CMAKE_PATH)

    assert "set(action_files" in cmake
    assert '"action/RotateToHeading.action"' in cmake
    assert "${action_files}" in cmake

    generator_start = cmake.index(
        "rosidl_generate_interfaces"
    )
    generator_text = cmake[generator_start:]

    assert "${msg_files}" in generator_text
    assert "${action_files}" in generator_text


def test_contract_test_is_registered() -> None:
    cmake = read_text(CMAKE_PATH)

    assert "find_package(ament_cmake_pytest REQUIRED)" in cmake
    assert "ament_add_pytest_test(" in cmake
    assert "test_rotate_to_heading_action_contract" in cmake

    assert (
        "test/test_rotate_to_heading_action_contract.py"
        in cmake
    )

    assert "TIMEOUT 60" in cmake


def test_package_declares_test_dependencies() -> None:
    root = ET.parse(PACKAGE_XML_PATH).getroot()

    test_dependencies = {
        element.text.strip()
        for element in root.findall("test_depend")
        if element.text
    }

    assert "ament_cmake_pytest" in test_dependencies
    assert "python3-pytest" in test_dependencies


def test_package_remains_interface_package() -> None:
    root = ET.parse(PACKAGE_XML_PATH).getroot()

    groups = {
        element.text.strip()
        for element in root.findall("member_of_group")
        if element.text
    }

    assert "rosidl_interface_packages" in groups
