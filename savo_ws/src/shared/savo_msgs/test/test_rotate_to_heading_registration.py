"""Regression tests for RotateToHeading ROSIDL registration."""

from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]

CMAKE = ROOT / "CMakeLists.txt"
PACKAGE_XML = ROOT / "package.xml"

ROTATE_ACTION = (
    ROOT /
    "action/RotateToHeading.action"
)

CONFIRM_ACTION = (
    ROOT /
    "action/ConfirmAprilTag.action"
)


def read_text(path: Path) -> str:
    """Read a UTF-8 package contract file."""

    return path.read_text(
        encoding="utf-8",
    )


def interface_sections(text: str):
    """Return uncommented action fields by section."""

    sections = [[]]

    for raw_line in text.splitlines():
        line = raw_line.split(
            "#",
            maxsplit=1,
        )[0].strip()

        if not line:
            continue

        if line == "---":
            sections.append([])
            continue

        sections[-1].append(line)

    return sections


def action_files_body(cmake: str) -> str:
    """Extract the ROSIDL action_files list."""

    match = re.search(
        r"set\s*\(\s*action_files"
        r"(?P<body>.*?)"
        r"\n\s*\)",
        cmake,
        flags=re.DOTALL,
    )

    assert match is not None

    return match.group("body")


def generator_body(cmake: str) -> str:
    """Extract rosidl_generate_interfaces arguments."""

    match = re.search(
        r"rosidl_generate_interfaces"
        r"\s*\("
        r"(?P<body>.*?)"
        r"\n\s*\)",
        cmake,
        flags=re.DOTALL,
    )

    assert match is not None

    return match.group("body")


def test_both_actions_are_registered() -> None:
    """Preserve AprilTag and rotate action generation."""

    cmake = read_text(CMAKE)
    body = action_files_body(cmake)

    assert (
        body.count(
            '"action/ConfirmAprilTag.action"'
        )
        == 1
    )

    assert (
        body.count(
            '"action/RotateToHeading.action"'
        )
        == 1
    )

    generated = generator_body(cmake)

    assert "${msg_files}" in generated
    assert "${action_files}" in generated


def test_rotate_action_schema_is_exact() -> None:
    """Keep the locked RotateToHeading schema."""

    assert ROTATE_ACTION.is_file()
    assert CONFIRM_ACTION.is_file()

    sections = interface_sections(
        read_text(ROTATE_ACTION)
    )

    assert sections == [
        [
            "float64 target_yaw_rad",
            "float64 max_duration_sec",
        ],
        [
            "bool success",
            "float64 final_yaw_rad",
            "float64 final_error_rad",
            "string reason",
        ],
        [
            "float64 current_yaw_rad",
            "float64 target_yaw_rad",
            "float64 error_rad",
            "float64 commanded_wz_rad_s",
            "float64 elapsed_sec",
            "bool safety_stop_active",
            "string state",
        ],
    ]


def test_registration_test_is_enabled() -> None:
    """Run this regression test through CTest."""

    cmake = read_text(CMAKE)

    assert (
        cmake.count(
            "test_rotate_to_heading_registration"
        )
        == 2
    )

    assert (
        cmake.count(
            "test/test_rotate_to_heading_registration.py"
        )
        == 1
    )


def test_package_exports_rosidl_runtime() -> None:
    """The interface package must export ROSIDL runtime."""

    package_root = ET.parse(
        PACKAGE_XML
    ).getroot()

    assert (
        package_root.findtext(
            "name",
            default="",
        ).strip()
        == "savo_msgs"
    )

    package_text = read_text(PACKAGE_XML)

    assert "rosidl_default_generators" in package_text
    assert "rosidl_default_runtime" in package_text
    assert "rosidl_interface_packages" in package_text
