from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(
        encoding="utf-8"
    )


def parse_version(value: str) -> tuple[int, int, int]:
    major, minor, patch = value.split(".")

    return (
        int(major),
        int(minor),
        int(patch),
    )


def test_package_contains_loc1b_or_later() -> None:
    package = ET.parse(
        ROOT / "package.xml"
    ).getroot()

    version = package.findtext("version")

    assert version is not None
    assert parse_version(version) >= (0, 3, 0)

    constants = read(
        "include/savo_locations/constants.hpp"
    )

    assert f'"{version}"' in constants


def test_registry_files_exist() -> None:
    for relative in (
        "include/savo_locations/registry.hpp",
        "src/registry.cpp",
        "test/unit/test_registry.cpp",
    ):
        assert (ROOT / relative).is_file()


def test_cmake_builds_registry() -> None:
    cmake = read("CMakeLists.txt")

    assert "src/registry.cpp" in cmake
    assert "test_location_registry" in cmake
    assert "test_phase1b_contracts" in cmake


def test_registry_supports_safe_resolution() -> None:
    header = read(
        "include/savo_locations/registry.hpp"
    )

    for required in (
        "kInvalidQuery",
        "kNotFound",
        "kAmbiguous",
        "kDisabled",
        "kRetired",
        "kMapMismatch",
        "kLocationId",
        "kDisplayName",
        "kAlias",
    ):
        assert required in header

    assert "enforce_map_context" in header
    assert "ambiguous_location_ids" in header


def test_registry_supports_revision_guards() -> None:
    header = read(
        "include/savo_locations/registry.hpp"
    )

    implementation = read(
        "src/registry.cpp"
    )

    assert "expected_current_revision" in header
    assert "kStaleRevision" in header
    assert "kRevisionSequenceError" in header

    assert (
        "current revision plus one"
        in implementation
    )


def test_registry_checks_identity_and_tag_conflicts() -> None:
    header = read(
        "include/savo_locations/registry.hpp"
    )

    implementation = read(
        "src/registry.cpp"
    )

    assert "kIdentityConflict" in header
    assert "kTagConflict" in header

    assert (
        "has_identity_collision_locked"
        in implementation
    )

    assert (
        "has_tag_collision_locked"
        in implementation
    )


def test_loc1b_registry_remains_dependency_isolated() -> None:
    registry_layer = "\n".join(
        (
            read("include/savo_locations/registry.hpp"),
            read("src/registry.cpp"),
        )
    )

    # The in-memory registry remains independent even
    # though later targets in the package use ROS/SQLite.
    for forbidden in (
        "rclcpp",
        "std_msgs",
        "geometry_msgs",
        "builtin_interfaces",
        "sqlite3",
        "sqlite_store",
        "sqlite_repository",
    ):
        assert forbidden not in registry_layer



