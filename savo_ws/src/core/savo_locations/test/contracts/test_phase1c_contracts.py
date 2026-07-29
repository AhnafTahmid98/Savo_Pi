from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(
        encoding="utf-8"
    )


def parse_version(
    value: str,
) -> tuple[int, int, int]:
    major, minor, patch = value.split(".")

    return (
        int(major),
        int(minor),
        int(patch),
    )


def test_package_contains_loc1c_or_later() -> None:
    package = ET.parse(
        ROOT / "package.xml"
    ).getroot()

    version = package.findtext("version")

    assert version is not None
    assert parse_version(version) >= (0, 4, 0)

    constants = read(
        "include/savo_locations/constants.hpp"
    )

    assert f'"{version}"' in constants


def test_candidate_catalog_files_exist() -> None:
    for relative in (
        "include/savo_locations/location_catalog.hpp",
        "src/location_catalog.cpp",
        "test/unit/test_location_catalog.cpp",
    ):
        assert (ROOT / relative).is_file()


def test_cmake_builds_location_catalog() -> None:
    cmake = read("CMakeLists.txt")

    assert "src/location_catalog.cpp" in cmake
    assert "test_location_catalog" in cmake
    assert "test_phase1c_contracts" in cmake


def test_candidate_lifecycle_is_explicit() -> None:
    header = read(
        "include/savo_locations/location_catalog.hpp"
    )

    for required in (
        "kRegistered",
        "kUpdated",
        "kRejected",
        "kNotPending",
        "kStaleRevision",
        "kTagConflict",
        "register_candidate",
        "replace_candidate",
        "reject_candidate",
    ):
        assert required in header


def test_approval_transaction_is_explicit() -> None:
    header = read(
        "include/savo_locations/location_catalog.hpp"
    )

    implementation = read(
        "src/location_catalog.cpp"
    )

    for required in (
        "kApproved",
        "kCandidateNotFound",
        "kCandidateNotPending",
        "kMissingApproachPose",
        "kLocationConflict",
        "approve_candidate",
    ):
        assert required in header

    assert (
        "location_record.source_candidate_id"
        in implementation
    )

    assert (
        "existing->second ="
        in implementation
    )

    assert (
        "locations_.insert(location_record)"
        in implementation
    )


def test_candidate_owns_mapping_evidence() -> None:
    model = read(
        "include/savo_locations/model.hpp"
    )

    implementation = read(
        "src/location_catalog.cpp"
    )

    assert "MapContext map;" in model
    assert "TagBinding tag;" in model
    assert "PoseData tag_pose_map;" in model

    assert "location.map = source.map" in implementation
    assert "location.tag = source.tag" in implementation
    assert (
        "location.tag_pose_map ="
        in implementation
    )


def test_loc1c_catalog_remains_dependency_isolated() -> None:
    cmake = read("CMakeLists.txt")

    catalog_layer = "\n".join(
        (
            read(
                "include/savo_locations/"
                "location_catalog.hpp"
            ),
            read("src/location_catalog.cpp"),
        )
    )

    # SQLite is allowed only in the dedicated LOC-2
    # storage layer. The candidate and approval catalog
    # must remain a pure in-memory domain component.
    assert "find_package(rclcpp" not in cmake
    assert "rclcpp::rclcpp" not in cmake
    assert "add_executable(" not in cmake

    assert "#include <sqlite3.h>" not in catalog_layer
    assert "sqlite3_" not in catalog_layer

    assert not (ROOT / "launch").exists()

    assert not (
        ROOT
        / "src"
        / "location_registry_node.cpp"
    ).exists()


