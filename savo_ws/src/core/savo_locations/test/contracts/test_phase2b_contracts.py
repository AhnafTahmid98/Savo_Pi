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


def test_package_contains_loc2b_or_later() -> None:
    package = ET.parse(
        ROOT / "package.xml"
    ).getroot()

    version = package.findtext("version")

    assert version is not None
    assert parse_version(version) >= (0, 6, 0)

    constants = read(
        "include/savo_locations/constants.hpp"
    )

    assert f'"{version}"' in constants


def test_repository_files_exist() -> None:
    for relative in (
        "include/savo_locations/sqlite_repository.hpp",
        "src/sqlite_repository.cpp",
        "test/storage/test_sqlite_repository.cpp",
    ):
        assert (ROOT / relative).is_file()


def test_cmake_builds_repository() -> None:
    cmake = read("CMakeLists.txt")

    assert "src/sqlite_repository.cpp" in cmake
    assert "test_sqlite_repository" in cmake
    assert "test_phase2b_contracts" in cmake

    assert (
        'SAVO_LOCATIONS_TEST_DB_DIR=\\"'
        in cmake
    )


def test_repository_has_typed_snapshot_api() -> None:
    header = read(
        "include/savo_locations/sqlite_repository.hpp"
    )

    assert "struct CatalogSnapshot" in header
    assert "std::vector<LocationRecordData>" in header
    assert "std::vector<CandidateRecordData>" in header

    assert "save_snapshot" in header
    assert "load_snapshot" in header


def test_snapshot_write_is_transactional() -> None:
    implementation = read(
        "src/sqlite_repository.cpp"
    )

    assert '"BEGIN IMMEDIATE;"' in implementation
    assert '"COMMIT;"' in implementation
    assert '"ROLLBACK;"' in implementation

    assert (
        "DELETE FROM location_candidates"
        in implementation
    )

    assert (
        "DELETE FROM locations"
        in implementation
    )


def test_repository_persists_location_identity_rows() -> None:
    implementation = read(
        "src/sqlite_repository.cpp"
    )

    assert "insert_location_identity" in implementation
    assert "alias_kind" in implementation
    assert "normalize_lookup_key(alias_text)" in implementation
    assert "reserves_identity" in implementation


def test_repository_persists_candidates() -> None:
    implementation = read(
        "src/sqlite_repository.cpp"
    )

    assert "INSERT INTO location_candidates" in implementation
    assert "INSERT INTO candidate_aliases" in implementation
    assert "approved_location_id" in implementation
    assert "review_reason" in implementation


def test_repository_validates_before_and_after_io() -> None:
    implementation = read(
        "src/sqlite_repository.cpp"
    )

    assert implementation.count(
        "validate_snapshot("
    ) >= 3

    assert "kCorruptData" in implementation
    assert "persisted catalog failed domain validation" in implementation


def test_loc2b_still_has_no_ros_runtime_node() -> None:
    cmake = read("CMakeLists.txt")

    assert "find_package(rclcpp" not in cmake
    assert "add_executable(" not in cmake

    assert not (
        ROOT
        / "src"
        / "location_registry_node.cpp"
    ).exists()

    assert not (ROOT / "launch").exists()
