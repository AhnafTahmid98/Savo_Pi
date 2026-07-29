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


def test_package_contains_loc2a_or_later() -> None:
    package = ET.parse(
        ROOT / "package.xml"
    ).getroot()

    version = package.findtext("version")

    assert version is not None
    assert parse_version(version) >= (0, 5, 0)

    dependencies = {
        (element.text or "").strip()
        for element in package.findall("depend")
    }

    assert "sqlite3" in dependencies

    constants = read(
        "include/savo_locations/constants.hpp"
    )

    assert f'"{version}"' in constants


def test_sqlite_foundation_files_exist() -> None:
    for relative in (
        "include/savo_locations/sqlite_schema.hpp",
        "include/savo_locations/sqlite_store.hpp",
        "src/sqlite_store.cpp",
        "config/storage.yaml",
        "test/storage/test_sqlite_store.cpp",
    ):
        assert (ROOT / relative).is_file()


def test_cmake_builds_storage_library() -> None:
    cmake = read("CMakeLists.txt")

    assert "find_package(SQLite3 REQUIRED)" in cmake
    assert "savo_locations_storage" in cmake
    assert "src/sqlite_store.cpp" in cmake
    assert "SQLite::SQLite3" in cmake
    assert "test_sqlite_store" in cmake
    assert "test_phase2a_contracts" in cmake


def test_schema_contains_required_tables() -> None:
    schema = read(
        "include/savo_locations/sqlite_schema.hpp"
    )

    for table in (
        "schema_migrations",
        "registry_metadata",
        "locations",
        "location_aliases",
        "location_candidates",
        "candidate_aliases",
        "location_events",
    ):
        assert f"CREATE TABLE IF NOT EXISTS {table}" in schema


def test_schema_has_partial_uniqueness() -> None:
    schema = read(
        "include/savo_locations/sqlite_schema.hpp"
    )

    assert (
        "idx_locations_active_tag_unique"
        in schema
    )

    assert "WHERE state != 2" in schema

    assert (
        "idx_location_aliases_active_identity_unique"
        in schema
    )

    assert "WHERE reserves_identity = 1" in schema

    assert (
        "idx_candidates_pending_tag_unique"
        in schema
    )

    assert "WHERE state = 1" in schema


def test_store_enforces_safe_sqlite_policy() -> None:
    implementation = read(
        "src/sqlite_store.cpp"
    )

    for required in (
        "PRAGMA foreign_keys=ON",
        "PRAGMA synchronous=NORMAL",
        "PRAGMA journal_mode=WAL",
        "sqlite3_busy_timeout",
        "SQLITE_OPEN_FULLMUTEX",
        "BEGIN IMMEDIATE",
        "PRAGMA integrity_check",
        "PRAGMA foreign_key_check",
    ):
        assert required in implementation


def test_store_has_fail_closed_schema_handling() -> None:
    header = read(
        "include/savo_locations/sqlite_store.hpp"
    )

    implementation = read(
        "src/sqlite_store.cpp"
    )

    assert "kSchemaTooNew" in header

    assert (
        "database schema is newer than this package"
        in implementation
    )

    assert (
        "kSupportedSqliteSchemaVersion"
        in implementation
    )


def test_loc2a_has_no_ros_runtime_node() -> None:
    cmake = read("CMakeLists.txt")

    assert "find_package(rclcpp" not in cmake
    assert "add_executable(" not in cmake

    assert not (
        ROOT
        / "src"
        / "location_registry_node.cpp"
    ).exists()

    assert not (ROOT / "launch").exists()
