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


def test_package_contains_loc2c_or_later() -> None:
    package = ET.parse(
        ROOT / "package.xml"
    ).getroot()

    version = package.findtext("version")

    assert version is not None
    assert parse_version(version) >= (0, 7, 0)

    constants = read(
        "include/savo_locations/constants.hpp"
    )

    assert f'"{version}"' in constants


def test_schema_version_two_is_locked() -> None:
    schema = read(
        "include/savo_locations/sqlite_schema.hpp"
    )

    assert (
        "kSupportedSqliteSchemaVersion{2U}"
        in schema
    )

    assert "kMigration002Sql" in schema

    assert (
        "location_events_reject_update"
        in schema
    )

    assert (
        "location_events_reject_delete"
        in schema
    )

    assert (
        "location_events is append-only"
        in schema
    )


def test_repository_has_bootstrap_api() -> None:
    header = read(
        "include/savo_locations/sqlite_repository.hpp"
    )

    assert "struct BootstrapReport" in header
    assert "SnapshotResult bootstrap(" in header
    assert "integrity_healthy" in header
    assert "last_event_sequence" in header


def test_repository_has_event_journal_api() -> None:
    header = read(
        "include/savo_locations/sqlite_repository.hpp"
    )

    assert "enum class PersistenceEventType" in header
    assert "struct PersistenceEvent" in header
    assert "append_event(" in header
    assert "list_events(" in header


def test_approval_commit_is_atomic() -> None:
    implementation = read(
        "src/sqlite_repository.cpp"
    )

    assert "commit_candidate_approval(" in implementation
    assert '"BEGIN IMMEDIATE;"' in implementation
    assert '"COMMIT;"' in implementation
    assert '"ROLLBACK;"' in implementation

    assert (
        "approval event append failed;"
        in implementation
    )

    assert (
        "snapshot was rolled back"
        in implementation
    )


def test_approval_delta_is_strict() -> None:
    implementation = read(
        "src/sqlite_repository.cpp"
    )

    assert "validate_approval_delta(" in implementation
    assert "candidate revision is stale" in implementation

    assert (
        "approval changed an unrelated candidate"
        in implementation
    )

    assert (
        "approval changed an existing location"
        in implementation
    )

    assert (
        "approval must create exactly one location"
        in implementation
    )


def test_loc2c_tests_exist() -> None:
    assert (
        ROOT
        / "test"
        / "storage"
        / "test_persistent_catalog.cpp"
    ).is_file()

    cmake = read("CMakeLists.txt")

    assert "test_persistent_catalog" in cmake
    assert "test_phase2c_contracts" in cmake


def test_loc2c_persistence_remains_ros_independent() -> None:
    persistence_layer = "\n".join(
        (
            read(
                "include/savo_locations/"
                "sqlite_schema.hpp"
            ),
            read(
                "include/savo_locations/"
                "sqlite_store.hpp"
            ),
            read(
                "include/savo_locations/"
                "sqlite_repository.hpp"
            ),
            read("src/sqlite_store.cpp"),
            read("src/sqlite_repository.cpp"),
        )
    )

    # Bootstrap, transactions and the append-only event
    # journal remain ROS-independent storage behavior.
    for forbidden in (
        "rclcpp",
        "std_msgs",
        "geometry_msgs",
        "builtin_interfaces",
        "savo_msgs",
    ):
        assert forbidden not in persistence_layer

