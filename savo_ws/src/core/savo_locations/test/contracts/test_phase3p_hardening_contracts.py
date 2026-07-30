from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def parse_version(value: str) -> tuple[int, ...]:
    return tuple(int(part) for part in value.split("."))


def test_package_contains_loc3p_hardening_or_later() -> None:
    package = ET.parse(ROOT / "package.xml").getroot()
    version = package.findtext("version")
    assert version is not None
    assert parse_version(version) >= (0, 11, 0)
    assert f'"{version}"' in read(
        "include/savo_locations/constants.hpp"
    )


def test_verified_storage_recovery_is_wired() -> None:
    header = read(
        "include/savo_locations/location_registry_node.hpp"
    )
    source = read("src/location_registry_node.cpp")
    names = read(
        "include/savo_locations/service_names.hpp"
    )

    for token in (
        "RecoverLocationStorage",
        "handle_recover_storage",
        "recovery_service_",
    ):
        assert token in header or token in source

    assert "kRecoverStorage" in names
    assert "/savo_locations/storage/recover" in names
    assert "integrity_check" in source
    assert "bootstrap" in source
    assert "hydrate_catalog" in source
    assert "persistent read/write registry recovered" in source


def test_failed_writes_remain_fail_closed() -> None:
    source = read("src/location_registry_node.cpp")

    assert 'state_ = "degraded_write"' in source
    assert "write_ready_ = false" in source
    degraded_body = source[
        source.index("finish_mutation_degraded"):
        source.index("finish_mutation_committed")
    ]
    assert "\n  ready_ = false;" not in degraded_body


def test_runtime_hardening_tests_are_registered() -> None:
    cmake = read("CMakeLists.txt")
    test_source = read(
        "test/ros/test_registry_hardening_node.cpp"
    )

    assert "test_registry_hardening_node" in cmake
    assert "test_phase3p_hardening_contracts" in cmake

    for token in (
        "ConcurrentApprovalsCommitExactlyOnce",
        "VerifiedRecoveryRestoresWritesAfterRollback",
        "force_location_event_failure",
        "RESULT_STORAGE_UNAVAILABLE",
        "last_event_sequence",
    ):
        assert token in test_source
