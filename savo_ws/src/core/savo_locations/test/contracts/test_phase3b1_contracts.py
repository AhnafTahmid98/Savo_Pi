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
    return tuple(
        int(part)
        for part in value.split(".")
    )


def test_package_is_loc3b1_or_later() -> None:
    package = ET.parse(
        ROOT / "package.xml"
    ).getroot()

    version = package.findtext("version")

    assert version is not None
    assert parse_version(version) >= (0, 9, 0)


def test_atomic_commit_contracts_exist() -> None:
    header = read(
        "include/savo_locations/"
        "sqlite_repository.hpp"
    )

    for fragment in (
        "CandidateRegistrationCommit",
        "LocationEnabledCommit",
        "commit_candidate_registration",
        "commit_candidate_approval",
        "commit_location_enabled",
        "kCandidateRegistrationDeltaInvalid",
        "kLocationEnabledDeltaInvalid",
    ):
        assert fragment in header


def test_registration_commit_is_transactional() -> None:
    source = read(
        "src/sqlite_repository.cpp"
    )

    for fragment in (
        "validate_registration_delta",
        "BEGIN IMMEDIATE;",
        "kCandidateRegistered",
        "candidate registration event append failed",
        "candidate registration persisted atomically",
    ):
        assert fragment in source


def test_enablement_commit_is_transactional() -> None:
    source = read(
        "src/sqlite_repository.cpp"
    )

    for fragment in (
        "validate_location_enabled_delta",
        "location already has the requested",
        "kLocationEnabledChanged",
        "location enablement event append failed",
        "location enablement persisted atomically",
    ):
        assert fragment in source


def test_approval_atomic_commit_is_preserved() -> None:
    source = read(
        "src/sqlite_repository.cpp"
    )

    for fragment in (
        "validate_approval_delta",
        "kCandidateApproved",
        "approval event append failed",
        "candidate approval persisted atomically",
    ):
        assert fragment in source


def test_loc3b1_tests_are_registered() -> None:
    cmake = read("CMakeLists.txt")

    for target in (
        "test_persistent_mutation_commits",
        "test_phase3b1_contracts",
    ):
        assert target in cmake


def test_loc3b1_atomic_foundation_remains_separate() -> None:
    source = read(
        "src/location_registry_node.cpp"
    )

    header = read(
        "include/savo_locations/"
        "location_registry_node.hpp"
    )

    combined = source + header

    package = ET.parse(ROOT / "package.xml").getroot()
    version = package.findtext("version")

    assert version is not None

    # LOC-3B1 itself did not expose writes. LOC-3B2 and later may
    # wire them, but must continue to use the atomic LOC-3B1 APIs.
    if parse_version(version) < (0, 10, 0):
        for forbidden in (
            "RegisterLocationCandidate",
            "ApproveLocation",
            "SetLocationEnabled",
            "handle_register",
            "handle_approve",
            "handle_set_enabled",
        ):
            assert forbidden not in combined
    else:
        for commit in (
            "commit_candidate_registration",
            "commit_candidate_approval",
            "commit_location_enabled",
        ):
            assert commit in source
