from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def parse_version(value: str) -> tuple[int, ...]:
    return tuple(int(part) for part in value.split("."))


def test_package_contains_loc3b2_or_later() -> None:
    package = ET.parse(ROOT / "package.xml").getroot()
    version = package.findtext("version")

    assert version is not None
    assert parse_version(version) >= (0, 10, 0)

    constants = read(
        "include/savo_locations/constants.hpp"
    )

    assert f'"{version}"' in constants


def test_write_service_types_are_wired() -> None:
    header = read(
        "include/savo_locations/location_registry_node.hpp"
    )
    source = read("src/location_registry_node.cpp")

    for token in (
        "RegisterLocationCandidate",
        "ApproveLocation",
        "GetLocationCandidate",
        "ListLocationCandidates",
        "RejectLocationCandidate",
        "SetLocationEnabled",
        "handle_register_candidate",
        "handle_approve_candidate",
        "handle_get_candidate",
        "handle_list_candidates",
        "handle_reject_candidate",
        "handle_set_enabled",
    ):
        assert token in header or token in source

    for service_name in (
        "kRegisterCandidate",
        "kApproveCandidate",
        "kGetCandidate",
        "kListCandidates",
        "kRejectCandidate",
        "kSetEnabled",
    ):
        assert service_name in source

    assert source.count("create_service<") >= 10


def test_event_topic_is_typed_and_post_commit() -> None:
    header = read(
        "include/savo_locations/location_registry_node.hpp"
    )
    source = read("src/location_registry_node.cpp")

    assert "savo_msgs::msg::LocationEvent" in header
    assert "kEvents" in source
    assert "publish_committed_event" in source
    assert ".reliable()" in source

    for commit, event in (
        (
            "commit_candidate_registration",
            "EVENT_CANDIDATE_REGISTERED",
        ),
        (
            "commit_candidate_approval",
            "EVENT_LOCATION_APPROVED",
        ),
        (
            "commit_candidate_rejection",
            "EVENT_CANDIDATE_REJECTED",
        ),
        (
            "commit_location_enabled",
            "EVENT_LOCATION_ENABLED",
        ),
    ):
        assert source.index(commit) < source.index(
            event,
            source.index(commit),
        )


def test_write_readiness_is_fail_closed() -> None:
    header = read(
        "include/savo_locations/location_registry_node.hpp"
    )
    source = read("src/location_registry_node.cpp")
    config = read("config/locations_node.yaml")

    for token in (
        "write_ready_",
        "mutation_in_progress_",
        "mutation_mutex_",
        "registry_write_ready",
        "finish_mutation_degraded",
    ):
        assert token in header or token in source

    assert "enable_write_services: true" in config
    assert "write_ready" in source
    assert "read_ready" in source
    assert 'state_ = "degraded_write"' in source


def test_mutations_use_atomic_repository_only() -> None:
    source = read("src/location_registry_node.cpp")

    assert "commit_candidate_registration" in source
    assert "commit_candidate_approval" in source
    assert "commit_location_enabled" in source

    assert "save_snapshot(" not in source
    assert "append_event(" not in source


def test_ros_conversions_cover_candidates_and_approval() -> None:
    header = read(
        "include/savo_locations/ros_conversions.hpp"
    )
    source = read("src/ros_conversions.cpp")

    for token in (
        "from_ros_candidate",
        "from_ros_approval_request",
        "to_ros_candidate_record",
    ):
        assert token in header
        assert token in source


def test_write_runtime_tests_are_registered() -> None:
    cmake = read("CMakeLists.txt")
    test_source = read(
        "test/ros/test_registry_write_node.cpp"
    )

    assert "test_registry_write_node" in cmake
    assert "test_phase3b2_contracts" in cmake

    for token in (
        "CommitsWritesPublishesEventsAndRestarts",
        "EventFailureRollsBackAndDisablesWrites",
        "RESULT_DUPLICATE_CANDIDATE_ID",
        "RESULT_STORAGE_UNAVAILABLE",
        "EVENT_LOCATION_DISABLED",
    ):
        assert token in test_source
