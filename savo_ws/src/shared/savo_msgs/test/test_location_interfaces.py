from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_location_interfaces_registered() -> None:
    cmake = read("CMakeLists.txt")

    for path in (
        "msg/LocationRecord.msg",
        "msg/LocationCandidate.msg",
        "msg/LocationEvent.msg",
        "srv/ResolveLocation.srv",
        "srv/GetLocation.srv",
        "srv/GetLocationCandidate.srv",
        "srv/ListLocationCandidates.srv",
        "srv/ListLocations.srv",
        "srv/RegisterLocationCandidate.srv",
        "srv/ApproveLocation.srv",
        "srv/RejectLocationCandidate.srv",
        "srv/ReviewLocationCandidate.srv",
        "srv/SetLocationEnabled.srv",
        "srv/RecoverLocationStorage.srv",
    ):
        assert f'"{path}"' in cmake


def test_map_revision_is_canonical() -> None:
    for path in (
        "msg/LocationRecord.msg",
        "msg/LocationCandidate.msg",
        "srv/ResolveLocation.srv",
        "srv/ListLocations.srv",
        "action/ConfirmAprilTag.action",
    ):
        text = read(path)

        assert "map_revision" in text
        assert "map_version" not in text


def test_navigation_resolution_guards() -> None:
    text = read("srv/ResolveLocation.srv")

    assert "bool enforce_map_context" in text
    assert "RESULT_AMBIGUOUS=3" in text
    assert "RESULT_MAP_MISMATCH=6" in text
    assert "string[] ambiguous_location_ids" in text


def test_revision_guards() -> None:
    assert (
        "uint64 expected_candidate_revision"
        in read("srv/ApproveLocation.srv")
    )

    assert (
        "uint64 expected_candidate_revision"
        in read("srv/RejectLocationCandidate.srv")
    )

    assert (
        "uint64 expected_record_revision"
        in read("srv/SetLocationEnabled.srv")
    )


def test_storage_recovery_contract() -> None:
    text = read("srv/RecoverLocationStorage.srv")

    for token in (
        "string actor_id",
        "RESULT_RECOVERED=0",
        "RESULT_NOT_ENABLED=2",
        "RESULT_INTEGRITY_FAILED=4",
        "uint64 last_event_sequence",
    ):
        assert token in text


def test_location_review_gateway_contract() -> None:
    lookup = read("srv/GetLocationCandidate.srv")
    candidate_list = read("srv/ListLocationCandidates.srv")
    review = read("srv/ReviewLocationCandidate.srv")

    for token in (
        "string candidate_id",
        "RESULT_FOUND=0",
        "savo_msgs/LocationCandidate candidate",
    ):
        assert token in lookup

    for token in (
        "STATE_FILTER_PENDING=1",
        "bool enforce_map_context",
        "savo_msgs/LocationCandidate[] candidates",
        "RESULT_INVALID_FILTER=1",
    ):
        assert token in candidate_list

    for token in (
        "DECISION_APPROVE=1",
        "DECISION_REJECT=2",
        "string request_id",
        "uint64 expected_candidate_revision",
        "string rejection_reason",
        "RESULT_SUPERVISOR_DENIED=7",
        "RESULT_STALE_REVISION=6",
        "bool completed",
        "savo_msgs/LocationRecord location",
    ):
        assert token in review
