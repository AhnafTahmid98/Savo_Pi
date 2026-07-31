from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_typed_location_integration_interfaces_are_registered() -> None:
    cmake = read("CMakeLists.txt")

    for path in (
        "action/RegisterMappedLocation.action",
        "action/NavigateToLocation.action",
        "srv/AuthorizeLocationOperation.srv",
        "srv/RejectLocationCandidate.srv",
        "srv/GetLocationCandidate.srv",
        "srv/ListLocationCandidates.srv",
        "srv/ReviewLocationCandidate.srv",
    ):
        assert f'"{path}"' in cmake


def test_mapping_registration_ownership_contract() -> None:
    text = read("action/RegisterMappedLocation.action")

    for token in (
        "string candidate_id",
        "string expected_family",
        "int32 expected_tag_id",
        "string map_id",
        "uint32 map_revision",
        "geometry_msgs/PoseStamped approach_pose",
        "savo_msgs/LocationCandidate candidate",
        "RESULT_SUPERVISOR_DENIED=2",
        "RESULT_REGISTRATION_REJECTED=6",
    ):
        assert token in text

    assert "LocationRecord" not in text


def test_navigation_uses_semantic_record_and_arrival_confirmation() -> None:
    text = read("action/NavigateToLocation.action")

    for token in (
        "string query",
        "bool enforce_map_context",
        "savo_msgs/LocationRecord location",
        "bool navigation_succeeded",
        "bool arrival_confirmed",
        "savo_msgs/AprilTagObservation final_observation",
        "STATE_NAVIGATING_TO_APPROACH_POSE=2",
        "STATE_CONFIRMING_ARRIVAL=3",
    ):
        assert token in text

    assert "geometry_msgs/PoseStamped tag_pose_map" not in text


def test_supervisor_authorization_is_permission_only() -> None:
    text = read("srv/AuthorizeLocationOperation.srv")

    for token in (
        "OP_REGISTER_LOCATION_CANDIDATE=1",
        "OP_APPROVE_LOCATION=2",
        "OP_NAVIGATE_TO_LOCATION=3",
        "OP_CONFIRM_LOCATION_ARRIVAL=4",
        "OP_REJECT_LOCATION_CANDIDATE=5",
        "bool motion_required",
        "bool authorized",
        "RESULT_SAFETY_BLOCKED=4",
        "builtin_interfaces/Time evaluated_at",
    ):
        assert token in text
