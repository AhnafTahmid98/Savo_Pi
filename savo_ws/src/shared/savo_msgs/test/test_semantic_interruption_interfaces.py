from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding='utf-8')


def test_semantic_interruption_interfaces_are_registered() -> None:
    cmake = read('CMakeLists.txt')

    assert '"msg/SemanticInterruptionStatus.msg"' in cmake
    assert '"srv/SubmitSemanticLocation.srv"' in cmake


def test_status_covers_the_complete_typed_lifecycle() -> None:
    status = read('msg/SemanticInterruptionStatus.msg')

    for token in (
        'uint32 CONTRACT_VERSION=1',
        'STATE_IDLE=0',
        'STATE_TAG_DETECTED=1',
        'STATE_PAUSING=2',
        'STATE_WAITING_FOR_MISSION_PAUSED=3',
        'STATE_WAITING_FOR_SEMANTICS=4',
        'STATE_REGISTERING=5',
        'STATE_RESUMING=6',
        'STATE_COMPLETED=7',
        'STATE_FAILED=8',
        'std_msgs/Header header',
        'savo_msgs/AprilTagObservation detector_observation',
        'savo_msgs/LocationCandidate location_candidate',
        'uint64 duplicate_observations_suppressed',
        'bool semantic_submission_received',
        'bool registration_started',
        'bool registration_complete',
        'bool resume_requested',
        'bool resume_complete',
    ):
        assert token in status


def test_submission_is_typed_and_contains_operator_semantics() -> None:
    service = read('srv/SubmitSemanticLocation.srv')

    for token in (
        'uint32 CONTRACT_VERSION=1',
        'string mission_id',
        'string actor_id',
        'string tag_family',
        'int32 tag_id',
        'string suggested_location_id',
        'string suggested_display_name',
        'string[] suggested_aliases',
        'string suggested_semantic_type',
        'bool approach_pose_valid',
        'geometry_msgs/PoseStamped approach_pose',
        'bool confirmation_pose_valid',
        'geometry_msgs/PoseStamped confirmation_pose',
        'builtin_interfaces/Duration timeout',
        'RESULT_ACCEPTED=0',
        'RESULT_NO_ACTIVE_INTERRUPTION=1',
        'RESULT_TAG_MISMATCH=2',
        'RESULT_MISSION_MISMATCH=3',
        'RESULT_INVALID_REQUEST=4',
        'RESULT_BUSY=5',
        'RESULT_REJECTED=6',
        'string candidate_id',
    ):
        assert token in service


def test_existing_coverage_state_is_reused() -> None:
    status = read('msg/AutonomousMappingStatus.msg')

    assert 'uint8 STATE_COVERAGE=19' in status
    assert 'STATE_COVERAGE_EXECUTING' not in status
