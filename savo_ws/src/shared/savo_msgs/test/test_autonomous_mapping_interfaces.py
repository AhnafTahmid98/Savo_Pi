from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_interfaces_are_registered() -> None:
    cmake = read("CMakeLists.txt")

    for path in (
        "msg/AutonomousMappingStatus.msg",
        "msg/FrontierExplorationStatus.msg",
        "action/RunAutonomousMapping.action",
        "srv/ControlAutonomousMapping.srv",
        "srv/ReviewAutonomousMappingRelease.srv",
        "srv/PrepareLocationRelease.srv",
        "srv/VerifyLocationRelease.srv",
        "srv/CommitLocationRelease.srv",
        "srv/RollbackLocationRelease.srv",
    ):
        assert f'"{path}"' in cmake


def test_status_contract_is_typed_and_mapping_owned() -> None:
    text = read("msg/AutonomousMappingStatus.msg")

    for token in (
        "uint32 CONTRACT_VERSION=4",
        "uint8 STRATEGY_NONE=0",
        "uint8 STRATEGY_FRONTIER=1",
        "uint8 STATE_WAITING_FOR_AUTHORITY=2",
        "uint8 STATE_EXPLORING=3",
        "uint8 STATE_PAUSED=5",
        "uint8 STATE_CANCELING=7",
        "uint8 STATE_COMPLETED=10",
        "uint8 STATE_COMPLETION_PENDING=13",
        "uint8 STATE_CAPTURING_START_POSE=14",
        "uint8 STATE_INITIAL_SCAN360=15",
        "uint8 STATE_INITIAL_HEAD_SCAN=16",
        "uint8 STATE_CONDITIONAL_SCAN360=17",
        "uint8 STATE_COVERAGE_PENDING=18",
        "uint8 STATE_RELEASING=25",
        "uint8 RESULT_SCAN_FAILED=10",
        "uint8 RESULT_START_POSE_UNAVAILABLE=11",
        "uint8 RESULT_LOCATION_VERIFICATION_FAILED=12",
        "uint8 RESULT_OPERATOR_REJECTED=13",
        "uint8 RESULT_RELEASE_FAILED=14",
        "uint8 RESULT_RELEASE_ROLLBACK_FAILED=15",
        "uint8 RESULT_GEOMETRY_INVALID=16",
        "uint8 RESULT_NOT_TERMINAL=255",
        "uint8 result_code",
        "string mission_id",
        "string map_id",
        "bool runtime_authorized",
        "bool handoff_active",
        "uint32 goals_succeeded",
        "uint32 goals_failed",
        "bool start_pose_capture_started",
        "bool start_pose_capture_complete",
        "bool start_pose_valid",
        "geometry_msgs/PoseStamped start_pose_map",
        "uint64 start_map_generation",
        "bool initial_scan360_complete",
        "bool initial_head_scan_complete",
        "uint32 conditional_scan360_completed",
        "string scan360_stage",
        "string head_scan_stage",
        "bool frontier_status_received",
        "string frontier_planning_status",
        "uint64 frontier_plan_sequence",
        "uint32 exhaustion_observations",
        "bool completion_candidate",
        "bool completion_confirmed",
        "string completion_reason",
        "bool coverage_planning_started",
        "uint64 coverage_plan_generation",
        "bool coverage_execution_active",
        "string coverage_mission_id",
        "float64 coverage_completion_ratio",
        "bool return_to_start_started",
        "bool return_to_start_succeeded",
        "float64 return_to_start_distance_m",
        "bool final_scan360_complete",
        "bool final_head_scan_complete",
        "bool map_save_started",
        "bool map_save_complete",
        "bool map_saved",
        "string saved_session_directory",
        "bool verification_started",
        "bool verification_complete",
        "bool map_verified",
        "string verification_reason",
        "bool location_verification_started",
        "bool location_verification_complete",
        "bool location_verification_passed",
        "uint64 review_generation",
        "bool approval_recorded",
        "bool release_succeeded",
        "string release_id",
        "bool rollback_required",
        "bool joint_active_release_verified",
    ):
        assert token in text

    assert "geometry_msgs/Twist" not in text
    assert "nav_msgs/Path" not in text


def test_frontier_status_reports_planner_evidence_without_json() -> None:
    text = read("msg/FrontierExplorationStatus.msg")

    for token in (
        "uint32 CONTRACT_VERSION=1",
        "EXHAUSTION_NO_FRONTIERS=1",
        "EXHAUSTION_NO_REACHABLE_FRONTIERS=2",
        "uint64 map_generation",
        "uint64 planned_map_generation",
        "uint64 plan_sequence",
        "string planning_status",
        "bool goal_pending",
        "uint32 detected_frontiers",
        "uint32 reachable_frontiers",
    ):
        assert token in text

    assert "nav_msgs/OccupancyGrid" not in text
    assert "geometry_msgs/PoseStamped" not in text


def test_action_starts_one_mission_and_returns_typed_status() -> None:
    text = read("action/RunAutonomousMapping.action")

    for token in (
        "uint32 CONTRACT_VERSION=3",
        "uint32 contract_version",
        "uint8 result_code",
        "string mission_id",
        "string actor_id",
        "string map_id",
        "uint8 strategy",
        "string authority_request_id",
        "uint64 authority_generation",
        "bool require_semantic",
        "bool auto_save",
        "bool require_quality_approval",
        "builtin_interfaces/Duration mission_timeout",
        "RESULT_READINESS_LOST=3",
        "RESULT_NAVIGATION_FAILED=4",
        "RESULT_TIMED_OUT=5",
        "RESULT_SAVE_FAILED=7",
        "RESULT_LOCATION_VERIFICATION_FAILED=12",
        "RESULT_OPERATOR_REJECTED=13",
        "RESULT_RELEASE_FAILED=14",
        "RESULT_RELEASE_ROLLBACK_FAILED=15",
        "RESULT_GEOMETRY_INVALID=16",
        "string map_release_id",
        "savo_msgs/AutonomousMappingStatus final_status",
        "savo_msgs/AutonomousMappingStatus status",
    ):
        assert token in text

    assert "geometry_msgs/PoseStamped" not in text
    assert "nav2_msgs" not in text


def test_control_service_adds_guarded_conditional_scan_request() -> None:
    text = read("srv/ControlAutonomousMapping.srv")

    for token in (
        "uint32 CONTRACT_VERSION=2",
        "COMMAND_PAUSE=1",
        "COMMAND_RESUME=2",
        "COMMAND_CANCEL=3",
        "COMMAND_REQUEST_SCAN360=4",
        "RESULT_NO_ACTIVE_MISSION=2",
        "RESULT_MISSION_MISMATCH=3",
        "RESULT_INVALID_STATE=4",
        "savo_msgs/AutonomousMappingStatus status",
    ):
        assert token in text

    assert "COMMAND_START" not in text


def test_am8_review_and_location_release_services_are_correlated() -> None:
    review = read("srv/ReviewAutonomousMappingRelease.srv")
    for token in (
        "uint32 CONTRACT_VERSION=1",
        "uint8 DECISION_APPROVE=1",
        "uint8 DECISION_REJECT=2",
        "string request_id",
        "string mission_id",
        "string map_id",
        "uint32 map_revision",
        "uint64 expected_review_generation",
        "string actor_id",
        "string requested_release_id",
        "uint8 RESULT_STALE_GENERATION=2",
        "uint8 RESULT_DUPLICATE_DECISION=4",
    ):
        assert token in review

    for path in (
        "srv/PrepareLocationRelease.srv",
        "srv/VerifyLocationRelease.srv",
        "srv/CommitLocationRelease.srv",
        "srv/RollbackLocationRelease.srv",
    ):
        text = read(path)
        assert "uint32 CONTRACT_VERSION=1" in text
        assert "string request_id" in text
        assert "string release_id" in text
        assert "string mission_id" in text
    for path in (
        "srv/VerifyLocationRelease.srv",
        "srv/CommitLocationRelease.srv",
        "srv/RollbackLocationRelease.srv",
    ):
        text = read(path)
        assert "string transaction_token" in text
        assert "string expected_snapshot_sha256" in text


def test_manifest_version_marks_new_interface_revision() -> None:
    root = ET.parse(ROOT / "package.xml").getroot()
    assert root.findtext("version") == "0.9.0"
