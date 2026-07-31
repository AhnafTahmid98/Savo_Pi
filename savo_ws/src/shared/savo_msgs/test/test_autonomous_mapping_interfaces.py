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
    ):
        assert f'"{path}"' in cmake


def test_status_contract_is_typed_and_mapping_owned() -> None:
    text = read("msg/AutonomousMappingStatus.msg")

    for token in (
        "uint32 CONTRACT_VERSION=1",
        "uint8 STRATEGY_NONE=0",
        "uint8 STRATEGY_FRONTIER=1",
        "uint8 STATE_WAITING_FOR_AUTHORITY=2",
        "uint8 STATE_EXPLORING=3",
        "uint8 STATE_PAUSED=5",
        "uint8 STATE_CANCELING=7",
        "uint8 STATE_COMPLETED=10",
        "uint8 STATE_COMPLETION_PENDING=13",
        "uint8 RESULT_NOT_TERMINAL=255",
        "uint8 result_code",
        "string mission_id",
        "string map_id",
        "bool runtime_authorized",
        "bool handoff_active",
        "uint32 goals_succeeded",
        "uint32 goals_failed",
        "bool frontier_status_received",
        "string frontier_planning_status",
        "uint64 frontier_plan_sequence",
        "uint32 exhaustion_observations",
        "bool completion_candidate",
        "bool completion_confirmed",
        "string completion_reason",
        "bool map_save_started",
        "bool map_save_complete",
        "bool map_saved",
        "string saved_session_directory",
        "bool verification_started",
        "bool verification_complete",
        "bool map_verified",
        "string verification_reason",
    ):
        assert token in text

    assert "geometry_msgs/Pose" not in text
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
        "uint32 contract_version",
        "uint8 result_code",
        "string mission_id",
        "string actor_id",
        "string map_id",
        "uint8 strategy",
        "bool auto_save",
        "bool require_quality_approval",
        "builtin_interfaces/Duration mission_timeout",
        "RESULT_READINESS_LOST=3",
        "RESULT_NAVIGATION_FAILED=4",
        "RESULT_TIMED_OUT=5",
        "RESULT_SAVE_FAILED=7",
        "savo_msgs/AutonomousMappingStatus final_status",
        "savo_msgs/AutonomousMappingStatus status",
    ):
        assert token in text

    assert "geometry_msgs/PoseStamped" not in text
    assert "nav2_msgs" not in text


def test_control_service_has_only_pause_resume_cancel() -> None:
    text = read("srv/ControlAutonomousMapping.srv")

    for token in (
        "COMMAND_PAUSE=1",
        "COMMAND_RESUME=2",
        "COMMAND_CANCEL=3",
        "RESULT_NO_ACTIVE_MISSION=2",
        "RESULT_MISSION_MISMATCH=3",
        "RESULT_INVALID_STATE=4",
        "savo_msgs/AutonomousMappingStatus status",
    ):
        assert token in text

    assert "COMMAND_START" not in text


def test_manifest_version_marks_new_interface_revision() -> None:
    root = ET.parse(ROOT / "package.xml").getroot()
    assert root.findtext("version") == "0.8.0"
