"""Validate the stable ExecuteCoveragePath action schema."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
ACTION = PACKAGE / "action/ExecuteCoveragePath.action"


def action_sections() -> list[list[str]]:
    """Return non-comment interface lines grouped by action section."""
    sections: list[list[str]] = [[]]

    for raw_line in ACTION.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()

        if line == "---":
            sections.append([])
        elif line and not line.startswith("#"):
            sections[-1].append(line)

    return sections


def test_execute_coverage_path_action_exists() -> None:
    """Require the shared Coverage execution action."""
    assert ACTION.is_file()
    assert ACTION.stat().st_size > 0


def test_execute_coverage_path_has_three_sections() -> None:
    """Require goal, result and feedback sections."""
    assert len(action_sections()) == 3


def test_goal_schema_is_stable() -> None:
    """Lock the public request contract."""
    goal, _, _ = action_sections()

    assert goal == [
        "uint32 CONTRACT_VERSION=1",
        "uint32 contract_version",
        "string mission_id",
        "nav_msgs/Path path",
        "builtin_interfaces/Duration execution_timeout",
    ]


def test_result_schema_is_stable() -> None:
    """Lock terminal states and final progress."""
    _, result, _ = action_sections()

    assert result == [
        "uint8 RESULT_SUCCEEDED=0",
        "uint8 RESULT_INVALID_REQUEST=1",
        "uint8 RESULT_NOT_READY=2",
        "uint8 RESULT_BUSY=3",
        "uint8 RESULT_BACKEND_UNAVAILABLE=4",
        "uint8 RESULT_BACKEND_REJECTED=5",
        "uint8 RESULT_ABORTED=6",
        "uint8 RESULT_CANCELED=7",
        "uint8 RESULT_TIMED_OUT=8",
        "uint8 RESULT_FEEDBACK_STALE=9",
        "uint8 RESULT_INTERNAL_ERROR=10",
        "bool success",
        "uint8 result_code",
        "string terminal_state",
        "string reason",
        "uint32 completed_waypoints",
        "uint32 total_waypoints",
        "float64 completion_ratio",
        "float64 remaining_distance_m",
    ]


def test_feedback_schema_is_stable() -> None:
    """Lock observable execution progress."""
    _, _, feedback = action_sections()

    assert feedback == [
        "uint8 STATE_WAITING_FOR_ADMISSION=0",
        "uint8 STATE_WAITING_FOR_BACKEND=1",
        "uint8 STATE_EXECUTING=2",
        "uint8 STATE_CANCELING=3",
        "uint32 NO_WAYPOINT=4294967295",
        "uint8 state",
        "string state_text",
        "string reason",
        "uint32 current_waypoint",
        "uint32 completed_waypoints",
        "uint32 total_waypoints",
        "float64 completion_ratio",
        "float64 remaining_distance_m",
        "builtin_interfaces/Duration elapsed_time",
        "builtin_interfaces/Duration estimated_time_remaining",
    ]


def test_action_does_not_expose_backend_or_velocity_authority() -> None:
    """Keep backend selection and physical motion outside the interface."""
    text = ACTION.read_text(encoding="utf-8")
    schema = "\n".join(
        line
        for section in action_sections()
        for line in section
    )

    forbidden_schema_tokens = (
        "Twist",
        "cmd_vel",
        "controller_id",
        "goal_checker_id",
        "progress_checker_id",
        "NavigateToPose",
        "FollowPath",
    )

    for token in forbidden_schema_tokens:
        assert token not in schema

    assert "savo_mapping is the production action client" in text
    assert "savo_nav is the production action server" in text
    assert "savo_control remains the final movement authority" in text
    assert "must never" in text
