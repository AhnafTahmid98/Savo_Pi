from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]

SOURCE = (
    ROOT /
    "src/nodes/rotate_to_heading_node.cpp"
)

CMAKE = ROOT / "CMakeLists.txt"


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_action_headers_and_aliases_exist() -> None:
    source = read_text(SOURCE)

    assert (
        '#include "rclcpp_action/rclcpp_action.hpp"'
        in source
    )

    assert (
        '#include "savo_msgs/action/'
        'rotate_to_heading.hpp"'
        in source
    )

    assert (
        "using RotateToHeading = "
        "savo_msgs::action::RotateToHeading;"
        in source
    )

    assert (
        "GoalHandleRotateToHeading"
        in source
    )


def test_action_server_uses_canonical_endpoint() -> None:
    source = read_text(SOURCE)

    assert (
        "rclcpp_action::create_server"
        "<RotateToHeading>"
        in source
    )

    assert "action_name_" in source

    assert (
        "topics::ROTATE_TO_HEADING_ACTION"
        in source
    )


def test_goal_validation_is_deterministic() -> None:
    source = read_text(SOURCE)

    required = (
        "action_goal_reserved_",
        "action_goal_active()",
        "!enabled_",
        "std::isfinite(goal->target_yaw_rad)",
        "std::isfinite(goal->max_duration_sec)",
        "!odom_fresh(now_s)",
        "safety_stop_active(now_s)",
        "GoalResponse::REJECT",
        "GoalResponse::ACCEPT_AND_EXECUTE",
    )

    for token in required:
        assert token in source


def test_only_one_action_goal_can_own_rotation() -> None:
    source = read_text(SOURCE)

    assert (
        "action_goal_reserved_ || "
        "action_goal_active() || active_"
        in source
    )

    assert (
        "action_goal_handle_ = goal_handle;"
        in source
    )

    assert (
        "action_goal_handle_.reset();"
        in source
    )


def test_native_cancel_is_acknowledged() -> None:
    source = read_text(SOURCE)

    assert (
        "rclcpp_action::CancelResponse "
        "handle_action_cancel"
        in source
    )

    assert (
        "CancelResponse::ACCEPT"
        in source
    )

    assert (
        "CancelResponse::REJECT"
        in source
    )

    assert (
        "finish_action_canceled"
        in source
    )

    assert (
        'finish_action_canceled("canceled")'
        in source
    )


def test_action_result_terminal_mappings_exist() -> None:
    source = read_text(SOURCE)

    assert "goal_handle->succeed(result)" in source
    assert "goal_handle->abort(result)" in source
    assert "goal_handle->canceled(result)" in source

    assert (
        'finish_action_succeeded("goal_reached")'
        in source
    )

    for reason in (
        "disabled",
        "inactive",
        "safety_stop",
        "no_target",
        "odom_stale",
        "timeout",
    ):
        assert (
            f'finish_action_aborted("{reason}")'
            in source
        )


def test_result_fields_are_populated() -> None:
    source = read_text(SOURCE)

    required = (
        "result->success",
        "result->final_yaw_rad",
        "result->final_error_rad",
        "result->reason",
    )

    for token in required:
        assert token in source


def test_feedback_fields_are_populated() -> None:
    source = read_text(SOURCE)

    required = (
        "feedback->current_yaw_rad",
        "feedback->target_yaw_rad",
        "feedback->error_rad",
        "feedback->commanded_wz_rad_s",
        "feedback->elapsed_sec",
        "feedback->safety_stop_active",
        "feedback->state",
        "publish_feedback(feedback)",
    )

    for token in required:
        assert token in source


def test_goal_duration_override_uses_default_fallback() -> None:
    source = read_text(SOURCE)

    assert (
        "goal->max_duration_sec > 0.0"
        in source
    )

    assert (
        "active_duration_limit_s()"
        in source
    )

    assert (
        "(now_s - active_start_s_) > "
        "active_duration_limit_s()"
        in source
    )


def test_legacy_topics_cannot_replace_action_goal() -> None:
    source = read_text(SOURCE)

    assert (
        "Ignoring legacy rotate target while "
        "an action goal owns rotation"
        in source
    )

    assert '"legacy_disabled"' in source
    assert '"legacy_cancel"' in source


def test_rotation_stays_on_auto_authority_lane() -> None:
    source = read_text(SOURCE)

    assert "topics::CMD_VEL_AUTO" in source
    assert "topics::CMD_VEL_RECOVERY" not in source


def test_action_server_contract_test_is_registered() -> None:
    cmake = read_text(CMAKE)

    assert (
        "test_rotate_to_heading_action_server_contract"
        in cmake
    )

    assert (
        "test/"
        "test_rotate_to_heading_action_server_contract.py"
        in cmake
    )


def test_action_server_has_one_create_call() -> None:
    source = read_text(SOURCE)

    matches = re.findall(
        r"rclcpp_action::create_server"
        r"<RotateToHeading>",
        source,
    )

    assert len(matches) == 1
