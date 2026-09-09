"""Static contracts for the dependency-gated Phase 3 startup architecture."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    """Read one bringup package file as UTF-8 text."""
    return (ROOT / relative).read_text(encoding="utf-8")


def test_stage_configuration_is_finite_and_dependency_ordered() -> None:
    """Every ordered startup stage has finite stabilization and timeout values."""
    config = yaml.safe_load(read("config/startup_stages.yaml"))[
        "bringup_readiness_node"
    ]["ros__parameters"]

    assert config["startup.evaluation_rate_hz"] == 5.0
    assert config["startup.status_publish_rate_hz"] == 2.0
    assert config["startup.observation_freshness_s"] == 3.0

    for role, stages in {
        "core": (
            "infrastructure",
            "hardware",
            "motion_safety",
            "sensor_stabilization",
            "localization",
            "supervisor",
            "navigation",
            "slam_foundation",
            "mapping_runtime",
            "head",
            "semantic_locations",
            "complete",
        ),
        "edge": (
            "infrastructure",
            "realsense",
            "vo",
            "obstacle_cloud",
            "bridge",
            "optional_apps",
            "complete",
        ),
    }.items():
        for stage in stages:
            prefix = f"startup.{role}.{stage}."
            settle = config[prefix + "minimum_settle_s"]
            stable = config[prefix + "stable_ready_s"]
            timeout = config[prefix + "startup_timeout_s"]
            assert settle >= 0.0
            assert stable >= 0.0
            assert timeout > settle + stable


def test_coordinator_is_process_only_and_preserves_real_tf_contract() -> None:
    """The coordinator gates processes without becoming motion authority."""
    source = read("src/nodes/bringup_readiness_node.cpp")

    assert '"infrastructure.parent_frame", "base_footprint"' in source
    assert '"infrastructure.child_frame", "base_link"' in source
    assert "/savo_supervisor/startup_ready" in source
    assert "/savo_nav/startup_readiness" in source
    assert "/savo_supervisor/system_ready" not in source

    for forbidden in (
        "ActionClient",
        "async_send_goal",
        "AuthorizeOperation",
        "ControlAutonomousMapping",
        "RotateToHeading",
        'publish<geometry_msgs::msg::Twist>',
    ):
        assert forbidden not in source


def test_stage_gate_requires_stable_minimum_quality_and_fails_closed() -> None:
    """Stage release requires stable quality and nonzero exits remain terminal."""
    core = read("src/bringup_contract.cpp")
    gate = read("src/nodes/startup_stage_gate_node.cpp")
    launch = read("savo_bringup/staged_launch.py")

    assert "input.quality == QualityLevel::kBelowMinimum" in core
    assert "stable_since_s_ = -1.0" in core
    assert "1.0 / startup_status_publish_rate_hz_" in read(
        "src/nodes/bringup_readiness_node.cpp"
    )
    assert "int exit_code_{3};" in gate
    assert "exit_code_ = 0;" in gate
    assert "if event.returncode != 0:" in launch
    assert "EmitEvent(" in launch
    assert "Shutdown(" in launch


def test_terminal_stage_completion_and_gate_release_are_exact() -> None:
    """Only final-stage readiness marks startup complete and releases its gate."""
    coordinator = read("src/nodes/bringup_readiness_node.cpp")
    gate = read("src/nodes/startup_stage_gate_node.cpp")

    ready_branch = coordinator.index("if (decision.ready) {")
    completed_append = coordinator.index(
        "completed_stages_.push_back(stage_name);", ready_branch
    )
    final_check = coordinator.index(
        "if (current_stage_ >= stages_.size()) {", completed_append
    )
    complete_assignment = coordinator.index(
        "startup_complete_ = true;", final_check
    )

    assert ready_branch < completed_append < final_check < complete_assignment
    assert (
        "startup_complete_ && decision.ready && !runtime_failed_"
        in coordinator
    )
    completed_branch = gate.index("if (Completed(payload)) {")
    assert gate.index("exit_code_ = 0;", completed_branch) > completed_branch
    failed_branch = gate.index('payload.find("\\\"state\\\":\\\"FAILED\\\"")')
    assert failed_branch < completed_branch
    assert gate.index("exit_code_ = 2;", failed_branch) < completed_branch


def test_complete_ready_payload_is_coherent() -> None:
    """Successful completion has one safe terminal reason and no fake input."""
    coordinator = read("src/nodes/bringup_readiness_node.cpp")

    assert 'stage_name == "complete"' in coordinator
    assert '"bringup_complete_safe_unarmed" : decision.reason' in coordinator
    assert 'AddStage("complete", complete);' in coordinator
    assert 'AddStage("complete", {});' in coordinator
    assert (
        'decision.failed && pending.empty() ?\n'
        "      std::vector<std::string>{reason} : pending"
        in coordinator
    )


def test_established_dependency_revalidation_is_fail_closed_and_debounced() -> None:
    """One boundary miss blocks; sustained loss becomes a terminal failure."""
    coordinator = read("src/nodes/bringup_readiness_node.cpp")
    contract = read("src/bringup_contract.cpp")

    assert "established_dependency_tracker_.Update(lost_signature)" in coordinator
    assert "input.dependencies_ready = false;" in coordinator
    assert "input.unrecoverable_failure = established.confirmed_loss;" in coordinator
    assert "established_dependency_revalidation_pending:" in coordinator
    assert "established_dependency_lost:" in coordinator
    assert "consecutive_loss_samples_ >= confirmation_samples_" in contract
    assert "decision.reason = terminal_failure_reason_;" in contract
    assert "runtime_failed_ = established.confirmed_loss;" in coordinator
    assert "decision.failed = runtime_failed_;" in coordinator
    assert "startup_complete_ && decision.ready && !runtime_failed_" in coordinator


def test_optional_components_use_launch_conditions_without_fake_stages() -> None:
    """Disabled optional components are controlled by ordinary conditions."""
    autonomous = read("launch/autonomous_mapping.launch.py")

    assert 'condition=IfCondition(LaunchConfiguration("start_head"))' in autonomous
    assert 'LaunchConfiguration("start_location_lifecycle")' in autonomous
    assert '"semantic_interruption_enabled": LaunchConfiguration(' in autonomous
    assert "StartupStageGroup" not in autonomous
    assert "bringup_readiness_node" not in autonomous


def test_launch_never_creates_motion_or_mission_authority() -> None:
    """Staged launch remains stopped, unarmed, and unable to submit goals."""
    autonomous = read("launch/autonomous_mapping.launch.py")

    assert 'default_value="STOP"' in autonomous
    assert '"supervisor_auto_arm", default_value="false"' in autonomous
    assert "async_send_goal" not in autonomous
    assert "ros2 action send_goal" not in autonomous
    assert "ActionClient" not in autonomous
    assert "TimerAction" not in autonomous
