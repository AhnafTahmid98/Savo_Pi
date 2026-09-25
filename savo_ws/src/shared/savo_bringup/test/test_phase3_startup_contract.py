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
            "slam_foundation",
            "navigation",
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
    assert "/savo_mapping/slam_health" in source
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
    assert "StartupStageGroup" in autonomous
    assert "bringup_readiness_node" in autonomous


def test_autonomous_stage_dependencies_are_slam_then_nav_then_runtime() -> None:
    """Live-map ownership is established before either downstream consumer."""
    coordinator = read("src/nodes/bringup_readiness_node.cpp")
    slam = coordinator.index('AddStage("slam_foundation"')
    navigation = coordinator.index('AddStage("navigation"')
    runtime = coordinator.index('AddStage("mapping_runtime"')
    assert slam < navigation < runtime
    assert '{"localization", "slam_lifecycle", "mapping"}' in coordinator
    assert (
        '{"slam_lifecycle", "mapping", "navigation_startup"}'
        in coordinator
    )


def test_mapping_runtime_liveness_is_required_only_after_runtime_launch() -> None:
    """Pre-runtime stages cannot wait for the not-yet-launched orchestrator."""
    coordinator = read("src/nodes/bringup_readiness_node.cpp")
    autonomous = read("launch/autonomous_mapping.launch.py")

    slam_block = coordinator.split(
        'if (require_mapping_) {', maxsplit=1
    )[1].split('if (require_navigation_) {', maxsplit=1)[0]
    navigation_block = coordinator.split(
        'if (require_navigation_) {', maxsplit=1
    )[1].split('if (require_mapping_runtime_) {', maxsplit=1)[0]
    runtime_block = coordinator.split(
        'if (require_mapping_runtime_) {', maxsplit=1
    )[1].split('if (require_head_) {', maxsplit=1)[0]

    assert '"mapping_runtime"' not in slam_block
    assert '"mapping_runtime"' not in navigation_block
    assert 'mapping_runtime.push_back("mapping_runtime")' in runtime_block

    assert 'name="mapping_runtime"' in autonomous
    assert 'actions=(mapping_runtime_launch,)' in autonomous
    staged_launch = read("savo_bringup/staged_launch.py")
    assert 'GroupAction(actions=[*group.actions, gate])' in staged_launch


def test_mapping_runtime_status_is_typed_liveness_not_mission_readiness() -> None:
    """Idle, unauthorized status is valid startup liveness evidence."""
    coordinator = read("src/nodes/bringup_readiness_node.cpp")
    callback = coordinator.split(
        "void SubscribeMappingRuntimeStatus()", maxsplit=1
    )[1].split("\n  void ", maxsplit=1)[0]

    assert 'savo_msgs/msg/autonomous_mapping_status.hpp' in coordinator
    assert '"/savo_mapping/autonomous/status"' in callback
    assert 'rclcpp::QoS(1).reliable().transient_local()' in callback
    assert (
        'message->contract_version ==\n'
        '        savo_msgs::msg::AutonomousMappingStatus::CONTRACT_VERSION'
        in callback
    )
    assert 'Mark("mapping_runtime", contract_valid, !contract_valid' in callback

    for forbidden_state_requirement in (
        'message->active',
        'message->runtime_authorized',
        'message->mapping_ready',
        'message->state',
        'STATE_IDLE',
        'STATE_EXPLORING',
    ):
        assert forbidden_state_requirement not in callback


def test_mapping_runtime_status_uses_existing_fail_closed_freshness() -> None:
    """Missing, stale, or wrong-version status cannot release the stage."""
    coordinator = read("src/nodes/bringup_readiness_node.cpp")
    observation_ready = coordinator.split(
        "bool ObservationReady(", maxsplit=1
    )[1].split("\n  savo_bringup::StartupStageInput", maxsplit=1)[0]

    assert 'found == observations_.end() || !found->second.seen' in observation_ready
    assert 'reason = key + "_not_observed";' in observation_ready
    assert 'if (!Fresh(found->second))' in observation_ready
    assert 'reason = key + "_stale";' in observation_ready
    assert 'found->second.failed || !found->second.ready' in observation_ready
    assert 'startup_observation_freshness_s_' in coordinator
    assert 'SubscribeMappingRuntimeStatus();' in coordinator


def test_dedicated_mapping_liveness_gate_preserves_stop_and_inert_startup() -> None:
    """Runtime startup observation cannot command or authorize the robot."""
    coordinator = read("src/nodes/bringup_readiness_node.cpp")
    autonomous = read("launch/autonomous_mapping.launch.py")

    assert 'default_value="STOP"' in autonomous
    assert '"control_startup_mode"' in autonomous
    assert 'async_send_goal' not in coordinator
    assert 'ActionClient' not in coordinator
    assert 'AuthorizeOperation' not in coordinator
    assert 'ControlAutonomousMapping' not in coordinator


def test_launch_never_creates_motion_or_mission_authority() -> None:
    """Simple timer staging remains unable to authorize or command motion."""
    autonomous = read("launch/autonomous_mapping.launch.py")

    assert 'default_value="STOP"' in autonomous
    assert '"supervisor_auto_arm", default_value="false"' in autonomous
    assert "async_send_goal" not in autonomous
    assert "ros2 action send_goal" not in autonomous
    assert "ActionClient" not in autonomous
    assert "TimerAction" in autonomous
    assert "StartupStageGroup" in autonomous
    assert "build_staged_sequence" in autonomous
