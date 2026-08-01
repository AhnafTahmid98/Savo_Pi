"""Static production contracts for AM-7 sequencing and ownership."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    """Read a package-relative UTF-8 source file."""
    return (PACKAGE / relative).read_text(encoding='utf-8')


def test_mission_orders_am7_before_save() -> None:
    """Coverage, return and final scans are explicit mission states."""
    mission = read('src/workflow/autonomous_mapping_mission.cpp')
    header = read('include/savo_mapping/autonomous_mapping_mission.hpp')

    for token in (
        'MissionState::CoveragePending',
        'MissionState::Coverage',
        'MissionState::ReturningToStart',
        'MissionState::FinalScan360',
        'MissionState::FinalHeadScan',
        'request_coverage_plan',
        'request_coverage_approve',
        'request_coverage_cancel',
        'request_return_to_start',
        'request_return_cancel',
    ):
        assert token in mission or token in header

    assert mission.index('if (inputs.require_coverage)') < mission.index(
        'automatic_map_save_requested'
    )


def test_orchestrator_uses_only_guarded_movement_boundaries() -> None:
    """AM-7 never calls raw Nav2 or internal Coverage approval directly."""
    source = read('src/nodes/autonomous_mapping_orchestrator_node.cpp')

    assert '"/savo_nav/navigation/navigate_to_pose"' in source
    assert '"/savo_mapping/coverage_operation/approve"' in source
    assert '"/savo_mapping/coverage_operation/cancel"' in source
    assert '"/navigate_to_pose"' not in source
    assert '"/follow_path"' not in source
    assert '"/savo_mapping/_internal/coverage_execution/approve"' not in source
    assert 'geometry_msgs/msg/twist' not in source.lower()
    assert 'cmd_vel' not in source


def test_launch_keeps_planning_manual_and_handoff_private() -> None:
    """Production launch composes one public supervisor gateway."""
    launch = read('launch/autonomous_mapping.launch.xml')

    assert 'coverage_mapping.launch.xml' in launch
    assert 'coverage_execution_handoff.launch.xml' in launch
    assert 'coverage_operation_orchestrator.launch.xml' in launch
    assert '<arg name="auto_plan" value="false"/>' in launch
    for endpoint in (
        '/savo_mapping/_internal/coverage_execution/approve',
        '/savo_mapping/_internal/coverage_execution/cancel',
        '/savo_mapping/_internal/coverage_execution/reset',
    ):
        assert endpoint in launch


def test_am7_configuration_is_fail_closed_and_bounded() -> None:
    """AM-7 production settings expose all safety-critical bounds."""
    config = read('config/autonomous_mapping_orchestrator.yaml')

    for token in (
        'coverage:',
        'planning_timeout_s:',
        'approval_timeout_s:',
        'execution_timeout_s:',
        'feedback_stale_timeout_s:',
        'cancel_timeout_s:',
        'scan360_cancel_timeout_s:',
        'head_scan_quiescence_timeout_s:',
        'maximum_restart_attempts:',
        'return_to_start:',
        'action_name: "/savo_nav/navigation/navigate_to_pose"',
        'position_tolerance_m:',
        'proximity_timeout_s:',
        'proximity_poll_period_s:',
        'maximum_attempts:',
        'final_sequence:',
        'require_final_scan360: true',
        'require_final_head_scan: true',
    ):
        assert token in config


def test_am7_safety_state_and_correlation_contracts() -> None:
    """Timeout quiescence and generation correlation remain explicit."""
    source = read('src/nodes/autonomous_mapping_orchestrator_node.cpp')
    helper = read('src/workflow/autonomous_mapping_am7.cpp')
    header = read('include/savo_mapping/autonomous_mapping_am7.hpp')
    gateway = read('src/nodes/coverage_operation_orchestrator_node.cpp')

    for token in (
        'GoalRequestPending',
        'AcceptedActive',
        'CancelPending',
        'VerifyingProximity',
        'return_goal_may_be_executing',
    ):
        assert token in helper or token in header or token in source

    for token in (
        'scan360_non_quiesced_fault',
        'head_scan_non_quiesced_fault',
        'coverage_non_quiesced_fault',
        'return_non_quiesced_fault',
        'coverage_plan_request_generation_stale',
        'coverage_plan_reset_generation_stale',
        'coverage_plan_map_generation_stale',
        'guarded_return_dispatch_error',
        'return_operation_epoch_',
    ):
        assert token in source or token in helper

    for token in ('feedback_received', 'feedback_sequence', 'feedback_age_s'):
        assert token in gateway
