from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE.parents[1]
SUPERVISOR_NODE = (
    SOURCE_ROOT / 'shared/savo_supervisor/src/supervisor_node.cpp'
)


def read(relative: str) -> str:
    return (PACKAGE / relative).read_text(encoding='utf-8')


def compact(text: str) -> str:
    """Collapse C++ formatting while retaining exact interface tokens."""
    return ' '.join(text.split())


def test_mapping_status_qos_matches_authoritative_supervisor_consumer() -> None:
    producer = compact(read('src/nodes/mapping_supervisor_node.cpp'))
    consumer = compact(SUPERVISOR_NODE.read_text(encoding='utf-8'))

    assert 'status_topic_, qos::state_qos());' in producer
    assert (
        'mapping_status_topic_, '
        'rclcpp::QoS(1).transient_local().reliable(),'
    ) in consumer


def test_coverage_status_qos_matches_autonomous_orchestrator_consumer() -> None:
    producer = compact(read('src/nodes/coverage_mapper_node.cpp'))
    consumer = compact(read('src/nodes/autonomous_mapping_orchestrator_node.cpp'))

    assert 'status_topic_, qos::state_qos());' in producer
    assert 'coverage_planner_status_topic_, retained_qos,' in consumer


def test_coverage_operation_status_qos_matches_retained_consumer() -> None:
    producer = compact(
        read('src/nodes/coverage_operation_orchestrator_node.cpp')
    )
    consumer = compact(read('src/nodes/autonomous_mapping_orchestrator_node.cpp'))

    assert 'status_topic_, qos::state_qos());' in producer
    assert 'coverage_operation_status_topic_, retained_qos,' in consumer


def test_topic_specific_fix_preserves_generic_streaming_status_qos() -> None:
    qos_source = compact(read('src/ros/qos_profiles.cpp'))
    status_body = qos_source.split('rclcpp::QoS status_qos()', maxsplit=1)[1]
    status_body = status_body.split('rclcpp::QoS state_qos()', maxsplit=1)[0]
    state_body = qos_source.split('rclcpp::QoS state_qos()', maxsplit=1)[1]
    state_body = state_body.split('rclcpp::QoS command_qos()', maxsplit=1)[0]

    assert '.reliable()' in status_body
    assert '.durability_volatile()' in status_body
    assert '.reliable()' in state_body
    assert '.transient_local()' in state_body


def test_orchestrator_is_registered_and_installed() -> None:
    cmake = read('CMakeLists.txt')

    for token in (
        'src/workflow/autonomous_mapping_mission.cpp',
        'src/workflow/autonomous_mapping_am7.cpp',
        'src/workflow/frontier_completion_detector.cpp',
        'include/savo_mapping/autonomous_mapping_mission.hpp',
        'include/savo_mapping/frontier_completion_detector.hpp',
        'src/nodes/autonomous_mapping_orchestrator_node.cpp',
        'autonomous_mapping_orchestrator_node',
        'config/autonomous_mapping_orchestrator.yaml',
        'launch/autonomous_mapping_orchestrator.launch.xml',
        'test_autonomous_mapping_mission',
        'test_autonomous_mapping_am7',
        'test_frontier_completion_detector',
        'test_autonomous_mapping_orchestrator_runtime',
        'test_autonomous_mapping_sequencer_runtime',
        'test_autonomous_mapping_sequencer_contract',
    ):
        assert token in cmake


def test_orchestrator_uses_typed_public_boundary() -> None:
    source = read('src/nodes/autonomous_mapping_orchestrator_node.cpp')

    for token in (
        'savo_msgs/action/run_autonomous_mapping.hpp',
        'savo_msgs/msg/autonomous_mapping_status.hpp',
        'savo_msgs/msg/frontier_exploration_status.hpp',
        'savo_msgs/srv/control_autonomous_mapping.hpp',
        'savo_msgs/srv/authorize_operation.hpp',
        'rclcpp_action::create_server<RunMission>',
        'create_service<ControlMission>',
        'MissionCommand::Pause',
        'MissionCommand::Resume',
        'MissionCommand::Cancel',
        'MissionCommand::RequestScan360',
        'COMMAND_CHECK',
        'COMMAND_RELEASE',
        'authority_generation',
    ):
        assert token in source


def test_orchestrator_uses_guarded_nav_and_public_save_boundaries() -> None:
    source = read('src/nodes/autonomous_mapping_orchestrator_node.cpp')

    forbidden = (
        '"/navigate_to_pose"',
        '"/savo_nav/exploration/navigate_to_pose"',
        'slam_toolbox/srv/serialize_pose_graph',
        'slam_toolbox/srv/save_map',
        'nav_msgs/msg/path.hpp',
    )

    for token in forbidden:
        assert token not in source

    for token in (
        'geometry_msgs/msg/pose_stamped.hpp',
        'TfPoseReader',
        'capture_start_pose_request',
        'nav2_msgs/action/navigate_to_pose.hpp',
        '"/savo_nav/navigation/navigate_to_pose"',
        'dispatch_return_to_start',
        'evaluate_planar_proximity',
        'session::verify_saved_map_session',
    ):
        assert token in source


def test_pause_and_cancel_use_existing_guarded_handoff_cancel() -> None:
    source = read('src/nodes/autonomous_mapping_orchestrator_node.cpp')
    config = read('config/autonomous_mapping_orchestrator.yaml')

    assert '"/savo_mapping/exploration_goal/cancel"' in source
    assert (
        'handoff_cancel_service: '
        '"/savo_mapping/exploration_goal/cancel"'
    ) in config
    assert 'request_handoff_cancel' in source
    assert 'goal_handle->is_canceling()' in source


def test_commands_flow_through_mapping_mode_manager_topics() -> None:
    source = read('src/nodes/autonomous_mapping_orchestrator_node.cpp')

    for token in (
        'topics::MODE_CMD',
        'topics::START_SESSION_CMD',
        'topics::CANCEL_SESSION_CMD',
        'publish_string(mode_command_publisher_, "autonomous:frontier")',
        'publish_string(mode_command_publisher_, "monitor_only")',
        'publish_string(mode_command_publisher_, "autonomous:scan360")',
    ):
        assert token in source


def test_motion_dispatch_waits_for_lease_bound_control_mode() -> None:
    source = compact(
        read('src/nodes/autonomous_mapping_orchestrator_node.cpp')
    )
    config = read('config/autonomous_mapping_orchestrator.yaml')

    for token in (
        'control_mode_command_topic: "/savo_control/mode_cmd"',
        'control_mode_state_topic: "/savo_control/mode_state"',
    ):
        assert token in config

    for token in (
        'control_mode_owned_ && authority_validated_',
        'inputs_.supervisor_authorized && !authority_resume_required_',
        'authority_acquire_on_admission_ = goal->authority_generation == 0U',
        'AuthorizeOperation::Request::COMMAND_ACQUIRE',
        'response->authority_generation > 0U',
        'decision.request_frontier_mode && nav_mode_observed',
        'decision.request_scan360_mode && auto_mode_observed',
        'decision.request_scan360_start && auto_mode_observed',
        'decision.request_coverage_approve && nav_mode_observed',
        'decision.request_return_to_start && nav_mode_observed',
        'primary_failure_reason_ = "supervisor_mapping_authority_lost"',
        'terminal_control_stop_pending_ = true',
        'control_mode_state_ == LowLevelControlMode::Stop',
        'const bool control_mode_command_due = stop_command_required ||',
    ):
        assert token in source

    assert 'goal->authority_generation > 0U' not in source
    assert 'mission_request.authority_generation = authority_generation_' in (
        source
    )


def test_completion_detection_is_typed_and_routes_save_publicly() -> None:
    source = read('src/nodes/autonomous_mapping_orchestrator_node.cpp')
    mission = read('src/workflow/autonomous_mapping_mission.cpp')
    detector = read('src/workflow/frontier_completion_detector.cpp')

    for token in (
        'FrontierCompletionDetector',
        'handle_frontier_status',
        'completion.minimum_exhaustion_observations',
        'completion.minimum_stable_duration_s',
        'completion.frontier_status_timeout_s',
    ):
        assert token in source

    assert 'MissionState::CompletionPending' in mission
    assert 'MissionState::Saving' in mission
    assert 'MissionState::Verifying' in mission
    assert 'frontier_exhaustion_confirmed' in detector
    assert 'create_client<Trigger>' in source
    assert 'session::verify_saved_map_session' in source
    assert 'save.map_session_service' in source


def test_frontier_explorer_publishes_typed_planner_evidence() -> None:
    source = read('src/nodes/frontier_explorer_node.cpp')
    config = read('config/frontier_mapping.yaml')

    for token in (
        'savo_msgs/msg/frontier_exploration_status.hpp',
        'typed_status_publisher_',
        'plan_sequence_',
        'last_planning_status_',
        'last_planned_map_generation_',
        'exhaustion_recheck_period_sec_',
    ):
        assert token in source

    assert '/savo_mapping/frontier_explorer/typed_status' in config


def test_frontier_handoff_ack_is_typed_and_sequence_correlated() -> None:
    explorer = read('src/nodes/frontier_explorer_node.cpp')
    handoff = read('src/nodes/exploration_goal_handoff_node.cpp')
    core = read('src/core/exploration_goal_handoff.cpp')
    config = read('config/frontier_mapping.yaml')

    for token in (
        'savo_msgs/msg/exploration_goal_status.hpp',
        'expected_handoff_sequence_',
        'expected_handoff_request_id_',
        'exploration::evaluate_pending_goal',
    ):
        assert token in explorer

    assert 'typed.sequence = machine_.sequence();' in handoff
    assert 'typed.request_id = machine_.request_id();' in handoff
    assert 'typed.terminal = exploration::is_terminal' in handoff
    assert 'observation.sequence == expected_sequence' in core
    assert 'observation.request_id' in core
    assert '/savo_mapping/exploration_goal/typed_status' in config
    assert 'observed_active_handoff_' not in explorer


def test_nav_is_requested_before_frontier_waits_for_nav_readiness() -> None:
    mission = read('src/workflow/autonomous_mapping_mission.cpp')
    manager = read('src/nodes/exploration_manager_node.cpp')
    runtime = read('src/workflow/exploration_runtime.cpp')

    waiting_block = mission.split(
        'if (!inputs.runtime_authorized) {', maxsplit=1
    )[1].split('}', maxsplit=1)[0]

    assert 'output.request_frontier_mode = true;' in waiting_block
    assert 'nav_readiness_subscription_' in manager
    assert 'std::chrono::steady_clock::now()' in manager
    assert 'control_mode_command_publisher_' not in manager
    assert 'if (!inputs.nav_ready)' in runtime


def test_launch_and_config_are_nonempty_and_consistent() -> None:
    launch = read('launch/autonomous_mapping_orchestrator.launch.xml')
    config = read('config/autonomous_mapping_orchestrator.yaml')

    assert 'autonomous_mapping_orchestrator_node' in launch
    assert 'autonomous_mapping_orchestrator.yaml' in launch
    assert 'name="initial_scan360_required" default="true"' in launch
    assert 'name="initial_head_scan_required" default="true"' in launch
    assert 'name="sequence.require_initial_scan360"' in launch
    assert 'value="$(var initial_scan360_required)"' in launch
    assert 'name="sequence.require_initial_head_scan"' in launch
    assert 'value="$(var initial_head_scan_required)"' in launch

    for endpoint in (
        '/savo_mapping/autonomous/run',
        '/savo_mapping/autonomous/control',
        '/savo_mapping/autonomous/status',
        '/savo_supervisor/authorize_operation',
        '/savo_mapping/exploration/runtime_enabled',
        '/savo_mapping/exploration_goal/state',
        '/savo_mapping/frontier_explorer/typed_status',
        '/savo_mapping/map_session/save',
        '/savo_mapping/scan360/state',
        '/savo_mapping/scan360/start',
        '/savo_mapping/scan360/cancel',
        '/savo_head/scan_state',
        '/savo_head/start_scan',
        '/savo_head/pause_scan',
        '/savo_head/resume_scan',
        '/savo_mapping/coverage/request_plan',
        '/savo_mapping/coverage/reset_plan',
        '/savo_mapping/coverage_operation/approve',
        '/savo_mapping/coverage_operation/cancel',
        '/savo_mapping/coverage_operation/reset',
        '/savo_nav/navigation/navigate_to_pose',
    ):
        assert endpoint in config
