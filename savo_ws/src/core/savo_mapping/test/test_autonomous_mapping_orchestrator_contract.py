from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (PACKAGE / relative).read_text(encoding='utf-8')


def test_orchestrator_is_registered_and_installed() -> None:
    cmake = read('CMakeLists.txt')

    for token in (
        'src/workflow/autonomous_mapping_mission.cpp',
        'src/workflow/frontier_completion_detector.cpp',
        'include/savo_mapping/autonomous_mapping_mission.hpp',
        'include/savo_mapping/frontier_completion_detector.hpp',
        'src/nodes/autonomous_mapping_orchestrator_node.cpp',
        'autonomous_mapping_orchestrator_node',
        'config/autonomous_mapping_orchestrator.yaml',
        'launch/autonomous_mapping_orchestrator.launch.xml',
        'test_autonomous_mapping_mission',
        'test_frontier_completion_detector',
        'test_autonomous_mapping_orchestrator_runtime',
    ):
        assert token in cmake


def test_orchestrator_uses_typed_public_boundary() -> None:
    source = read('src/nodes/autonomous_mapping_orchestrator_node.cpp')

    for token in (
        'savo_msgs/action/run_autonomous_mapping.hpp',
        'savo_msgs/msg/autonomous_mapping_status.hpp',
        'savo_msgs/msg/frontier_exploration_status.hpp',
        'savo_msgs/srv/control_autonomous_mapping.hpp',
        'rclcpp_action::create_server<RunMission>',
        'create_service<ControlMission>',
        'MissionCommand::Pause',
        'MissionCommand::Resume',
        'MissionCommand::Cancel',
    ):
        assert token in source


def test_orchestrator_uses_public_save_boundary_without_nav_bypass() -> None:
    source = read('src/nodes/autonomous_mapping_orchestrator_node.cpp')

    forbidden = (
        'nav2_msgs/action/navigate_to_pose',
        '"/navigate_to_pose"',
        '"/savo_nav/exploration/navigate_to_pose"',
        'slam_toolbox/srv/serialize_pose_graph',
        'slam_toolbox/srv/save_map',
        'nav_msgs/msg/path.hpp',
        'geometry_msgs/msg/pose_stamped.hpp',
    )

    for token in forbidden:
        assert token not in source


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
    ):
        assert token in source


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


def test_launch_and_config_are_nonempty_and_consistent() -> None:
    launch = read('launch/autonomous_mapping_orchestrator.launch.xml')
    config = read('config/autonomous_mapping_orchestrator.yaml')

    assert 'autonomous_mapping_orchestrator_node' in launch
    assert 'autonomous_mapping_orchestrator.yaml' in launch

    for endpoint in (
        '/savo_mapping/autonomous/run',
        '/savo_mapping/autonomous/control',
        '/savo_mapping/autonomous/status',
        '/savo_mapping/exploration/runtime_enabled',
        '/savo_mapping/exploration_goal/state',
        '/savo_mapping/frontier_explorer/typed_status',
        '/savo_mapping/map_session/save',
    ):
        assert endpoint in config
