"""Static contracts for the AM-5 autonomous mapping mission sequencer."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    """Read one package source file."""
    return (PACKAGE / relative).read_text(encoding='utf-8')


def test_mission_declares_the_locked_am5_prelude_states() -> None:
    """AM-5 orders start-pose, Scan360, head scan and frontier entry."""
    header = read('include/savo_mapping/autonomous_mapping_mission.hpp')
    source = read('src/workflow/autonomous_mapping_mission.cpp')

    for token in (
        'MissionState::CapturingStartPose',
        'MissionState::InitialScan360',
        'MissionState::InitialHeadScan',
        'MissionState::ConditionalScan360',
        'MissionCommand::RequestScan360',
        'MissionResult::ScanFailed',
        'MissionResult::StartPoseUnavailable',
    ):
        assert token in source or token in header

    start_pose = source.index('MissionState::CapturingStartPose')
    initial_scan = source.index('MissionState::InitialScan360', start_pose)
    initial_head = source.index('MissionState::InitialHeadScan', initial_scan)
    assert start_pose < initial_scan < initial_head
    assert 'return evaluate_frontier_entry(inputs);' in source


def test_orchestrator_uses_owned_component_boundaries() -> None:
    """Sequencing calls public Scan360/head services and reads TF evidence."""
    source = read('src/nodes/autonomous_mapping_orchestrator_node.cpp')

    for token in (
        'TfPoseReader',
        'sequence.start_pose_target_frame',
        'sequence.start_pose_source_frame',
        'sequence.scan360_start_service',
        'sequence.scan360_cancel_service',
        'sequence.head_scan_start_service',
        'sequence.head_scan_pause_service',
        'sequence.head_scan_resume_service',
        'create_client<Trigger>',
        'request_scan360_start',
        'request_head_scan_start',
        'scan360_operation_timeout',
        'head_scan_operation_timeout',
        'scan360_operation_epoch_',
        'head_scan_operation_epoch_',
    ):
        assert token in source

    for forbidden in (
        'geometry_msgs/msg/twist.hpp',
        '/cmd_vel',
        'nav2_msgs/action/navigate_to_pose',
        'slam_toolbox/srv/save_map',
        'slam_toolbox/srv/serialize_pose_graph',
    ):
        assert forbidden not in source


def test_launch_composes_scan360_without_autostart() -> None:
    """The mapping launch starts the server, not an uncommanded scan."""
    launch = read('launch/autonomous_mapping.launch.xml')
    config = read('config/autonomous_mapping_orchestrator.yaml')
    scan_config = read('config/scan360_mapping.yaml')

    assert 'scan360_mapping.launch.xml' in launch
    assert 'name="auto_start"' in launch
    assert 'value="false"' in launch
    assert '/savo_mapping/scan360/start' in config
    assert '/savo_mapping/scan360/cancel' in config
    assert '/savo_head/start_scan' in config
    assert '/savo_head/pause_scan' in config
    assert '/savo_head/resume_scan' in config
    assert 'start_service: "/savo_mapping/scan360/start"' in scan_config
    assert 'cancel_service: "/savo_mapping/scan360/cancel"' in scan_config


def test_am5_runtime_fixture_is_registered_and_isolated() -> None:
    """The production orchestrator is exercised with isolated ROS fixtures."""
    cmake = read('CMakeLists.txt')
    runtime = read('test/test_autonomous_mapping_sequencer_runtime.py')

    assert 'test_autonomous_mapping_sequencer_runtime' in cmake
    assert 'ROS_DOMAIN_ID=224' in cmake
    assert 'AUTONOMOUS_ORCHESTRATOR_EXECUTABLE' in cmake
    assert '/test/am5_' in runtime
    assert 'TransformBroadcaster' in runtime
    assert 'COMMAND_REQUEST_SCAN360' in runtime
    assert '/cmd_vel' not in runtime
