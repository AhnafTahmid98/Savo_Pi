import hashlib
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / 'src/nodes/scan360_mapper_node.cpp'
RUNTIME_TEST = ROOT / 'test/test_scan360_mapper_node_runtime.py'
CMAKE = ROOT / 'CMakeLists.txt'

IMMUTABLE_HASHES = {
    'package.xml':
        '235cedf79a50b5e3f8b4d1477f5f8e7f0415503b0f0db775c24637007c9c34e1',
    'include/savo_mapping/scan360_planner.hpp':
        '304b4a06c22461f8fcc66d5858d51f13c53f8df00f70805a35babdb013bc1399',
    'src/scan360/scan360_planner.cpp':
        'a555f4e58054241650ec0571d0b726c3c3b3e923ea8979f84a06cea13dd0cfe7',
    'test/test_scan360_planner.cpp':
        '9e927c9f9406f7a6eae567070bc03e9c82f969c94eb29cf69d2d62d6e84bca6f',
    'include/savo_mapping/scan360_controller.hpp':
        '409dc1bb161d720cb750b86983e14cb468259aace1abc5ff62755a5eadff41ee',
    'src/scan360/scan360_controller.cpp':
        'ea4f16420da8d11c3f5893a9c7646278bdea5aeb72e202814572457e8736fdda',
    'test/test_scan360_controller.cpp':
        '46fada4afbe74ba6c3cb498a545d2661e5ab915f9bec5f6a8893aec79c5c9e72',
    'include/savo_mapping/scan360_orchestrator.hpp':
        '1574719075f293befacea0425afe7e89a9d58144e4f2f17df2f81b245c126016',
    'src/scan360/scan360_orchestrator.cpp':
        '9028813a8fdf4ab4505160404adf65b3ad9d045e3a06803903aebd01e74c232e',
    'test/test_scan360_orchestrator.cpp':
        '156dd224a8eefb316dcd405c8434a9075d840292df4655ed254344454796888e',
    'include/savo_mapping/scan360_rotate_action_client.hpp':
        '71d2107ea05d87071583906dc2fd3a44fb51f7fc8497fa502351ee04afe1844e',
    'src/ros/scan360_rotate_action_client.cpp':
        '02196e5562a8f161b1bc396d0e5589020976c6e9365d0a4a0b170d5954e8a821',
    'test/test_scan360_rotate_action_client_runtime.cpp':
        '8448fb3e94e53cd6bc4a1397954b6eb6e8812b87d48f694e6ac864ed9c202aa3',
    'include/savo_mapping/scan360_rotate_action_binding.hpp':
        '21e1cbf25181dbfa81a9c2de98546be2bfc5d94f3ecf22c1b2dfd32c1835bb96',
    'src/ros/scan360_rotate_action_binding.cpp':
        '7d84cca994f3ab46b21d51556c87c20a9ae49e4eb3a91b1d0b00baf597090403',
    'test/test_scan360_rotate_action_binding.cpp':
        '027584eaec8187aca1ef4b92e38608447f00a6de5fa6390515ee1a65be6c2f1b',
}


def read(path: Path) -> str:
    return path.read_text(encoding='utf-8')


def test_node_is_populated_and_uses_validated_odom_yaw() -> None:
    assert NODE.is_file()
    assert NODE.stat().st_size > 0
    source = read(NODE)

    for token in (
        'nav_msgs::msg::Odometry',
        'std::string{topics::ODOM_FILTERED}',
        '"odom_frame"',
        '"odom"',
        'message->header.frame_id != odom_frame_',
        'std::isfinite(orientation.x)',
        'std::isfinite(orientation.y)',
        'std::isfinite(orientation.z)',
        'std::isfinite(orientation.w)',
        'norm_squared',
        'kMinimumQuaternionNorm',
        'std::atan2(',
        'std::isfinite(yaw_rad)',
        'scan360::normalize_yaw(yaw_rad)',
        'SteadyClock::now()',
        'yaw_stale_timeout_sec_',
        'odom_is_fresh()',
    ):
        assert token in source


def test_safe_defaults_and_native_options_are_explicit() -> None:
    source = read(NODE)

    assert re.search(
        r'"auto_start"\s*,\s*false',
        source,
    )
    assert '"/savo_control/rotate_to_heading"' in source

    for token in (
        '"enabled"',
        '"action_name"',
        '"odom_topic"',
        '"yaw_stale_timeout_sec"',
        '"sweep_angle_rad"',
        '"step_angle_rad"',
        '"direction"',
        '"settle_duration_sec"',
        '"rotation_max_duration_sec"',
        '"tick_period_sec"',
        '"server_wait_timeout_sec"',
        '"goal_response_timeout_sec"',
        '"feedback_stale_timeout_sec"',
        '"cancel_timeout_sec"',
        '"execution_grace_timeout_sec"',
        '"start_service"',
        '"cancel_service"',
    ):
        assert token in source


def test_real_component_chain_is_used_without_duplicate_sources() -> None:
    source = read(NODE)

    for token in (
        'scan360::make_plan(',
        'scan360::Scan360Controller',
        'scan360::Scan360Orchestrator',
        'make_scan360_rotate_action_callbacks(',
        'NativeClient::create(',
        'controller_.load_plan(',
        'controller_.handle(',
        'orchestrator_->dispatch(',
        'orchestrator_->tick()',
        'orchestrator_->take_events()',
    ):
        assert token in source

    assert '++current_target' not in source
    assert '--current_target' not in source
    assert 'current_target_index_' not in source
    assert 'relative_yaw_rad =' not in source
    assert 'RotationClientState::' not in source


def test_every_controller_action_is_handled_explicitly() -> None:
    source = read(NODE)
    controller_header = read(
        ROOT / 'include/savo_mapping/scan360_controller.hpp'
    )
    match = re.search(
        r'enum\s+class\s+ControllerAction\s*\{(?P<body>.*?)\};',
        controller_header,
        flags=re.DOTALL,
    )
    assert match
    actions = re.findall(r'\b([A-Z][A-Za-z0-9_]*)\b', match.group('body'))
    assert actions

    for action in actions:
        assert source.count(
            f'case scan360::ControllerAction::{action}:'
        ) == 1

    decision_switch = re.search(
        r'switch\s*\(\s*decision\.action\s*\)'
        r'\s*\{(?P<body>.*?)\n\s*\}',
        source,
        flags=re.DOTALL,
    )
    assert decision_switch
    assert 'default:' not in decision_switch.group('body')
    assert 'kMaximumDecisionSteps' in source
    assert 'kMaximumEventRounds' in source


def test_public_start_and_cancel_services_are_guarded() -> None:
    source = read(NODE)

    for token in (
        '#include <std_srvs/srv/trigger.hpp>',
        'create_control_services()',
        'create_service<Trigger>(',
        'handle_start_request',
        'handle_cancel_request',
        'scan360_already_active',
        'scan360_shutdown_in_progress',
        'reset_scan_runtime()',
        'scan360::ControllerEvent::OperatorCancel',
    ):
        assert token in source

    assert '/savo_mapping/scan360/start' in source
    assert '/savo_mapping/scan360/cancel' in source


def test_cmake_builds_installs_and_registers_node_once() -> None:
    cmake = read(CMAKE)

    assert len(re.findall(
        r'add_executable\s*\(\s*scan360_mapper_node\b',
        cmake,
    )) == 1
    assert cmake.count('src/nodes/scan360_mapper_node.cpp') == 1

    core = re.search(
        r'add_library\s*\(\s*\$\{PROJECT_NAME\}_core\b'
        r'(?P<body>.*?)\)',
        cmake,
        flags=re.DOTALL,
    )
    assert core
    assert 'scan360_mapper_node.cpp' not in core.group('body')

    link = re.search(
        r'target_link_libraries\s*\(\s*scan360_mapper_node\b'
        r'(?P<body>.*?)\)',
        cmake,
        flags=re.DOTALL,
    )
    assert link
    for target in (
        '${PROJECT_NAME}_core',
        'savo_mapping_scan360_rotate_action_client',
        'savo_mapping_scan360_rotate_action_binding',
    ):
        assert target in link.group('body')

    dependency_block = re.search(
        r'ament_target_dependencies\s*\(\s*scan360_mapper_node\b'
        r'(?P<body>.*?)\)',
        cmake,
        flags=re.DOTALL,
    )
    assert dependency_block
    assert 'std_srvs' in dependency_block.group('body')

    assert len(re.findall(
        r'ament_add_pytest_test\s*\(\s*'
        r'test_scan360_mapper_node_runtime\b',
        cmake,
    )) == 1
    assert len(re.findall(
        r'ament_add_pytest_test\s*\(\s*'
        r'test_scan360_mapper_node_contract\b',
        cmake,
    )) == 1


def test_runtime_fixture_is_local_and_never_names_production_endpoint() -> None:
    runtime = read(RUNTIME_TEST)

    assert '/fixture/scan360_mapper_' in runtime
    assert 'ROS_LOCALHOST_ONLY' in runtime
    assert 'ROS_DOMAIN_ID' in runtime
    assert '/savo_control/rotate_to_heading' not in runtime


def test_no_velocity_or_hardware_authority_is_added() -> None:
    combined = read(NODE) + read(RUNTIME_TEST)
    velocity_root = 'cmd' + '_vel'
    forbidden = (
        'geometry_msgs::msg::' + 'Tw' + 'ist',
        '/' + velocity_root,
        velocity_root,
        'linear' + '.x',
        'linear' + '.y',
        'angular' + '.z',
        'create_publisher<geometry_msgs',
        '/de' + 'v/',
        'GPIO',
        'I2C',
    )

    for token in forbidden:
        assert token not in combined


def test_immutable_implementation_hashes_are_preserved() -> None:
    for relative_path, expected_hash in IMMUTABLE_HASHES.items():
        path = ROOT / relative_path
        assert path.is_file()
        actual_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        assert actual_hash == expected_hash, (
            f'{relative_path}: expected {expected_hash}, got {actual_hash}'
        )
