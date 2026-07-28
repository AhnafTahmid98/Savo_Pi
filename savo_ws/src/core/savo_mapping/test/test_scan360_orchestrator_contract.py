import hashlib
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]

ORCHESTRATOR_HEADER = (
    ROOT /
    'include/savo_mapping/'
    'scan360_orchestrator.hpp'
)

ORCHESTRATOR_SOURCE = (
    ROOT /
    'src/scan360/'
    'scan360_orchestrator.cpp'
)

ORCHESTRATOR_TEST = (
    ROOT /
    'test/test_scan360_orchestrator.cpp'
)

ACTION_CLIENT_HEADER = (
    ROOT /
    'include/savo_mapping/'
    'scan360_rotate_action_client.hpp'
)

ACTION_CLIENT_SOURCE = (
    ROOT /
    'src/ros/'
    'scan360_rotate_action_client.cpp'
)

CMAKE = ROOT / 'CMakeLists.txt'


IMMUTABLE_HASHES = {
    'include/savo_mapping/scan360_controller.hpp':
        '409dc1bb161d720cb750b86983e14cb468259aace1abc5ff62755a5eadff41ee',
    'src/scan360/scan360_controller.cpp':
        'ea4f16420da8d11c3f5893a9c7646278bdea5aeb72e202814572457e8736fdda',
    'include/savo_mapping/scan360_planner.hpp':
        '304b4a06c22461f8fcc66d5858d51f13c53f8df00f70805a35babdb013bc1399',
    'src/scan360/scan360_planner.cpp':
        'a555f4e58054241650ec0571d0b726c3c3b3e923ea8979f84a06cea13dd0cfe7',
    'include/savo_mapping/scan360_rotate_action_client.hpp':
        '71d2107ea05d87071583906dc2fd3a44fb51f7fc8497fa502351ee04afe1844e',
    'src/ros/scan360_rotate_action_client.cpp':
        '02196e5562a8f161b1bc396d0e5589020976c6e9365d0a4a0b170d5954e8a821',
    'package.xml':
        '50a6cc4b2475696cac067dcfac64f46e743fd0bfbb9a797b6dfaed46b3346ee7',
    'include/savo_mapping/scan360_orchestrator.hpp':
        '1574719075f293befacea0425afe7e89a9d58144e4f2f17df2f81b245c126016',
    'src/scan360/scan360_orchestrator.cpp':
        '9028813a8fdf4ab4505160404adf65b3ad9d045e3a06803903aebd01e74c232e',
    'test/test_scan360_orchestrator.cpp':
        '156dd224a8eefb316dcd405c8434a9075d840292df4655ed254344454796888e',
}


RUNTIME_SCAFFOLDS = (
    'src/nodes/scan360_mapper_node.cpp',
    'config/scan360_mapping.yaml',
    'config/profiles/scan360_real_robot.yaml',
    'launch/scan360_mapping.launch.xml',
    'rviz/scan360_mapping.rviz',
)


def read(path: Path) -> str:
    return path.read_text(encoding='utf-8')


def assert_contains(
    text: str,
    token: str,
    path: Path,
) -> None:
    assert token in text, (
        f'{path}: missing required contract token {token!r}'
    )


def assert_not_contains(
    text: str,
    token: str,
    path: Path,
) -> None:
    assert token not in text, (
        f'{path}: contains forbidden contract token {token!r}'
    )


def test_b3b_immutable_files_retain_locked_hashes() -> None:
    for relative_path, expected_hash in IMMUTABLE_HASHES.items():
        path = ROOT / relative_path
        assert path.is_file(), (
            f'{path}: immutable file is missing'
        )

        actual_hash = hashlib.sha256(
            path.read_bytes()
        ).hexdigest()

        assert actual_hash == expected_hash, (
            f'{path}: immutable hash changed; '
            f'expected {expected_hash}, got {actual_hash}'
        )


def test_runtime_scaffolds_remain_exactly_zero_byte() -> None:
    for relative_path in RUNTIME_SCAFFOLDS:
        path = ROOT / relative_path
        assert path.is_file(), (
            f'{path}: runtime scaffold is missing'
        )
        assert path.stat().st_size == 0, (
            f'{path}: runtime scaffold must remain zero-byte'
        )


def test_orchestrator_namespace_ownership_is_locked() -> None:
    namespace_pattern = re.compile(
        r'\bnamespace\s+savo_mapping::scan360\s*\{'
    )
    parent_only_pattern = re.compile(
        r'\bnamespace\s+savo_mapping\s*\{'
    )

    for path in (
        ORCHESTRATOR_HEADER,
        ORCHESTRATOR_SOURCE,
    ):
        text = read(path)
        assert namespace_pattern.search(text), (
            f'{path}: missing namespace '
            'savo_mapping::scan360 definition'
        )
        assert not parent_only_pattern.search(text), (
            f'{path}: orchestrator must not be defined '
            'in parent-only namespace savo_mapping'
        )

    test_source = read(ORCHESTRATOR_TEST)
    assert_contains(
        test_source,
        'savo_mapping::scan360',
        ORCHESTRATOR_TEST,
    )


def test_orchestrator_core_remains_ros_independent() -> None:
    forbidden_tokens = (
        'rclcpp',
        'rclcpp_action',
        'savo_msgs/action',
        'geometry_msgs',
        'nav_msgs',
        'std_msgs',
        'tf2_ros',
        'create_publisher',
        'create_subscription',
        'create_service',
        'create_client',
        'create_wall_timer',
        'create_timer',
    )

    for path in (
        ORCHESTRATOR_HEADER,
        ORCHESTRATOR_SOURCE,
    ):
        text = read(path)
        for token in forbidden_tokens:
            assert_not_contains(text, token, path)


def test_orchestrator_has_no_motion_authority() -> None:
    forbidden_patterns = (
        re.compile(r'\bTwist\b'),
        re.compile(r'geometry_msgs::msg::Twist'),
        re.compile(r'(?<![A-Za-z0-9_])/cmd_vel(?![A-Za-z0-9_])'),
        re.compile(r'\bcmd_vel\b'),
        re.compile(r'\bcmd_vel_auto\b'),
        re.compile(r'\bangular\s*\.\s*z\b'),
        re.compile(r'\blinear\s*\.\s*[xy]\b'),
    )

    for path in (
        ORCHESTRATOR_HEADER,
        ORCHESTRATOR_SOURCE,
        ORCHESTRATOR_TEST,
    ):
        text = read(path)
        for pattern in forbidden_patterns:
            assert not pattern.search(text), (
                f'{path}: forbidden motion-authority '
                f'pattern {pattern.pattern!r}'
            )

    combined = (
        read(ORCHESTRATOR_HEADER) +
        read(ORCHESTRATOR_SOURCE)
    )
    assert_not_contains(
        combined,
        'create_publisher',
        ORCHESTRATOR_SOURCE,
    )


def test_required_controller_mappings_are_present() -> None:
    source = read(ORCHESTRATOR_SOURCE)

    for token in (
        'ControllerAction::IssueRotationRequest',
        'ControllerAction::RequestCancel',
        'ControllerEvent::MotionAccepted',
        'ControllerEvent::TargetReached',
        'ControllerEvent::MotionRejected',
        'ControllerEvent::MotionFailed',
        'ControllerEvent::CancelAcknowledged',
        'ControllerEvent::CancelRejected',
    ):
        assert_contains(source, token, ORCHESTRATOR_SOURCE)


def test_immediate_success_preserves_event_order() -> None:
    source = read(ORCHESTRATOR_SOURCE)
    success_branch_match = re.search(
        r'case\s+RotationClientState::Succeeded\s*:'
        r'(?P<body>.*?)'
        r'case\s+RotationClientState::Canceled\s*:',
        source,
        flags=re.DOTALL,
    )

    assert success_branch_match, (
        f'{ORCHESTRATOR_SOURCE}: Succeeded branch '
        'could not be isolated'
    )

    success_branch = success_branch_match.group('body')
    accepted_position = success_branch.find(
        'emit_motion_accepted()'
    )
    reached_position = success_branch.find(
        'ControllerEvent::TargetReached'
    )

    assert accepted_position >= 0, (
        f'{ORCHESTRATOR_SOURCE}: immediate-success '
        'branch does not emit MotionAccepted'
    )
    assert reached_position >= 0, (
        f'{ORCHESTRATOR_SOURCE}: immediate-success '
        'branch does not emit TargetReached'
    )
    assert accepted_position < reached_position, (
        f'{ORCHESTRATOR_SOURCE}: immediate-success '
        'order must be MotionAccepted then TargetReached'
    )

    test_source = read(ORCHESTRATOR_TEST)
    assert_contains(
        test_source,
        'ImmediateSuccessPreservesControllerEventOrder',
        ORCHESTRATOR_TEST,
    )


def test_callback_abstraction_and_rotation_phases_are_complete() -> None:
    header = read(ORCHESTRATOR_HEADER)

    for callback in (
        'request_rotation',
        'request_cancel',
        'tick',
        'snapshot',
    ):
        assert_contains(
            header,
            callback,
            ORCHESTRATOR_HEADER,
        )

    for phase in (
        'Idle',
        'Pending',
        'Active',
        'Canceling',
        'Succeeded',
        'Canceled',
        'Rejected',
        'Failed',
    ):
        assert re.search(
            rf'\b{re.escape(phase)}\b',
            header,
        ), (
            f'{ORCHESTRATOR_HEADER}: missing rotation '
            f'client phase {phase!r}'
        )

    for path in (
        ORCHESTRATOR_HEADER,
        ORCHESTRATOR_SOURCE,
    ):
        text = read(path)
        assert_not_contains(
            text,
            'Scan360RotateActionClient',
            path,
        )


def test_validation_reason_equivalents_are_locked() -> None:
    source = read(ORCHESTRATOR_SOURCE)

    canonical_to_locked_marker = {
        'scan360_callbacks_incomplete':
            'scan360 rotation callbacks are incomplete',
        'scan360_rotation_duration_invalid':
            'scan360 rotation duration must be positive and finite',
        'scan360_controller_target_missing':
            'scan360_target_missing',
        'scan360_controller_target_invalid':
            'scan360_target_yaw_not_finite',
        'scan360_rotation_request_rejected':
            'scan360_rotation_request_rejected',
        'scan360_cancel_request_rejected':
            'scan360_cancel_request_rejected',
    }

    for canonical_name, locked_marker in (
        canonical_to_locked_marker.items()
    ):
        assert locked_marker in source, (
            f'{ORCHESTRATOR_SOURCE}: missing locked '
            f'validation marker {locked_marker!r} '
            f'for canonical contract {canonical_name!r}'
        )


def test_required_cpp_fixture_coverage_is_preserved() -> None:
    test_source = read(ORCHESTRATOR_TEST)

    canonical_to_locked_test = {
        'RejectsIncompleteCallbacks':
            'IncompleteCallbacksAreRejected',
        'RejectsInvalidDuration':
            'InvalidDurationIsRejected',
        'MissingTargetProducesMotionRejected':
            'MissingTargetProducesMotionRejected',
        'NonFiniteTargetProducesMotionRejected':
            'NonFiniteTargetProducesMotionRejected',
        'DispatchesRotationRequest':
            'RotationRequestForwardsYawAndDuration',
        'ActiveProducesMotionAcceptedOnce':
            'ActiveStateProducesMotionAcceptedExactlyOnce',
        'ImmediateSuccessPreservesEventOrder':
            'ImmediateSuccessPreservesControllerEventOrder',
        'SuccessAfterActiveProducesTargetReached':
            'SuccessAfterActiveProducesTargetReachedOnce',
        'CancelProducesAcknowledgement':
            'CancelSuccessProducesCancelAcknowledged',
        'RejectedRequestProducesMotionRejected':
            'SynchronousRequestRejectionProducesMotionRejected',
        'RuntimeFailureProducesMotionFailed':
            'RuntimeFailureProducesMotionFailed',
        'CancelFailureProducesCancelRejected':
            'CancelFailureProducesCancelRejected',
        'IdleWithoutPendingOperationEmitsNothing':
            'IdleWithoutPendingOperationEmitsNothing',
        'ResetClearsQueuedAndTerminalState':
            'ResetClearsQueuedAndTerminalState',
    }

    for canonical_name, locked_name in (
        canonical_to_locked_test.items()
    ):
        assert locked_name in test_source, (
            f'{ORCHESTRATOR_TEST}: missing locked '
            f'GTest {locked_name!r} for canonical '
            f'coverage {canonical_name!r}'
        )

    for helper in (
        'construct_with_incomplete_callbacks',
        'construct_with_duration',
    ):
        assert_contains(
            test_source,
            helper,
            ORCHESTRATOR_TEST,
        )

    temporary_constructor_pattern = re.compile(
        r'EXPECT_THROW\s*\(\s*'
        r'scan360::Scan360Orchestrator\s*\(',
        flags=re.DOTALL,
    )
    assert not temporary_constructor_pattern.search(
        test_source
    ), (
        f'{ORCHESTRATOR_TEST}: temporary '
        'Scan360Orchestrator construction inside '
        'EXPECT_THROW is not cppcheck-compatible'
    )


def test_cmake_registration_is_unique_and_linked() -> None:
    cmake = read(CMAKE)

    assert cmake.count(
        'src/scan360/scan360_orchestrator.cpp'
    ) == 1, (
        f'{CMAKE}: orchestrator core source must be '
        'registered exactly once'
    )

    gtest_registrations = re.findall(
        r'ament_add_gtest\s*\(\s*'
        r'test_scan360_orchestrator\b',
        cmake,
    )
    assert len(gtest_registrations) == 1, (
        f'{CMAKE}: test_scan360_orchestrator GTest '
        f'registration count is {len(gtest_registrations)}, '
        'expected 1'
    )

    link_match = re.search(
        r'target_link_libraries\s*\(\s*'
        r'test_scan360_orchestrator\b'
        r'(?P<body>.*?)\)',
        cmake,
        flags=re.DOTALL,
    )
    assert link_match, (
        f'{CMAKE}: missing test_scan360_orchestrator '
        'target_link_libraries block'
    )
    assert '${PROJECT_NAME}_core' in link_match.group('body'), (
        f'{CMAKE}: test_scan360_orchestrator must '
        'link to ${{PROJECT_NAME}}_core'
    )

    pytest_registrations = re.findall(
        r'ament_add_pytest_test\s*\(\s*'
        r'test_scan360_orchestrator_contract\b',
        cmake,
    )
    assert len(pytest_registrations) == 1, (
        f'{CMAKE}: orchestrator contract pytest '
        f'registration count is {len(pytest_registrations)}, '
        'expected 1'
    )

    assert cmake.count(
        'test/test_scan360_orchestrator_contract.py'
    ) == 1, (
        f'{CMAKE}: orchestrator contract path must '
        'appear exactly once'
    )

    action_contract_registrations = re.findall(
        r'ament_add_pytest_test\s*\(\s*'
        r'test_scan360_rotate_action_client_contract\b',
        cmake,
    )
    assert len(action_contract_registrations) == 1, (
        f'{CMAKE}: existing action-client contract '
        'registration count is '
        f'{len(action_contract_registrations)}, expected 1'
    )


def test_native_ros_action_boundary_remains_separate() -> None:
    action_client = (
        read(ACTION_CLIENT_HEADER) +
        read(ACTION_CLIENT_SOURCE)
    )
    orchestrator = (
        read(ORCHESTRATOR_HEADER) +
        read(ORCHESTRATOR_SOURCE)
    )

    for token in (
        'rclcpp_action',
        'savo_msgs::action::RotateToHeading',
        '/savo_control/rotate_to_heading',
    ):
        assert_contains(
            action_client,
            token,
            ACTION_CLIENT_SOURCE,
        )

    for token in (
        'rclcpp_action',
        'savo_msgs::action::RotateToHeading',
        '/savo_control/rotate_to_heading',
        'scan360_rotate_action_client.hpp',
    ):
        assert_not_contains(
            orchestrator,
            token,
            ORCHESTRATOR_SOURCE,
        )

    assert_contains(
        read(ORCHESTRATOR_HEADER),
        'scan360_controller.hpp',
        ORCHESTRATOR_HEADER,
    )


def test_cpp_fixture_is_hardware_and_ros_isolated() -> None:
    test_source = read(ORCHESTRATOR_TEST)

    forbidden_patterns = (
        re.compile(r'\brclcpp::init\b'),
        re.compile(r'\bExecutor\b'),
        re.compile(r'\bcreate_node\b'),
        re.compile(r'\bsend_goal\b'),
        re.compile(r'\bpublish\s*\('),
        re.compile(r'(?<![A-Za-z0-9_])/dev(?:/|\b)'),
        re.compile(
            r'\b(?:GPIO|I2C|serial|camera|LiDAR|motor)\b',
            flags=re.IGNORECASE,
        ),
        re.compile(r'/savo_control/rotate_to_heading'),
    )

    for pattern in forbidden_patterns:
        assert not pattern.search(test_source), (
            f'{ORCHESTRATOR_TEST}: fixture contains '
            f'forbidden ROS/hardware pattern '
            f'{pattern.pattern!r}'
        )
