import hashlib
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]

BINDING_HEADER = (
    ROOT /
    'include/savo_mapping/'
    'scan360_rotate_action_binding.hpp'
)

BINDING_SOURCE = (
    ROOT /
    'src/ros/'
    'scan360_rotate_action_binding.cpp'
)

BINDING_TEST = (
    ROOT /
    'test/test_scan360_rotate_action_binding.cpp'
)

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

ACTION_CLIENT_HEADER = (
    ROOT /
    'include/savo_mapping/'
    'scan360_rotate_action_client.hpp'
)

CONTROLLER_HEADER = (
    ROOT /
    'include/savo_mapping/'
    'scan360_controller.hpp'
)

CONTROLLER_SOURCE = (
    ROOT /
    'src/scan360/'
    'scan360_controller.cpp'
)

PLANNER_HEADER = (
    ROOT /
    'include/savo_mapping/'
    'scan360_planner.hpp'
)

PLANNER_SOURCE = (
    ROOT /
    'src/scan360/'
    'scan360_planner.cpp'
)

CMAKE = ROOT / 'CMakeLists.txt'


IMMUTABLE_HASHES = {
    'package.xml':
        'fa2cf79fff914fa222b89038999c45a84ca296223685860578f5ff245227684a',
    'include/savo_mapping/scan360_controller.hpp':
        '409dc1bb161d720cb750b86983e14cb468259aace1abc5ff62755a5eadff41ee',
    'src/scan360/scan360_controller.cpp':
        'ea4f16420da8d11c3f5893a9c7646278bdea5aeb72e202814572457e8736fdda',
    'include/savo_mapping/scan360_planner.hpp':
        '304b4a06c22461f8fcc66d5858d51f13c53f8df00f70805a35babdb013bc1399',
    'src/scan360/scan360_planner.cpp':
        'a555f4e58054241650ec0571d0b726c3c3b3e923ea8979f84a06cea13dd0cfe7',
    'include/savo_mapping/scan360_orchestrator.hpp':
        '1574719075f293befacea0425afe7e89a9d58144e4f2f17df2f81b245c126016',
    'src/scan360/scan360_orchestrator.cpp':
        '9028813a8fdf4ab4505160404adf65b3ad9d045e3a06803903aebd01e74c232e',
    'test/test_scan360_orchestrator.cpp':
        '156dd224a8eefb316dcd405c8434a9075d840292df4655ed254344454796888e',
    'test/test_scan360_orchestrator_contract.py':
        '9b0edd7a5d951f7dbbe11a9e19764c676b6fb1fa4763a28a09714854e9dffcb8',
    'include/savo_mapping/scan360_rotate_action_client.hpp':
        '71d2107ea05d87071583906dc2fd3a44fb51f7fc8497fa502351ee04afe1844e',
    'src/ros/scan360_rotate_action_client.cpp':
        '02196e5562a8f161b1bc396d0e5589020976c6e9365d0a4a0b170d5954e8a821',
    'test/test_scan360_rotate_action_client_runtime.cpp':
        '8448fb3e94e53cd6bc4a1397954b6eb6e8812b87d48f694e6ac864ed9c202aa3',
    'test/test_scan360_rotate_action_client_contract.py':
        '09bcfaad8f0350d22d21b2fdcdce2dfd8c725f6698e743e83dccb6e6811d816e',
    'include/savo_mapping/scan360_rotate_action_binding.hpp':
        '21e1cbf25181dbfa81a9c2de98546be2bfc5d94f3ecf22c1b2dfd32c1835bb96',
    'src/ros/scan360_rotate_action_binding.cpp':
        '7d84cca994f3ab46b21d51556c87c20a9ae49e4eb3a91b1d0b00baf597090403',
    'test/test_scan360_rotate_action_binding.cpp':
        '027584eaec8187aca1ef4b92e38608447f00a6de5fa6390515ee1a65be6c2f1b',
}


NATIVE_STATE_MAPPING = (
    (('kIdle',), 'Idle'),
    (
        (
            'kWaitingForServer',
            'kWaitingForGoalResponse',
        ),
        'Pending',
    ),
    (('kActive',), 'Active'),
    (('kCanceling',), 'Canceling'),
    (('kSucceeded',), 'Succeeded'),
    (('kCanceled',), 'Canceled'),
    (('kRejected',), 'Rejected'),
    (
        (
            'kAborted',
            'kTimedOut',
            'kFailed',
        ),
        'Failed',
    ),
)


def read(path: Path) -> str:
    return path.read_text(encoding='utf-8')


def assert_contains(
    text: str,
    token: str,
    path: Path,
) -> None:
    assert token in text, (
        f'{path}: missing required token {token!r}'
    )


def assert_not_contains(
    text: str,
    token: str,
    path: Path,
) -> None:
    assert token not in text, (
        f'{path}: contains forbidden token {token!r}'
    )


def extract_binding_conversion() -> str:
    source = read(BINDING_SOURCE)
    match = re.search(
        r'RotationClientSnapshot\s*'
        r'map_scan360_rotate_action_snapshot\s*\('
        r'.*?\)\s*\{(?P<body>.*?)'
        r'\n\}\s*\n\s*'
        r'Scan360RotationCallbacks',
        source,
        flags=re.DOTALL,
    )
    assert match, (
        f'{BINDING_SOURCE}: unable to isolate native '
        'snapshot conversion function'
    )
    return match.group('body')


def extract_binding_factory() -> str:
    source = read(BINDING_SOURCE)
    match = re.search(
        r'Scan360RotationCallbacks\s*'
        r'make_scan360_rotate_action_callbacks\s*\('
        r'.*?\)\s*\{(?P<body>.*?)'
        r'\n\}\s*\n\s*\}'
        r'\s*// namespace',
        source,
        flags=re.DOTALL,
    )
    assert match, (
        f'{BINDING_SOURCE}: unable to isolate callback factory'
    )
    return match.group('body')


def extract_cmake_call(
    command: str,
    target: str,
) -> str:
    cmake = read(CMAKE)
    match = re.search(
        rf'{re.escape(command)}\s*\(\s*'
        rf'{re.escape(target)}\b'
        r'(?P<body>.*?)\)',
        cmake,
        flags=re.DOTALL,
    )
    assert match, (
        f'{CMAKE}: missing {command} call for target {target}'
    )
    return match.group('body')


def test_b3c_immutable_hashes_are_locked() -> None:
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


def test_binding_namespace_ownership_is_locked() -> None:
    namespace_pattern = re.compile(
        r'\bnamespace\s+savo_mapping::scan360\s*\{'
    )
    parent_only_pattern = re.compile(
        r'\bnamespace\s+savo_mapping\s*\{'
    )

    for path in (
        BINDING_HEADER,
        BINDING_SOURCE,
    ):
        text = read(path)
        assert namespace_pattern.search(text), (
            f'{path}: missing namespace '
            'savo_mapping::scan360 definition'
        )
        assert not parent_only_pattern.search(text), (
            f'{path}: binding must not be defined in '
            'parent-only namespace savo_mapping'
        )

    test_source = read(BINDING_TEST)
    assert re.search(
        r'\bnamespace\s+scan360\s*=\s*'
        r'savo_mapping::scan360\s*;',
        test_source,
    ), (
        f'{BINDING_TEST}: fixture must use the exact '
        'savo_mapping::scan360 namespace'
    )


def test_dependency_direction_is_preserved() -> None:
    binding_header = read(BINDING_HEADER)

    for include in (
        '#include "savo_mapping/scan360_orchestrator.hpp"',
        '#include "savo_mapping/scan360_rotate_action_client.hpp"',
    ):
        assert_contains(
            binding_header,
            include,
            BINDING_HEADER,
        )

    forbidden_dependency = (
        'scan360_rotate_action_binding.hpp',
        'scan360_rotate_action_client.hpp',
    )

    for path in (
        ORCHESTRATOR_HEADER,
        ORCHESTRATOR_SOURCE,
    ):
        text = read(path)
        for token in forbidden_dependency:
            assert_not_contains(text, token, path)

    for path in (
        CONTROLLER_HEADER,
        CONTROLLER_SOURCE,
        PLANNER_HEADER,
        PLANNER_SOURCE,
    ):
        assert_not_contains(
            read(path),
            'scan360_rotate_action_binding.hpp',
            path,
        )


def test_binding_has_separate_library_and_linkage() -> None:
    cmake = read(CMAKE)
    target = (
        'savo_mapping_scan360_rotate_action_binding'
    )

    core_match = re.search(
        r'add_library\s*\(\s*'
        r'\$\{PROJECT_NAME\}_core\b'
        r'(?P<body>.*?)\)',
        cmake,
        flags=re.DOTALL,
    )
    assert core_match, (
        f'{CMAKE}: unable to isolate ${{PROJECT_NAME}}_core'
    )
    assert (
        'src/ros/scan360_rotate_action_binding.cpp'
        not in core_match.group('body')
    ), (
        f'{CMAKE}: ROS binding source must not be '
        'registered in ${{PROJECT_NAME}}_core'
    )

    library_registrations = re.findall(
        rf'add_library\s*\(\s*{target}\b',
        cmake,
    )
    assert len(library_registrations) == 1, (
        f'{CMAKE}: binding library registration count is '
        f'{len(library_registrations)}, expected 1'
    )

    assert not re.search(
        rf'add_executable\s*\(\s*{target}\b',
        cmake,
    ), (
        f'{CMAKE}: binding target {target} must not '
        'be an executable or node'
    )

    link_body = extract_cmake_call(
        'target_link_libraries',
        target,
    )

    for dependency in (
        '${PROJECT_NAME}_core',
        'savo_mapping_scan360_rotate_action_client',
    ):
        assert dependency in link_body, (
            f'{CMAKE}: binding target {target} is missing '
            f'link dependency {dependency}'
        )


def test_public_factory_rejects_null_and_keeps_ownership() -> None:
    header = read(BINDING_HEADER)
    source = read(BINDING_SOURCE)
    factory = extract_binding_factory()

    assert re.search(
        r'Scan360RotationCallbacks\s*'
        r'make_scan360_rotate_action_callbacks\s*\(',
        header,
    ), (
        f'{BINDING_HEADER}: missing public callback factory '
        'returning Scan360RotationCallbacks'
    )
    assert_contains(
        header,
        'Scan360RotateActionClient::SharedPtr client',
        BINDING_HEADER,
    )
    assert_contains(
        source,
        'scan360_rotate_action_client_missing',
        BINDING_SOURCE,
    )

    assert factory.count('[client]') == 4, (
        f'{BINDING_SOURCE}: all four callbacks must '
        'capture shared client ownership; found '
        f'{factory.count("[client]")} shared captures'
    )

    for forbidden_capture in (
        '[&client]',
        'client.get()',
        '[client_ptr',
        '[&]',
    ):
        assert_not_contains(
            factory,
            forbidden_capture,
            BINDING_SOURCE,
        )


def test_callback_forwarding_is_exact() -> None:
    factory = extract_binding_factory()

    forwarding_patterns = {
        'target yaw and maximum duration':
            r'client->request_rotation\s*\(\s*'
            r'target_yaw_rad\s*,\s*max_duration_sec\s*\)',
        'cancellation request':
            r'client->request_cancel\s*\(\s*\)',
        'tick':
            r'client->tick\s*\(\s*\)',
        'snapshot read':
            r'client->snapshot\s*\(\s*\)',
    }

    for behavior, pattern in forwarding_patterns.items():
        assert re.search(pattern, factory), (
            f'{BINDING_SOURCE}: missing exact forwarding '
            f'for {behavior}; pattern {pattern!r}'
        )

    assert_contains(
        factory,
        'map_scan360_rotate_action_snapshot',
        BINDING_SOURCE,
    )


def test_all_native_states_have_exact_phase_mapping() -> None:
    native_header = read(ACTION_CLIENT_HEADER)
    conversion = extract_binding_conversion()

    enum_match = re.search(
        r'enum\s+class\s+State\b.*?\{'
        r'(?P<body>.*?)\};',
        native_header,
        flags=re.DOTALL,
    )
    assert enum_match, (
        f'{ACTION_CLIENT_HEADER}: native State enum '
        'could not be isolated'
    )

    native_states = re.findall(
        r'\b(k[A-Z][A-Za-z0-9_]*)\b'
        r'(?:\s*=\s*[^,]+)?\s*,',
        enum_match.group('body'),
    )

    expected_states = tuple(
        state
        for group, _ in NATIVE_STATE_MAPPING
        for state in group
    )
    assert tuple(native_states) == expected_states, (
        f'{ACTION_CLIENT_HEADER}: native state set changed; '
        f'expected {expected_states}, got {tuple(native_states)}'
    )

    for native_state in native_states:
        count = conversion.count(
            f'NativeState::{native_state}'
        )
        assert count == 1, (
            f'{BINDING_SOURCE}: native state {native_state} '
            f'appears {count} times in conversion, expected 1'
        )

    for native_group, phase in NATIVE_STATE_MAPPING:
        cases = ''.join(
            rf'case\s+NativeState::{state}\s*:\s*'
            for state in native_group
        )
        pattern = (
            cases +
            rf'mapped\.state\s*=\s*'
            rf'RotationClientState::{phase}\s*;\s*'
            r'break\s*;'
        )
        assert re.search(pattern, conversion), (
            f'{BINDING_SOURCE}: missing locked mapping '
            f'{native_group!r} -> RotationClientState::{phase}'
        )

    assert 'default:' not in conversion, (
        f'{BINDING_SOURCE}: conversion switch must not '
        'contain a default branch'
    )

    switch_match = re.search(
        r'switch\s*\(\s*snapshot\.state\s*\)'
        r'\s*\{(?P<body>.*?)\n\s*\}',
        conversion,
        flags=re.DOTALL,
    )
    assert switch_match, (
        f'{BINDING_SOURCE}: state conversion switch '
        'could not be isolated'
    )
    assert 'reason' not in switch_match.group('body'), (
        f'{BINDING_SOURCE}: phase conversion must not '
        'use native reason text'
    )


def test_native_reason_is_preserved_generically() -> None:
    conversion = extract_binding_conversion()

    assignment_pattern = re.compile(
        r'mapped\.reason\s*=\s*snapshot\.reason\s*;'
    )
    assert assignment_pattern.search(conversion), (
        f'{BINDING_SOURCE}: native reason must be copied '
        'unchanged into the mapped snapshot'
    )

    assert conversion.count('mapped.reason') == 1, (
        f'{BINDING_SOURCE}: mapped reason must have exactly '
        'one generic assignment'
    )

    stable_reasons = (
        'goal_reached',
        'canceled',
        'timeout',
        'safety_stop',
        'odom_stale',
        'disabled',
        'scan360_action_server_unavailable',
        'scan360_goal_response_timeout',
        'scan360_goal_rejected',
        'scan360_feedback_stale',
        'scan360_rotation_timeout',
        'scan360_cancel_timeout',
        'scan360_cancel_rejected',
        'scan360_action_unknown_result',
    )

    for reason in stable_reasons:
        assert reason not in conversion, (
            f'{BINDING_SOURCE}: conversion must preserve '
            f'{reason!r} generically, not special-case it'
        )


def test_binding_does_not_duplicate_orchestration() -> None:
    source = read(BINDING_SOURCE)

    for token in (
        'ControllerEvent::MotionAccepted',
        'ControllerEvent::TargetReached',
        'ControllerEvent::MotionRejected',
        'ControllerEvent::MotionFailed',
        'ControllerEvent::CancelAcknowledged',
        'ControllerEvent::CancelRejected',
        'take_events',
        'events_.push_back',
        'emit_motion_accepted',
        'emit_terminal',
    ):
        assert_not_contains(source, token, BINDING_SOURCE)


def test_binding_creates_no_ros_runtime_entities() -> None:
    forbidden_patterns = (
        re.compile(r'\brclcpp::Node\b'),
        re.compile(r'\brclcpp_action::create_client\b'),
        re.compile(r'\bcreate_wall_timer\b'),
        re.compile(r'\bcreate_timer\b'),
        re.compile(r'\bcreate_publisher\b'),
        re.compile(r'\bcreate_subscription\b'),
        re.compile(r'\bcreate_service\b'),
        re.compile(r'\bSingleThreadedExecutor\b'),
        re.compile(r'\bMultiThreadedExecutor\b'),
        re.compile(r'\brclcpp::spin\b'),
        re.compile(
            r'(?:make_shared|new)\s*<\s*'
            r'(?:savo_mapping::)?'
            r'Scan360RotateActionClient\s*>',
        ),
        re.compile(
            r'\bScan360RotateActionClient::create\s*\('
        ),
    )

    for path in (
        BINDING_HEADER,
        BINDING_SOURCE,
    ):
        text = read(path)
        for pattern in forbidden_patterns:
            assert not pattern.search(text), (
                f'{path}: binding contains forbidden ROS '
                f'runtime pattern {pattern.pattern!r}'
            )


def test_binding_has_no_motion_authority() -> None:
    forbidden_patterns = (
        re.compile(r'geometry_msgs::msg::Twist'),
        re.compile(r'\bTwist\b'),
        re.compile(
            r'(?<![A-Za-z0-9_])/cmd_vel'
            r'(?![A-Za-z0-9_])'
        ),
        re.compile(r'\bcmd_vel\b'),
        re.compile(r'\bcmd_vel_auto\b'),
        re.compile(r'\blinear\s*\.\s*[xy]\b'),
        re.compile(r'\bangular\s*\.\s*z\b'),
        re.compile(r'\bcreate_publisher\b'),
    )

    for path in (
        BINDING_HEADER,
        BINDING_SOURCE,
        BINDING_TEST,
    ):
        text = read(path)
        for pattern in forbidden_patterns:
            assert not pattern.search(text), (
                f'{path}: forbidden motion-authority '
                f'pattern {pattern.pattern!r}'
            )


def test_binding_gtest_coverage_and_fixture_isolation() -> None:
    test_source = read(BINDING_TEST)

    required_tests = (
        'NullNativeClientIsRejected',
        'EveryNativeStateMapsToExpectedPhase',
        'NativeReasonIsPreservedExactly',
        'CallbacksRetainNativeClientLifetime',
        'RotationAndCancellationRequestsAreForwarded',
        'TickIsForwardedToNativeClient',
        'BindingAddsNoControllerEventOrdering',
    )

    for test_name in required_tests:
        assert_contains(
            test_source,
            test_name,
            BINDING_TEST,
        )

    for token in (
        'target_yaw_rad()',
        'max_duration_sec()',
        'request_cancel()',
        'callbacks.tick()',
        'weak_client.expired()',
    ):
        assert_contains(
            test_source,
            token,
            BINDING_TEST,
        )

    for native_state in (
        'kIdle',
        'kWaitingForServer',
        'kWaitingForGoalResponse',
        'kActive',
        'kCanceling',
        'kSucceeded',
        'kCanceled',
        'kRejected',
        'kAborted',
        'kTimedOut',
        'kFailed',
    ):
        assert_contains(
            test_source,
            f'NativeClient::State::{native_state}',
            BINDING_TEST,
        )

    assert_contains(
        test_source,
        '"/fixture/scan360_rotate_action_binding_"',
        BINDING_TEST,
    )
    assert_contains(
        test_source,
        'next_identifier_.fetch_add(1)',
        BINDING_TEST,
    )
    assert_contains(
        test_source,
        'std::to_string(identifier)',
        BINDING_TEST,
    )
    assert_not_contains(
        test_source,
        '/savo_control/rotate_to_heading',
        BINDING_TEST,
    )


def test_binding_gtest_has_no_hardware_access() -> None:
    test_source = read(BINDING_TEST)

    forbidden_patterns = (
        re.compile(r'(?<![A-Za-z0-9_])/dev(?:/|\b)'),
        re.compile(
            r'\b(?:GPIO|I2C|serial|camera|LiDAR|'
            r'motor|UPS)\b',
            flags=re.IGNORECASE,
        ),
        re.compile(
            r'\b(?:[0-9]{1,3}\.){3}[0-9]{1,3}\b'
        ),
        re.compile(
            r'\b(?:robot[-_]?savo|savo[-_]?pi)\b',
            flags=re.IGNORECASE,
        ),
    )

    for pattern in forbidden_patterns:
        assert not pattern.search(test_source), (
            f'{BINDING_TEST}: fixture contains forbidden '
            f'hardware/host pattern {pattern.pattern!r}'
        )


def test_cmake_registration_installation_and_contract_are_unique() -> None:
    cmake = read(CMAKE)
    binding_target = (
        'savo_mapping_scan360_rotate_action_binding'
    )
    test_target = (
        'test_scan360_rotate_action_binding'
    )
    contract_target = (
        'test_scan360_rotate_action_binding_contract'
    )

    assert cmake.count(
        'src/ros/scan360_rotate_action_binding.cpp'
    ) == 1, (
        f'{CMAKE}: binding source registration must '
        'appear exactly once'
    )

    assert len(re.findall(
        rf'add_library\s*\(\s*{binding_target}\b',
        cmake,
    )) == 1, (
        f'{CMAKE}: binding library {binding_target} '
        'must be created exactly once'
    )

    assert len(re.findall(
        rf'ament_add_gtest\s*\(\s*{test_target}\b',
        cmake,
    )) == 1, (
        f'{CMAKE}: binding GTest {test_target} must '
        'be registered exactly once'
    )

    test_link_body = extract_cmake_call(
        'target_link_libraries',
        test_target,
    )
    assert binding_target in test_link_body, (
        f'{CMAKE}: binding GTest {test_target} must '
        f'link to {binding_target}'
    )

    install_match = re.search(
        r'install\s*\(\s*TARGETS\s+'
        rf'{binding_target}\b'
        r'(?P<body>.*?)\)',
        cmake,
        flags=re.DOTALL,
    )
    assert install_match, (
        f'{CMAKE}: binding library {binding_target} '
        'is not installed'
    )
    for destination in (
        'ARCHIVE DESTINATION lib',
        'LIBRARY DESTINATION lib',
        'RUNTIME DESTINATION bin',
    ):
        assert destination in install_match.group('body'), (
            f'{CMAKE}: binding install block is missing '
            f'{destination!r}'
        )

    assert (
        'include/savo_mapping/'
        'scan360_rotate_action_binding.hpp'
        in cmake
    ), (
        f'{CMAKE}: public binding header is not in '
        'the package install list'
    )

    contract_registrations = re.findall(
        rf'ament_add_pytest_test\s*\(\s*'
        rf'{contract_target}\b',
        cmake,
    )
    assert len(contract_registrations) == 1, (
        f'{CMAKE}: binding contract registration count is '
        f'{len(contract_registrations)}, expected 1'
    )

    assert cmake.count(
        'test/test_scan360_rotate_action_binding_contract.py'
    ) == 1, (
        f'{CMAKE}: binding contract path must appear once'
    )

    build_testing = cmake.find('if(BUILD_TESTING)')
    contract_registration = cmake.find(contract_target)
    build_testing_end = cmake.rfind('endif()')

    assert (
        build_testing >= 0 and
        build_testing < contract_registration <
        build_testing_end
    ), (
        f'{CMAKE}: binding contract must be registered '
        'inside BUILD_TESTING'
    )
