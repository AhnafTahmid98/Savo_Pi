from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

HEADER = (
    ROOT /
    'include/savo_mapping/'
    'scan360_rotate_action_client.hpp'
)

SOURCE = (
    ROOT /
    'src/ros/'
    'scan360_rotate_action_client.cpp'
)

CMAKE = ROOT / 'CMakeLists.txt'
PACKAGE = ROOT / 'package.xml'


def read(path: Path) -> str:
    return path.read_text(encoding='utf-8')


def test_adapter_files_exist() -> None:
    assert HEADER.is_file()
    assert SOURCE.is_file()


def test_native_action_type_and_endpoint_are_locked() -> None:
    combined = read(HEADER) + read(SOURCE)

    assert (
        'savo_msgs::action::RotateToHeading'
        in combined
    )

    assert (
        '"/savo_control/rotate_to_heading"'
        in combined
    )


def test_factory_requires_a_ros_node() -> None:
    header = read(HEADER)

    assert 'static SharedPtr create' in header

    assert (
        'const rclcpp::Node::SharedPtr & node'
        in header
    )

    assert (
        'std::enable_shared_from_this'
        in header
    )


def test_public_transport_api_is_present() -> None:
    header = read(HEADER)

    assert 'bool request_rotation(' in header
    assert 'bool request_cancel(' in header
    assert 'void tick();' in header

    assert (
        'Snapshot snapshot() const;'
        in header
    )

    assert (
        'void set_update_callback('
        in header
    )


def test_state_model_covers_complete_action_lifecycle() -> None:
    header = read(HEADER)

    for token in (
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
        assert token in header


def test_all_watchdog_options_are_explicit() -> None:
    header = read(HEADER)

    for token in (
        'server_wait_timeout_sec',
        'goal_response_timeout_sec',
        'feedback_stale_timeout_sec',
        'cancel_timeout_sec',
        'execution_grace_sec',
    ):
        assert token in header


def test_goal_yaw_is_normalized_before_dispatch() -> None:
    combined = read(HEADER) + read(SOURCE)

    assert 'normalize_yaw' in combined

    assert (
        'std::atan2('
        in combined
    )

    assert (
        'goal.target_yaw_rad'
        in combined
    )

    assert (
        'snapshot_.target_yaw_rad'
        in combined
    )


def test_one_active_rotation_owner_is_enforced() -> None:
    source = read(SOURCE)

    assert 'if (busy_locked())' in source
    assert 'return false;' in source

    assert (
        'bool Scan360RotateActionClient::'
        'busy_locked()'
        in source
    )


def test_native_goal_callbacks_are_all_wired() -> None:
    source = read(SOURCE)

    assert 'goal_response_callback' in source
    assert 'feedback_callback' in source
    assert 'result_callback' in source
    assert 'async_send_goal' in source


def test_native_cancel_request_and_ack_are_wired() -> None:
    combined = read(HEADER) + read(SOURCE)

    assert 'async_cancel_goal' in combined
    assert 'handle_cancel_response' in combined
    assert 'cancel_acknowledged' in combined
    assert 'goals_canceling' in combined


def test_terminal_success_requires_goal_reached() -> None:
    source = read(SOURCE)

    assert (
        'rclcpp_action::ResultCode::SUCCEEDED'
        in source
    )

    assert (
        'action_reason == "goal_reached"'
        in source
    )

    assert (
        'wrapped_result.result->success'
        in source
    )

    assert 'State::kSucceeded' in source


def test_deterministic_transport_reasons_are_present() -> None:
    source = read(SOURCE)

    for reason in (
        'scan360_action_server_unavailable',
        'scan360_goal_response_timeout',
        'scan360_goal_rejected',
        'scan360_feedback_stale',
        'scan360_rotation_timeout',
        'scan360_cancel_timeout',
        'scan360_cancel_rejected',
        'scan360_action_unknown_result',
    ):
        assert reason in source


def test_adapter_has_no_direct_velocity_authority() -> None:
    combined = read(HEADER) + read(SOURCE)

    for forbidden in (
        'geometry_msgs::msg::Twist',
        'cmd_vel',
        'angular.z',
        'linear.x',
        'linear.y',
    ):
        assert forbidden not in combined


def test_existing_scan360_core_remains_ros_independent() -> None:
    core_paths = (
        ROOT /
        'include/savo_mapping/'
        'scan360_controller.hpp',

        ROOT /
        'src/scan360/'
        'scan360_controller.cpp',

        ROOT /
        'include/savo_mapping/'
        'scan360_planner.hpp',

        ROOT /
        'src/scan360/'
        'scan360_planner.cpp',
    )

    for path in core_paths:
        text = read(path)

        assert 'rclcpp_action' not in text
        assert 'RotateToHeading' not in text
        assert 'cmd_vel' not in text


def test_package_declares_action_dependencies() -> None:
    package = read(PACKAGE)

    assert (
        '<depend>rclcpp_action</depend>'
        in package
    )

    assert (
        '<depend>savo_msgs</depend>'
        in package
    )

    assert (
        '<test_depend>'
        'ament_cmake_pytest'
        '</test_depend>'
        in package
    )


def test_cmake_builds_and_installs_adapter_library() -> None:
    cmake = read(CMAKE)

    assert (
        'find_package(rclcpp_action REQUIRED)'
        in cmake
    )

    assert (
        'find_package(savo_msgs REQUIRED)'
        in cmake
    )

    assert (
        'add_library('
        'savo_mapping_scan360_rotate_action_client'
        in cmake
    )

    assert (
        'src/ros/'
        'scan360_rotate_action_client.cpp'
        in cmake
    )

    assert (
        'ament_export_dependencies('
        'rclcpp rclcpp_action savo_msgs)'
        in cmake
    )


def test_contract_test_is_registered() -> None:
    cmake = read(CMAKE)

    assert (
        'find_package('
        'ament_cmake_pytest REQUIRED)'
        in cmake
    )

    assert (
        'test_scan360_rotate_action_client_contract'
        in cmake
    )

    assert (
        'test/'
        'test_scan360_rotate_action_client_contract.py'
        in cmake
    )

    assert 'TIMEOUT 60' in cmake


def test_runtime_fixture_is_registered() -> None:
    cmake = read(CMAKE)

    assert (
        'test_scan360_rotate_action_client_runtime'
        in cmake
    )

    assert (
        'test/'
        'test_scan360_rotate_action_client_runtime.cpp'
        in cmake
    )

    assert (
        'savo_mapping_scan360_rotate_action_client'
        in cmake
    )

    assert 'TIMEOUT 60' in cmake


def test_runtime_fixture_is_strictly_isolated() -> None:
    runtime = read(
        ROOT /
        'test/'
        'test_scan360_rotate_action_client_runtime.cpp'
    )

    assert (
        '/fixture/scan360_rotate_to_heading_'
        in runtime
    )

    assert (
        '"/savo_control/rotate_to_heading"'
        not in runtime
    )

    for forbidden in (
        'geometry_msgs::msg::Twist',
        'cmd_vel',
        'angular.z',
        'linear.x',
        'linear.y',
    ):
        assert forbidden not in runtime


def test_cppcheck_excludes_only_runtime_fixture() -> None:
    cmake = read(CMAKE)

    assert cmake.count('ament_cppcheck(') == 1
    assert cmake.count('EXCLUDE') >= 1
    assert (
        '${CMAKE_CURRENT_SOURCE_DIR}/test/'
        'test_scan360_rotate_action_client_runtime.cpp'
        in cmake
    )
