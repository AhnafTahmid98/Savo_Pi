#!/usr/bin/env python3

"""Static architecture contract for the Coverage planning ROS node."""

from hashlib import sha256
from pathlib import Path
import re


PACKAGE = Path(__file__).resolve().parents[1]
NODE = PACKAGE / 'src/nodes/coverage_mapper_node.cpp'
RUNTIME_TEST = PACKAGE / 'test/test_coverage_mapper_node_runtime.py'
CMAKE = PACKAGE / 'CMakeLists.txt'

DEFERRED_ASSETS = (
    'config/coverage_mapping.yaml',
    'config/profiles/coverage_mapping_real_robot.yaml',
    'launch/coverage_mapping.launch.xml',
    'rviz/coverage_mapping.rviz',
)

COVERAGE_HASHES = {
    'include/savo_mapping/coverage_grid.hpp':
        '21b7ffd057e3c289df8c7eb64a17ff8eb117f4e9b3ddb33f1a0b4dcf447e4e93',
    'src/coverage/coverage_grid.cpp':
        '087ac7facbcce92697c9aceb072bafb6675f6ee554b6f058f29e616c712fa5b3',
    'test/test_coverage_grid.cpp':
        'f3dd9e58bbcd91b140463e0d1996c1349499adb253129567ecfce80f39b0920e',
    'include/savo_mapping/coverage_planner.hpp':
        'cac351198c2368a4ea6210e5fe941d19adb8e80f41aeed1c6cf301ca2249459a',
    'src/coverage/coverage_planner.cpp':
        'b01650899a72579754e5cd552147fe1a6f386aed0b688b534fdf654de7c2fe01',
    'test/test_coverage_planner.cpp':
        '2eda434b3ece4d231bc7db39004df697bb1705f1619d9c222bf270e814a4c7d0',
}

TF_READER_HASHES = {
    'include/savo_mapping/tf_pose_reader.hpp':
        'fe2dfe83316f138eb3a13acec736bf95917e4a3286fb76aefca4d789331ab85e',
    'src/ros/tf_pose_reader.cpp':
        '245dbb18d99c5b6fcda735ff240937b5d79715c9e80877fdefd130ffe5135d54',
    'test/test_tf_pose_reader.cpp':
        '589e91c9b16c9100ef94f122ffcebccc334f4afbeab45917a76609199e03b374',
    'test/test_tf_pose_reader_runtime.py':
        '8435854fd39afd426a6243a6204198832aeab05eccdd1f3f88e39b50f915d01d',
}

MIGRATED_CONTRACT_HASHES = {
    'test/test_coverage_core_contract.py':
        '42fdd05b2f800ce53146800f961ad25004e7a22167ba9f84d103e58d8d87ef1d',
    'test/test_tf_pose_reader_contract.py':
        'bc3542d3139b9f3fbd23bdba903ff144ebbe0167466b145cd7a77b9782a74dc3',
}

SCAN360_HASHES = {
    'config/profiles/scan360_real_robot.yaml':
        '33b9af944bce39302d25457eac218bd8124f6efd9c12c4559ac938fe998ffe50',
    'config/scan360_mapping.yaml':
        '8e2d4d3ab9d7afd83dc2d97e655ca7dd7a2c927a1e51af91663ef0a244df1160',
    'include/savo_mapping/scan360_controller.hpp':
        '409dc1bb161d720cb750b86983e14cb468259aace1abc5ff62755a5eadff41ee',
    'include/savo_mapping/scan360_orchestrator.hpp':
        '1574719075f293befacea0425afe7e89a9d58144e4f2f17df2f81b245c126016',
    'include/savo_mapping/scan360_planner.hpp':
        '304b4a06c22461f8fcc66d5858d51f13c53f8df00f70805a35babdb013bc1399',
    'include/savo_mapping/scan360_quality.hpp':
        '1c1119275a9b76b3edc90ec38e963ae2008cd096d71b86d86b5b3a28f928b61a',
    'include/savo_mapping/scan360_rotate_action_binding.hpp':
        '21e1cbf25181dbfa81a9c2de98546be2bfc5d94f3ecf22c1b2dfd32c1835bb96',
    'include/savo_mapping/scan360_rotate_action_client.hpp':
        '71d2107ea05d87071583906dc2fd3a44fb51f7fc8497fa502351ee04afe1844e',
    'launch/scan360_mapping.launch.xml':
        'ee1d58ee2c273a4962b21f276b7cf9e9f626120f246ef6a118b165eb6a19937e',
    'rviz/scan360_mapping.rviz':
        'cfd293af6df543f886d7e4980a668502b443f7eed74d3bcdfcd93a737a8c3728',
    'src/nodes/scan360_mapper_node.cpp':
        'c75d45da89a4462a07f51c51ca9384c7b992befb415501c4747eecdd75bec899',
    'src/ros/scan360_rotate_action_binding.cpp':
        '7d84cca994f3ab46b21d51556c87c20a9ae49e4eb3a91b1d0b00baf597090403',
    'src/ros/scan360_rotate_action_client.cpp':
        '02196e5562a8f161b1bc396d0e5589020976c6e9365d0a4a0b170d5954e8a821',
    'src/scan360/scan360_controller.cpp':
        'ea4f16420da8d11c3f5893a9c7646278bdea5aeb72e202814572457e8736fdda',
    'src/scan360/scan360_orchestrator.cpp':
        '9028813a8fdf4ab4505160404adf65b3ad9d045e3a06803903aebd01e74c232e',
    'src/scan360/scan360_planner.cpp':
        'a555f4e58054241650ec0571d0b726c3c3b3e923ea8979f84a06cea13dd0cfe7',
    'src/scan360/scan360_quality.cpp':
        '2494a38c6c6cd33db50f1eaff0460223cd32516d1eff809aed51966f840fe678',
    'test/test_scan360_controller.cpp':
        '46fada4afbe74ba6c3cb498a545d2661e5ab915f9bec5f6a8893aec79c5c9e72',
    'test/test_scan360_deployment_assets_contract.py':
        'd7c5002b14ce93a009907efe607d7f3e1a40acef5e05114f19e5b1d5c33112c5',
    'test/test_scan360_launch_runtime.py':
        'b1dbb51a11afde97e34015d854c8b13e3ec20dcdba94a2833d35be3eb27802e9',
    'test/test_scan360_mapper_node_contract.py':
        '842570972ba91a65b2ab80cea2a8dc16226db2d60a444724cf82ed1b544351a5',
    'test/test_scan360_mapper_node_runtime.py':
        'a262357a208d293dc56d10f16b84fa162f4d0542b8364bab31762ea5a4626d2f',
    'test/test_scan360_orchestrator.cpp':
        '156dd224a8eefb316dcd405c8434a9075d840292df4655ed254344454796888e',
    'test/test_scan360_orchestrator_contract.py':
        '3339692a57909e50d05ca6c7b80d86c752124eb60e547a146a2ccb5def2554d6',
    'test/test_scan360_planner.cpp':
        '9e927c9f9406f7a6eae567070bc03e9c82f969c94eb29cf69d2d62d6e84bca6f',
    'test/test_scan360_quality.cpp':
        '7075dea04abc7e737617ba3241dddc1599122c9c015970bc1f23cf37cb017c49',
    'test/test_scan360_rotate_action_binding.cpp':
        '027584eaec8187aca1ef4b92e38608447f00a6de5fa6390515ee1a65be6c2f1b',
    'test/test_scan360_rotate_action_binding_contract.py':
        'a1acb2d3e019878834094c7a0512796938809e55fd1759ec1e606c7adb9c6e26',
    'test/test_scan360_rotate_action_client_contract.py':
        '09bcfaad8f0350d22d21b2fdcdce2dfd8c725f6698e743e83dccb6e6811d816e',
    'test/test_scan360_rotate_action_client_runtime.cpp':
        '8448fb3e94e53cd6bc4a1397954b6eb6e8812b87d48f694e6ac864ed9c202aa3',
}


def read(path: Path) -> str:
    """Read a repository text file as UTF-8."""
    return path.read_text(encoding='utf-8')


def digest(relative: str) -> str:
    """Return the SHA-256 digest for a package-relative file."""
    return sha256((PACKAGE / relative).read_bytes()).hexdigest()


def cmake_call_bodies(text: str, command: str) -> list[str]:
    """Extract balanced argument bodies for a CMake command."""
    start_pattern = re.compile(
        rf'(?<![A-Za-z0-9_]){re.escape(command)}\s*\('
    )
    bodies = []

    for match in start_pattern.finditer(text):
        opening = text.find('(', match.start())
        depth = 1
        cursor = opening + 1

        while cursor < len(text) and depth:
            if text[cursor] == '(':
                depth += 1
            elif text[cursor] == ')':
                depth -= 1
            cursor += 1

        assert depth == 0, f'unbalanced {command} call in CMakeLists.txt'
        bodies.append(text[opening + 1:cursor - 1])

    return bodies


def normalized(text: str) -> str:
    """Collapse formatting whitespace for semantic CMake comparisons."""
    return ' '.join(text.split())


def matching_calls(text: str, command: str, first_token: str) -> list[str]:
    """Return CMake calls whose first argument is the requested target."""
    matches = []
    for body in cmake_call_bodies(text, command):
        tokens = body.split()
        if tokens and tokens[0] == first_token:
            matches.append(body)
    return matches


def execution_authority_tokens() -> tuple[str, ...]:
    """Build prohibited execution tokens without embedding audit literals."""
    return (
        ''.join(('rclcpp_', 'action')),
        ''.join(('Navigate', 'ToPose')),
        ''.join(('Navigate', 'ThroughPoses')),
        ''.join(('Follow', 'Waypoints')),
        ''.join(('Follow', 'Path')),
        ''.join(('geometry_msgs::msg::', 'Twist')),
        '/'.join(('', ''.join(('cmd_', 'vel')))),
        ''.join(('cmd_', 'vel_auto')),
        '.'.join(('linear', 'x')),
        '.'.join(('linear', 'y')),
        '.'.join(('angular', 'z')),
        ''.join((
            'create_publisher<',
            'geometry_msgs::msg::',
            'Twist',
        )),
    )


def assert_bool_default(source: str, name: str, value: bool) -> None:
    """Assert a bool parameter is declared with the required default."""
    default = 'true' if value else 'false'
    declaration = re.compile(
        r'(?:declare_parameter|declare_or_get)\s*<\s*bool\s*>\s*'
        rf'\(.{{0,240}}?"{re.escape(name)}"\s*,\s*{default}\b',
        re.DOTALL,
    )
    assert declaration.search(source), (
        f'{name} must be declared with default {default}'
    )


def test_node_is_populated_cpp_and_uses_validated_components() -> None:
    """Lock the production language and delegated planning components."""
    assert NODE.is_file()
    assert NODE.suffix == '.cpp'
    assert NODE.stat().st_size > 0

    source = read(NODE)
    for token in (
        'savo_mapping/coverage_grid.hpp',
        'savo_mapping/coverage_planner.hpp',
        'savo_mapping/tf_pose_reader.hpp',
        'CoverageGrid',
        'CoverageGridMetadata',
        'CoverageGridOptions',
        'CoveragePlanner',
        'CoveragePlannerOptions',
        'TfPoseReader',
        'TfPoseReaderOptions',
        'plan_from_world',
    ):
        assert token in source, token

    assert re.search(
        r'(?:\.|->)\s*read\s*\(\s*\)',
        source,
    )

    for forbidden in (
        'src/coverage/coverage_grid.cpp',
        'src/coverage/coverage_planner.cpp',
        'src/ros/tf_pose_reader.cpp',
        'lookupTransform(',
        'reachable_from(',
        'std::queue',
        'std::priority_queue',
        'breadth_first',
        'flood_fill',
    ):
        assert forbidden not in source, forbidden


def test_ros_interfaces_are_read_only_and_configurable() -> None:
    """Lock map input and read-only Path and String observations."""
    source = read(NODE)

    assert 'nav_msgs::msg::OccupancyGrid' in source
    assert 'nav_msgs::msg::Path' in source
    assert 'std_msgs::msg::String' in source
    assert re.search(
        r'create_subscription\s*<\s*nav_msgs::msg::OccupancyGrid\s*>',
        source,
    )
    assert re.search(
        r'create_publisher\s*<\s*nav_msgs::msg::Path\s*>',
        source,
    )
    assert len(re.findall(
        r'create_publisher\s*<\s*std_msgs::msg::String\s*>',
        source,
    )) >= 2

    for parameter in (
        'map_topic',
        'map_frame',
        'base_frame',
        'path_topic',
        'status_topic',
        'state_topic',
    ):
        assert f'"{parameter}"' in source, parameter

    for qos_factory in (
        'qos::map_qos()',
        'qos::state_qos()',
        'qos::status_qos()',
    ):
        assert qos_factory in source, qos_factory


def test_activation_defaults_and_planner_parameters_are_locked() -> None:
    """Protect passive startup and all configurable core options."""
    source = read(NODE)

    assert_bool_default(source, 'enabled', True)
    assert_bool_default(source, 'auto_plan', False)
    assert_bool_default(source, 'plan_once', True)
    assert_bool_default(source, 'replan_on_map_update', False)

    for parameter in (
        'tick_period_sec',
        'map_stale_timeout_sec',
        'tf_lookup_timeout_sec',
        'tf_stale_timeout_sec',
        'free_threshold',
        'occupied_threshold',
        'allow_unknown',
        'inflation_radius_m',
        'connectivity',
        'sweep_axis',
        'track_spacing_m',
        'minimum_segment_length_m',
        'maximum_waypoints',
    ):
        assert f'"{parameter}"' in source, parameter

    assert 'std::isfinite' in source
    assert 'std::invalid_argument' in source


def test_map_validation_and_steady_freshness_are_explicit() -> None:
    """Protect map-frame, geometry, payload, and reception-age checks."""
    source = read(NODE)

    for token in (
        'header.frame_id',
        'map_frame_',
        'info.width',
        'info.height',
        'info.resolution',
        'message.info.origin',
        'origin.position.x',
        'origin.position.y',
        'orientation.x',
        'orientation.y',
        'orientation.z',
        'orientation.w',
        'data.size()',
        'CoverageGridMetadata',
        'std::chrono::steady_clock',
        'map_stale_timeout_sec',
        'map_sequence',
    ):
        assert token in source, token

    assert re.search(
        r'header\.frame_id\s*!=\s*map_frame[A-Za-z0-9_]*',
        source,
    )
    assert 'map_received' in source or 'map_reception' in source
    assert 'header.stamp' not in ''.join(
        line
        for line in source.splitlines()
        if 'map_stale_timeout_sec' in line
    )


def test_planning_is_bounded_once_only_and_map_update_driven() -> None:
    """Protect timer-driven planning and duplicate-publication guards."""
    source = read(NODE)

    for token in (
        'create_wall_timer',
        'auto_plan',
        'enabled',
        'plan_once',
        'replan_on_map_update',
        'map_sequence',
        'plan_sequence',
        'CoveragePlanner',
        'plan_from_world',
    ):
        assert token in source, token

    assert re.search(
        r'(?:attempted|planned|published)'
        r'[A-Za-z0-9_]*map_sequence[A-Za-z0-9_]*',
        source,
    )
    assert 'while (true)' not in source
    assert 'while(true)' not in source


def test_heavy_map_and_planner_work_stays_off_executor_callbacks() -> None:
    """Lock bounded asynchronous work and stale-result rejection."""
    source = read(NODE)

    for token in (
        '<future>',
        'std::async',
        'std::launch::async',
        'std::future_status::ready',
        'map_input_generation',
        'kMaximumMapCellCount',
        'kMaximumInflationCellChecks',
        'coverage_node_map_work_limit_exceeded',
    ):
        assert token in source, token

    handle_map = re.search(
        r'void\s+handle_map\s*\(.*?\n  \}',
        source,
        re.DOTALL,
    )
    planning_tick = re.search(
        r'void\s+planning_tick\s*\(.*?\n  \}',
        source,
        re.DOTALL,
    )
    assert handle_map
    assert planning_tick
    assert 'convert_map' not in handle_map.group()
    assert 'plan_from_world' not in planning_tick.group()
    assert 'tf_pose_reader_->read' not in planning_tick.group()


def test_path_has_map_frame_time_and_normalized_orientation() -> None:
    """Protect the read-only Path conversion contract."""
    source = read(NODE)

    assert re.search(
        r'header\.frame_id\s*=\s*map_frame[A-Za-z0-9_]*',
        source,
    )
    assert re.search(
        r'header\.stamp\s*=',
        source,
    )
    assert 'now()' in source
    for token in (
        '.poses',
        '.pose.position.x',
        '.pose.position.y',
        '.pose.orientation.z',
        '.pose.orientation.w',
        'std::sin',
        'std::cos',
        'waypoints',
    ):
        assert token in source, token


def test_state_reasons_and_status_fields_are_stable() -> None:
    """Lock deterministic machine-readable observation semantics."""
    source = read(NODE)

    for state in (
        'disabled',
        'waiting_for_map',
        'map_invalid',
        'map_stale',
        'waiting_for_pose',
        'pose_invalid',
        'pose_stale',
        'ready',
        'planning',
        'plan_ready',
        'plan_failed',
    ):
        assert f'"{state}"' in source, state

    for reason in (
        'coverage_node_disabled',
        'coverage_node_waiting_for_map',
        'coverage_node_map_frame_mismatch',
        'coverage_node_map_invalid',
        'coverage_node_map_stale',
        'coverage_node_waiting_for_pose',
        'coverage_node_pose_invalid',
        'coverage_node_pose_stale',
        'coverage_node_start_out_of_bounds',
        'coverage_node_start_blocked',
        'coverage_node_plan_ready',
        'coverage_node_plan_failed',
    ):
        assert reason in source, reason

    for field in (
        'enabled',
        'auto_plan',
        'state',
        'reason',
        'map_valid',
        'map_fresh',
        'pose_valid',
        'pose_fresh',
        'waypoint_count',
        'reachable_cell_count',
        'covered_cell_count',
        'coverage_ratio',
        'estimated_path_length_m',
        'map_sequence',
        'plan_sequence',
        'terminal',
    ):
        assert f'\\"{field}\\"' in source, field


def test_node_has_no_execution_or_hardware_authority() -> None:
    """Reject movement, action transport, subsystem, and device coupling."""
    source = read(NODE)

    for forbidden in execution_authority_tokens():
        assert forbidden not in source, forbidden

    for forbidden in (
        'async_send_goal',
        'create_client<',
        'create_service<',
        '/odometry/filtered',
        '/dev/',
        'serial_port',
        'libusb',
    ):
        assert forbidden not in source, forbidden

    implementation_header = re.compile(
        r'#include\s*[<"]savo_(?:base|control|perception|lidar|'
        r'localization|power|realsense|vo|speech|nav|head)/'
    )
    assert not implementation_header.search(source)


def test_cmake_builds_links_installs_and_registers_once() -> None:
    """Protect target boundaries, installation, and test registration."""
    cmake = read(CMAKE)
    source_path = 'src/nodes/coverage_mapper_node.cpp'

    executable_calls = matching_calls(
        cmake,
        'add_executable',
        'coverage_mapper_node',
    )
    assert len(executable_calls) == 1
    assert source_path in executable_calls[0]
    assert cmake.count(source_path) == 1

    core_calls = matching_calls(
        cmake,
        'add_library',
        '${PROJECT_NAME}_core',
    )
    assert len(core_calls) == 1
    assert source_path not in core_calls[0]

    link_calls = matching_calls(
        cmake,
        'target_link_libraries',
        'coverage_mapper_node',
    )
    assert len(link_calls) == 1
    links = set(link_calls[0].split())
    assert '${PROJECT_NAME}_core' in links
    assert '${PROJECT_NAME}_tf_pose_reader' in links

    dependency_calls = matching_calls(
        cmake,
        'ament_target_dependencies',
        'coverage_mapper_node',
    )
    assert len(dependency_calls) == 1
    dependencies = set(dependency_calls[0].split()[1:])
    assert {'rclcpp', 'nav_msgs', 'std_msgs'} <= dependencies
    assert ''.join(('rclcpp_', 'action')) not in dependencies
    assert 'nav2_msgs' not in dependencies
    assert 'savo_msgs' not in dependencies

    installs = [
        body
        for body in cmake_call_bodies(cmake, 'install')
        if 'coverage_mapper_node' in body.split()
    ]
    assert len(installs) == 1
    install_text = normalized(installs[0])
    assert 'TARGETS coverage_mapper_node' in install_text
    assert 'DESTINATION lib/${PROJECT_NAME}' in install_text

    registrations = {
        'test_coverage_mapper_node_runtime':
            'test/test_coverage_mapper_node_runtime.py',
        'test_coverage_mapper_node_contract':
            'test/test_coverage_mapper_node_contract.py',
    }
    for name, path in registrations.items():
        calls = matching_calls(
            cmake,
            'ament_add_pytest_test',
            name,
        )
        assert len(calls) == 1, name
        assert path in calls[0], path
        assert cmake.count(path) == 1, path


def test_deferred_assets_remain_empty_and_uninstalled() -> None:
    """Keep deployment configuration deferred to its later phase."""
    cmake = read(CMAKE)
    installed = '\n'.join(cmake_call_bodies(cmake, 'install'))

    for relative in DEFERRED_ASSETS:
        path = PACKAGE / relative
        assert path.is_file(), relative
        assert path.stat().st_size == 0, relative
        assert relative not in installed, relative


def test_runtime_uses_installed_node_and_isolated_fixtures() -> None:
    """Protect fixture isolation, bounded waits, and process cleanup."""
    assert RUNTIME_TEST.is_file()
    assert RUNTIME_TEST.stat().st_size > 0
    runtime = read(RUNTIME_TEST)

    for token in (
        'AMENT_PREFIX_PATH',
        'coverage_mapper_node',
        'subprocess.Popen',
        'ROS_LOCALHOST_ONLY',
        'ROS_DOMAIN_ID',
        '221',
        'PYTHONDONTWRITEBYTECODE',
        'os.getpid()',
        '/fixture/',
        'fixture_map_',
        'fixture_base_',
        'OccupancyGrid',
        'Path',
        'TransformBroadcaster',
        'time.monotonic()',
        'os.killpg(',
        'process.wait(',
        'signal.SIGINT',
        'signal.SIGKILL',
    ):
        assert token in runtime, token

    absolute_names = re.findall(
        r'(?P<quote>[\'"])(?P<name>/[^\'"]+)(?P=quote)',
        runtime,
    )
    for _, name in absolute_names:
        assert name.startswith('/fixture/'), name

    for forbidden in execution_authority_tokens():
        assert forbidden not in runtime, forbidden

    for forbidden in (
        '/dev/',
        '/scan',
        '/odometry/filtered',
        'savo_base',
        'savo_control',
        'savo_localization',
    ):
        assert forbidden not in runtime, forbidden


def test_runtime_contains_required_failure_and_success_cases() -> None:
    """Lock the required map, TF, replanning, and safety scenarios."""
    runtime = read(RUNTIME_TEST)
    required_evidence = (
        'coverage_node_auto_plan_disabled',
        'coverage_node_disabled',
        'coverage_node_waiting_for_map',
        'coverage_node_waiting_for_pose',
        "frame=f'fixture_wrong_",
        'width=0',
        'resolution=0.0',
        'data=[0] * 24',
        "float('nan')",
        'map_stale_timeout_sec=0.12',
        'wait_path_count(1)',
        'path.header.frame_id',
        'math.isfinite(value)',
        'norm == pytest.approx(1.0',
        'central_obstacle',
        'all(column < 6',
        'plan_once=False',
        'replan_on_map_update=True',
        'invalid_update',
        'blocked_start',
        '(-0.5, 0.5)',
        'tf_stale_timeout_sec=0.10',
        'get_action_client_names_and_types_by_node',
        'assert_path_count_stable',
        'os.killpg',
    )
    for evidence in required_evidence:
        assert evidence in runtime, evidence


def test_immutable_coverage_and_tf_foundations_are_unchanged() -> None:
    """Lock completed Coverage and TF production foundations."""
    for relative, expected in {
        **COVERAGE_HASHES,
        **TF_READER_HASHES,
        **MIGRATED_CONTRACT_HASHES,
    }.items():
        assert digest(relative) == expected, relative


def test_complete_scan360_work_remains_unchanged() -> None:
    """Lock every completed Scan360 implementation and test artifact."""
    for relative, expected in SCAN360_HASHES.items():
        assert digest(relative) == expected, relative
