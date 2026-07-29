#!/usr/bin/env python3

"""Permanent architecture contract for the production TF pose reader."""

import hashlib
from pathlib import Path
import re


PACKAGE = Path(__file__).resolve().parents[1]
HEADER = PACKAGE / 'include/savo_mapping/tf_pose_reader.hpp'
SOURCE = PACKAGE / 'src/ros/tf_pose_reader.cpp'
UNIT_TEST = PACKAGE / 'test/test_tf_pose_reader.cpp'
RUNTIME_TEST = PACKAGE / 'test/test_tf_pose_reader_runtime.py'
CMAKE = PACKAGE / 'CMakeLists.txt'

COVERAGE_CORE_HASHES = {
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
        '3d57f984e2118fc382da6c41d8d31173bb77507ee269c5afdafba14f5df4cb40',
    'test/test_scan360_launch_runtime.py':
        'b1dbb51a11afde97e34015d854c8b13e3ec20dcdba94a2833d35be3eb27802e9',
    'test/test_scan360_mapper_node_contract.py':
        '12a41679779f0b39c07d1231331334a61d49775a46b0193ccc0a6a0433d0d755',
    'test/test_scan360_mapper_node_runtime.py':
        'a262357a208d293dc56d10f16b84fa162f4d0542b8364bab31762ea5a4626d2f',
    'test/test_scan360_orchestrator.cpp':
        '156dd224a8eefb316dcd405c8434a9075d840292df4655ed254344454796888e',
    'test/test_scan360_orchestrator_contract.py':
        '9b0edd7a5d951f7dbbe11a9e19764c676b6fb1fa4763a28a09714854e9dffcb8',
    'test/test_scan360_planner.cpp':
        '9e927c9f9406f7a6eae567070bc03e9c82f969c94eb29cf69d2d62d6e84bca6f',
    'test/test_scan360_quality.cpp':
        '7075dea04abc7e737617ba3241dddc1599122c9c015970bc1f23cf37cb017c49',
    'test/test_scan360_rotate_action_binding.cpp':
        '027584eaec8187aca1ef4b92e38608447f00a6de5fa6390515ee1a65be6c2f1b',
    'test/test_scan360_rotate_action_binding_contract.py':
        'e37d9bf4e45a64e2dc7ecef895bfc9448878de8c1e115f107259eeed468cf1b5',
    'test/test_scan360_rotate_action_client_contract.py':
        '09bcfaad8f0350d22d21b2fdcdce2dfd8c725f6698e743e83dccb6e6811d816e',
    'test/test_scan360_rotate_action_client_runtime.cpp':
        '8448fb3e94e53cd6bc4a1397954b6eb6e8812b87d48f694e6ac864ed9c202aa3',
}


def read(path):
    return path.read_text(encoding='utf-8')


def sha256(relative):
    return hashlib.sha256(
        (PACKAGE / relative).read_bytes()
    ).hexdigest()


def test_reader_sources_exist_and_are_non_empty():
    for path in (HEADER, SOURCE, UNIT_TEST, RUNTIME_TEST):
        assert path.is_file(), path
        assert path.stat().st_size > 0, path


def test_reader_is_a_separate_installed_ros_library():
    cmake = read(CMAKE)
    core = re.search(
        r'add_library\(\$\{PROJECT_NAME\}_core'
        r'(?P<body>.*?)\n\)',
        cmake,
        re.DOTALL,
    )
    assert core
    assert 'src/ros/tf_pose_reader.cpp' not in core.group('body')
    assert len(re.findall(
        r'add_library\(\$\{PROJECT_NAME\}_tf_pose_reader\s+'
        r'src/ros/tf_pose_reader\.cpp\s*\)',
        cmake,
        re.DOTALL,
    )) == 1
    assert (
        '${PROJECT_NAME}_tf_pose_reader_targets'
        in cmake
    )
    assert cmake.count('src/ros/tf_pose_reader.cpp') == 1
    assert cmake.count(
        'include/savo_mapping/tf_pose_reader.hpp'
    ) == 1


def test_reader_has_only_required_ros_dependencies():
    cmake = read(CMAKE)
    dependency_block = re.search(
        r'ament_target_dependencies\(\s*'
        r'\$\{PROJECT_NAME\}_tf_pose_reader'
        r'(?P<body>.*?)\)',
        cmake,
        re.DOTALL,
    )
    assert dependency_block
    dependencies = set(
        dependency_block.group('body').split()
    )
    assert dependencies == {
        'geometry_msgs',
        'rclcpp',
        'tf2',
        'tf2_ros',
    }


def test_unit_runtime_and_contract_tests_are_registered_once():
    cmake = read(CMAKE)
    registrations = (
        r'ament_add_gtest\(\s*test_tf_pose_reader\s+',
        r'ament_add_pytest_test\(\s*'
        r'test_tf_pose_reader_runtime\s+',
        r'ament_add_pytest_test\(\s*'
        r'test_tf_pose_reader_contract\s+',
    )
    for registration in registrations:
        assert len(re.findall(registration, cmake)) == 1
    assert cmake.count('test/test_tf_pose_reader_runtime.py') == 1
    assert cmake.count('test/test_tf_pose_reader_contract.py') == 1


def test_production_reader_is_lookup_only():
    production = read(HEADER) + '\n' + read(SOURCE)
    assert 'tf2_ros::Buffer' in production
    assert 'lookupTransform(' in production
    for forbidden in (
        'TransformBroadcaster',
        'StaticTransformBroadcaster',
        'TransformListener',
        'create_publisher',
        'create_subscription',
        'rclcpp_action',
        'NavigateToPose',
        'FollowWaypoints',
        'geometry_msgs::msg::Twist',
        'cmd_vel',
    ):
        assert forbidden not in production, forbidden


def test_options_snapshot_and_validation_contract_are_present():
    header = read(HEADER)
    source = read(SOURCE)
    for token in (
        'TfPoseReaderOptions',
        'TfPoseSnapshot',
        'TfPoseReader',
        'target_frame',
        'source_frame',
        'lookup_timeout_sec',
        'stale_timeout_sec',
        'transform_stamp',
        'lookup_duration_sec',
        'std::chrono::steady_clock',
        'tf2::TimePointZero',
        'tf2::Duration::zero()',
        'std::isfinite(translation.x)',
        'std::isfinite(translation.y)',
        'std::isfinite(translation.z)',
        'std::isfinite(quaternion.x)',
        'std::isfinite(quaternion.y)',
        'std::isfinite(quaternion.z)',
        'std::isfinite(quaternion.w)',
        'quaternion_norm',
        'normalize_yaw',
        'age_sec > stale_timeout_sec',
    ):
        assert token in header + '\n' + source, token


def test_all_stable_reasons_are_locked():
    combined = read(HEADER) + '\n' + read(SOURCE)
    for reason in (
        'tf_pose_ready',
        'tf_pose_options_invalid',
        'tf_pose_target_frame_empty',
        'tf_pose_source_frame_empty',
        'tf_pose_lookup_timeout_invalid',
        'tf_pose_stale_timeout_invalid',
        'tf_pose_transform_unavailable',
        'tf_pose_lookup_timeout',
        'tf_pose_extrapolation_error',
        'tf_pose_connectivity_error',
        'tf_pose_invalid_argument',
        'tf_pose_translation_invalid',
        'tf_pose_quaternion_invalid',
        'tf_pose_yaw_invalid',
        'tf_pose_timestamp_invalid',
        'tf_pose_transform_stale',
        'tf_pose_clock_invalid',
    ):
        assert reason in combined, reason


def test_runtime_uses_only_isolated_fixture_frames():
    runtime = read(RUNTIME_TEST)
    assert 'ROS_LOCALHOST_ONLY' in runtime
    assert "ROS_DOMAIN_ID'] = '220'" in runtime
    assert 'fixture_map_' in runtime
    assert 'fixture_base_' in runtime
    assert 'TransformBroadcaster' in runtime
    assert "'map'" not in runtime
    assert "'base_link'" not in runtime
    for forbidden in (
        '/dev/',
        '/scan',
        '/odometry/filtered',
        'NavigateToPose',
        'FollowWaypoints',
        'Twist',
        '/cmd_vel',
        'cmd_vel_auto',
        'savo_base',
        'savo_control',
        'savo_localization',
        'savo_nav',
    ):
        assert forbidden not in runtime, forbidden


def test_coverage_core_hashes_remain_locked():
    for relative, expected in COVERAGE_CORE_HASHES.items():
        assert sha256(relative) == expected, relative


def test_complete_scan360_hashes_remain_locked():
    for relative, expected in SCAN360_HASHES.items():
        assert sha256(relative) == expected, relative
