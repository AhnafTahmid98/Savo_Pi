#!/usr/bin/env python3

"""Static deployment contract for Coverage Mapping assets."""

import hashlib
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
BASE_CONFIG = ROOT / 'config/coverage_mapping.yaml'
ROBOT_PROFILE = ROOT / 'config/profiles/coverage_mapping_real_robot.yaml'
LAUNCH = ROOT / 'launch/coverage_mapping.launch.xml'
RVIZ = ROOT / 'rviz/coverage_mapping.rviz'
CMAKE = ROOT / 'CMakeLists.txt'

ASSETS = (BASE_CONFIG, ROBOT_PROFILE, LAUNCH, RVIZ)

NODE_PARAMETERS = {
    'enabled',
    'auto_plan',
    'plan_once',
    'replan_on_map_update',
    'map_topic',
    'map_frame',
    'base_frame',
    'path_topic',
    'status_topic',
    'state_topic',
    'request_plan_service',
    'reset_plan_service',
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
}

IMMUTABLE_HASHES = {
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
    'include/savo_mapping/tf_pose_reader.hpp':
        'fe2dfe83316f138eb3a13acec736bf95917e4a3286fb76aefca4d789331ab85e',
    'src/ros/tf_pose_reader.cpp':
        '245dbb18d99c5b6fcda735ff240937b5d79715c9e80877fdefd130ffe5135d54',
    'test/test_tf_pose_reader.cpp':
        '589e91c9b16c9100ef94f122ffcebccc334f4afbeab45917a76609199e03b374',
    'test/test_tf_pose_reader_runtime.py':
        'ef6fdcf5a4fb369cc6de2a9864e0f63081b9aea8906eacac80f9bdc2c3df28c1',
    'test/test_coverage_mapper_node_runtime.py':
        'c5931b27b75b73039cfe4fd69eec2a143c40da3d3e7112fdd0127f25cedc99ae',
    'config/scan360_mapping.yaml':
        '16f769954cc92e28e97c7347a5c649bd7261c2da28abcc7c84e30c48baaed8ed',
    'launch/scan360_mapping.launch.xml':
        'ee1d58ee2c273a4962b21f276b7cf9e9f626120f246ef6a118b165eb6a19937e',
    'rviz/scan360_mapping.rviz':
        'cfd293af6df543f886d7e4980a668502b443f7eed74d3bcdfcd93a737a8c3728',
}

MIGRATED_CONTRACTS = (
    ROOT / 'test/test_coverage_core_contract.py',
    ROOT / 'test/test_tf_pose_reader_contract.py',
    ROOT / 'test/test_coverage_mapper_node_contract.py',
)


def read(path: Path) -> str:
    return path.read_text(encoding='utf-8')


def load_yaml(path: Path):
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def node_parameters(path: Path) -> dict:
    document = load_yaml(path)
    assert isinstance(document, dict), f'{path}: YAML root must be a mapping'
    assert set(document) == {'coverage_mapper_node'}, (
        f'{path}: expected only coverage_mapper_node at the YAML root'
    )
    node = document['coverage_mapper_node']
    assert set(node) == {'ros__parameters'}, (
        f'{path}: expected only ros__parameters below the node root'
    )
    parameters = node['ros__parameters']
    assert isinstance(parameters, dict), (
        f'{path}: ros__parameters must be a mapping'
    )
    return parameters


def test_assets_exist_and_are_non_empty() -> None:
    for path in ASSETS:
        assert path.is_file(), f'{path}: deployment asset is missing'
        assert path.stat().st_size > 0, f'{path}: deployment asset is empty'


def test_base_config_matches_declared_node_contract_and_safe_defaults(
) -> None:
    parameters = node_parameters(BASE_CONFIG)
    assert set(parameters) == NODE_PARAMETERS, (
        f'{BASE_CONFIG}: keys differ from the node contract; '
        f'missing={NODE_PARAMETERS - set(parameters)}, '
        f'extra={set(parameters) - NODE_PARAMETERS}'
    )
    assert parameters['enabled'] is True
    assert parameters['auto_plan'] is False
    assert parameters['plan_once'] is True
    assert parameters['replan_on_map_update'] is False
    assert parameters['map_topic'] == '/map'
    assert parameters['map_frame'] == 'map'
    assert parameters['base_frame'] == 'base_link'
    assert parameters['path_topic'] == '/savo_mapping/coverage/path'
    assert parameters['status_topic'] == '/savo_mapping/coverage/status'
    assert parameters['state_topic'] == '/savo_mapping/coverage/state'
    assert (
        parameters['request_plan_service']
        == '/savo_mapping/coverage/request_plan'
    )
    assert (
        parameters['reset_plan_service']
        == '/savo_mapping/coverage/reset_plan'
    )
    assert parameters['allow_unknown'] is False
    assert parameters['connectivity'] in {'four', 'eight'}
    assert parameters['sweep_axis'] in {'automatic', 'rows', 'columns'}
    assert (
        0
        <= parameters['free_threshold']
        < parameters['occupied_threshold']
        <= 100
    )
    assert 0.01 <= parameters['tick_period_sec'] <= 1.0
    assert 0.0 < parameters['tf_lookup_timeout_sec'] <= 2.0
    for name in (
        'map_stale_timeout_sec',
        'tf_stale_timeout_sec',
        'inflation_radius_m',
        'minimum_segment_length_m',
    ):
        value = parameters[name]
        assert isinstance(value, (int, float)) and math.isfinite(value)
        assert value >= 0.0, f'{BASE_CONFIG}: {name} must be non-negative'
    assert parameters['track_spacing_m'] > 0.0
    assert 0 < parameters['maximum_waypoints'] <= 100000


def test_real_robot_profile_is_conservative_and_non_authoritative() -> None:
    parameters = node_parameters(ROBOT_PROFILE)
    assert set(parameters) <= NODE_PARAMETERS
    assert parameters['auto_plan'] is False
    assert parameters['plan_once'] is True
    assert parameters['allow_unknown'] is False
    assert parameters['inflation_radius_m'] == 0.24
    assert parameters['map_stale_timeout_sec'] > 0.0
    assert parameters['tf_stale_timeout_sec'] >= 0.0
    profile_text = read(ROBOT_PROFILE).lower()
    for token in ('pwm', 'motor', 'acceleration'):
        assert token not in profile_text, (
            f'{ROBOT_PROFILE}: profile must not own {token} settings'
        )
    assert 'floor validation' in profile_text


def test_launch_is_safe_reusable_and_profile_overlay_is_exclusive() -> None:
    root = ET.parse(LAUNCH).getroot()
    assert root.tag == 'launch'
    arguments = {
        element.attrib['name']: element.attrib.get('default')
        for element in root.findall('arg')
    }
    required = {
        'params_file',
        'profile_file',
        'use_real_robot_profile',
        'enabled',
        'auto_plan',
        'plan_once',
        'replan_on_map_update',
        'map_topic',
        'map_frame',
        'base_frame',
        'path_topic',
        'status_topic',
        'state_topic',
        'use_sim_time',
    }
    assert required <= set(arguments), f'{LAUNCH}: launch arguments missing'
    assert arguments['auto_plan'] == 'false'
    assert arguments['plan_once'] == 'true'
    assert arguments['replan_on_map_update'] == 'false'
    assert arguments['use_real_robot_profile'] == 'false'

    nodes = root.findall('node')
    assert len(nodes) == 2, (
        f'{LAUNCH}: expected two mutually exclusive parameter branches'
    )
    for node in nodes:
        assert node.attrib.get('pkg') == 'savo_mapping'
        assert node.attrib.get('exec') == 'coverage_mapper_node'
        assert node.attrib.get('name') == 'coverage_mapper_node'
        assert node.attrib.get('output') == 'screen'
    assert {node.attrib.get('if') for node in nodes} == {
        None,
        '$(var use_real_robot_profile)',
    }
    assert {node.attrib.get('unless') for node in nodes} == {
        None,
        '$(var use_real_robot_profile)',
    }
    profile_node = next(
        node for node in nodes
        if node.attrib.get('if') == '$(var use_real_robot_profile)'
    )
    assert any(
        param.attrib.get('from') == '$(var profile_file)'
        for param in profile_node.findall('param')
    )
    launch_text = read(LAUNCH)
    assert 'config/coverage_mapping.yaml' in launch_text
    for package in (
        'savo_nav',
        'nav2_bringup',
        'savo_control',
        'savo_base',
        'savo_localization',
    ):
        assert f'pkg="{package}"' not in launch_text


def test_assets_are_installed_linted_and_tests_registered_once() -> None:
    cmake = read(CMAKE)
    expected_counts = {
        'config/coverage_mapping.yaml': 1,
        'config/profiles/coverage_mapping_real_robot.yaml': 1,
        'launch/coverage_mapping.launch.xml': 2,
        'rviz/coverage_mapping.rviz': 1,
    }
    for relative, expected_count in expected_counts.items():
        assert cmake.count(relative) == expected_count, (
            f'{CMAKE}: {relative} registration count is not {expected_count}'
        )
    assert cmake.count('test_coverage_deployment_assets_contract') == 2
    assert cmake.count('test/test_coverage_deployment_assets_contract.py') == 1
    assert cmake.count('test_coverage_launch_runtime') == 3
    assert cmake.count('test/test_coverage_launch_runtime.py') == 1


def test_rviz_is_observer_only_and_displays_the_coverage_path() -> None:
    document = load_yaml(RVIZ)
    manager = document.get('Visualization Manager')
    assert isinstance(manager, dict), f'{RVIZ}: Visualization Manager missing'
    displays = manager.get('Displays', [])
    classes = {
        display.get('Class')
        for display in displays
        if isinstance(display, dict)
    }
    required = {
        'rviz_default_plugins/Grid',
        'rviz_default_plugins/TF',
        'rviz_default_plugins/RobotModel',
        'rviz_default_plugins/Map',
        'rviz_default_plugins/LaserScan',
        'rviz_default_plugins/Odometry',
        'rviz_default_plugins/Path',
    }
    assert required <= classes
    text = read(RVIZ)
    for topic in (
        '/map',
        '/scan',
        '/odometry/filtered',
        '/savo_mapping/coverage/path',
    ):
        assert topic in text, f'{RVIZ}: missing observer topic {topic}'
    tools = {
        tool.get('Class')
        for tool in manager.get('Tools', [])
        if isinstance(tool, dict)
    }
    assert tools == {
        'rviz_default_plugins/Interact',
        'rviz_default_plugins/MoveCamera',
        'rviz_default_plugins/Select',
        'rviz_default_plugins/FocusCamera',
        'rviz_default_plugins/Measure',
    }
    velocity_root = 'cmd' + '_vel'
    forbidden = (
        '/goal_pose',
        '/initialpose',
        '/clicked_point',
        '/' + velocity_root,
        '/' + velocity_root + '_auto',
    )
    for topic in forbidden:
        assert topic not in text, f'{RVIZ}: forbidden output topic {topic}'


def test_no_navigation_or_motion_authority_is_configured() -> None:
    production_text = '\n'.join(read(path) for path in ASSETS).lower()
    forbidden = (
        'navigatetopose',
        'navigatethroughposes',
        'followwaypoints',
        'followpath',
        'computepathtopose',
        'rclcpp_action',
        'geometry_msgs/msg/' + 'tw' + 'ist',
        'cmd' + '_vel',
    )
    for token in forbidden:
        assert token not in production_text, (
            f'Coverage deployment assets must not contain {token}'
        )


def test_obsolete_deferred_asset_assertions_are_removed() -> None:
    asset_names = tuple(str(path.relative_to(ROOT)) for path in ASSETS)
    for contract in MIGRATED_CONTRACTS:
        text = read(contract)
        for asset_name in asset_names:
            assert asset_name not in text, (
                f'{contract}: obsolete deferred assertion remains for '
                f'{asset_name}'
            )
        assert not re.search(
            r'(?:deferred.*coverage|coverage.*deferred)',
            text,
            flags=re.IGNORECASE,
        ), f'{contract}: obsolete deferred Coverage wording remains'


def test_immutable_coverage_tf_and_scan360_hashes_are_unchanged() -> None:
    for relative, expected in IMMUTABLE_HASHES.items():
        path = ROOT / relative
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        assert actual == expected, (
            f'{path}: immutable hash changed; expected {expected}, '
            f'got {actual}'
        )
