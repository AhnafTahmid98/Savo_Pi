import hashlib
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]

BASE_CONFIG = ROOT / 'config/scan360_mapping.yaml'
ROBOT_PROFILE = ROOT / 'config/profiles/scan360_real_robot.yaml'
LAUNCH = ROOT / 'launch/scan360_mapping.launch.xml'
RVIZ = ROOT / 'rviz/scan360_mapping.rviz'
CMAKE = ROOT / 'CMakeLists.txt'

ASSETS = (
    BASE_CONFIG,
    ROBOT_PROFILE,
    LAUNCH,
    RVIZ,
)

NODE_PARAMETERS = {
    'enabled',
    'auto_start',
    'action_name',
    'odom_topic',
    'odom_frame',
    'status_topic',
    'state_topic',
    'start_service',
    'cancel_service',
    'yaw_stale_timeout_sec',
    'sweep_angle_rad',
    'step_angle_rad',
    'direction',
    'settle_duration_sec',
    'rotation_max_duration_sec',
    'tick_period_sec',
    'server_wait_timeout_sec',
    'goal_response_timeout_sec',
    'feedback_stale_timeout_sec',
    'cancel_timeout_sec',
    'execution_grace_timeout_sec',
}

IMMUTABLE_HASHES = {
    'src/nodes/scan360_mapper_node.cpp':
        '755b0906a5f2009323b6567691f7d845bc0dc43892fd56668a19a5f4579675ab',
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
    'include/savo_mapping/scan360_quality.hpp':
        '1c1119275a9b76b3edc90ec38e963ae2008cd096d71b86d86b5b3a28f928b61a',
    'src/scan360/scan360_quality.cpp':
        '2494a38c6c6cd33db50f1eaff0460223cd32516d1eff809aed51966f840fe678',
    'test/test_scan360_quality.cpp':
        '7075dea04abc7e737617ba3241dddc1599122c9c015970bc1f23cf37cb017c49',
    'package.xml':
        'd5151971a13db517151b2fa7014ab926cbadf8733b9281b1b39a79ba8ae261cd',
}


def read(path: Path) -> str:
    return path.read_text(encoding='utf-8')


def load_yaml(path: Path):
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def node_parameters(path: Path) -> dict:
    document = load_yaml(path)
    assert isinstance(document, dict), f'{path}: YAML root must be a mapping'
    assert set(document) == {'scan360_mapper_node'}, (
        f'{path}: expected only the scan360_mapper_node root'
    )
    node = document['scan360_mapper_node']
    assert set(node) == {'ros__parameters'}, (
        f'{path}: expected only ros__parameters below the node root'
    )
    parameters = node['ros__parameters']
    assert isinstance(parameters, dict), (
        f'{path}: ros__parameters must be a mapping'
    )
    return parameters


def test_four_deployment_assets_exist_and_are_non_empty() -> None:
    for path in ASSETS:
        assert path.is_file(), f'{path}: deployment asset is missing'
        assert path.stat().st_size > 0, f'{path}: deployment asset is empty'


def test_base_configuration_matches_the_real_node_contract() -> None:
    parameters = node_parameters(BASE_CONFIG)
    assert set(parameters) == NODE_PARAMETERS, (
        f'{BASE_CONFIG}: keys differ from the declared node parameters; '
        f'missing={NODE_PARAMETERS - set(parameters)}, '
        f'extra={set(parameters) - NODE_PARAMETERS}'
    )
    assert parameters['enabled'] is True
    assert parameters['auto_start'] is False
    assert parameters['action_name'] == '/savo_control/rotate_to_heading'
    assert parameters['odom_topic'] == '/odometry/filtered'
    assert parameters['odom_frame'] == 'odom'
    assert parameters['status_topic'] == '/savo_mapping/scan360/status'
    assert parameters['state_topic'] == '/savo_mapping/scan360/state'
    assert parameters['start_service'] == '/savo_mapping/scan360/start'
    assert parameters['cancel_service'] == '/savo_mapping/scan360/cancel'
    assert parameters['direction'] in {'clockwise', 'counter_clockwise'}

    positive = NODE_PARAMETERS - {
        'enabled',
        'auto_start',
        'action_name',
        'odom_topic',
        'odom_frame',
        'status_topic',
        'state_topic',
        'start_service',
        'cancel_service',
        'direction',
        'settle_duration_sec',
        'execution_grace_timeout_sec',
    }
    for name in positive:
        value = parameters[name]
        assert isinstance(value, (int, float)) and math.isfinite(value), (
            f'{BASE_CONFIG}: {name} must be finite'
        )
        assert value > 0.0, f'{BASE_CONFIG}: {name} must be positive'

    for name in ('settle_duration_sec', 'execution_grace_timeout_sec'):
        value = parameters[name]
        assert isinstance(value, (int, float)) and math.isfinite(value), (
            f'{BASE_CONFIG}: {name} must be finite'
        )
        assert value >= 0.0, f'{BASE_CONFIG}: {name} must be non-negative'

    assert 0.01 <= parameters['tick_period_sec'] <= 1.0
    assert parameters['sweep_angle_rad'] <= 2.0 * math.pi + 1.0e-9
    assert parameters['step_angle_rad'] <= 2.0 * math.pi


def test_real_robot_profile_is_a_safe_conservative_overlay() -> None:
    parameters = node_parameters(ROBOT_PROFILE)
    assert parameters['auto_start'] is False, (
        f'{ROBOT_PROFILE}: loading the profile must not start rotation'
    )
    assert set(parameters) <= NODE_PARAMETERS, (
        f'{ROBOT_PROFILE}: profile contains undeclared node parameters'
    )
    forbidden_control_terms = (
        'motor',
        'pwm',
        'velocity',
        'acceleration',
        'twist',
    )
    lowered = read(ROBOT_PROFILE).lower()
    for term in forbidden_control_terms:
        assert term not in lowered, (
            f'{ROBOT_PROFILE}: profile must not own {term} configuration'
        )


def test_launch_is_safe_reusable_and_uses_supported_interfaces() -> None:
    root = ET.parse(LAUNCH).getroot()
    assert root.tag == 'launch', f'{LAUNCH}: XML root must be launch'
    arguments = {
        element.attrib['name']: element.attrib.get('default')
        for element in root.findall('arg')
    }
    required = {
        'params_file',
        'profile_file',
        'use_real_robot_profile',
        'enabled',
        'auto_start',
        'action_name',
        'odom_topic',
        'odom_frame',
        'use_sim_time',
    }
    assert required <= set(arguments), f'{LAUNCH}: launch arguments missing'
    assert arguments['auto_start'] == 'false'
    assert arguments['use_real_robot_profile'] == 'false'
    assert arguments['action_name'] == '/savo_control/rotate_to_heading'

    nodes = root.findall('node')
    assert len(nodes) == 2, (
        f'{LAUNCH}: expected mutually exclusive base and profile branches'
    )
    assert {node.attrib.get('if') for node in nodes} == {
        None,
        '$(var use_real_robot_profile)',
    }
    assert {node.attrib.get('unless') for node in nodes} == {
        None,
        '$(var use_real_robot_profile)',
    }
    for node in nodes:
        for name, expected in {
            'pkg': 'savo_mapping',
            'exec': 'scan360_mapper_node',
            'name': 'scan360_mapper_node',
            'output': 'screen',
        }.items():
            assert node.attrib.get(name) == expected, (
                f'{LAUNCH}: conditional node branch has wrong {name}'
            )
    profile_branches = [
        node for node in nodes
        if node.attrib.get('if') == '$(var use_real_robot_profile)'
    ]
    assert len(profile_branches) == 1
    assert any(
        parameter.attrib.get('from') == '$(var profile_file)'
        for parameter in profile_branches[0].findall('param')
    ), f'{LAUNCH}: real-robot branch must load the profile'
    launch_text = read(LAUNCH)
    assert 'config/scan360_mapping.yaml' in launch_text
    assert '$(var action_name)' in launch_text
    for forbidden_package in ('savo_control', 'savo_base'):
        package_pattern = f'pkg="{forbidden_package}"'
        assert package_pattern not in launch_text, (
            f'{LAUNCH}: must not launch {forbidden_package}'
        )
    assert 'motor' not in launch_text.lower()


def test_assets_are_installed_and_registered_exactly_once() -> None:
    cmake = read(CMAKE)
    expected = {
        'config/scan360_mapping.yaml':
            'share/${PROJECT_NAME}/config',
        'config/profiles/scan360_real_robot.yaml':
            'share/${PROJECT_NAME}/config/profiles',
        'launch/scan360_mapping.launch.xml':
            'share/${PROJECT_NAME}/launch',
        'rviz/scan360_mapping.rviz':
            'share/${PROJECT_NAME}/rviz',
    }
    for relative_path, destination in expected.items():
        assert cmake.count(relative_path) == (
            2 if relative_path == 'launch/scan360_mapping.launch.xml' else 1
        ), (
            f'{CMAKE}: {relative_path} has an unexpected registration count'
        )
        asset_index = cmake.index(relative_path)
        destination_index = cmake.index(destination, asset_index)
        assert destination_index > asset_index, (
            f'{CMAKE}: {relative_path} is not installed to {destination}'
        )
    assert cmake.count('test_scan360_deployment_assets_contract') == 2
    assert cmake.count('test_scan360_launch_runtime') == 3


def test_rviz_is_observer_only_and_uses_authoritative_topics() -> None:
    document = load_yaml(RVIZ)
    assert isinstance(document, dict), f'{RVIZ}: RViz YAML must parse'
    manager = document.get('Visualization Manager')
    assert isinstance(manager, dict), f'{RVIZ}: Visualization Manager missing'
    classes = {
        display.get('Class')
        for display in manager.get('Displays', [])
        if isinstance(display, dict)
    }
    required = {
        'rviz_default_plugins/Grid',
        'rviz_default_plugins/TF',
        'rviz_default_plugins/RobotModel',
        'rviz_default_plugins/Map',
        'rviz_default_plugins/LaserScan',
        'rviz_default_plugins/Odometry',
    }
    assert required <= classes, f'{RVIZ}: required observer displays missing'
    text = read(RVIZ)
    for topic in (
        '/robot_description',
        '/map',
        '/scan',
        '/odometry/filtered',
    ):
        assert topic in text, f'{RVIZ}: missing observer topic {topic}'

    tool_classes = {
        tool.get('Class')
        for tool in manager.get('Tools', [])
        if isinstance(tool, dict)
    }
    assert tool_classes == {
        'rviz_default_plugins/Interact',
        'rviz_default_plugins/MoveCamera',
        'rviz_default_plugins/Select',
        'rviz_default_plugins/FocusCamera',
        'rviz_default_plugins/Measure',
    }, f'{RVIZ}: tool list must remain observer-only'

    forbidden_topics = (
        '/goal_pose',
        '/initialpose',
        '/clicked_point',
        '/' + 'cmd' + '_vel',
        '/' + 'cmd' + '_vel_auto',
    )
    for topic in forbidden_topics:
        assert topic not in text, f'{RVIZ}: forbidden output topic {topic}'


def test_no_motion_authority_or_production_goal_exists_in_tests() -> None:
    source_files = (
        Path(__file__),
        ROOT / 'test/test_scan360_launch_runtime.py',
    )
    velocity_token = 'cmd' + '_vel'
    twist_token = 'Twi' + 'st'
    production_endpoint = '/savo_control/' + 'rotate_to_heading'
    for path in source_files:
        text = read(path)
        assert velocity_token not in text, (
            f'{path}: test source names a velocity command interface'
        )
        assert twist_token not in text, (
            f'{path}: test source names a direct motion message'
        )
    assert production_endpoint not in read(
        ROOT / 'test/test_scan360_launch_runtime.py'
    ), 'launch runtime test must never name the production action endpoint'


def test_obsolete_scaffold_contracts_are_removed_only_where_required() -> None:
    contracts = (
        ROOT / 'test/test_scan360_mapper_node_contract.py',
        ROOT / 'test/test_scan360_orchestrator_contract.py',
        ROOT / 'test/test_scan360_rotate_action_client_contract.py',
        ROOT / 'test/test_scan360_rotate_action_binding_contract.py',
    )
    for path in contracts:
        text = read(path)
        for asset in ASSETS:
            assert str(asset.relative_to(ROOT)) not in text, (
                f'{path}: obsolete zero-byte assertion remains for {asset}'
            )
        assert not re.search(
            r'stat\(\)\.st_size\s*==\s*0',
            text,
        ), f'{path}: obsolete Scan360 zero-byte assertion remains'


def test_immutable_scan360_implementation_hashes_are_unchanged() -> None:
    for relative_path, expected_hash in IMMUTABLE_HASHES.items():
        path = ROOT / relative_path
        actual_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        assert actual_hash == expected_hash, (
            f'{path}: immutable hash changed; expected {expected_hash}, '
            f'got {actual_hash}'
        )
