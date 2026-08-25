# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Phase 8A source, ownership, and configuration contracts."""

import ast
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[2]
RAW_TOPIC = '/camera/camera/depth/color/points'
FILTERED_TOPIC = '/savo_perception/obstacles/points'

PHASE_FILES = (
    'include/savo_perception/obstacle_cloud_filter.hpp',
    'include/savo_perception/obstacle_cloud_filter_node.hpp',
    'src/filtering/obstacle_cloud_filter.cpp',
    'src/nodes/obstacle_cloud_filter_node.cpp',
    'config/edge/obstacle_cloud_filter.yaml',
    'launch/obstacle_cloud_filter.launch.py',
    'test/unit/test_obstacle_cloud_filter.cpp',
    'test/contracts/test_phase8a_obstacle_cloud_contracts.py',
    'test/fixtures/obstacle_cloud_filter_fixture.py',
)


def read(relative_path):
    """Read a package file as UTF-8 text."""
    return (PACKAGE / relative_path).read_text(encoding='utf-8')


def test_phase_files_exist_and_have_licenses():
    """Require every Phase 8A artifact and its source license header."""
    for relative_path in PHASE_FILES:
        path = PACKAGE / relative_path
        assert path.is_file()
        assert path.stat().st_size > 0

        if path.suffix in {'.cpp', '.hpp', '.py'}:
            text = path.read_text(encoding='utf-8')
            assert 'Copyright 2026 Ahnaf Tahmid' in text[:200]
            assert 'SPDX-License-Identifier: LicenseRef-Proprietary' in text[:250]


def test_package_dependencies_are_minimal():
    """Require the ROS interfaces without introducing PCL."""
    root = ET.parse(PACKAGE / 'package.xml').getroot()
    dependencies = {
        element.text
        for element in root
        if element.tag in {'depend', 'test_depend'}
    }
    assert {'sensor_msgs', 'tf2', 'tf2_ros'} <= dependencies
    assert {
        'ament_cmake_gtest',
        'ament_cmake_pytest',
        'python3-pytest',
        'python3-yaml',
    } <= dependencies
    assert not any('pcl' in dependency.lower() for dependency in dependencies)


def test_cmake_registers_and_installs_phase_targets():
    """Require the reusable library, node, GTest, and contract test."""
    cmake = read('CMakeLists.txt')
    for token in (
        'savo_perception_obstacle_cloud_filter',
        'obstacle_cloud_filter_node',
        'test_obstacle_cloud_filter',
        'test_phase8a_obstacle_cloud_contracts',
        'config',
        'launch',
    ):
        assert token in cmake


def test_topic_constants_match_yaml_contract():
    """Keep C++ constants and YAML ownership topics identical."""
    header = read('include/savo_perception/topic_names.hpp')
    topics = yaml.safe_load(read('config/topics.yaml'))
    obstacle = topics['topics']['obstacle_cloud']

    assert obstacle['raw_points'] == RAW_TOPIC
    assert obstacle['filtered_points'] == FILTERED_TOPIC
    for topic in obstacle.values():
        assert topic in header

    ownership = topics['ownership']['filtered_obstacle_cloud']
    assert ownership['message_type'] == 'sensor_msgs/msg/PointCloud2'
    assert ownership['semantics'] == 'obstacle_only'
    assert ownership['clearing_supported'] is False


def test_edge_configuration_is_safe_and_ordered():
    """Validate the production parameter bounds and obstacle semantics."""
    config = yaml.safe_load(
        read('config/edge/obstacle_cloud_filter.yaml')
    )['obstacle_cloud_filter_node']['ros__parameters']

    assert config['input_topic'] == RAW_TOPIC
    assert config['output_topic'] == FILTERED_TOPIC
    assert config['output_frame'] == 'base_link'
    assert config['min_range_m'] >= 0.0
    assert config['max_range_m'] > config['min_range_m']
    assert config['max_height_m'] > config['min_height_m']
    assert config['voxel_size_m'] > 0.0
    assert config['self_max_x_m'] > config['self_min_x_m']
    assert config['self_max_y_m'] > config['self_min_y_m']
    assert config['self_max_z_m'] > config['self_min_z_m']
    assert config['max_output_points'] > 0


def test_launch_starts_only_the_filter_node():
    """Keep RealSense ownership out of the perception launch file."""
    path = PACKAGE / 'launch/obstacle_cloud_filter.launch.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    constants = {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    assert 'obstacle_cloud_filter_node' in constants
    assert 'realsense2_camera' not in constants

    core_launch = read('launch/perception_bringup.launch.py')
    assert 'obstacle_cloud_filter_node' not in core_launch


def test_production_source_has_no_forbidden_authority():
    """Reject motion, control, recovery, safety, and hardware publishers."""
    source = read('src/nodes/obstacle_cloud_filter_node.cpp')
    forbidden = (
        '/cmd_vel',
        '/cmd_vel_nav',
        '/cmd_vel_safe',
        '/cmd_vel_recovery',
        '/savo_control/mode_cmd',
        '/savo_control/recovery_request',
        'geometry_msgs::msg::Twist',
    )
    for token in forbidden:
        assert token not in source

    assert RAW_TOPIC not in re.findall(
        r'create_publisher[^;]+;',
        source,
        flags=re.DOTALL,
    )


def test_pointcloud_layout_contract_matches_real_d435_storage():
    """Accept only safe single-row padding and retain field validation."""
    header = read('include/savo_perception/obstacle_cloud_filter.hpp')
    library = read('src/filtering/obstacle_cloud_filter.cpp')
    node = read('src/nodes/obstacle_cloud_filter_node.cpp')
    fixture = read('test/fixtures/obstacle_cloud_filter_fixture.py')
    hardware = read(
        'test/fixtures/obstacle_cloud_filter_hardware_fixture.py'
    )

    assert 'PointCloudStorageLayout' in header
    assert 'layout.height != 1U' in library
    assert 'layout.row_step < minimum_row_step' in library
    assert 'layout.data_size != declared_data_size' in library
    assert 'checked_multiply' in library
    assert 'big_endian_cloud_not_supported' in node
    assert 'unsupported_xyz_field_type' in node
    assert 'xyz_field_outside_point_step' in node
    assert 'point_step - field_offset' in node
    assert 'padded_realsense_cloud' in fixture
    assert 'height != 1 and row_step != minimum_row_step' in hardware
    assert 'filtered output row must be compact' in hardware


def test_readme_documents_filtered_architecture():
    """Require obstacle-only ownership and hardware-pending documentation."""
    readme = read('README.md')
    assert '## Phase 8A obstacle-cloud filtering' in readme
    assert FILTERED_TOPIC in readme
    assert 'obstacle-only' in readme
    assert 'clearing remains false' in readme
    assert 'real D435 hardware validation remains pending' in readme
    assert 'raw RealSense cloud directly into Nav2' not in readme
