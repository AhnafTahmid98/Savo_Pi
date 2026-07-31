"""Contracts for the package-local autonomous mapping launch."""

from pathlib import Path
import re
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch/autonomous_mapping.launch.xml'


def test_autonomous_mapping_launch_is_valid_and_complete() -> None:
    """The launch composes every mapping-owned AM-1 through AM-5 process."""
    tree = ET.parse(LAUNCH)
    root = tree.getroot()

    arguments = {
        element.attrib['name'] for element in root.findall('arg')
    }
    assert {
        'map_id',
        'map_output_root',
        'allow_map_overwrite',
        'use_sim_time',
        'slam_autostart',
        'slam_params_file',
        'mapping_profile_file',
        'frontier_params_file',
        'handoff_params_file',
        'exploration_manager_params_file',
        'orchestrator_params_file',
        'scan360_params_file',
        'scan360_profile_file',
        'scan360_use_real_robot_profile',
        'map_frame',
        'base_frame',
    }.issubset(arguments)

    executables = {
        element.attrib['exec'] for element in root.findall('node')
    }
    assert {
        'mapping_mode_manager_node',
        'mapping_supervisor_node',
        'slam_lifecycle_health_bridge_node',
    }.issubset(executables)

    includes = {
        element.attrib['file'] for element in root.findall('include')
    }
    assert any('online_async_launch.py' in value for value in includes)
    assert any('map_session_manager.launch.xml' in value for value in includes)
    assert any('frontier_mapping.launch.xml' in value for value in includes)
    assert any('scan360_mapping.launch.xml' in value for value in includes)
    assert any(
        'autonomous_mapping_orchestrator.launch.xml' in value
        for value in includes
    )


def test_autonomous_launch_starts_safe_and_preserves_ownership() -> None:
    """Launch starts idle and does not absorb hardware or Nav2 ownership."""
    text = LAUNCH.read_text(encoding='utf-8')

    assert 'name="authority.initial_mode"' in text
    assert 'value="monitor_only"' in text
    assert 'name="authority.initial_session_state"' in text
    assert 'value="idle"' in text
    assert 'name="enabled"' in text
    assert 'value="true"' in text

    forbidden_packages = {
        'pkg="savo_base"',
        'pkg="savo_control"',
        'pkg="savo_lidar"',
        'pkg="savo_localization"',
        'pkg="savo_nav"',
        'pkg="savo_perception"',
        'pkg="savo_power"',
        'pkg="savo_supervisor"',
    }
    for token in forbidden_packages:
        assert token not in text

    assert '/savo_mapping/map_session/save' not in text
    assert '/navigate_to_pose' not in text
    assert 'name="auto_start"' in text
    assert 'value="false"' in text


def test_all_launch_variables_are_declared() -> None:
    """Every XML launch variable has a top-level declaration."""
    text = LAUNCH.read_text(encoding='utf-8')
    tree = ET.parse(LAUNCH)
    declared = {
        element.attrib['name'] for element in tree.getroot().findall('arg')
    }

    referenced = set(re.findall(r'\$\(var ([a-zA-Z0-9_]+)\)', text))
    assert referenced <= declared
