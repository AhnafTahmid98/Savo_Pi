# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Verify Phase 4L-B3 Coverage admission contracts."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def read(relative_path):
    """Read a package file."""
    return (
        PACKAGE_ROOT / relative_path
    ).read_text(encoding='utf-8')


def load_yaml(relative_path):
    """Load one package YAML file."""
    return yaml.safe_load(read(relative_path))


def test_validator_is_registered_in_core():
    """Ensure the validator is compiled into savo_nav_core."""
    cmake = read('CMakeLists.txt')

    assert 'src/core/coverage_path_validator.cpp' in cmake
    assert 'test_coverage_path_validator' in cmake


def test_savo_msgs_dependency_is_declared():
    """Ensure the custom action package is a dependency."""
    cmake = read('CMakeLists.txt')
    package_xml = read('package.xml')

    assert 'find_package(savo_msgs REQUIRED)' in cmake
    assert '<depend>savo_msgs</depend>' in package_xml


def test_coverage_action_names_are_locked():
    """Ensure public, hidden, and Nav2 names are stable."""
    names = read('include/savo_nav/action_names.hpp')

    assert '/savo_nav/coverage/execute_path' in names
    assert '/savo_nav/_internal/coverage/execute_path' in names
    assert '/follow_path' in names


def test_action_server_manifest_contains_coverage():
    """Ensure Coverage and FollowPath are documented."""
    manifest = load_yaml('config/action_servers.yaml')

    public = manifest['actions']['public']
    internal = manifest['actions']['internal']

    assert (
        public['coverage_execute_path']['type']
        == 'savo_msgs/action/ExecuteCoveragePath'
    )

    assert (
        internal['nav2_follow_path']['type']
        == 'nav2_msgs/action/FollowPath'
    )

    assert (
        manifest['constraints']['maximum_active_nav2_goals']
        == 1
    )


def test_gate_configuration_contains_coverage():
    """Ensure the outer admission gate has both endpoints."""
    config = load_yaml('config/goal_admission_gate.yaml')
    parameters = config['goal_admission_gate_node']['ros__parameters']

    assert (
        parameters['public_coverage_action']
        == '/savo_nav/coverage/execute_path'
    )

    assert (
        parameters['internal_coverage_action']
        == '/savo_nav/_internal/coverage/execute_path'
    )


def test_gateway_configuration_contains_follow_path():
    """Ensure the gateway declares Coverage and FollowPath."""
    config = load_yaml('config/goal_gateway.yaml')
    parameters = config['goal_gateway_node']['ros__parameters']

    assert (
        parameters['coverage_action_name']
        == '/savo_nav/coverage/execute_path'
    )

    assert (
        parameters['nav2_follow_path_action_name']
        == '/follow_path'
    )


def test_launch_loads_gate_configuration_and_remapping():
    """Ensure launch uses the gate YAML and hidden endpoint."""
    launch = read('launch/saved_map_navigation.launch.py')

    assert "'goal_admission_gate.yaml'" in launch
    assert 'parameters=[goal_admission_gate_params]' in launch

    assert (
        "'/savo_nav/coverage/execute_path'"
        in launch
    )

    assert (
        "'/savo_nav/_internal/coverage/execute_path'"
        in launch
    )
