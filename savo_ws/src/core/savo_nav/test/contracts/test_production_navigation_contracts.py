# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Validate fail-closed startup from an immutable AM-8 joint release."""

import ast
import hashlib
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / 'launch/production_navigation.launch.py'
MAP_CONTEXT_SOURCE = ROOT / 'src/nodes/active_map_context_node.cpp'
MAP_CONTEXT_CONFIG = ROOT / 'config/active_map_context.yaml'
CMAKE = ROOT / 'CMakeLists.txt'


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _verifier_namespace():
    """Load only the launch file's ROS-independent verification helpers."""
    tree = ast.parse(LAUNCH.read_text(encoding='utf-8'))
    keep_functions = {
        '_sha256',
        '_load_yaml',
        '_required_string',
        '_resolved_regular_file',
        '_require_descendant',
        '_verify_artifacts',
        '_resolve_active_release',
    }
    selected = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            names = {alias.name for alias in node.names}
            if names <= {'hashlib', 'yaml'}:
                selected.append(node)
        elif isinstance(node, ast.ImportFrom) and node.module == 'pathlib':
            selected.append(node)
        elif isinstance(node, ast.Assign):
            selected.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name in keep_functions:
            selected.append(node)
    module = ast.Module(body=selected, type_ignores=[])
    namespace = {}
    exec(compile(module, str(LAUNCH), 'exec'), namespace)
    return namespace


def _write(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding='utf-8')
    return path


def _build_release(tmp_path, measurement_state='locked'):
    production_root = tmp_path / 'production'
    release_id = 'campus-main-r7'
    release_dir = production_root / 'releases' / release_id
    release_dir.mkdir(parents=True)

    geometry_text = yaml.safe_dump(
        {
            'metadata': {
                'profile_id': 'robot_savo_core_v1',
                'measurement_state': measurement_state,
            }
        },
        sort_keys=False,
    )
    current_geometry = _write(tmp_path / 'current_geometry.yaml', geometry_text)

    files = {
        'map_image': _write(release_dir / 'map.pgm', 'P2\n1 1\n255\n0\n'),
        'map_yaml': _write(
            release_dir / 'map.yaml',
            'image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n',
        ),
        'posegraph': _write(release_dir / 'map.posegraph', 'posegraph\n'),
        'data': _write(release_dir / 'map.data', 'data\n'),
        'quality_report': _write(
            release_dir / 'quality.yaml',
            'passed: true\nmap_id: campus_main\n',
        ),
        'source_manifest': _write(
            release_dir / 'source_manifest.yaml',
            'verified: true\n',
        ),
        'location_snapshot': _write(
            release_dir / 'locations.json',
            '{"locations": []}\n',
        ),
        'location_snapshot_metadata': _write(
            release_dir / 'locations_metadata.json',
            '{"count": 0}\n',
        ),
        'operator_approval': _write(
            release_dir / 'operator_approval.yaml',
            'approved: true\n',
        ),
        'geometry_profile': _write(
            release_dir / 'geometry.yaml',
            geometry_text,
        ),
    }

    artifacts = []
    for role, path in files.items():
        artifacts.append(
            {
                'role': role,
                'path': path.name,
                'size_bytes': path.stat().st_size,
                'sha256': _sha256(path),
            }
        )

    manifest = {
        'schema_version': 2,
        'release_id': release_id,
        'map_id': 'campus_main',
        'map_revision': 7,
        'frame_id': 'map',
        'immutable': True,
        'quality': {'passed': True, 'operator_approved': True},
        'navigation': {'eligible': True},
        'geometry_profile_name': 'robot_savo_core_v1',
        'geometry_profile_digest': _sha256(files['geometry_profile']),
        'location_snapshot_digest': _sha256(files['location_snapshot']),
        'artifacts': artifacts,
    }
    manifest_path = _write(
        release_dir / 'release_manifest.yaml',
        yaml.safe_dump(manifest, sort_keys=False),
    )
    active = {
        'schema_version': 1,
        'active': True,
        'release_id': release_id,
        'map_id': 'campus_main',
        'frame_id': 'map',
        'release_directory': str(release_dir),
        'release_manifest': str(manifest_path),
        'release_manifest_sha256': _sha256(manifest_path),
        'map_yaml': str(files['map_yaml']),
        'quality_report': str(files['quality_report']),
    }
    active_path = _write(
        production_root / 'active_map.yaml',
        yaml.safe_dump(active, sort_keys=False),
    )
    return production_root, active_path, current_geometry, files


def test_verified_joint_release_resolves_real_map_identity(tmp_path):
    """Accept a complete immutable joint release and return its identity."""
    production_root, active, geometry, _ = _build_release(tmp_path)
    resolve = _verifier_namespace()['_resolve_active_release']
    result = resolve(active, geometry, production_root)
    assert result == {
        'map_yaml': str(
            production_root / 'releases/campus-main-r7/map.yaml'
        ),
        'map_id': 'campus_main',
        'map_revision': 7,
        'map_release_id': 'campus-main-r7',
    }


def test_release_artifact_tampering_is_rejected(tmp_path):
    """Re-hash immutable artifacts on every production startup."""
    production_root, active, geometry, files = _build_release(tmp_path)
    files['map_image'].write_text('tampered\n', encoding='utf-8')
    resolve = _verifier_namespace()['_resolve_active_release']
    with pytest.raises(RuntimeError, match='artifact_(size|hash)_mismatch'):
        resolve(active, geometry, production_root)


def test_provisional_geometry_is_rejected(tmp_path):
    """Keep unrestricted production navigation behind AM-0B geometry lock."""
    production_root, active, geometry, _ = _build_release(
        tmp_path,
        measurement_state='provisional',
    )
    resolve = _verifier_namespace()['_resolve_active_release']
    with pytest.raises(RuntimeError, match='geometry_profile_not_locked'):
        resolve(active, geometry, production_root)


def test_production_launch_wires_release_identity_and_context_sync():
    """Pass the verified release to Nav2, gateway, and supervisor."""
    source = LAUNCH.read_text(encoding='utf-8')
    assert "'map_release_id': release['map_release_id']" in source
    assert "'map_revision': str(release['map_revision'])" in source
    assert "executable='active_map_context_node'" in source
    assert "default_value='/var/lib/robot_savo/maps/production'" in source


def test_active_map_context_node_uses_typed_supervisor_service():
    """Require retrying, correlated map-context synchronization."""
    source = MAP_CONTEXT_SOURCE.read_text(encoding='utf-8')
    assert 'savo_msgs/srv/update_map_context.hpp' in source
    assert 'COMMAND_SET_SAVED_RELEASE' in source
    assert 'request->map_release_id = map_release_id_' in source
    assert 'response->active_map_release_id == map_release_id_' in source
    assert 'waiting_for_supervisor_service' in source
    assert 'service_was_ready_' in source

    config = yaml.safe_load(MAP_CONTEXT_CONFIG.read_text(encoding='utf-8'))
    parameters = config['active_map_context_node']['ros__parameters']
    assert parameters['supervisor_service'] == (
        '/savo_supervisor/update_map_context'
    )
    assert parameters['approved'] is True

    cmake = CMAKE.read_text(encoding='utf-8')
    assert 'active_map_context_node' in cmake


def test_production_path_enforces_release_to_nav2_gate_chain():
    """Require every fail-closed stage before a goal can reach Nav2."""
    launch_source = LAUNCH.read_text(encoding='utf-8')
    readiness_source = (
        ROOT / 'src/nodes/navigation_readiness_node.cpp'
    ).read_text(encoding='utf-8')
    gateway_source = (
        ROOT / 'src/nodes/goal_gateway_node.cpp'
    ).read_text(encoding='utf-8')
    action_names = (
        ROOT / 'include/savo_nav/action_names.hpp'
    ).read_text(encoding='utf-8')

    assert 'release = _resolve_active_release(' in launch_source
    assert 'geometry_profile_not_locked' in launch_source
    assert "executable='active_map_context_node'" in launch_source
    assert "'require_map_context_sync': 'true'" in launch_source
    assert "'map_release_id': release['map_release_id']" in launch_source
    assert 'map_context_synchronized_' in readiness_source
    assert 'policy.require_readiness = true' in gateway_source
    assert 'goal_acceptance_allowed' in gateway_source
    assert '/savo_nav/_internal/navigation/navigate_to_pose' in action_names
    assert 'kNav2NavigateToPose' in gateway_source
