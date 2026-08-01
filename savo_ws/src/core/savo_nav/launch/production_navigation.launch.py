# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Launch saved-map navigation from a verified active AM-8 release."""

import hashlib
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


_REQUIRED_JOINT_ARTIFACTS = {
    'map_yaml',
    'map_image',
    'posegraph',
    'data',
    'quality_report',
    'source_manifest',
    'location_snapshot',
    'location_snapshot_metadata',
    'operator_approval',
    'geometry_profile',
}


def _sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(65536), b''):
            digest.update(block)
    return digest.hexdigest()


def _load_yaml(path, label):
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f'{label}_missing_or_empty:{path}')
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict):
        raise RuntimeError(f'{label}_must_be_mapping:{path}')
    return document


def _required_string(document, key, label):
    value = document.get(key)
    if not isinstance(value, str) or not value.strip():
        raise RuntimeError(f'{label}_missing:{key}')
    return value.strip()


def _resolved_regular_file(path, label):
    try:
        resolved = path.resolve(strict=True)
    except (FileNotFoundError, OSError) as exception:
        raise RuntimeError(f'{label}_missing:{path}') from exception
    if not resolved.is_file() or resolved.stat().st_size == 0:
        raise RuntimeError(f'{label}_missing_or_empty:{resolved}')
    return resolved


def _require_descendant(path, root, label):
    resolved_path = path.resolve(strict=True)
    resolved_root = root.resolve(strict=True)
    try:
        resolved_path.relative_to(resolved_root)
    except ValueError as exception:
        raise RuntimeError(f'{label}_outside_release_root:{resolved_path}') from exception
    return resolved_path


def _verify_artifacts(manifest, release_directory):
    artifact_nodes = manifest.get('artifacts')
    if not isinstance(artifact_nodes, list):
        raise RuntimeError('active_release_artifacts_missing')

    artifacts = {}
    for artifact in artifact_nodes:
        if not isinstance(artifact, dict):
            raise RuntimeError('active_release_artifact_invalid')

        role = _required_string(artifact, 'role', 'active_release_artifact')
        if role in artifacts:
            raise RuntimeError(f'active_release_duplicate_artifact:{role}')

        relative_path = Path(
            _required_string(artifact, 'path', 'active_release_artifact')
        )
        if relative_path.is_absolute() or '..' in relative_path.parts:
            raise RuntimeError(f'active_release_artifact_path_invalid:{role}')

        artifact_path = _require_descendant(
            release_directory / relative_path,
            release_directory,
            f'active_release_artifact_{role}',
        )
        artifact_path = _resolved_regular_file(
            artifact_path,
            f'active_release_artifact_{role}',
        )

        size_bytes = artifact.get('size_bytes')
        if not isinstance(size_bytes, int) or size_bytes <= 0:
            raise RuntimeError(f'active_release_artifact_size_invalid:{role}')
        if artifact_path.stat().st_size != size_bytes:
            raise RuntimeError(f'active_release_artifact_size_mismatch:{role}')

        expected_sha = _required_string(
            artifact,
            'sha256',
            'active_release_artifact',
        )
        if len(expected_sha) != 64 or _sha256(artifact_path) != expected_sha:
            raise RuntimeError(f'active_release_artifact_hash_mismatch:{role}')

        artifacts[role] = artifact_path

    missing = sorted(_REQUIRED_JOINT_ARTIFACTS.difference(artifacts))
    if missing:
        raise RuntimeError(
            'active_release_required_artifacts_missing:' + ','.join(missing)
        )
    return artifacts


def _resolve_active_release(
    active_contract_path,
    geometry_profile_path,
    production_root,
):
    production_root = production_root.resolve(strict=True)
    releases_root = (production_root / 'releases').resolve(strict=True)
    active_contract_path = _resolved_regular_file(
        active_contract_path,
        'active_map_contract',
    )
    active = _load_yaml(active_contract_path, 'active_map_contract')

    if active.get('schema_version') != 1:
        raise RuntimeError('active_map_schema_version_unsupported')
    if active.get('active') is not True:
        raise RuntimeError('no_active_production_map_release')

    map_id = _required_string(active, 'map_id', 'active_map_contract')
    release_id = _required_string(active, 'release_id', 'active_map_contract')
    frame_id = _required_string(active, 'frame_id', 'active_map_contract')
    if frame_id != 'map':
        raise RuntimeError('active_map_frame_must_be_map')

    release_directory = _require_descendant(
        Path(
            _required_string(
                active,
                'release_directory',
                'active_map_contract',
            )
        ),
        releases_root,
        'active_release_directory',
    )
    if release_directory.parent != releases_root:
        raise RuntimeError('active_release_directory_must_be_direct_child')
    if release_directory.name != release_id:
        raise RuntimeError('active_release_directory_id_mismatch')

    map_yaml = _resolved_regular_file(
        Path(_required_string(active, 'map_yaml', 'active_map_contract')),
        'active_map_yaml',
    )
    release_manifest = _resolved_regular_file(
        Path(
            _required_string(
                active,
                'release_manifest',
                'active_map_contract',
            )
        ),
        'active_release_manifest',
    )
    quality_report = _resolved_regular_file(
        Path(
            _required_string(
                active,
                'quality_report',
                'active_map_contract',
            )
        ),
        'active_quality_report',
    )

    for path, label in (
        (map_yaml, 'active_map_yaml'),
        (release_manifest, 'active_release_manifest'),
        (quality_report, 'active_quality_report'),
    ):
        _require_descendant(path, release_directory, label)

    if release_manifest != release_directory / 'release_manifest.yaml':
        raise RuntimeError('active_release_manifest_path_invalid')

    expected_manifest_sha = _required_string(
        active,
        'release_manifest_sha256',
        'active_map_contract',
    )
    if len(expected_manifest_sha) != 64:
        raise RuntimeError('active_release_manifest_sha256_invalid')
    if _sha256(release_manifest) != expected_manifest_sha:
        raise RuntimeError('active_release_manifest_sha256_mismatch')

    manifest = _load_yaml(release_manifest, 'release_manifest')
    if manifest.get('schema_version') != 2:
        raise RuntimeError('joint_release_manifest_v2_required')
    if manifest.get('immutable') is not True:
        raise RuntimeError('active_release_must_be_immutable')
    if manifest.get('release_id') != release_id:
        raise RuntimeError('active_release_id_mismatch')
    if manifest.get('map_id') != map_id:
        raise RuntimeError('active_release_map_id_mismatch')
    if manifest.get('frame_id') != 'map':
        raise RuntimeError('active_release_frame_mismatch')

    map_revision = manifest.get('map_revision')
    if not isinstance(map_revision, int) or map_revision <= 0:
        raise RuntimeError('active_release_map_revision_invalid')

    navigation = manifest.get('navigation')
    quality = manifest.get('quality')
    if not isinstance(navigation, dict) or navigation.get('eligible') is not True:
        raise RuntimeError('active_release_not_navigation_eligible')
    if (
        not isinstance(quality, dict)
        or quality.get('passed') is not True
        or quality.get('operator_approved') is not True
    ):
        raise RuntimeError('active_release_quality_not_approved')

    artifacts = _verify_artifacts(manifest, release_directory)
    if map_yaml != artifacts['map_yaml']:
        raise RuntimeError('active_map_yaml_artifact_mismatch')
    if quality_report != artifacts['quality_report']:
        raise RuntimeError('active_quality_report_artifact_mismatch')

    quality_document = _load_yaml(quality_report, 'quality_report')
    if quality_document.get('passed') is not True:
        raise RuntimeError('active_quality_report_not_passed')
    if quality_document.get('map_id') != map_id:
        raise RuntimeError('active_quality_report_map_id_mismatch')

    map_document = _load_yaml(map_yaml, 'map_yaml')
    image_value = _required_string(map_document, 'image', 'map_yaml')
    map_image = _require_descendant(
        map_yaml.parent / image_value,
        release_directory,
        'active_map_image',
    )
    map_image = _resolved_regular_file(map_image, 'active_map_image')
    if map_image != artifacts['map_image']:
        raise RuntimeError('active_map_image_artifact_mismatch')

    geometry_profile_path = _resolved_regular_file(
        geometry_profile_path,
        'geometry_profile',
    )
    geometry = _load_yaml(geometry_profile_path, 'geometry_profile')
    metadata = geometry.get('metadata')
    if not isinstance(metadata, dict):
        raise RuntimeError('geometry_profile_metadata_missing')
    if metadata.get('measurement_state') != 'locked':
        raise RuntimeError('geometry_profile_not_locked')
    if metadata.get('profile_id') != manifest.get('geometry_profile_name'):
        raise RuntimeError('geometry_profile_name_mismatch')

    geometry_digest = manifest.get('geometry_profile_digest')
    if not isinstance(geometry_digest, str) or len(geometry_digest) != 64:
        raise RuntimeError('geometry_profile_digest_invalid')
    if _sha256(geometry_profile_path) != geometry_digest:
        raise RuntimeError('geometry_profile_digest_mismatch')
    if _sha256(artifacts['geometry_profile']) != geometry_digest:
        raise RuntimeError('release_geometry_profile_digest_mismatch')

    location_digest = manifest.get('location_snapshot_digest')
    if not isinstance(location_digest, str) or len(location_digest) != 64:
        raise RuntimeError('location_snapshot_digest_invalid')
    if _sha256(artifacts['location_snapshot']) != location_digest:
        raise RuntimeError('location_snapshot_digest_mismatch')

    return {
        'map_yaml': str(map_yaml),
        'map_id': map_id,
        'map_revision': map_revision,
        'map_release_id': release_id,
    }


def _launch_verified_release(context):
    production_root = Path(
        LaunchConfiguration('production_map_root').perform(context)
    )
    active_contract_value = LaunchConfiguration(
        'active_map_contract'
    ).perform(context)
    active_contract_path = (
        Path(active_contract_value)
        if active_contract_value
        else production_root / 'active_map.yaml'
    )
    geometry_profile_path = Path(
        LaunchConfiguration('geometry_profile').perform(context)
    )

    release = _resolve_active_release(
        active_contract_path,
        geometry_profile_path,
        production_root,
    )

    package_share = FindPackageShare('savo_nav').perform(context)
    saved_map_launch = (
        Path(package_share) / 'launch' / 'saved_map_navigation.launch.py'
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(saved_map_launch)),
        launch_arguments={
            'map': release['map_yaml'],
            'map_id': release['map_id'],
            'map_revision': str(release['map_revision']),
            'map_release_id': release['map_release_id'],
            'params_file': LaunchConfiguration('params_file'),
            'readiness_params': LaunchConfiguration('readiness_params'),
            'autostart': LaunchConfiguration('autostart'),
            'start_readiness': LaunchConfiguration('start_readiness'),
            'start_goal_gateway': LaunchConfiguration('start_goal_gateway'),
            'require_map_context_sync': 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    map_context = Node(
        package='savo_nav',
        executable='active_map_context_node',
        name='active_map_context_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_map_context_sync')),
        parameters=[
            LaunchConfiguration('map_context_params'),
            {
                'map_id': release['map_id'],
                'map_revision': release['map_revision'],
                'map_release_id': release['map_release_id'],
            },
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            LaunchConfiguration('log_level'),
        ],
    )

    return [navigation, map_context]


def generate_launch_description():
    """Build the fail-closed production navigation launch description."""
    nav_share = FindPackageShare('savo_nav')
    description_share = FindPackageShare('savo_description')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'production_map_root',
                default_value='/var/lib/robot_savo/maps/production',
                description='AM-8 production map release root.',
            ),
            DeclareLaunchArgument(
                'active_map_contract',
                default_value='',
                description=(
                    'Optional active_map.yaml path; defaults to the '
                    'production root.'
                ),
            ),
            DeclareLaunchArgument(
                'geometry_profile',
                default_value=PathJoinSubstitution(
                    [
                        description_share,
                        'config',
                        'profiles',
                        'robot_savo_core_v1.yaml',
                    ]
                ),
                description='Locked geometry profile matching the release.',
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=PathJoinSubstitution(
                    [nav_share, 'config', 'nav2_saved_map.yaml']
                ),
                description='Nav2 parameter profile.',
            ),
            DeclareLaunchArgument(
                'readiness_params',
                default_value=PathJoinSubstitution(
                    [nav_share, 'config', 'readiness.yaml']
                ),
                description='Navigation readiness profile.',
            ),
            DeclareLaunchArgument(
                'map_context_params',
                default_value=PathJoinSubstitution(
                    [nav_share, 'config', 'active_map_context.yaml']
                ),
                description='Supervisor map-context synchronization profile.',
            ),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('autostart', default_value='true'),
            DeclareLaunchArgument('start_readiness', default_value='true'),
            DeclareLaunchArgument('start_goal_gateway', default_value='true'),
            DeclareLaunchArgument(
                'start_map_context_sync',
                default_value='true',
            ),
            DeclareLaunchArgument('log_level', default_value='info'),
            OpaqueFunction(function=_launch_verified_release),
        ]
    )
