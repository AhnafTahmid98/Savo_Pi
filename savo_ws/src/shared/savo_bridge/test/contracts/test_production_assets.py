from __future__ import annotations

from pathlib import Path
import subprocess

import yaml


PACKAGE = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (PACKAGE / relative).read_text(encoding='utf-8')


def test_edge_configuration_uses_guarded_authority() -> None:
    config = read('config/savo_bridge.edge.yaml')

    required = (
        'command_server.execution_mode: live',
        'command_server.socket_mode: 432',
        'command_server.socket_gid: 10001',
        'command_server.allowed_peer_uids:',
        '- 10001',
        'command_dispatcher.navigation_action_name: /savo_nav/navigation/navigate_to_pose',
        'command_dispatcher.location_resolve_service: /savo_locations/resolve',
        'command_dispatcher.active_map_id: ""',
        'command_dispatcher.active_map_revision: 0',
        'command_dispatcher.require_active_map_context: false',
        'snapshot_path: /run/savo_bridge/snapshot.json',
    )
    for marker in required:
        assert marker in config

    forbidden = (
        'command_dispatcher.navigation_action_name: /navigate_to_pose',
        'manual_velocity_topic: /cmd_vel_safe',
        'external_stop_topic: /safety/stop',
    )
    for marker in forbidden:
        assert marker not in config


def test_installer_is_inert_without_explicit_start() -> None:
    installer = read('scripts/install_edge_runtime.sh')
    assert 'START_SERVICE=false' in installer
    assert '--start' in installer
    assert 'systemctl enable --now "${SERVICE_NAME}"' in installer
    assert 'service was not enabled or started' in installer


def test_shell_assets_parse() -> None:
    for relative in (
        'scripts/run_edge_bridge.sh',
        'scripts/install_edge_runtime.sh',
    ):
        result = subprocess.run(
            ['bash', '-n', str(PACKAGE / relative)],
            check=False,
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stderr


def test_systemd_runtime_contract_is_shared_and_fail_closed() -> None:
    service = read('systemd/savo_bridge.service.in')
    assert 'Group=savomind-bridge' in service
    assert 'RuntimeDirectory=savo_bridge' in service
    assert 'RuntimeDirectoryMode=0770' in service
    assert 'UMask=0007' in service
    assert 'ReadWritePaths=/run/savo_bridge' in service
    assert 'Environment=SAVO_BRIDGE_RUNTIME_DIR=/run/savo_bridge' in service
    assert 'ExecStart=' in service
    assert 'ExecStartPre=' not in service


def test_launch_requires_observed_map_context() -> None:
    launch = read('launch/edge_bridge.launch.py')
    normalized_launch = launch.replace(chr(39), chr(34))
    assert 'DeclareLaunchArgument("active_map_id", default_value="")' in normalized_launch
    assert 'DeclareLaunchArgument("active_map_revision", default_value="0")' in normalized_launch
    assert 'default_value="saved_map"' not in normalized_launch
    assert 'executable="savo_bridge_node"' in normalized_launch
    assert 'OpaqueFunction(function=_launch_bridge)' in launch
    assert 'unsupported Bridge robot_mode policy' in launch


def test_edge_launch_resolves_a_private_command_runtime() -> None:
    launch = read('launch/edge_bridge.launch.py')
    assert (
        "DeclareLaunchArgument('runtime_directory', default_value='')"
        in launch
    )
    assert "os.environ.get('SAVO_BRIDGE_RUNTIME_DIR', '')" in launch
    assert "return f'/tmp/savo_bridge-runtime-{os.geteuid()}'" in launch
    assert "runtime_directory, 'command.sock'" in launch
    assert "runtime_directory, 'snapshot.json'" in launch


def test_command_server_parent_validation_remains_fail_closed() -> None:
    source = read('src/command_server.cpp')
    required = (
        'metadata.st_uid != ::geteuid()',
        '(permissions & S_IWOTH) != 0',
        'S_ISLNK(metadata.st_mode)',
        '::mkdir(current.c_str(), S_IRWXU)',
        '::chmod(current.c_str(), S_IRWXU | S_IRWXG)',
        'command_server_parent_directory_unsafe',
    )
    for marker in required:
        assert marker in source


def test_systemd_and_runner_do_not_invent_active_map() -> None:
    service = read('systemd/savo_bridge.service.in')
    runner = read('scripts/run_edge_bridge.sh')
    assert 'SAVO_ACTIVE_MAP_ID' not in service
    assert 'SAVO_ACTIVE_MAP_REVISION' not in service
    assert 'saved_map' not in runner
    assert 'active_map_id:=' not in runner
    assert 'active_map_revision:=' not in runner


def mode_parameters(mode: str) -> dict:
    document = yaml.safe_load(read(f'config/modes/{mode}.yaml'))
    return document['/savo_bridge/savo_bridge_node']['ros__parameters']


def test_mode_observation_contracts_are_fail_closed_and_minimal() -> None:
    config = read('config/savo_bridge.edge.yaml')
    common = yaml.safe_load(config)[
        '/savo_bridge/savo_bridge_node'
    ]['ros__parameters']
    common_topics = {
        '/savo_control/mode_state',
        '/savo_control/external_stop',
        '/safety/stop',
        '/cmd_vel_safe',
    }
    assert set(common['observation_topics']) == common_topics
    assert '/edge_ups_node' not in config
    assert '/edge_bringup_readiness_node' not in config

    bridge_source = read('src/bridge_node.cpp')
    assert 'evidence.core_evidence_configured;' in bridge_source
    assert '!evidence.edge_evidence_configured ||' in bridge_source
    assert 'evidence.edge_visible;' in bridge_source
    assert 'evidence.dds_active &&' in bridge_source
    assert 'health.required_topics_ready &&' in bridge_source

    safe = mode_parameters('safe_idle')
    assert set(safe['observation_topics']) == common_topics
    assert safe['command_dispatcher.require_active_map_context'] is False

    manual = mode_parameters('manual_mapping')
    assert set(manual['observation_topics']) == common_topics | {
        '/savo_mapping/status'
    }
    assert '/savo_nav/readiness' not in manual['observation_topics']
    assert '/savo_nav/map_context/status' not in manual['observation_topics']

    autonomous = mode_parameters('autonomous_mapping')
    assert set(autonomous['observation_topics']) == common_topics | {
        '/savo_mapping/status',
        '/savo_mapping/autonomous/status',
        '/savo_nav/readiness',
    }
    assert '/savo_nav/map_context/status' not in autonomous[
        'observation_topics'
    ]
    assert autonomous['command_dispatcher.require_active_map_context'] is False

    saved = mode_parameters('saved_map_navigation')
    assert set(saved['observation_topics']) == common_topics | {
        '/savo_nav/readiness',
        '/savo_nav/map_context/status',
    }
    assert saved['command_dispatcher.require_active_map_context'] is True

    for parameters in (common, safe, manual, autonomous, saved):
        topics = parameters['observation_topics']
        assert len(topics) == len(parameters['observation_requirements'])
        assert len(topics) == len(parameters['observation_stale_after_ms'])
        assert set(parameters['observation_requirements']) == {'required'}
        assert '/savo_speech/readiness' not in topics
        assert all('/savo_ui' not in topic for topic in topics)


def test_optional_graph_evidence_arrays_use_typed_cpp_defaults() -> None:
    config = read('config/savo_bridge.edge.yaml')
    common = yaml.safe_load(config)[
        '/savo_bridge/savo_bridge_node'
    ]['ros__parameters']
    bridge_source = read('src/bridge_node.cpp')

    optional_arrays = (
        'core_evidence_nodes',
        'edge_evidence_nodes',
        'edge_evidence_topics',
    )
    for parameter_name in optional_arrays:
        # ROS 2 Jazzy parses an untyped YAML [] as PARAMETER_NOT_SET. Omitting
        # optional arrays lets the typed C++ declaration supply an empty vector.
        assert parameter_name not in common
        assert common.get(parameter_name, []) == []
        assert (
            f'"{parameter_name}",\n'
            '    std::vector<std::string>{}'
        ) in bridge_source

    assert common['core_evidence_topics'] == [
        '/savo_control/mode_state'
    ]
    assert '/edge_ups_node' not in config
    assert '/edge_bringup_readiness_node' not in config


def test_saved_map_readiness_uses_fresh_synchronized_runtime_context() -> None:
    source = read('src/bridge_node.cpp')
    for marker in (
        'dispatcher_snapshot.map_context_synchronized',
        'dispatcher_snapshot.map_context_observed',
        'dispatcher_snapshot.map_context_age_ms',
        'dispatcher_snapshot.active_map_id.empty()',
        'dispatcher_snapshot.active_map_revision > 0U',
    ):
        assert marker in source
