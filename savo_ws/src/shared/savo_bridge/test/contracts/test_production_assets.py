from __future__ import annotations

from pathlib import Path
import subprocess


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
        'command_dispatcher.require_active_map_context: true',
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
    assert 'ExecStart=' in service
    assert 'ExecStartPre=' not in service


def test_launch_requires_observed_map_context() -> None:
    launch = read('launch/edge_bridge.launch.py')
    normalized_launch = launch.replace(chr(39), chr(34))
    assert 'DeclareLaunchArgument("active_map_id", default_value="")' in normalized_launch
    assert 'DeclareLaunchArgument("active_map_revision", default_value="0")' in normalized_launch
    assert 'default_value="saved_map"' not in normalized_launch
    assert 'executable="savo_bridge_node"' in normalized_launch
