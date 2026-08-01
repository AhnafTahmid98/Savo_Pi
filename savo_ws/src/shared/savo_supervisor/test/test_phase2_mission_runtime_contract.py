# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / 'src' / 'supervisor_node.cpp'


def test_phase2_supervisor_contracts_are_wired():
    source = NODE.read_text()
    for token in (
        '/savo_supervisor/authorize_operation',
        '/savo_supervisor/update_map_context',
        '/savo_mapping/status',
        '/savo_nav/status',
        '/savo_nav/heartbeat',
        '/savo_head/status',
        '/savo_locations/status',
        '/savo_locations/heartbeat',
        'operation_authorization_revoked',
        'can_start_autonomous_mapping',
        'semantic_mapping_ready',
    ):
        assert token in source


def test_phase2_is_permission_only():
    source = NODE.read_text()
    forbidden_publishers = (
        '/cmd_vel',
        '/cmd_vel_nav',
        '/savo_mapping/mode_cmd',
        '/savo_mapping/start_session_cmd',
    )
    for topic in forbidden_publishers:
        assert f'create_publisher<{topic}' not in source
