# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Source contracts for the Phase 2 mission-authority runtime fixture."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / 'test' / 'runtime'


def test_phase2_runtime_assets_cover_stateful_authority() -> None:
    fixture = (RUNTIME / 'phase2_fixture.py').read_text(encoding='utf-8')
    probe = (RUNTIME / 'phase2_supervisor_probe.py').read_text(encoding='utf-8')
    runner = (RUNTIME / 'run_phase2_mission_authority_test.sh').read_text(
        encoding='utf-8')

    for token in (
        '/savo_mapping/status',
        '/savo_nav/status',
        '/savo_head/status',
        '/savo_locations/status',
        '/savo_mapping/autonomous/run',
        '/savo_control/rotate_to_heading',
        '/savo_nav/coverage/execute_path',
        '/savo_head/apriltag/confirm',
    ):
        assert token in fixture

    for token in (
        'COMMAND_ACQUIRE',
        'COMMAND_RESUME',
        'COMMAND_RELEASE',
        'RESULT_OPERATION_CONFLICT',
        'drop_navigation',
        'safety_stop',
        'REVOKED',
        'COMMAND_SET_SAVED_RELEASE',
        'PHASE_2_MISSION_AUTHORITY_RUNTIME_COMPLETE',
    ):
        assert token in probe

    assert 'ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-226}"' in runner
    assert 'phase2_fixture.py' in runner
    assert 'phase2_supervisor_probe.py' in runner
