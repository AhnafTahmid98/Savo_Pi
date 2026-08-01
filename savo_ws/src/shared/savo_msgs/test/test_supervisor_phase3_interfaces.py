# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_phase3_system_service_is_registered():
    cmake = (ROOT / 'CMakeLists.txt').read_text()
    assert 'srv/ManageSystemState.srv' in cmake


def test_system_service_has_startup_and_shutdown_authority():
    text = (ROOT / 'srv' / 'ManageSystemState.srv').read_text()
    for token in (
        'COMMAND_ARM',
        'COMMAND_DISARM',
        'COMMAND_BEGIN_SHUTDOWN',
        'COMMAND_CLEAR_FAULT_LATCH',
        'uint8 command',
        'fault_latched',
        'shutdown_requested',
        'remote_commands_ready',
        'system_generation',
    ):
        assert token in text
