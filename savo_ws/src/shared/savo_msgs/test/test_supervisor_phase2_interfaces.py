# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_phase2_services_are_registered():
    cmake = (ROOT / 'CMakeLists.txt').read_text()
    assert 'srv/AuthorizeOperation.srv' in cmake
    assert 'srv/UpdateMapContext.srv' in cmake


def test_authorize_operation_contract_is_typed():
    text = (ROOT / 'srv' / 'AuthorizeOperation.srv').read_text()
    for token in (
        'COMMAND_ACQUIRE',
        'COMMAND_RELEASE',
        'COMMAND_PAUSE',
        'COMMAND_RESUME',
        'OP_START_AUTONOMOUS_MAPPING',
        'OP_RUN_SCAN360',
        'OP_RUN_COVERAGE',
        'OP_NAVIGATE_TO_LOCATION',
        'authority_generation',
    ):
        assert token in text


def test_map_context_contract_distinguishes_live_and_saved():
    text = (ROOT / 'srv' / 'UpdateMapContext.srv').read_text()
    assert 'COMMAND_SET_LIVE_MAPPING' in text
    assert 'COMMAND_SET_SAVED_RELEASE' in text
    assert 'map_release_id' in text
    assert 'approved' in text
