# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Location review gateway ownership and deployment contracts."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_gateway_uses_authoritative_candidate_and_supervisor() -> None:
    """The gateway authorizes stored candidate context before mutation."""
    source = (
        ROOT / 'src/nodes/location_review_gateway_node.cpp'
    ).read_text()

    for token in (
        'GetLocationCandidate',
        'OP_APPROVE_LOCATION',
        'OP_REJECT_LOCATION_CANDIDATE',
        'candidate.map_id',
        'candidate.map_revision',
        'ApproveLocation',
        'RejectLocationCandidate',
        'RESULT_STALE_REVISION',
        'MultiThreadedExecutor',
    ):
        assert token in source

    assert 'sqlite3_' not in source
    assert '/cmd_vel' not in source
    assert 'NavigateTo' not in source


def test_gateway_has_bounded_fail_closed_dependencies() -> None:
    """Every external dependency is bounded and failures are explicit."""
    source = (
        ROOT / 'src/nodes/location_review_gateway_node.cpp'
    ).read_text()

    for token in (
        'dependency_wait_timeout_s',
        'operation_timeout_s',
        'wait_for_service',
        'wait_until_ready',
        'RESULT_SUPERVISOR_UNAVAILABLE',
        'RESULT_REGISTRY_UNAVAILABLE',
        'RESULT_TIMED_OUT',
        'RESULT_BUSY',
    ):
        assert token in source


def test_semantic_launch_starts_review_gateway() -> None:
    """The semantic mapping entrypoint starts the review gateway."""
    launch = (ROOT / 'launch/semantic_mapping.launch.xml').read_text()
    config = (ROOT / 'config/semantic_landmarks.yaml').read_text()
    cmake = (ROOT / 'CMakeLists.txt').read_text()

    assert 'exec="location_review_gateway_node"' in launch
    assert 'location_review_gateway_node:' in config
    assert 'candidate_lookup_service: /savo_locations/candidates/get' in config
    assert 'authorization_service: /savo_supervisor/authorize_location_operation' in config
    assert 'approval_service: /savo_locations/candidates/approve' in config
    assert 'rejection_service: /savo_locations/candidates/reject' in config
    assert 'src/nodes/location_review_gateway_node.cpp' in cmake
    assert 'test_location_review_gateway_contract.py' in cmake
