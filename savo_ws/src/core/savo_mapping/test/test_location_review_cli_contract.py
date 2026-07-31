# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Operator review CLI ownership and deployment contracts."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_cli_uses_read_only_discovery_and_authorized_review() -> None:
    """The operator CLI cannot call raw registry mutation services."""
    source = (ROOT / 'src/tools/location_review_cli.cpp').read_text()

    for token in (
        'ListLocationCandidates',
        'GetLocationCandidate',
        'ReviewLocationCandidate',
        '/savo_locations/candidates/list',
        '/savo_locations/candidates/get',
        '/savo_mapping/locations/review',
        'DECISION_APPROVE',
        'DECISION_REJECT',
    ):
        assert token in source

    assert 'ApproveLocation' not in source
    assert 'RejectLocationCandidate' not in source
    assert '/savo_locations/candidates/approve' not in source
    assert '/savo_locations/candidates/reject' not in source


def test_cli_supports_review_queue_and_operator_validation() -> None:
    """The CLI exposes deterministic review and validation commands."""
    source = (ROOT / 'src/tools/location_review_cli.cpp').read_text()

    for token in (
        'location_review_cli list',
        'location_review_cli show',
        'location_review_cli inspect',
        'location_review_cli approve',
        'location_review_cli reject',
        '--actor',
        '--reason',
        '--revision',
        '--request-id',
        '--state all|pending|approved|rejected',
        '--map-id ID --map-revision REV',
        '--json',
        '"schema_version", 1',
        'RESULT_CANDIDATE_NOT_FOUND',
        'RESULT_SUPERVISOR_UNAVAILABLE',
        'RESULT_REGISTRY_UNAVAILABLE',
        'RESULT_TIMED_OUT',
        'candidate revision must be non-zero',
        '--reason is required for rejection',
    ):
        assert token in source


def test_cli_and_runtime_test_are_built_and_installed() -> None:
    """The build installs the CLI and runs the gateway integration suite."""
    cmake = (ROOT / 'CMakeLists.txt').read_text()
    runtime = (
        ROOT / 'test/test_location_review_gateway_runtime.py'
    ).read_text()

    for token in (
        'src/tools/location_review_cli.cpp',
        'TARGETS location_review_cli',
        'test_location_review_cli_contract',
        'test_location_review_gateway_runtime',
        'LOCATION_REVIEW_GATEWAY_EXECUTABLE',
        'LOCATION_REVIEW_CLI_EXECUTABLE',
    ):
        assert token in cmake

    for token in (
        'RESULT_APPROVED',
        'RESULT_REJECTED',
        'RESULT_STALE_REVISION',
        'RESULT_SUPERVISOR_DENIED',
        'RESULT_BUSY',
        'RESULT_TIMED_OUT',
        'RESULT_REGISTRY_REJECTED',
        'RESULT_REGISTRY_UNAVAILABLE',
        'run_cli',
    ):
        assert token in runtime
