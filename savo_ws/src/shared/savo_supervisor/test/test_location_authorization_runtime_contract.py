# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""LOC-3P-C supervisor authorization runtime source contracts."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_supervisor_exposes_permission_only_service() -> None:
    """Supervisor grants permission without executing owned operations."""
    source = (ROOT / 'src/supervisor_node.cpp').read_text()
    cmake = (ROOT / 'CMakeLists.txt').read_text()

    assert '/savo_supervisor/authorize_location_operation' in source
    assert 'AuthorizeLocationOperation' in source
    assert 'LocationAuthorizationEvaluator' in source
    assert 'OP_NAVIGATE_TO_LOCATION' in source
    assert 'motion_required ||' in source
    assert 'location_authorization_code_to_ros' in source

    for forbidden in (
        'RegisterLocationCandidate',
        'ResolveLocation',
        'NavigateToPose',
        'ConfirmAprilTag',
        'sqlite3_',
    ):
        assert forbidden not in source

    assert 'savo_msgs' in cmake
