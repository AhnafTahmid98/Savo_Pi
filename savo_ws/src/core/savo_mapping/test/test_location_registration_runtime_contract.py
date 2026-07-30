# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""LOC-3P-C mapping-to-location runtime source contracts."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_mapping_runtime_uses_owned_typed_dependencies() -> None:
    """Mapping uses typed owners and never takes persistence or motion."""
    source = (
        ROOT / 'src/nodes/mapped_location_registration_node.cpp'
    ).read_text()
    cmake = (ROOT / 'CMakeLists.txt').read_text()

    assert 'RegisterMappedLocation' in source
    assert '/savo_mapping/locations/register' in source
    assert '/savo_head/apriltag/confirm' in source
    assert '/savo_supervisor/authorize_location_operation' in source
    assert '/savo_locations/candidates/register' in source
    assert 'OP_REGISTER_LOCATION_CANDIDATE' in source
    assert 'SemanticLandmarkRecorder' in source
    assert 'derive_approach_pose' in source
    assert 'RESULT_CANCELED' in source
    assert 'source_component = "savo_mapping"' in source
    assert 'candidate.review_reason.clear();' in source
    assert 'pending_operator_review' not in source

    forbidden = (
        'sqlite3_open',
        'sqlite3_exec',
        '/navigate_to_pose',
        'tag_pose_map = approach_pose',
    )
    for text in forbidden:
        assert text not in source

    assert 'mapped_location_registration_node' in cmake
