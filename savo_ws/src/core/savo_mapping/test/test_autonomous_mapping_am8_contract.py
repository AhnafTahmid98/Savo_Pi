"""Source-level AM-8 ownership and fail-closed release contracts."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ORCHESTRATOR = (
    ROOT / 'src' / 'nodes' / 'autonomous_mapping_orchestrator_node.cpp'
)


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding='utf-8')


def test_mission_has_ordered_am8_states_and_results() -> None:
    header = read('include/savo_mapping/autonomous_mapping_mission.hpp')
    source = read('src/workflow/autonomous_mapping_mission.cpp')
    for token in (
        'VerifyingLocations = 23',
        'AwaitingApproval = 24',
        'Releasing = 25',
        'LocationVerificationFailed = 12',
        'OperatorRejected = 13',
        'ReleaseFailed = 14',
        'ReleaseRollbackFailed = 15',
        'GeometryInvalid = 16',
    ):
        assert token in header
    verifying_location_branch = source.index(
        'if (snapshot_.state == MissionState::VerifyingLocations)'
    )
    assert source.index('MissionState::VerifyingLocations') < source.index(
        'MissionState::AwaitingApproval', verifying_location_branch
    )
    assert 'request_joint_release' in source
    assert 'request_release_rollback' in source


def test_orchestrator_uses_typed_location_authority_only() -> None:
    source = ORCHESTRATOR.read_text(encoding='utf-8')
    for token in (
        'ListLocationCandidates',
        'ListLocations',
        'PrepareLocationRelease',
        'VerifyLocationRelease',
        'CommitLocationRelease',
        'RollbackLocationRelease',
        '"/savo_mapping/autonomous/review_release"',
        'location_snapshot_sha256',
        'active_joint_release.yaml',
        'discover_incomplete_release_transactions',
        'attempt_startup_release_recovery',
    ):
        assert token in source
    assert 'sqlite3' not in source.lower()
    assert 'cmd_vel' not in source
    assert 'NavigateToPose' in source  # guarded AM-7 return-to-start only


def test_release_artifacts_bind_approval_locations_and_geometry() -> None:
    source = read('src/session/production_map_release.cpp')
    for token in (
        '"location_snapshot"',
        '"location_snapshot_metadata"',
        '"operator_approval"',
        '"geometry_profile"',
        'manifest["location_snapshot_digest"]',
        'manifest["geometry_profile_digest"]',
        'manifest["approval_timestamp_unix_ns"]',
        'discard_unpromoted_release',
    ):
        assert token in source


def test_launch_wires_locked_geometry_into_orchestrator() -> None:
    orchestrator_launch = read(
        'launch/autonomous_mapping_orchestrator.launch.xml'
    )
    parent_launch = read('launch/autonomous_mapping.launch.xml')
    for text in (orchestrator_launch, parent_launch):
        assert 'geometry_profile' in text
        assert 'require_locked_geometry' in text
        assert 'allow_provisional_geometry' in text
    assert 'default="true"' in orchestrator_launch
    assert 'default="false"' in orchestrator_launch
