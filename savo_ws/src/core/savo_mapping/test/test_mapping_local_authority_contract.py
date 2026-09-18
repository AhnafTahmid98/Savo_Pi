# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Check that autonomous authority has no system-Supervisor transport."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_orchestrator_owns_authority_without_supervisor_client():
    source = (ROOT / 'src/nodes/autonomous_mapping_orchestrator_node.cpp').read_text()
    assert 'local_authority::Authority' in source
    assert 'local_authority::Health' in source
    assert '/savo_supervisor/' not in source
    assert 'create_client<AuthorizeOperation>' not in source
    assert 'supervisor_mapping_authority_lost' not in source
    assert 'local_authority_.acquire(' in source
    assert 'local_authority_.revalidate(' in source
    assert 'local_authority_.release()' in source
    assert 'terminal_stop_confirmed' in source


def test_child_endpoint_is_check_only():
    source = (ROOT / 'src/nodes/autonomous_mapping_orchestrator_node.cpp').read_text()
    assert '/savo_mapping/autonomous/authorize_phase' in source
    assert 'request->command == AuthorizeOperation::Request::COMMAND_CHECK' in source
    assert 'request->expected_generation == authority_generation_' in source


def test_dedicated_launch_enables_direct_health_and_not_system_supervisor():
    source_root = ROOT.parents[1]
    launch = (source_root / 'shared/savo_bringup/launch/autonomous_mapping.launch.py')
    source = launch.read_text()
    assert '"start_supervisor", default_value="false"' in source
    xml = (ROOT / 'launch/autonomous_mapping.launch.xml').read_text()
    assert 'name="local_health.enabled" value="true"' in xml
    assert 'name="local_health.edge_ups_required"' in xml
    assert '"local_health_edge_ups_required": LaunchConfiguration(' in source


def test_autonomous_children_check_scoped_parent_and_standalone_stays_supported():
    for name in ('coverage_operation_orchestrator_node',
                 'mapped_location_registration_node', 'location_review_gateway_node'):
        source = (ROOT / f'src/nodes/{name}.cpp').read_text()
        assert '"mapping_local_authority", false' in source
        assert '/savo_mapping/autonomous/authority' in source
        assert 'same_lease(' in source
    coverage = (ROOT / 'src/nodes/coverage_operation_orchestrator_node.cpp').read_text()
    assert 'COMMAND_CHECK' in coverage
    assert 'OP_RUN_COVERAGE' in coverage


def test_semantic_revalidation_has_explicit_semantic_loss_reason():
    source = (ROOT / 'include/savo_mapping/local_mapping_authority.hpp').read_text()
    assert 'mapping_local_semantic_unavailable' in source
    revalidate = source.split('bool revalidate(', 1)[1].split('void pause()', 1)[0]
    assert 'health.ready(false, false, now)' in revalidate
    assert 'mapping_local_semantic_unavailable' in revalidate
