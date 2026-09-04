from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (PACKAGE / relative).read_text(encoding='utf-8')


def test_mapping_commands_are_typed_and_quality_gated() -> None:
    protocol = read('src/command_protocol.cpp')
    assert '"start_autonomous_mapping"' in protocol
    assert '"request_scan360"' in protocol
    assert 'auto_save and operator quality approval' in protocol
    assert 'require_quality_approval' in protocol
    assert 'require_semantic' in protocol


def test_mapping_action_requires_exact_supervisor_lease_first() -> None:
    dispatcher = read('src/ros_command_dispatcher.cpp')
    for token in (
        'COMMAND_ACQUIRE',
        'OP_START_AUTONOMOUS_MAPPING',
        'bridge_supervisor_authorization_rejected',
        'goal.authority_request_id',
        'goal.authority_generation',
        'goal.require_semantic',
    ):
        assert token in dispatcher


def test_status_queries_cover_navigation_mapping_and_supervisor() -> None:
    protocol = read('src/command_protocol.cpp')
    dispatcher = read('src/ros_command_dispatcher.cpp')
    for token in (
        'query_navigation_state',
        'query_mapping_state',
        'query_supervisor_state',
    ):
        assert token in protocol
    for field in (
        '"map_save"',
        '"verification"',
        '"locations"',
        '"review"',
        '"release"',
        '"release_id"',
    ):
        assert field in dispatcher


def test_bridge_has_no_generic_ros_escape_hatch() -> None:
    readme = read('README.md')
    assert 'no generic ROS' in readme
    assert 'arbitrary poses' in readme
    assert 'Operator approval is never accepted from SavoMind' in readme


def test_production_runner_does_not_invent_active_map() -> None:
    service = read('systemd/savo_bridge.service.in')
    runner = read('scripts/run_edge_bridge.sh')
    corpus = service + runner
    assert 'SAVO_ACTIVE_MAP_ID=saved_map' not in corpus
    assert 'SAVO_ACTIVE_MAP_REVISION=1' not in corpus
