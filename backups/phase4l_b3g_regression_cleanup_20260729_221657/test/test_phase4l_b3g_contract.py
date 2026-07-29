"""Phase 4L-B3G unified Coverage operation contracts."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / 'src/nodes/coverage_operation_orchestrator_node.cpp'
CONFIG = ROOT / 'config/coverage_operation_orchestrator.yaml'
LAUNCH = ROOT / 'launch/coverage_operation_orchestrator.launch.xml'
HANDOFF_LAUNCH = ROOT / 'launch/coverage_execution_handoff.launch.xml'


def read(path: Path) -> str:
    """Read one source asset."""
    return path.read_text(encoding='utf-8')


def test_orchestrator_has_no_motion_or_action_authority():
    """B3G must only proxy approval services."""
    text = read(NODE)
    forbidden = (
        'async_send_goal',
        'create_client<ExecuteCoveragePath',
        'geometry_msgs::msg::Twist',
        '/cmd_vel',
        'nav2_msgs',
    )
    for token in forbidden:
        assert token not in text


def test_public_and_internal_approval_endpoints_are_distinct():
    """The public operator service must not expose B3F directly."""
    text = read(CONFIG)
    assert '/savo_mapping/coverage_operation/approve' in text
    assert '/savo_mapping/_internal/coverage_execution/approve' in text
    assert text.count('/savo_mapping/coverage_operation/approve') == 1
    assert text.count('/savo_mapping/_internal/coverage_execution/approve') == 1


def test_supervisor_is_a_required_approval_input():
    """B3G consumes the monitor-only supervisor state."""
    text = read(NODE)
    assert 'parse_supervisor_authorization' in text
    assert 'supervisor_authorized' in text
    assert 'coverage_operation_supervisor_not_authorized' in text


def test_path_publication_cannot_call_internal_approve():
    """The orchestrator never subscribes to the Coverage path."""
    text = read(NODE)
    assert 'nav_msgs::msg::Path' not in text
    assert '/savo_mapping/coverage/path' not in text
    assert 'handle_public_approve' in text


def test_supervisor_loss_routes_to_cancel_not_motion():
    """Authorization loss can only request B3F cancellation."""
    text = read(NODE)
    assert 'cancel_on_supervisor_loss' in text
    assert 'internal_cancel_client_' in text
    assert 'supervisor_loss_cancel_requested' in text


def test_launch_and_config_are_installable_assets():
    """B3G owns explicit launch and parameter files."""
    assert LAUNCH.exists()
    assert CONFIG.exists()
    assert '<node' in read(LAUNCH)
    assert 'coverage_operation_orchestrator_node' in read(LAUNCH)
    assert 'approve_service' in read(CONFIG)


def test_b3f_handoff_still_has_parameterized_service_names():
    """Bringup can hide the original B3F Trigger services."""
    text = read(HANDOFF_LAUNCH)
    assert 'approve_service' in text
    assert 'cancel_service' in text
    assert 'reset_service' in text


def test_handoff_status_is_retained_for_late_joining_orchestrator():
    """B3G must receive the latest B3F candidate snapshot."""
    handoff_node = (
        ROOT / 'src/nodes/coverage_execution_handoff_node.cpp'
    ).read_text(encoding='utf-8')
    orchestrator = read(NODE)
    assert 'status_topic_,' in handoff_node
    assert 'reliable().transient_local()' in handoff_node
    assert 'handoff_status_topic_' in orchestrator
    assert 'latched_qos' in orchestrator
