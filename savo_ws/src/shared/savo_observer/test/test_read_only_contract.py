"""Fail if observer runtime gains a robot mutation interface."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = [ROOT / 'src', ROOT / 'launch', ROOT / 'rviz', ROOT / 'dashboard']
PROHIBITED = {
    'create_client<', 'create_service<', 'rclcpp_action', 'async_send_goal',
    'SetGoal', 'SetInitialPose', 'PublishPoint', 'Teleop',
}


def test_runtime_has_no_service_action_or_unsafe_rviz_api():
    for directory in RUNTIME:
        for path in directory.rglob('*'):
            if (
                not path.is_file()
                or '__pycache__' in path.parts
                or path.suffix in {'.pyc', '.pyo'}
            ):
                continue

            try:
                source = path.read_text(encoding='utf-8')
            except UnicodeDecodeError:
                continue

            for token in PROHIBITED:
                assert token not in source, (path, token)


def test_ros_publishers_are_confined_to_observer_telemetry():
    telemetry = (ROOT / 'src/nodes/observer_telemetry_node.cpp').read_text(
        encoding='utf-8'
    )
    dashboard = (ROOT / 'src/nodes/observer_dashboard_node.cpp').read_text(
        encoding='utf-8'
    )
    assert 'create_publisher<' in telemetry
    assert 'output_namespace + ' in telemetry
    assert 'create_publisher<' not in dashboard
    for forbidden in ('/cmd_vel', '/goal_pose', '/initialpose', '/mode_cmd'):
        assert forbidden not in telemetry
        assert forbidden not in dashboard
