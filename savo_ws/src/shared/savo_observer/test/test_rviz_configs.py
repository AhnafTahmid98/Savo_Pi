"""Validate the complete read-only RViz view set."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
RVIZ = ROOT / 'rviz'
EXPECTED = {
    'overview.rviz', 'robot_model.rviz', 'tf.rviz', 'sensors.rviz',
    'safety.rviz', 'localization.rviz', 'mapping_overview.rviz',
    'manual_mapping.rviz', 'autonomous_mapping.rviz', 'coverage_mapping.rviz',
    'scan360_mapping.rviz', 'map_quality.rviz', 'navigation.rviz',
    'costmaps.rviz', 'locations_apriltags.rviz', 'full_debug.rviz',
}
UNSAFE_TOOLS = {'SetGoal', 'SetInitialPose', 'PublishPoint', 'Teleop'}


def _documents():
    for path in sorted(RVIZ.glob('*.rviz')):
        yield path, yaml.safe_load(path.read_text(encoding='utf-8'))


def test_view_set_is_complete_and_nonempty():
    assert {path.name for path in RVIZ.glob('*.rviz')} == EXPECTED
    assert all(path.stat().st_size > 0 for path in RVIZ.glob('*.rviz'))


def test_rviz_documents_have_valid_roots_and_safe_tools():
    for path, document in _documents():
        assert isinstance(document, dict), path
        assert 'Panels' in document, path
        manager = document.get('Visualization Manager')
        assert isinstance(manager, dict), path
        assert manager.get('Global Options', {}).get('Fixed Frame') in {
            'map', 'odom', 'base_footprint', 'base_link'
        }, path
        tools = manager.get('Tools', [])
        for tool in tools:
            class_name = tool.get('Class', '').rsplit('/', 1)[-1]
            assert class_name not in UNSAFE_TOOLS, (path, class_name)


def test_topics_and_qos_are_explicit_and_high_bandwidth_is_disabled():
    for path, document in _documents():
        displays = document['Visualization Manager'].get('Displays', [])
        for display in displays:
            class_name = display.get('Class', '')
            topic = display.get('Topic')
            if isinstance(topic, dict):
                assert topic.get('Value'), (path, display.get('Name'))
            if class_name.endswith('/Map') and isinstance(topic, dict):
                assert topic.get('Durability Policy') == 'Transient Local', path
            if class_name.endswith('/LaserScan') and isinstance(topic, dict):
                assert topic.get('Reliability Policy') == 'Best Effort', path
            if class_name.endswith(('/PointCloud2', '/Image')):
                assert display.get('Enabled') is False, (path, display.get('Name'))


def test_tf_view_includes_fixed_sensors_and_dynamic_head_chain():
    source = (RVIZ / 'tf.rviz').read_text(encoding='utf-8')
    for frame in (
        'base_footprint', 'base_link', 'laser_frame', 'imu_link',
        'camera_link', 'tof_left_link', 'tof_right_link',
        'ultrasonic_front_link', 'pantilt_mount_link', 'pantilt_pan_link',
        'pantilt_tilt_link', 'pi_camera_link', 'pi_camera_optical_frame',
    ):
        assert f'{frame}:' in source
