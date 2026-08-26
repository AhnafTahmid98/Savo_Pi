"""Validate the complete read-only RViz view set."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
RVIZ = ROOT / 'rviz'
TOPICS = ROOT / 'config' / 'topics.yaml'
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


def _display_topics(document):
    return {
        display.get('Name'): display.get('Topic', {}).get('Value')
        for display in document['Visualization Manager']['Displays']
    }


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
                assert display.get('Value') is False, (path, display.get('Name'))
                assert topic.get('Reliability Policy') == 'Best Effort', path


def test_full_debug_has_complete_approved_spatial_context():
    document = yaml.safe_load(
        (RVIZ / 'full_debug.rviz').read_text(encoding='utf-8')
    )
    assert _display_topics(document) == {
        'RobotModel': None,
        'TF': None,
        'Map': '/map',
        'LaserScan': '/scan',
        'FilteredOdometry': '/odometry/filtered',
        'WheelOdometry': '/wheel/odom',
        'VisualOdometry': '/vo/odom',
        'RawDepthCloud': '/camera/camera/depth/color/points',
        'FilteredObstacleCloud': '/savo_perception/obstacles/points',
        'D435ColorImage': '/camera/camera/color/image_raw',
        'HeadCameraImage': '/savo_head/camera/image_raw',
        'GlobalCostmap': '/global_costmap/costmap',
        'LocalCostmap': '/local_costmap/costmap',
        'GlobalPlan': '/plan',
        'Footprint': '/local_costmap/published_footprint',
        'CoveragePath': '/savo_mapping/coverage/path',
        'ExplorationSelectedGoal': (
            '/savo_mapping/exploration/selected_goal'
        ),
    }


def test_navigation_and_sensor_views_have_read_only_context():
    navigation = yaml.safe_load(
        (RVIZ / 'navigation.rviz').read_text(encoding='utf-8')
    )
    navigation_topics = _display_topics(navigation)
    assert navigation_topics['RobotModel'] is None
    assert navigation_topics['TF'] is None
    assert navigation_topics['FilteredOdometry'] == '/odometry/filtered'
    assert navigation_topics['FilteredObstacleCloud'] == (
        '/savo_perception/obstacles/points'
    )

    sensors = yaml.safe_load(
        (RVIZ / 'sensors.rviz').read_text(encoding='utf-8')
    )
    sensor_topics = _display_topics(sensors)
    assert sensor_topics['DepthPointCloud'] == (
        '/camera/camera/depth/color/points'
    )
    assert sensor_topics['FilteredObstacleCloud'] == (
        '/savo_perception/obstacles/points'
    )
    assert sensor_topics['D435ColorImage'] == (
        '/camera/camera/color/image_raw'
    )
    assert sensor_topics['HeadCameraImage'] == '/savo_head/camera/image_raw'


def test_selected_exploration_goal_is_present_in_mapping_views():
    for name in (
        'mapping_overview.rviz',
        'autonomous_mapping.rviz',
        'full_debug.rviz',
    ):
        document = yaml.safe_load((RVIZ / name).read_text(encoding='utf-8'))
        displays = document['Visualization Manager']['Displays']
        selected_goal = next(
            display
            for display in displays
            if display.get('Name') == 'ExplorationSelectedGoal'
        )
        assert selected_goal['Class'].endswith('/Pose')
        assert selected_goal['Topic']['Value'] == (
            '/savo_mapping/exploration/selected_goal'
        )


def test_observer_uses_only_approved_spatial_topics():
    source = '\n'.join(
        path.read_text(encoding='utf-8')
        for path in [*sorted(RVIZ.glob('*.rviz')), TOPICS]
    )
    assert '/savo_nav/markers' not in source
    assert '/savo_nav/current_goal' not in source
    assert '/savo_realsense/status' not in source

    catalog = yaml.safe_load(TOPICS.read_text(encoding='utf-8'))
    assert catalog['spatial_topics'] == {
        'map': '/map',
        'map_updates': '/map_updates',
        'scan': '/scan',
        'odometry_filtered': '/odometry/filtered',
        'wheel_odometry': '/wheel/odom',
        'visual_odometry': '/vo/odom',
        'global_costmap': '/global_costmap/costmap',
        'local_costmap': '/local_costmap/costmap',
        'global_plan': '/plan',
        'footprint': '/local_costmap/published_footprint',
        'realsense_points': '/camera/camera/depth/color/points',
        'filtered_obstacles': '/savo_perception/obstacles/points',
        'coverage_path': '/savo_mapping/coverage/path',
        'exploration_selected_goal': (
            '/savo_mapping/exploration/selected_goal'
        ),
        'd435_color_image': '/camera/camera/color/image_raw',
        'head_camera_image': '/savo_head/camera/image_raw',
    }


def test_tf_view_includes_fixed_sensors_and_dynamic_head_chain():
    source = (RVIZ / 'tf.rviz').read_text(encoding='utf-8')
    for frame in (
        'base_footprint', 'base_link', 'laser_frame', 'imu_link',
        'camera_link', 'tof_left_link', 'tof_right_link',
        'ultrasonic_front_link', 'pantilt_mount_link', 'pantilt_pan_link',
        'pantilt_tilt_link', 'pi_camera_link', 'pi_camera_optical_frame',
    ):
        assert f'{frame}:' in source
