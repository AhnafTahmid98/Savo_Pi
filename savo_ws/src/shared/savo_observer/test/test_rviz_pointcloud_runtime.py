"""Validate temporary PointCloud2 enablement without ROS or an RViz GUI."""

from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from savo_observer.rviz_config import (
    create_pointcloud_runtime_config,
    parse_launch_boolean,
    remove_runtime_config,
)


ROOT = Path(__file__).resolve().parents[1]
RVIZ = ROOT / 'rviz'


def _displays(document):
    return document['Visualization Manager']['Displays']


def _is_pointcloud(display):
    return str(display.get('Class', '')).endswith('/PointCloud2')


def test_launch_boolean_parser_is_explicit_and_fail_closed():
    """Accept normal boolean forms and reject every ambiguous value."""
    for value in (True, 'true', 'TRUE', 'yes', 'on', '1'):
        assert parse_launch_boolean(value, 'enable_pointclouds') is True
    for value in (False, 'false', 'FALSE', 'no', 'off', '0'):
        assert parse_launch_boolean(value, 'enable_pointclouds') is False
    for value in ('', 'enabled', 'truthy', '2', None):
        with pytest.raises(ValueError, match='enable_pointclouds must be'):
            parse_launch_boolean(value, 'enable_pointclouds')


def test_runtime_copies_enable_only_existing_pointcloud_displays():
    """Preserve source files, topics, QoS, images, and unrelated displays."""
    pointcloud_count = 0
    views_without_pointclouds = 0
    pointcloud_topics = set()

    for source_path in sorted(RVIZ.glob('*.rviz')):
        source_bytes = source_path.read_bytes()
        source_document = yaml.safe_load(source_bytes)
        runtime_path, enabled_count = create_pointcloud_runtime_config(
            source_path
        )

        try:
            assert runtime_path != source_path
            assert runtime_path.parent != source_path.parent
            runtime_document = yaml.safe_load(
                runtime_path.read_text(encoding='utf-8')
            )
            source_displays = _displays(source_document)
            runtime_displays = _displays(runtime_document)
            assert len(runtime_displays) == len(source_displays)

            expected_enabled = 0
            for source, runtime in zip(source_displays, runtime_displays):
                expected = deepcopy(source)
                if _is_pointcloud(source):
                    expected_enabled += 1
                    pointcloud_count += 1
                    pointcloud_topics.add(source['Topic']['Value'])
                    expected['Enabled'] = True
                    if 'Value' in expected:
                        expected['Value'] = True
                assert runtime == expected

            assert enabled_count == expected_enabled
            if expected_enabled == 0:
                views_without_pointclouds += 1
        finally:
            remove_runtime_config(runtime_path)

        assert not runtime_path.exists()
        assert source_path.read_bytes() == source_bytes

    assert pointcloud_count > 0
    assert views_without_pointclouds > 0
    assert pointcloud_topics == {
        '/camera/camera/depth/color/points',
        '/savo_perception/obstacles/points',
    }


def test_user_config_enables_pointcloud_without_enabling_image(tmp_path):
    """Apply the same selective behavior to a user-supplied RViz file."""
    source_path = tmp_path / 'user.rviz'
    document = {
        'Visualization Manager': {
            'Displays': [
                {
                    'Class': 'rviz_default_plugins/PointCloud2',
                    'Enabled': False,
                    'Name': 'Cloud',
                    'Topic': {
                        'Reliability Policy': 'Best Effort',
                        'Value': '/custom/cloud',
                    },
                    'Value': False,
                },
                {
                    'Class': 'rviz_default_plugins/Image',
                    'Enabled': False,
                    'Name': 'Image',
                    'Topic': {
                        'Reliability Policy': 'Best Effort',
                        'Value': '/custom/image',
                    },
                    'Value': False,
                },
            ]
        }
    }
    source_path.write_text(
        yaml.safe_dump(document, sort_keys=False),
        encoding='utf-8',
    )
    source_bytes = source_path.read_bytes()
    runtime_path, enabled_count = create_pointcloud_runtime_config(source_path)

    try:
        runtime = yaml.safe_load(runtime_path.read_text(encoding='utf-8'))
        cloud, image = _displays(runtime)
        assert enabled_count == 1
        assert cloud['Enabled'] is True
        assert cloud['Value'] is True
        assert cloud['Topic'] == document['Visualization Manager']['Displays'][
            0
        ]['Topic']
        assert image == document['Visualization Manager']['Displays'][1]
    finally:
        remove_runtime_config(runtime_path)

    assert source_path.read_bytes() == source_bytes
