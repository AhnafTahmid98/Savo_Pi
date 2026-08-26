"""Validate temporary high-bandwidth RViz enablement without ROS or a GUI."""

from copy import deepcopy
from pathlib import Path

import pytest
from savo_observer.rviz_config import (
    create_runtime_config,
    parse_launch_boolean,
    remove_runtime_config,
)
import yaml


ROOT = Path(__file__).resolve().parents[1]
RVIZ = ROOT / 'rviz'


def _displays(document):
    return document['Visualization Manager']['Displays']


def _display_kind(display):
    class_name = str(display.get('Class', ''))
    if class_name.endswith('/Image'):
        return 'images'
    if class_name.endswith('/PointCloud2'):
        return 'pointclouds'
    return None


def test_launch_boolean_parser_is_explicit_and_fail_closed():
    """Accept normal boolean forms and reject every ambiguous value."""
    for name in ('enable_camera_preview', 'enable_pointclouds'):
        for value in (True, 'true', 'TRUE', 'yes', 'on', '1'):
            assert parse_launch_boolean(value, name) is True
        for value in (False, 'false', 'FALSE', 'no', 'off', '0'):
            assert parse_launch_boolean(value, name) is False
        for value in ('', 'enabled', 'truthy', '2', None):
            with pytest.raises(ValueError, match=f'{name} must be'):
                parse_launch_boolean(value, name)


@pytest.mark.parametrize(
    ('enable_camera_preview', 'enable_pointclouds'),
    [(False, False), (True, False), (False, True), (True, True)],
)
def test_runtime_copy_enables_only_requested_display_classes(
    enable_camera_preview,
    enable_pointclouds,
):
    """Cover default-disabled, camera-only, cloud-only, and combined modes."""
    requested = {
        'images': enable_camera_preview,
        'pointclouds': enable_pointclouds,
    }
    totals = {'images': 0, 'pointclouds': 0}
    topics = {'images': set(), 'pointclouds': set()}

    for source_path in sorted(RVIZ.glob('*.rviz')):
        source_bytes = source_path.read_bytes()
        source_document = yaml.safe_load(source_bytes)
        runtime_path, enabled_counts = create_runtime_config(
            source_path,
            enable_camera_preview=enable_camera_preview,
            enable_pointclouds=enable_pointclouds,
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

            expected_counts = {'images': 0, 'pointclouds': 0}
            for source, runtime in zip(source_displays, runtime_displays):
                expected = deepcopy(source)
                kind = _display_kind(source)
                if kind is not None:
                    totals[kind] += 1
                    topics[kind].add(source['Topic']['Value'])
                    if requested[kind]:
                        expected_counts[kind] += 1
                        expected['Enabled'] = True
                        if 'Value' in expected:
                            expected['Value'] = True
                assert runtime == expected

            assert enabled_counts == expected_counts
        finally:
            remove_runtime_config(runtime_path)

        assert not runtime_path.exists()
        assert source_path.read_bytes() == source_bytes

    assert totals['images'] > 0
    assert totals['pointclouds'] > 0
    assert topics['images'] == {
        '/camera/camera/color/image_raw',
        '/savo_head/camera/image_raw',
    }
    assert topics['pointclouds'] == {
        '/camera/camera/depth/color/points',
        '/savo_perception/obstacles/points',
    }


def test_user_config_combines_options_without_changing_topics_or_qos(tmp_path):
    """Apply both independent flags to a user-supplied RViz file."""
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
    runtime_path, enabled_counts = create_runtime_config(
        source_path,
        enable_camera_preview=True,
        enable_pointclouds=True,
    )

    try:
        runtime = yaml.safe_load(runtime_path.read_text(encoding='utf-8'))
        cloud, image = _displays(runtime)
        assert enabled_counts == {'images': 1, 'pointclouds': 1}
        assert cloud['Enabled'] is True
        assert cloud['Value'] is True
        assert image['Enabled'] is True
        assert image['Value'] is True
        for index, display in enumerate((cloud, image)):
            assert display['Topic'] == document['Visualization Manager'][
                'Displays'
            ][index]['Topic']
    finally:
        remove_runtime_config(runtime_path)

    assert source_path.read_bytes() == source_bytes
