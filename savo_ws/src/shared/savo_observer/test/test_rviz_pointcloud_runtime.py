"""Validate temporary high-bandwidth RViz enablement without ROS or a GUI."""

from copy import deepcopy
from pathlib import Path

import pytest
from savo_observer.rviz_config import (
    create_runtime_config,
    D435_COMPRESSED_IMAGE_TOPIC,
    D435_IMAGE_TOPICS,
    D435_OBSERVER_IMAGE_BASE_TOPIC,
    D435_RAW_IMAGE_TOPIC,
    parse_d435_image_transport,
    parse_launch_boolean,
    RAW_D435_POINTCLOUD_TOPIC,
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
        if display['Topic']['Value'] == RAW_D435_POINTCLOUD_TOPIC:
            return 'raw_d435_pointclouds'
        return 'pointclouds'
    return None


def test_launch_boolean_parser_is_explicit_and_fail_closed():
    """Accept normal boolean forms and reject every ambiguous value."""
    for name in (
        'enable_camera_preview',
        'enable_pointclouds',
        'enable_raw_d435_pointcloud',
    ):
        for value in (True, 'true', 'TRUE', 'yes', 'on', '1'):
            assert parse_launch_boolean(value, name) is True
        for value in (False, 'false', 'FALSE', 'no', 'off', '0'):
            assert parse_launch_boolean(value, name) is False
        for value in ('', 'enabled', 'truthy', '2', None):
            with pytest.raises(ValueError, match=f'{name} must be'):
                parse_launch_boolean(value, name)


def test_d435_image_transport_parser_is_explicit_and_fail_closed():
    """Accept raw/compressed and reject unsupported transport modes."""
    assert D435_OBSERVER_IMAGE_BASE_TOPIC == (
        '/savo_observer/d435/color/image_raw'
    )
    assert D435_COMPRESSED_IMAGE_TOPIC == (
        '/savo_observer/d435/color/image_raw/compressed'
    )
    assert D435_RAW_IMAGE_TOPIC == '/camera/camera/color/image_raw'
    assert D435_IMAGE_TOPICS == {
        'compressed': D435_COMPRESSED_IMAGE_TOPIC,
        'raw': D435_RAW_IMAGE_TOPIC,
    }
    assert parse_d435_image_transport('raw') == 'raw'
    assert parse_d435_image_transport('RAW') == 'raw'
    assert parse_d435_image_transport('compressed') == 'compressed'
    assert parse_d435_image_transport('COMPRESSED') == 'compressed'
    for value in ('', 'theora', 'auto', 'jpeg', None):
        with pytest.raises(ValueError, match='must be raw or compressed'):
            parse_d435_image_transport(value)


@pytest.mark.parametrize(
    (
        'enable_camera_preview',
        'enable_pointclouds',
        'enable_raw_d435_pointcloud',
    ),
    [
        (False, False, False),
        (True, False, False),
        (False, True, False),
        (False, False, True),
        (True, True, False),
        (True, False, True),
        (False, True, True),
        (True, True, True),
    ],
)
@pytest.mark.parametrize('d435_image_transport', ['raw', 'compressed'])
def test_runtime_copy_enables_only_requested_display_classes(
    enable_camera_preview,
    enable_pointclouds,
    enable_raw_d435_pointcloud,
    d435_image_transport,
):
    """Cover independent camera, filtered-cloud, and raw-cloud options."""
    requested = {
        'images': enable_camera_preview,
        'pointclouds': enable_pointclouds,
        'raw_d435_pointclouds': enable_raw_d435_pointcloud,
    }
    totals = {
        'images': 0,
        'pointclouds': 0,
        'raw_d435_pointclouds': 0,
    }
    topics = {kind: set() for kind in totals}

    for source_path in sorted(RVIZ.glob('*.rviz')):
        source_bytes = source_path.read_bytes()
        source_document = yaml.safe_load(source_bytes)
        runtime_path, enabled_counts = create_runtime_config(
            source_path,
            enable_camera_preview=enable_camera_preview,
            enable_pointclouds=enable_pointclouds,
            enable_raw_d435_pointcloud=enable_raw_d435_pointcloud,
            d435_image_transport=d435_image_transport,
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

            expected_counts = {kind: 0 for kind in totals}
            for source, runtime in zip(source_displays, runtime_displays):
                expected = deepcopy(source)
                kind = _display_kind(source)
                if kind is not None:
                    totals[kind] += 1
                    topics[kind].add(source['Topic']['Value'])
                    if source.get('Name') == 'D435ColorImage':
                        expected['Topic']['Value'] = D435_IMAGE_TOPICS[
                            d435_image_transport
                        ]
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
    assert totals['raw_d435_pointclouds'] > 0
    assert topics['images'] == {
        '/savo_observer/d435/color/image_raw/compressed',
        '/savo_head/camera/image_raw/compressed',
    }
    assert topics['pointclouds'] == {
        '/savo_perception/obstacles/points',
    }
    assert topics['raw_d435_pointclouds'] == {
        '/camera/camera/depth/color/points',
    }


def test_user_config_combines_options_without_changing_qos(tmp_path):
    """Apply all independent flags to a user-supplied RViz file."""
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
                    'Class': 'rviz_default_plugins/PointCloud2',
                    'Enabled': False,
                    'Name': 'RawDepthCloud',
                    'Topic': {
                        'Reliability Policy': 'Best Effort',
                        'Value': RAW_D435_POINTCLOUD_TOPIC,
                    },
                    'Value': False,
                },
                {
                    'Class': 'rviz_default_plugins/Image',
                    'Enabled': False,
                    'Name': 'D435ColorImage',
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
        enable_raw_d435_pointcloud=True,
        d435_image_transport='raw',
    )

    try:
        runtime = yaml.safe_load(runtime_path.read_text(encoding='utf-8'))
        cloud, raw_cloud, image = _displays(runtime)
        assert enabled_counts == {
            'images': 1,
            'pointclouds': 1,
            'raw_d435_pointclouds': 1,
        }
        for display in (cloud, raw_cloud, image):
            assert display['Enabled'] is True
            assert display['Value'] is True
        assert image['Enabled'] is True
        assert image['Value'] is True
        assert image['Topic']['Value'] == '/camera/camera/color/image_raw'
        for index, display in enumerate((cloud, raw_cloud)):
            assert display['Topic'] == document[
                'Visualization Manager'
            ]['Displays'][index]['Topic']
    finally:
        remove_runtime_config(runtime_path)

    assert source_path.read_bytes() == source_bytes


@pytest.mark.parametrize('name', ['sensors.rviz', 'full_debug.rviz'])
def test_camera_preview_enables_both_approved_displays(name):
    """Enable both D435 and compressed head previews in runtime copies."""
    runtime_path, enabled_counts = create_runtime_config(
        RVIZ / name,
        enable_camera_preview=True,
    )

    try:
        runtime = yaml.safe_load(runtime_path.read_text(encoding='utf-8'))
        displays = {
            display['Name']: display for display in _displays(runtime)
        }
        assert enabled_counts['images'] == 2
        for display_name in ('D435ColorImage', 'HeadCameraCompressed'):
            assert displays[display_name]['Enabled'] is True
            assert displays[display_name]['Value'] is True
        assert displays['D435ColorImage']['Topic']['Value'] == (
            '/savo_observer/d435/color/image_raw/compressed'
        )
        assert displays['HeadCameraCompressed']['Topic']['Value'] == (
            '/savo_head/camera/image_raw/compressed'
        )
        assert displays['HeadCameraCompressed']['Topic'][
            'Reliability Policy'
        ] == 'Best Effort'
    finally:
        remove_runtime_config(runtime_path)
