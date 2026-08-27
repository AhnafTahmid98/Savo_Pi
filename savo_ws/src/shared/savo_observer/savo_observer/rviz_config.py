"""Create bounded runtime RViz configurations for observer-only options."""

from pathlib import Path
import tempfile

import yaml


TRUE_VALUES = frozenset({'1', 'on', 'true', 'yes'})
FALSE_VALUES = frozenset({'0', 'off', 'false', 'no'})
D435_RAW_IMAGE_TOPIC = '/camera/camera/color/image_raw'
D435_OBSERVER_IMAGE_BASE_TOPIC = '/savo_observer/d435/color/image_raw'
D435_COMPRESSED_IMAGE_TOPIC = f'{D435_OBSERVER_IMAGE_BASE_TOPIC}/compressed'
D435_IMAGE_TOPICS = {
    'compressed': D435_COMPRESSED_IMAGE_TOPIC,
    'raw': D435_RAW_IMAGE_TOPIC,
}
RAW_D435_POINTCLOUD_TOPIC = '/camera/camera/depth/color/points'


def parse_launch_boolean(value, name):
    """Parse an ordinary ROS launch boolean or fail without enabling it."""
    if isinstance(value, bool):
        return value

    normalized = str(value).strip().lower()
    if normalized in TRUE_VALUES:
        return True
    if normalized in FALSE_VALUES:
        return False
    raise ValueError(
        f'{name} must be one of true/false, yes/no, on/off, or 1/0; '
        f'received {value!r}'
    )


def parse_d435_image_transport(value):
    """Accept only the two supported D435 observer image transports."""
    normalized = str(value).strip().lower()
    if normalized not in D435_IMAGE_TOPICS:
        raise ValueError(
            'd435_image_transport must be raw or compressed; '
            f'received {value!r}'
        )
    return normalized


def enable_optional_displays(
    document,
    *,
    enable_camera_preview=False,
    enable_pointclouds=False,
    enable_raw_d435_pointcloud=False,
    d435_image_transport='compressed',
):
    """Enable only requested top-level high-bandwidth RViz displays."""
    if not isinstance(document, dict):
        raise ValueError('RViz configuration root must be a mapping')

    manager = document.get('Visualization Manager')
    if not isinstance(manager, dict):
        raise ValueError('RViz configuration lacks Visualization Manager')

    displays = manager.get('Displays')
    if not isinstance(displays, list):
        raise ValueError('RViz configuration lacks a Displays list')

    d435_image_transport = parse_d435_image_transport(
        d435_image_transport
    )
    enabled_counts = {
        'images': 0,
        'pointclouds': 0,
        'raw_d435_pointclouds': 0,
    }
    for display in displays:
        if not isinstance(display, dict):
            raise ValueError('RViz Displays entries must be mappings')

        class_name = str(display.get('Class', ''))
        topic = display.get('Topic')
        if (
            class_name.endswith('/Image')
            and display.get('Name') == 'D435ColorImage'
        ):
            if not isinstance(topic, dict):
                raise ValueError('D435ColorImage must have a Topic mapping')
            topic['Value'] = D435_IMAGE_TOPICS[d435_image_transport]

        if class_name.endswith('/Image') and enable_camera_preview:
            enabled_counts['images'] += 1
        elif class_name.endswith('/PointCloud2'):
            if not isinstance(topic, dict):
                raise ValueError('PointCloud2 display must have a Topic mapping')
            if topic.get('Value') == RAW_D435_POINTCLOUD_TOPIC:
                if not enable_raw_d435_pointcloud:
                    continue
                enabled_counts['raw_d435_pointclouds'] += 1
            elif enable_pointclouds:
                enabled_counts['pointclouds'] += 1
            else:
                continue
        else:
            continue

        display['Enabled'] = True
        if 'Value' in display:
            display['Value'] = True

    return enabled_counts


def create_runtime_config(
    source_path,
    *,
    enable_camera_preview=False,
    enable_pointclouds=False,
    enable_raw_d435_pointcloud=False,
    d435_image_transport='compressed',
):
    """Write one temporary copy with requested optional displays enabled."""
    source = Path(source_path)
    try:
        document = yaml.safe_load(source.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(
            f'could not read RViz configuration {source}: {error}'
        ) from error

    enabled_counts = enable_optional_displays(
        document,
        enable_camera_preview=enable_camera_preview,
        enable_pointclouds=enable_pointclouds,
        enable_raw_d435_pointcloud=enable_raw_d435_pointcloud,
        d435_image_transport=d435_image_transport,
    )
    with tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        prefix='savo_observer_rviz_',
        suffix='.rviz',
        delete=False,
    ) as stream:
        yaml.safe_dump(document, stream, sort_keys=False)
        runtime_path = Path(stream.name)

    return runtime_path, enabled_counts


def remove_runtime_config(path):
    """Remove an observer-owned temporary RViz configuration if present."""
    Path(path).unlink(missing_ok=True)
