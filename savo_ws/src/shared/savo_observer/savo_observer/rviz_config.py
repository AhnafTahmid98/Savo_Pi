"""Create bounded runtime RViz configurations for observer-only options."""

import tempfile
from pathlib import Path

import yaml


TRUE_VALUES = frozenset({'1', 'on', 'true', 'yes'})
FALSE_VALUES = frozenset({'0', 'off', 'false', 'no'})


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


def enable_pointcloud_displays(document):
    """Enable only top-level PointCloud2 displays in an RViz document."""
    if not isinstance(document, dict):
        raise ValueError('RViz configuration root must be a mapping')

    manager = document.get('Visualization Manager')
    if not isinstance(manager, dict):
        raise ValueError('RViz configuration lacks Visualization Manager')

    displays = manager.get('Displays')
    if not isinstance(displays, list):
        raise ValueError('RViz configuration lacks a Displays list')

    enabled_count = 0
    for display in displays:
        if not isinstance(display, dict):
            raise ValueError('RViz Displays entries must be mappings')

        class_name = str(display.get('Class', ''))
        if not class_name.endswith('/PointCloud2'):
            continue

        display['Enabled'] = True
        if 'Value' in display:
            display['Value'] = True
        enabled_count += 1

    return enabled_count


def create_pointcloud_runtime_config(source_path):
    """Write a temporary PointCloud2-enabled copy of one RViz file."""
    source = Path(source_path)
    try:
        document = yaml.safe_load(source.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(
            f'could not read RViz configuration {source}: {error}'
        ) from error

    enabled_count = enable_pointcloud_displays(document)
    with tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        prefix='savo_observer_pointclouds_',
        suffix='.rviz',
        delete=False,
    ) as stream:
        yaml.safe_dump(document, stream, sort_keys=False)
        runtime_path = Path(stream.name)

    return runtime_path, enabled_count


def remove_runtime_config(path):
    """Remove an observer-owned temporary RViz configuration if present."""
    Path(path).unlink(missing_ok=True)
