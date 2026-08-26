"""Create bounded runtime RViz configurations for observer-only options."""

from pathlib import Path
import tempfile

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


def enable_optional_displays(
    document,
    *,
    enable_camera_preview=False,
    enable_pointclouds=False,
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

    enabled_counts = {'images': 0, 'pointclouds': 0}
    for display in displays:
        if not isinstance(display, dict):
            raise ValueError('RViz Displays entries must be mappings')

        class_name = str(display.get('Class', ''))
        if class_name.endswith('/Image') and enable_camera_preview:
            enabled_counts['images'] += 1
        elif class_name.endswith('/PointCloud2') and enable_pointclouds:
            enabled_counts['pointclouds'] += 1
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
