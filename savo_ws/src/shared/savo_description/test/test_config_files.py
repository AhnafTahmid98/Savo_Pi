"""Validate required Robot Savo description configuration files."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_required_config_files_exist_and_are_valid_yaml() -> None:
    """Require non-empty YAML mappings for every description contract."""
    required_files = [
        "robot_dimensions.yaml",
        "wheel_geometry.yaml",
        "sensor_mounts.yaml",
        "frame_names.yaml",
        "costmap_frames.yaml",
        "description_params.yaml",
    ]

    config_dir = PACKAGE_ROOT / "config"

    for filename in required_files:
        path = config_dir / filename
        assert path.is_file()
        assert path.stat().st_size > 0
        with path.open(encoding="utf-8") as stream:
            parsed = yaml.safe_load(stream)
        assert isinstance(parsed, dict)
        assert parsed, f"{path}: YAML mapping is empty"
