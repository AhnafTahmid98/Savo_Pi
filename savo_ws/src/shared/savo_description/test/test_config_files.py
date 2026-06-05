from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_required_config_files_exist():
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
        assert (config_dir / filename).is_file()