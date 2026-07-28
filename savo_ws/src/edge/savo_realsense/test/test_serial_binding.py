# Copyright 2026 Ahnaf Tahmid

"""Contract tests for the production D435 serial binding."""

from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EXPECTED_SERIAL = "801212070967"


def load_camera_parameters(config_name: str) -> dict:
    with (PACKAGE_ROOT / "config" / config_name).open(
        "r",
        encoding="utf-8",
    ) as config_file:
        config = yaml.safe_load(config_file)

    return config["/camera/camera"]["ros__parameters"]


@pytest.mark.parametrize(
    "config_name",
    [
        "realsense_d435_camera.yaml",
        "realsense_pointcloud_camera.yaml",
    ],
)
def test_production_d435_profiles_bind_the_required_serial(
    config_name: str,
) -> None:
    params = load_camera_parameters(config_name)

    assert params["serial_no"] == EXPECTED_SERIAL
    assert isinstance(params["serial_no"], str)


def test_pointcloud_launch_forces_the_serial_override_to_string() -> None:
    launch_text = (
        PACKAGE_ROOT / "launch" / "realsense_pointcloud.launch.py"
    ).read_text(encoding="utf-8")
    compact = " ".join(launch_text.split())

    assert 'LaunchConfiguration("config_file")' in launch_text
    assert '"realsense_pointcloud_camera.yaml"' in launch_text
    assert 'LaunchConfiguration("serial_no")' in launch_text
    assert 'default_value="801212070967"' in launch_text
    assert "value_type=str" in launch_text
    assert '{"serial_no": serial_no}' in launch_text

    assert 'package="realsense2_camera"' in launch_text
    assert 'executable="realsense2_camera_node"' in launch_text
    assert 'namespace="camera"' in launch_text
    assert 'name="camera"' in launch_text
    assert compact.count("Node(") == 1
