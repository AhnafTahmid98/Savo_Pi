"""Lock the LiDAR scan frame to the shared Robot Savo TF contract."""

from pathlib import Path

import yaml

from savo_lidar.constants import DEFAULT_FRAME_ID
from savo_lidar.models.lidar_config import LidarDriverConfig


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LOCKED_FRAME = "laser_frame"


def _load_yaml(relative_path: str) -> dict:
    with (PACKAGE_ROOT / relative_path).open(encoding="utf-8") as stream:
        parsed = yaml.safe_load(stream)
    assert isinstance(parsed, dict)
    return parsed


def test_python_defaults_use_laser_frame() -> None:
    """Keep Python defaults aligned with the shared description."""
    assert DEFAULT_FRAME_ID == LOCKED_FRAME
    assert LidarDriverConfig().frame_id == LOCKED_FRAME


def test_all_runtime_profiles_use_laser_frame() -> None:
    """Reject a launch profile that would publish scans in an orphan frame."""
    runtime_files = (
        "config/lidar_driver.yaml",
        "config/profiles/bench_test.yaml",
        "config/profiles/dryrun_sim.yaml",
        "config/profiles/real_rplidar_a1.yaml",
        "config/profiles/mapping_rplidar_a1.yaml",
        "config/profiles/nav_rplidar_a1.yaml",
    )

    for relative_path in runtime_files:
        document = _load_yaml(relative_path)
        node_parameters = next(iter(document.values()))["ros__parameters"]
        assert node_parameters["frame_id"] == LOCKED_FRAME


def test_diagnostics_expect_laser_frame() -> None:
    """Make frame diagnostics detect any future contract drift."""
    diagnostics = _load_yaml("config/diagnostics.yaml")
    expected = diagnostics["diagnostics"]["ros__parameters"]["expected_frame_id"]
    assert expected == LOCKED_FRAME


def test_cpp_defaults_use_laser_frame() -> None:
    """Keep C++ driver and data-model defaults aligned with TF."""
    runtime_files = (
        "include/savo_lidar/scan_types.hpp",
        "include/savo_lidar/diagnostics.hpp",
        "include/savo_lidar/serial_config.hpp",
        "src/nodes/lidar_driver_node.cpp",
    )

    for relative_path in runtime_files:
        text = (PACKAGE_ROOT / relative_path).read_text()
        assert '"laser_frame"' in text
        assert '"laser"' not in text


def test_cpp_real_driver_bins_hardware_angles_and_applies_configured_transform() -> None:
    """Keep the production C++ acquisition path wired to measured-angle bins."""
    driver = (PACKAGE_ROOT / "src/drivers/rplidar_driver.cpp").read_text()
    assembler = (
        PACKAGE_ROOT / "src/drivers/scan_frame_assembler.cpp"
    ).read_text()
    compensator = (
        PACKAGE_ROOT / "src/drivers/scan_angle_compensator.cpp"
    ).read_text()
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()

    assert "sample.angle_rad = measurement.angle_rad" in assembler
    assert "bin_scan_samples_by_angle(" in driver
    assert "config_.inverted" in driver
    assert "config_.angle_offset_rad" in driver
    assert "scan.ranges_m.assign" in compensator
    assert "std::numeric_limits<float>::infinity()" in compensator
    assert "scan.intensities.assign" in compensator
    assert "scan_angle_compensator.cpp" in cmake
    assert "test_scan_angle_compensator.cpp" in cmake
