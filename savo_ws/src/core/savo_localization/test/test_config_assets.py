"""Validate installed localization configuration references."""

from pathlib import Path
import re

import yaml

from savo_localization.constants import (
    DEFAULT_ENCODER_GPIO_MAP,
    DEFAULT_ENCODER_INVERT_MAP,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
VO_OVERLAY = "config/ekf_vo_input_optional.yaml"
STALE_OVERLAY = "vo_fusion_optional.yaml"
ENCODER_CONFIGS = (
    "config/encoders.yaml",
    "config/profiles/bench_encoders_4wheel.yaml",
    "config/profiles/wheel_odom_4enc.yaml",
)
EXPECTED_ENCODER_GPIOS = {
    "fl_a_gpio": 20,
    "fl_b_gpio": 21,
    "fr_a_gpio": 13,
    "fr_b_gpio": 25,
    "rl_a_gpio": 24,
    "rl_b_gpio": 23,
    "rr_a_gpio": 12,
    "rr_b_gpio": 26,
}
EXPECTED_ENCODER_WHEELS = {
    "FL": (20, 21),
    "FR": (13, 25),
    "RL": (24, 23),
    "RR": (12, 26),
}


def test_health_overlays_only_use_declared_runtime_parameters() -> None:
    """Prevent safety-looking YAML knobs from being silently ignored."""
    source = (PACKAGE_ROOT / "src/localization_health_node.cpp").read_text(
        encoding="utf-8"
    )
    declared = set(
        re.findall(r'declare_parameter<[^>]+>\(\s*"([^"]+)"', source)
    )
    assert declared

    unknown_by_file = {}
    for path in sorted((PACKAGE_ROOT / "config").rglob("*.yaml")):
        document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        parameters = document.get("localization_health_node", {}).get(
            "ros__parameters", {}
        )
        unknown = sorted(set(parameters) - declared)
        if unknown:
            unknown_by_file[str(path.relative_to(PACKAGE_ROOT))] = unknown

    assert unknown_by_file == {}


def test_optional_vo_overlay_exists_and_is_valid_yaml() -> None:
    """Require the actual VO overlay consumed by launch and diagnostics."""
    path = PACKAGE_ROOT / VO_OVERLAY
    assert path.is_file()
    assert path.stat().st_size > 0

    with path.open(encoding="utf-8") as stream:
        parsed = yaml.safe_load(stream)
    assert isinstance(parsed, dict)
    assert parsed


def test_launch_and_dump_script_reference_the_installed_overlay() -> None:
    """Prevent a stale filename from breaking VO-enabled localization."""
    launch_text = (PACKAGE_ROOT / "launch/ekf.launch.py").read_text()
    dump_text = (PACKAGE_ROOT / "scripts/dump_effective_params.py").read_text()
    bringup_text = (PACKAGE_ROOT / "launch/localization_bringup.launch.py").read_text()

    for text in (launch_text, dump_text, bringup_text):
        assert "ekf_vo_input_optional.yaml" in text
        assert STALE_OVERLAY not in text


def test_cmake_installs_localization_config_directory() -> None:
    """Ensure the corrected overlay is available from package share."""
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()
    assert "config/" in cmake
    assert "DESTINATION share/${PROJECT_NAME}/config" in cmake


def test_bno055_calibration_persistence_production_contract() -> None:
    """Lock production restore defaults, persistent storage, and explicit capture."""
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config/imu.yaml").read_text(encoding="utf-8")
    )["imu_node"]["ros__parameters"]
    assert config["reset_on_start"] is True
    assert config["calibration_restore_enabled"] is True
    assert config["calibration_require_verified_restore"] is True
    assert config["calibration_profile_path"] == (
        "/var/lib/robot_savo/localization/bno055_calibration.yaml"
    )
    assert config["calibration_save_service"] == (
        "/savo_localization/save_imu_calibration"
    )

    for launch_name in (
        "imu.launch.py",
        "localization_bench_imu.launch.py",
        "localization_bringup.launch.py",
    ):
        launch = (PACKAGE_ROOT / "launch" / launch_name).read_text(encoding="utf-8")
        assert '"calibration_restore_enabled"' in launch
        assert 'default_value="true"' in launch
        assert (
            '"calibration_restore_enabled": calibration_restore_enabled' in launch
        )


def test_bno055_production_uses_real_i2c_and_fake_is_test_only() -> None:
    """Keep the fake backend out of every installed production target."""
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config/imu.yaml").read_text(encoding="utf-8")
    )["imu_node"]["ros__parameters"]
    assert config["i2c_bus"] == 1
    assert config["i2c_address"] == 0x28

    driver = (PACKAGE_ROOT / "src/bno055_driver.cpp").read_text(encoding="utf-8")
    real_bus = (PACKAGE_ROOT / "src/i2c_bus.cpp").read_text(encoding="utf-8")
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "std::make_unique<I2CBus>(i2c_bus)" in driver
    assert 'return "/dev/i2c-" + std::to_string(bus_number_)' in real_bus
    assert "src/i2c_bus.cpp" in cmake
    production_sources = cmake.split("set(BNO055_DRIVER_SOURCES", 1)[1].split(
        ")", 1
    )[0]
    assert "fake_i2c_bus" not in production_sources

    fake = PACKAGE_ROOT / "test/fake_i2c_bus.hpp"
    assert fake.is_file()
    assert not (PACKAGE_ROOT / "include/fake_i2c_bus.hpp").exists()
    for relative in (
        "test/test_bno055_driver.cpp",
        "test/test_bno055_calibration.cpp",
    ):
        assert '#include "fake_i2c_bus.hpp"' in (
            PACKAGE_ROOT / relative
        ).read_text(encoding="utf-8")

    production_corpus = "\n".join(
        path.read_text(encoding="utf-8", errors="replace")
        for directory in ("include", "src", "launch", "config")
        for path in (PACKAGE_ROOT / directory).rglob("*")
        if path.is_file()
    )
    assert "FakeI2CBus" not in production_corpus
    assert "fake_i2c_bus" not in production_corpus


def test_bno055_ros_dependencies_match_production_contract() -> None:
    """Require only the service and YAML dependencies used by persistence."""
    package_xml = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    for dependency in ("std_srvs", "yaml_cpp_vendor"):
        assert f"<depend>{dependency}</depend>" in package_xml
        assert f"find_package({dependency} REQUIRED)" in cmake
    assert "std_srvs" in cmake
    assert "target_link_libraries(savo_bno055_driver yaml-cpp)" in cmake


def test_bno055_restore_diagnostics_and_manual_save_are_explicit() -> None:
    """Require auditable restore state and operator-triggered persistence."""
    imu_source = (PACKAGE_ROOT / "src/imu_node.cpp").read_text(encoding="utf-8")
    health_source = (PACKAGE_ROOT / "src/localization_health_node.cpp").read_text(
        encoding="utf-8"
    )
    producer_source = (PACKAGE_ROOT / "src/producer_health.cpp").read_text(
        encoding="utf-8"
    )

    for key in (
        "calibration_profile_present",
        "calibration_profile_loaded",
        "calibration_restore_attempted",
        "calibration_profile_verified",
        "calibration_restore_failed",
    ):
        assert key in imu_source
    assert "save_calibration_callback" in imu_source
    assert "live status must be exactly 3/3/3/3" in (
        PACKAGE_ROOT / "src/bno055_calibration.cpp"
    ).read_text(encoding="utf-8")
    assert "ProducerHealthConsumer" in health_source
    assert 'object.at("health_state")' in producer_source


def test_real_robot_encoder_gpio_mapping_and_direction_flags() -> None:
    """Keep all real-hardware encoder configs aligned with Robot Savo wiring."""
    for relative_path in ENCODER_CONFIGS:
        config = yaml.safe_load(
            (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")
        )
        params = config["wheel_odom_node"]["ros__parameters"]

        assert {
            name: params[name] for name in EXPECTED_ENCODER_GPIOS
        } == EXPECTED_ENCODER_GPIOS
        assert {
            name: params[name]
            for name in ("invert_fl", "invert_fr", "invert_rl", "invert_rr")
        } == {
            "invert_fl": False,
            "invert_fr": False,
            "invert_rl": False,
            "invert_rr": False,
        }


def test_python_encoder_defaults_match_real_robot_wiring() -> None:
    """Keep Python model fallbacks aligned with the production configuration."""
    assert DEFAULT_ENCODER_GPIO_MAP == EXPECTED_ENCODER_WHEELS
    assert DEFAULT_ENCODER_INVERT_MAP == {
        wheel: False for wheel in EXPECTED_ENCODER_WHEELS
    }


def test_vo_fusion_contract_matches_planar_ekf_base() -> None:
    """VO must measure odom -> base_footprint without taking TF ownership."""
    overlay = yaml.safe_load((PACKAGE_ROOT / VO_OVERLAY).read_text(encoding="utf-8"))
    params = overlay["ekf_filter_node"]["ros__parameters"]

    assert params["odom1"] == "/vo/odom"
    assert params["odom1_config"] == [
        True,
        True,
        False,
        False,
        False,
        True,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
    ]
    assert params["odom1_differential"] is False
    assert params["odom1_relative"] is True

    frames = yaml.safe_load(
        (PACKAGE_ROOT / "config/frames.yaml").read_text(encoding="utf-8")
    )
    ekf_frames = frames["ekf_filter_node"]["ros__parameters"]
    assert ekf_frames["base_link_frame"] == "base_footprint"

    health_source = (
        PACKAGE_ROOT / "src/localization_health_node.cpp"
    ).read_text(encoding="utf-8")
    assert "record_odom(vo_tracker_, message, odom_frame_id_, base_frame_id_)" in health_source
