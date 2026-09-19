import py_compile
from pathlib import Path

import pytest

from savo_power.models.power_status import BatterySource
from savo_power.models.ups_reading import make_ups_reading
from savo_power.nodes import ups_hat_node_py
from savo_power.policy.power_policy import PowerPolicyThresholds
from savo_power.ros.params import UpsNodeParams


def test_ups_hat_python_node_module_compiles():
    py_compile.compile(
        "savo_power/nodes/ups_hat_node_py.py",
        doraise=True,
    )


def test_ups_hat_python_node_exports_runtime_helpers():
    for name in [
        "UpsHatNodeState",
        "source_to_topic",
        "source_to_python_node_name",
        "validate_ups_source",
        "create_timer_period_s",
        "create_ups_driver_from_params",
        "read_from_driver",
        "make_error_reading",
        "reading_to_publish_text",
        "build_startup_summary",
        "main_core",
        "main_edge",
        "main",
    ]:
        assert hasattr(ups_hat_node_py, name), name


def test_ups_hat_core_and_edge_topics_are_locked():
    assert ups_hat_node_py.source_to_topic("core_ups") == "/savo_power/core/ups"
    assert ups_hat_node_py.source_to_topic("edge_ups") == "/savo_power/edge/ups"


def test_ups_hat_core_and_edge_python_node_names_are_locked():
    assert (
        ups_hat_node_py.source_to_python_node_name("core_ups")
        == "core_ups_node_py"
    )
    assert (
        ups_hat_node_py.source_to_python_node_name("edge_ups")
        == "edge_ups_node_py"
    )


def test_ups_hat_source_validation_accepts_only_ups_roles():
    assert ups_hat_node_py.validate_ups_source("core_ups") == BatterySource.CORE_UPS
    assert ups_hat_node_py.validate_ups_source("edge_ups") == BatterySource.EDGE_UPS

    with pytest.raises(Exception):
        ups_hat_node_py.validate_ups_source("base_battery")


def test_ups_hat_timer_period_helper_is_safe():
    assert ups_hat_node_py.create_timer_period_s(1.0) == 1.0
    assert ups_hat_node_py.create_timer_period_s(2.0) == 0.5
    assert ups_hat_node_py.create_timer_period_s(0.0) > 0.0


def test_ups_hat_node_state_defaults_are_safe():
    state = ups_hat_node_py.UpsHatNodeState()

    assert state.publish_count == 0
    assert state.error_count == 0
    assert state.last_error == ""


def test_power_producers_have_one_truthful_rate_control() -> None:
    files = (
        "include/savo_power/power_node_params.hpp",
        "src/nodes/core_ups_node.cpp",
        "src/nodes/edge_ups_node.cpp",
        "src/nodes/ups_hat_node.cpp",
        "src/nodes/kit_battery_node.cpp",
        "savo_power/ros/params.py",
        "savo_power/nodes/ups_hat_node_py.py",
        "savo_power/nodes/kit_battery_node_py.py",
    )

    for path in files:
        assert "sample_rate_hz" not in Path(path).read_text(), path


def test_ups_hat_error_reading_preserves_source():
    reading = ups_hat_node_py.make_error_reading(
        BatterySource.CORE_UPS,
        "i2c failed",
    )

    assert reading.source == BatterySource.CORE_UPS

    published = ups_hat_node_py.reading_to_publish_text(reading)

    assert "core_ups" in published
    assert "i2c failed" in published


def test_python_ups_factory_uses_bus_factory_contract(monkeypatch):
    captured = {}

    def fake_factory(**kwargs):
        captured.update(kwargs)
        return object()

    monkeypatch.setattr(ups_hat_node_py, "make_ups_hat_driver", fake_factory)
    params = UpsNodeParams(
        source=BatterySource.CORE_UPS,
        i2c_bus=1,
        address=0x36,
    )

    driver = ups_hat_node_py.create_ups_driver_from_params(params)

    assert driver is not None
    assert captured == {
        "source": BatterySource.CORE_UPS,
        "bus_id": 1,
        "address": 0x36,
    }


def test_python_ups_node_applies_configured_thresholds():
    reading = make_ups_reading(
        BatterySource.CORE_UPS,
        voltage_v=3.30,
        capacity_pct=50.0,
    )
    thresholds = PowerPolicyThresholds(
        ups_low_voltage_v=3.40,
        ups_critical_voltage_v=3.35,
    )

    classified = ups_hat_node_py.apply_configured_policy(reading, thresholds)

    assert classified.state.value == "critical"
    assert classified.ok is True


def test_python_ups_factory_failure_returns_error_and_retries():
    attempts = 0

    def failing_factory():
        nonlocal attempts
        attempts += 1
        raise OSError("simulated bus unavailable")

    driver = None
    for _ in range(3):
        reading, driver = ups_hat_node_py.read_with_recovery(
            driver,
            failing_factory,
            BatterySource.CORE_UPS,
            PowerPolicyThresholds(),
        )
        assert driver is None
        assert reading.state.value == "error"
        assert reading.ok is False
        assert "simulated bus unavailable" in reading.error_message

    assert attempts == 3


def test_python_ups_failed_driver_is_closed_before_retry():
    class FailedDriver:
        def __init__(self):
            self.close_count = 0

        def read(self):
            return ups_hat_node_py.make_error_reading(
                BatterySource.CORE_UPS,
                "simulated read failure",
            )

        def close(self):
            self.close_count += 1

    failed_driver = FailedDriver()

    reading, retained_driver = ups_hat_node_py.read_with_recovery(
        failed_driver,
        lambda: pytest.fail("existing driver must be used"),
        BatterySource.CORE_UPS,
        PowerPolicyThresholds(),
    )

    assert reading.state.value == "error"
    assert retained_driver is None
    assert failed_driver.close_count == 1


def test_ups_hat_publish_text_is_json_like():
    reading = ups_hat_node_py.make_error_reading(
        BatterySource.EDGE_UPS,
        "dryrun error",
    )

    text = ups_hat_node_py.reading_to_publish_text(reading)

    assert "edge_ups" in text
    assert "dryrun error" in text


def test_ups_hat_cpp_sources_exist():
    for path in [
        "src/nodes/core_ups_node.cpp",
        "src/nodes/edge_ups_node.cpp",
        "src/nodes/ups_hat_node.cpp",
        "include/savo_power/ups_hat_node.hpp",
        "src/drivers/ups_hat_driver.cpp",
        "include/savo_power/ups_hat_driver.hpp",
    ]:
        assert Path(path).is_file(), path


def test_core_and_edge_ups_cpp_nodes_have_ros_runtime_shape():
    expected_node_names = {
        "src/nodes/core_ups_node.cpp": "CoreUpsNode",
        "src/nodes/edge_ups_node.cpp": "EdgeUpsNode",
    }

    for path, node_name in expected_node_names.items():
        text = Path(path).read_text()

        assert "rclcpp::init" in text
        assert "rclcpp::spin" in text
        assert "rclcpp::shutdown" in text
        assert node_name in text


def test_core_ups_callback_recovers_without_terminating_process():
    source = Path("src/nodes/core_ups_node.cpp").read_text()

    assert "catch (const std::exception & error)" in source
    assert "Core UPS timer callback failed" in source
    assert "reset_driver()" in source
    assert "reading.state == PowerState::ERROR" in source
    assert "publish_error_reading(last_error_)" in source


def test_ups_hat_cpp_contract_mentions_core_edge_and_driver():
    text = (
        Path("src/nodes/ups_hat_node.cpp").read_text()
        + "\n"
        + Path("include/savo_power/ups_hat_node.hpp").read_text()
        + "\n"
        + Path("src/drivers/ups_hat_driver.cpp").read_text()
        + "\n"
        + Path("include/savo_power/ups_hat_driver.hpp").read_text()
    )

    assert "core_ups" in text
    assert "edge_ups" in text
    assert "UpsHat" in text or "ups" in text.lower()


def test_ups_hat_script_wrappers_exist_and_call_python_node():
    expected = {
        "scripts/core_ups_node_py": "main_core",
        "scripts/edge_ups_node_py": "main_edge",
    }

    for script, entry in expected.items():
        path = Path(script)

        assert path.is_file(), script

        text = path.read_text()

        assert "savo_power.nodes.ups_hat_node_py" in text
        assert entry in text


def test_cmake_references_ups_hat_sources_and_nodes():
    text = Path("CMakeLists.txt").read_text()

    assert "src/drivers/ups_hat_driver.cpp" in text
    assert "src/nodes/ups_hat_node.cpp" in text
    assert "src/nodes/core_ups_node.cpp" in text
    assert "src/nodes/edge_ups_node.cpp" in text
    assert "core_ups_node" in text
    assert "edge_ups_node" in text


def test_ups_hat_node_does_not_enable_shutdown_or_motor_control():
    files = [
        "savo_power/nodes/ups_hat_node_py.py",
        "src/nodes/core_ups_node.cpp",
        "src/nodes/edge_ups_node.cpp",
        "src/nodes/ups_hat_node.cpp",
        "include/savo_power/ups_hat_node.hpp",
        "src/drivers/ups_hat_driver.cpp",
        "include/savo_power/ups_hat_driver.hpp",
        "scripts/core_ups_node_py",
        "scripts/edge_ups_node_py",
    ]

    combined = "\n".join(
        Path(path).read_text(errors="ignore")
        for path in files
        if Path(path).is_file()
    ).lower()

    for forbidden in [
        "automatic_shutdown_enabled: true",
        "automatic_shutdown_enabled=true",
        "shutdown:=true",
        "/cmd_vel",
        "motor_pwm",
        "wheel_speed",
        "pca9685",
    ]:
        assert forbidden not in combined
