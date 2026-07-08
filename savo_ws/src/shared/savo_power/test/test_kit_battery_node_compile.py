import py_compile
from pathlib import Path

from savo_power.nodes import kit_battery_node_py


def test_kit_battery_python_node_module_compiles():
    py_compile.compile(
        "savo_power/nodes/kit_battery_node_py.py",
        doraise=True,
    )


def test_kit_battery_python_node_exports_runtime_helpers():
    for name in [
        "KitBatteryNodeState",
        "base_battery_topic",
        "base_battery_python_node_name",
        "create_timer_period_s",
        "create_ads7830_driver_from_params",
        "read_from_driver",
        "make_error_reading",
        "reading_to_publish_text",
        "build_startup_summary",
        "main",
    ]:
        assert hasattr(kit_battery_node_py, name), name


def test_kit_battery_python_node_uses_base_battery_identity():
    assert kit_battery_node_py.base_battery_topic() == "/savo_power/base/battery"
    assert kit_battery_node_py.base_battery_python_node_name() == "base_battery_node_py"


def test_kit_battery_timer_period_helper_is_safe():
    assert kit_battery_node_py.create_timer_period_s(1.0) == 1.0
    assert kit_battery_node_py.create_timer_period_s(2.0) == 0.5
    assert kit_battery_node_py.create_timer_period_s(0.0) > 0.0


def test_kit_battery_state_dataclass_defaults_are_safe():
    state = kit_battery_node_py.KitBatteryNodeState()

    assert state.publish_count == 0
    assert state.error_count == 0
    assert state.last_error == ""


def test_kit_battery_cpp_sources_exist():
    for path in [
        "src/nodes/base_battery_node.cpp",
        "src/nodes/kit_battery_node.cpp",
        "include/savo_power/kit_battery_node.hpp",
    ]:
        assert Path(path).is_file(), path


def test_kit_battery_cpp_node_has_ros_runtime_shape():
    text = Path("src/nodes/base_battery_node.cpp").read_text()

    assert "rclcpp::init" in text
    assert "rclcpp::spin" in text
    assert "rclcpp::shutdown" in text
    assert "KitBatteryNode" in text


def test_kit_battery_cpp_implementation_mentions_ads7830_and_base_battery():
    text = (
        Path("src/nodes/kit_battery_node.cpp").read_text()
        + "\n"
        + Path("include/savo_power/kit_battery_node.hpp").read_text()
    )

    assert "Ads7830" in text or "ads7830" in text.lower()
    assert "base_battery" in text or "BASE_BATTERY" in text


def test_kit_battery_wrappers_exist_and_call_python_node():
    for script in [
        "scripts/base_battery_node_py",
        "scripts/kit_battery_node_py",
    ]:
        text = Path(script).read_text()

        assert "savo_power.nodes.kit_battery_node_py" in text
        assert "main" in text


def test_cmake_references_kit_battery_sources_and_base_node():
    text = Path("CMakeLists.txt").read_text()

    assert "src/nodes/kit_battery_node.cpp" in text
    assert "src/nodes/base_battery_node.cpp" in text
    assert "base_battery_node" in text


def test_kit_battery_node_does_not_enable_shutdown_or_motor_control():
    combined = "\n".join(
        Path(path).read_text(errors="ignore")
        for path in [
            "savo_power/nodes/kit_battery_node_py.py",
            "src/nodes/base_battery_node.cpp",
            "src/nodes/kit_battery_node.cpp",
            "include/savo_power/kit_battery_node.hpp",
        ]
    ).lower()

    for forbidden in [
        "automatic_shutdown_enabled: true",
        "automatic_shutdown_enabled=true",
        "shutdown:=true",
        "/cmd_vel",
        "motor_pwm",
        "wheel_speed",
    ]:
        assert forbidden not in combined
