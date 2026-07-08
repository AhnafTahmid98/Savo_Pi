import ast
import py_compile
import stat
from pathlib import Path


EXPECTED_SCRIPTS = {
    "base_battery_node_py",
    "core_ups_node_py",
    "edge_ups_node_py",
    "kit_battery_node_py",
    "power_aggregator_node_py",
    "power_dashboard_node_py",
    "power_health_node_py",
    "power_i2c_check",
    "ups_check",
    "kit_battery_check",
    "power_status_echo",
    "power_dashboard_echo",
}


SCRIPT_IMPORTS = {
    "base_battery_node_py": "from savo_power.nodes.kit_battery_node_py import main",
    "core_ups_node_py": "from savo_power.nodes.ups_hat_node_py import main_core",
    "edge_ups_node_py": "from savo_power.nodes.ups_hat_node_py import main_edge",
    "kit_battery_node_py": "from savo_power.nodes.kit_battery_node_py import main",
    "power_aggregator_node_py": "from savo_power.nodes.power_aggregator_node_py import main",
    "power_dashboard_node_py": "from savo_power.nodes.power_dashboard_node_py import main",
    "power_health_node_py": "from savo_power.nodes.power_health_node_py import main",
    "power_i2c_check": "from savo_power.diagnostics.i2c_power_check import main",
    "ups_check": "from savo_power.diagnostics.ups_check import main",
    "kit_battery_check": "from savo_power.diagnostics.kit_battery_check import main",
}


def script_path(name):
    return Path("scripts") / name


def script_text(name):
    return script_path(name).read_text()


def test_expected_scripts_exist():
    actual = {
        path.name
        for path in Path("scripts").iterdir()
        if path.is_file()
    }

    assert EXPECTED_SCRIPTS.issubset(actual)


def test_scripts_are_executable():
    for name in EXPECTED_SCRIPTS:
        mode = script_path(name).stat().st_mode

        assert mode & stat.S_IXUSR, name


def test_scripts_have_python_shebang():
    for name in EXPECTED_SCRIPTS:
        assert script_text(name).startswith("#!/usr/bin/env python3"), name


def test_scripts_compile_as_python():
    for name in EXPECTED_SCRIPTS:
        py_compile.compile(str(script_path(name)), doraise=True)


def test_wrapper_scripts_import_expected_entry_points():
    for name, expected_import in SCRIPT_IMPORTS.items():
        text = script_text(name)

        assert expected_import in text, name
        assert 'if __name__ == "__main__":' in text, name


def test_node_wrapper_scripts_call_main_functions():
    expected_calls = {
        "core_ups_node_py": "main_core()",
        "edge_ups_node_py": "main_edge()",
        "base_battery_node_py": "main()",
        "kit_battery_node_py": "main()",
        "power_aggregator_node_py": "main()",
        "power_health_node_py": "main()",
        "power_dashboard_node_py": "main()",
    }

    for name, call in expected_calls.items():
        assert call in script_text(name), name


def test_diagnostic_wrapper_scripts_call_main():
    for name in ("power_i2c_check", "ups_check", "kit_battery_check"):
        text = script_text(name)

        assert "main()" in text, name


def test_power_status_echo_has_topic_aliases():
    text = script_text("power_status_echo")

    assert "TOPIC_ALIASES" in text
    assert '"status": c.STATUS_TOPIC' in text
    assert '"health": c.HEALTH_TOPIC' in text
    assert '"dashboard": c.DASHBOARD_TOPIC' in text
    assert '"dashboard_text": c.DASHBOARD_TEXT_TOPIC' in text
    assert '"core_ups": c.CORE_UPS_TOPIC' in text
    assert '"edge_ups": c.EDGE_UPS_TOPIC' in text
    assert '"base_battery": c.BASE_BATTERY_TOPIC' in text


def test_power_status_echo_uses_ros2_topic_echo():
    text = script_text("power_status_echo")

    assert 'cmd = ["ros2", "topic", "echo", topic]' in text
    assert "subprocess.call(cmd)" in text
    assert "--once" in text


def test_power_dashboard_echo_selects_text_by_default_and_json_option():
    text = script_text("power_dashboard_echo")

    assert "return c.DASHBOARD_TEXT_TOPIC" in text
    assert "return c.DASHBOARD_TOPIC" in text
    assert "--json" in text
    assert "--once" in text
    assert 'cmd = ["ros2", "topic", "echo", topic]' in text


def test_setup_py_console_scripts_include_node_and_diagnostic_wrappers():
    text = Path("setup.py").read_text()

    expected = [
        "core_ups_node_py = savo_power.nodes.ups_hat_node_py:main_core",
        "edge_ups_node_py = savo_power.nodes.ups_hat_node_py:main_edge",
        "base_battery_node_py = savo_power.nodes.kit_battery_node_py:main",
        "power_aggregator_node_py = savo_power.nodes.power_aggregator_node_py:main",
        "power_health_node_py = savo_power.nodes.power_health_node_py:main",
        "power_dashboard_node_py = savo_power.nodes.power_dashboard_node_py:main",
        "power_i2c_check = savo_power.diagnostics.i2c_power_check:main",
        "ups_check = savo_power.diagnostics.ups_check:main",
        "kit_battery_check = savo_power.diagnostics.kit_battery_check:main",
    ]

    for item in expected:
        assert item in text


def test_setup_py_parses_as_python():
    ast.parse(Path("setup.py").read_text())


def test_cmake_installs_scripts_directory():
    text = Path("CMakeLists.txt").read_text()

    assert "file(GLOB SAVO_POWER_SCRIPTS" in text
    assert "install(" in text
    assert "PROGRAMS ${SAVO_POWER_SCRIPTS}" in text
    assert "DESTINATION lib/${PROJECT_NAME}" in text


def test_no_script_enables_shutdown():
    for name in EXPECTED_SCRIPTS:
        text = script_text(name)

        assert "automatic_shutdown_enabled=True" not in text
        assert "automatic_shutdown_enabled = True" not in text
        assert "shutdown:=true" not in text
