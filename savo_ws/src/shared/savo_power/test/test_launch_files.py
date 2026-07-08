import ast
from pathlib import Path


LAUNCH_FILES = [
    "launch/power_core.launch.py",
    "launch/power_edge.launch.py",
    "launch/power_bringup.launch.py",
    "launch/power_diag.launch.py",
    "launch/power_dryrun.launch.py",
]


def read(path):
    return Path(path).read_text()


def assert_python_file_parses(path):
    ast.parse(read(path))


def test_launch_files_exist():
    for path in LAUNCH_FILES:
        assert Path(path).is_file(), path


def test_launch_files_parse_as_python():
    for path in LAUNCH_FILES:
        assert_python_file_parses(path)


def test_core_launch_uses_cpp_nodes_by_default():
    text = read("launch/power_core.launch.py")

    assert 'executable="core_ups_node"' in text
    assert 'executable="base_battery_node"' in text
    assert 'executable="power_aggregator_node"' in text
    assert 'executable="power_health_node"' in text
    assert 'executable="power_dashboard_node"' in text

    assert "UnlessCondition(use_python_fallback)" in text


def test_core_launch_has_python_fallback_nodes():
    text = read("launch/power_core.launch.py")

    assert 'executable="core_ups_node_py"' in text
    assert 'executable="base_battery_node_py"' in text
    assert 'executable="power_aggregator_node_py"' in text
    assert 'executable="power_health_node_py"' in text
    assert 'executable="power_dashboard_node_py"' in text

    assert "IfCondition(use_python_fallback)" in text


def test_core_launch_uses_core_configs():
    text = read("launch/power_core.launch.py")

    assert '"power_common.yaml"' in text
    assert '"topics.yaml"' in text
    assert '"profiles", "core_real_robot_v1.yaml"' in text

    assert '"core", "core_ups.yaml"' in text
    assert '"core", "kit_battery.yaml"' in text
    assert '"core", "power_aggregator.yaml"' in text
    assert '"core", "power_health.yaml"' in text
    assert '"core", "dashboard.yaml"' in text


def test_edge_launch_runs_edge_ups_by_default():
    text = read("launch/power_edge.launch.py")

    assert 'executable="edge_ups_node"' in text
    assert 'executable="edge_ups_node_py"' in text

    assert '"profiles", "edge_real_robot_v1.yaml"' in text
    assert '"edge", "edge_ups.yaml"' in text

    assert "UnlessCondition(use_python_fallback)" in text
    assert "IfCondition(use_python_fallback)" in text


def test_edge_launch_keeps_health_dashboard_optional():
    text = read("launch/power_edge.launch.py")

    assert 'DeclareLaunchArgument(\n            "enable_power_health"' in text
    assert 'DeclareLaunchArgument(\n            "enable_power_dashboard"' in text

    assert 'default_value="false"' in text

    assert 'executable="power_health_node"' in text
    assert 'executable="power_health_node_py"' in text
    assert 'executable="power_dashboard_node"' in text
    assert 'executable="power_dashboard_node_py"' in text

    assert "_enabled_with_cpp(enable_power_health, use_python_fallback)" in text
    assert "_enabled_with_python(enable_power_dashboard, use_python_fallback)" in text


def test_bringup_launch_includes_core_and_edge_launches():
    text = read("launch/power_bringup.launch.py")

    assert '"power_core.launch.py"' in text
    assert '"power_edge.launch.py"' in text

    assert 'default_value="core"' in text
    assert "role:=core" not in text

    assert '"use_python_fallback": use_python_fallback' in text
    assert '"enable_power_health": enable_edge_power_health' in text
    assert '"enable_power_dashboard": enable_edge_power_dashboard' in text


def test_bringup_role_condition_supports_core_edge_both():
    text = read("launch/power_bringup.launch.py")

    assert "Power bringup role: core, edge, or both." in text
    assert "_role_condition(role, \"core\")" in text
    assert "_role_condition(role, \"edge\")" in text
    assert "'both'" in text


def test_diag_launch_runs_diagnostic_modules_only():
    text = read("launch/power_diag.launch.py")

    assert "ExecuteProcess" in text
    assert "savo_power.diagnostics.i2c_power_check" in text
    assert "savo_power.diagnostics.ups_check" in text
    assert "savo_power.diagnostics.kit_battery_check" in text

    assert 'executable="core_ups_node"' not in text
    assert 'executable="edge_ups_node"' not in text
    assert 'executable="base_battery_node"' not in text


def test_diag_launch_uses_diagnostic_config():
    text = read("launch/power_diag.launch.py")

    assert "SAVO_POWER_DIAGNOSTIC_CONFIG" in text
    assert '"diagnostic.yaml"' in text

    assert '"run_i2c_check"' in text
    assert '"run_ups_check"' in text
    assert '"run_kit_battery_check"' in text


def test_dryrun_launch_includes_full_cpp_stack():
    text = read("launch/power_dryrun.launch.py")

    assert '"profiles", "dryrun_sim.yaml"' in text

    assert 'executable="core_ups_node"' in text
    assert 'executable="edge_ups_node"' in text
    assert 'executable="base_battery_node"' in text
    assert 'executable="power_aggregator_node"' in text
    assert 'executable="power_health_node"' in text
    assert 'executable="power_dashboard_node"' in text

    assert "UnlessCondition(use_python_fallback)" in text


def test_dryrun_launch_includes_full_python_fallback_stack():
    text = read("launch/power_dryrun.launch.py")

    assert 'executable="core_ups_node_py"' in text
    assert 'executable="edge_ups_node_py"' in text
    assert 'executable="base_battery_node_py"' in text
    assert 'executable="power_aggregator_node_py"' in text
    assert 'executable="power_health_node_py"' in text
    assert 'executable="power_dashboard_node_py"' in text

    assert "IfCondition(use_python_fallback)" in text


def test_all_launches_use_savo_power_package():
    for path in LAUNCH_FILES:
        text = read(path)

        assert "savo_power" in text, path


def test_launch_files_do_not_enable_shutdown():
    for path in LAUNCH_FILES:
        text = read(path)

        assert "automatic_shutdown_enabled:=true" not in text
        assert "shutdown:=true" not in text
