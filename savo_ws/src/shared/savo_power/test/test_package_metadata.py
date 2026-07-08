import ast
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_NAME = "savo_power"


def package_xml_root():
    path = Path("package.xml")

    assert path.is_file()

    return ET.fromstring(path.read_text())


def package_depend_values(tag):
    root = package_xml_root()

    return {
        item.text.strip()
        for item in root.findall(tag)
        if item.text and item.text.strip()
    }


def setup_text():
    path = Path("setup.py")

    assert path.is_file()

    return path.read_text()


def cmake_text():
    path = Path("CMakeLists.txt")

    assert path.is_file()

    return path.read_text()


def test_package_xml_has_basic_metadata():
    root = package_xml_root()

    assert root.findtext("name") == PACKAGE_NAME
    assert root.findtext("version") == "0.1.0"
    assert root.findtext("license") == "Proprietary"

    maintainer = root.find("maintainer")
    assert maintainer is not None
    assert maintainer.text.strip() == "Ahnaf Tahmid"
    assert maintainer.attrib["email"] == "tahmidahnaf998@gmail.com"


def test_package_xml_uses_ament_cmake_build_type():
    root = package_xml_root()
    export = root.find("export")

    assert export is not None
    assert export.findtext("build_type") == "ament_cmake"


def test_package_xml_has_required_buildtool_dependencies():
    buildtools = package_depend_values("buildtool_depend")

    assert "ament_cmake" in buildtools
    assert "ament_cmake_python" in buildtools


def test_package_xml_has_required_runtime_dependencies():
    deps = package_depend_values("depend")

    assert "rclcpp" in deps
    assert "rclpy" in deps
    assert "std_msgs" in deps
    assert "diagnostic_msgs" in deps
    assert "savo_msgs" in deps


def test_package_xml_has_required_exec_dependencies():
    exec_deps = package_depend_values("exec_depend")

    assert "launch" in exec_deps
    assert "launch_ros" in exec_deps
    assert "python3-yaml" in exec_deps
    assert "ament_index_python" in exec_deps
    assert "python3-smbus" in exec_deps


def test_package_xml_has_test_dependencies():
    test_deps = package_depend_values("test_depend")

    assert "ament_lint_auto" in test_deps
    assert "ament_lint_common" in test_deps
    assert "ament_cmake_pytest" in test_deps
    assert "python3-pytest" in test_deps


def test_setup_py_parses_and_names_package():
    text = setup_text()

    ast.parse(text)

    assert 'package_name = "savo_power"' in text
    assert "name=package_name" in text
    assert 'version="0.1.0"' in text
    assert 'maintainer="Ahnaf Tahmid"' in text
    assert 'maintainer_email="tahmidahnaf998@gmail.com"' in text


def test_setup_py_installs_python_package_metadata():
    text = setup_text()

    assert "find_packages" in text
    assert '("share/ament_index/resource_index/packages", ["resource/" + package_name])' in text
    assert '("share/" + package_name, ["package.xml"])' in text


def test_setup_py_has_python_fallback_console_entries():
    text = setup_text()

    expected_entries = [
        "core_ups_node_py = savo_power.nodes.ups_hat_node_py:main_core",
        "edge_ups_node_py = savo_power.nodes.ups_hat_node_py:main_edge",
        "base_battery_node_py = savo_power.nodes.kit_battery_node_py:main",
        "power_aggregator_node_py = savo_power.nodes.power_aggregator_node_py:main",
        "power_health_node_py = savo_power.nodes.power_health_node_py:main",
        "power_dashboard_node_py = savo_power.nodes.power_dashboard_node_py:main",
    ]

    for entry in expected_entries:
        assert entry in text


def test_setup_py_has_diagnostic_console_entries():
    text = setup_text()

    expected_entries = [
        "power_i2c_check = savo_power.diagnostics.i2c_power_check:main",
        "ups_check = savo_power.diagnostics.ups_check:main",
        "kit_battery_check = savo_power.diagnostics.kit_battery_check:main",
    ]

    for entry in expected_entries:
        assert entry in text


def test_cmake_parses_required_project_setup():
    text = cmake_text()

    assert "cmake_minimum_required(VERSION 3.16)" in text
    assert "project(savo_power)" in text
    assert "set(CMAKE_CXX_STANDARD 17)" in text
    assert "find_package(ament_cmake REQUIRED)" in text
    assert "find_package(ament_cmake_python REQUIRED)" in text


def test_cmake_has_required_ros_dependencies():
    text = cmake_text()

    assert "find_package(rclcpp REQUIRED)" in text
    assert "find_package(std_msgs REQUIRED)" in text
    assert "find_package(diagnostic_msgs REQUIRED)" in text
    assert "find_package(savo_msgs REQUIRED)" in text


def test_cmake_builds_cpp_default_library_sources():
    text = cmake_text()

    expected_sources = [
        "src/state/power_state.cpp",
        "src/policy/power_policy.cpp",
        "src/drivers/linux_i2c_bus.cpp",
        "src/drivers/ups_hat_driver.cpp",
        "src/drivers/ads7830_driver.cpp",
        "src/aggregation/power_aggregator.cpp",
        "src/health/power_health.cpp",
        "src/nodes/ups_hat_node.cpp",
        "src/nodes/kit_battery_node.cpp",
    ]

    for source in expected_sources:
        assert source in text


def test_cmake_builds_cpp_default_nodes():
    text = cmake_text()

    expected_nodes = [
        "savo_power_add_node(core_ups_node src/nodes/core_ups_node.cpp)",
        "savo_power_add_node(edge_ups_node src/nodes/edge_ups_node.cpp)",
        "savo_power_add_node(base_battery_node src/nodes/base_battery_node.cpp)",
        "savo_power_add_node(power_aggregator_node src/nodes/power_aggregator_node.cpp)",
        "savo_power_add_node(power_health_node src/nodes/power_health_node.cpp)",
        "savo_power_add_node(power_dashboard_node src/nodes/power_dashboard_node.cpp)",
    ]

    for node in expected_nodes:
        assert node in text


def test_cmake_installs_python_package_and_runtime_assets():
    text = cmake_text()

    assert "ament_python_install_package(${PROJECT_NAME})" in text

    assert "foreach(dir config launch)" in text
    assert "DESTINATION share/${PROJECT_NAME}/${dir}" in text

    assert "file(GLOB SAVO_POWER_SCRIPTS" in text
    assert "PROGRAMS ${SAVO_POWER_SCRIPTS}" in text
    assert "DESTINATION lib/${PROJECT_NAME}" in text


def test_cmake_exports_library_and_dependencies():
    text = cmake_text()

    assert "ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)" in text
    assert "ament_export_include_directories(include)" in text
    assert "ament_export_dependencies(" in text
    assert "ament_package()" in text


def test_resource_marker_exists():
    path = Path("resource") / PACKAGE_NAME

    assert path.is_file()


def test_expected_top_level_directories_exist():
    for path in [
        "include",
        "src",
        "savo_power",
        "config",
        "launch",
        "scripts",
        "test",
        "resource",
    ]:
        assert Path(path).exists(), path


def test_python_package_has_init_file():
    assert Path("savo_power/__init__.py").is_file()


def test_no_metadata_enables_shutdown():
    for path in [
        Path("package.xml"),
        Path("setup.py"),
        Path("CMakeLists.txt"),
    ]:
        text = path.read_text()

        assert "automatic_shutdown_enabled: true" not in text
        assert "automatic_shutdown_enabled=True" not in text
        assert "shutdown:=true" not in text
