import re
from pathlib import Path


CMAKE_PATH = Path("CMakeLists.txt")


EXPECTED_LIBRARY_SOURCES = [
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


EXPECTED_CPP_NODES = {
    "core_ups_node": "src/nodes/core_ups_node.cpp",
    "edge_ups_node": "src/nodes/edge_ups_node.cpp",
    "base_battery_node": "src/nodes/base_battery_node.cpp",
    "power_aggregator_node": "src/nodes/power_aggregator_node.cpp",
    "power_health_node": "src/nodes/power_health_node.cpp",
    "power_dashboard_node": "src/nodes/power_dashboard_node.cpp",
}


def cmake_text():
    assert CMAKE_PATH.is_file()

    return CMAKE_PATH.read_text()



def compact_cmake_text():
    return re.sub(r"\s+", " ", cmake_text())


def source_paths_from_cmake():
    text = cmake_text()

    return sorted(set(re.findall(r"src/[A-Za-z0-9_./-]+\.cpp", text)))


def test_cmake_file_exists():
    assert CMAKE_PATH.is_file()


def test_cmake_referenced_cpp_sources_exist():
    for source in source_paths_from_cmake():
        assert Path(source).is_file(), source


def test_expected_library_sources_exist():
    for source in EXPECTED_LIBRARY_SOURCES:
        assert Path(source).is_file(), source


def test_expected_cpp_node_sources_exist():
    for target, source in EXPECTED_CPP_NODES.items():
        assert Path(source).is_file(), target


def test_expected_library_sources_are_in_cmake():
    text = cmake_text()

    for source in EXPECTED_LIBRARY_SOURCES:
        assert source in text


def test_expected_cpp_nodes_are_in_cmake():
    text = cmake_text()

    for target, source in EXPECTED_CPP_NODES.items():
        expected = f"savo_power_add_node({target} {source})"

        assert expected in text


def test_cmake_uses_cpp17():
    text = cmake_text()

    assert "set(CMAKE_CXX_STANDARD 17)" in text
    assert "set(CMAKE_CXX_STANDARD_REQUIRED ON)" in text


def test_cmake_has_required_ros_packages():
    text = cmake_text()

    for package in [
        "ament_cmake",
        "ament_cmake_python",
        "rclcpp",
        "std_msgs",
        "diagnostic_msgs",
        "savo_msgs",
    ]:
        assert f"find_package({package} REQUIRED)" in text


def test_cmake_has_main_library_target():
    text = cmake_text()

    assert "add_library(${PROJECT_NAME}" in text
    assert "target_include_directories(${PROJECT_NAME}" in text
    assert "ament_target_dependencies(${PROJECT_NAME}" in text


def test_cmake_has_node_helper_function():
    text = cmake_text()

    assert "function(savo_power_add_node" in text
    assert "add_executable" in text
    assert "target_link_libraries" in text
    assert "ament_target_dependencies" in text


def test_cmake_installs_cpp_targets():
    text = cmake_text()

    assert "install(" in text
    assert "TARGETS" in text
    assert "${PROJECT_NAME}" in text

    for target in EXPECTED_CPP_NODES:
        assert target in text

    compact = compact_cmake_text()

    assert "ARCHIVE DESTINATION lib" in compact
    assert "LIBRARY DESTINATION lib" in compact
    assert (
        "RUNTIME DESTINATION lib/${PROJECT_NAME}" in compact
        or "DESTINATION lib/${PROJECT_NAME}" in compact
    )


def test_cmake_installs_headers():
    text = cmake_text()
    compact = compact_cmake_text()

    assert "install(" in text
    assert "DIRECTORY" in text
    assert "include/" in text
    assert "DESTINATION include" in compact


def test_cmake_installs_config_and_launch_dirs():
    text = cmake_text()
    compact = compact_cmake_text()

    assert "foreach(dir config launch)" in text
    assert "install(" in text
    assert "DIRECTORY" in text
    assert "${dir}/" in text
    assert "DESTINATION share/${PROJECT_NAME}/${dir}" in compact


def test_cmake_installs_python_package():
    text = cmake_text()

    assert "ament_python_install_package(${PROJECT_NAME})" in text


def test_cmake_installs_scripts_as_programs():
    text = cmake_text()

    assert "file(GLOB SAVO_POWER_SCRIPTS" in text
    assert "scripts/*" in text
    assert "install(" in text
    assert "PROGRAMS ${SAVO_POWER_SCRIPTS}" in text
    assert "DESTINATION lib/${PROJECT_NAME}" in text


def test_cmake_exports_library_and_dependencies():
    text = cmake_text()

    assert "ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)" in text
    assert "ament_export_include_directories(include)" in text
    assert "ament_export_dependencies(" in text
    assert "ament_package()" in text


def test_include_tree_has_public_headers():
    include_root = Path("include/savo_power")

    assert include_root.is_dir()

    headers = sorted(include_root.rglob("*.hpp"))

    assert headers


def test_cpp_sources_are_not_empty_placeholders():
    for source in EXPECTED_LIBRARY_SOURCES + list(EXPECTED_CPP_NODES.values()):
        path = Path(source)
        text = path.read_text().strip()

        assert len(text) > 80, source
        assert text != "int main() { return 0; }", source


def test_cpp_node_sources_use_rclcpp_runtime():
    for target, source in EXPECTED_CPP_NODES.items():
        text = Path(source).read_text()

        assert "rclcpp::init" in text, target
        assert "rclcpp::spin" in text, target
        assert "rclcpp::shutdown" in text, target


def test_no_cmake_source_enables_shutdown():
    paths = [CMAKE_PATH]
    paths.extend(Path(source) for source in EXPECTED_LIBRARY_SOURCES)
    paths.extend(Path(source) for source in EXPECTED_CPP_NODES.values())

    for path in paths:
        text = path.read_text()

        assert "automatic_shutdown_enabled: true" not in text, str(path)
        assert "automatic_shutdown_enabled=True" not in text, str(path)
        assert "shutdown:=true" not in text, str(path)
