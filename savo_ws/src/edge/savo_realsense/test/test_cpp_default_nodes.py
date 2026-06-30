# Copyright 2026 Ahnaf Tahmid
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read_file(path: str) -> str:
    return (PACKAGE_ROOT / path).read_text(encoding="utf-8")


def test_cmake_builds_camera_monitor_nodes_as_cpp_defaults() -> None:
    cmake = read_file("CMakeLists.txt")

    assert "add_executable(camera_health_node" in cmake
    assert "src/camera_health_main.cpp" in cmake
    assert "add_executable(camera_topic_monitor_node" in cmake
    assert "src/camera_topic_monitor_main.cpp" in cmake
    assert "add_executable(depth_front_min_node" in cmake
    assert "src/depth_front_min_main.cpp" in cmake


def test_python_monitor_nodes_are_fallback_only() -> None:
    cmake = read_file("CMakeLists.txt")
    setup = read_file("setup.py")

    assert "scripts/camera_health_node_py" in cmake
    assert "scripts/camera_topic_monitor_node_py" in cmake
    assert "camera_health_node_py = savo_realsense.nodes.camera_health_node:main" in setup
    assert (
        "camera_topic_monitor_node_py = "
        "savo_realsense.nodes.camera_topic_monitor_node:main"
    ) in setup

    assert "scripts/camera_health_node\n" not in cmake
    assert "scripts/camera_topic_monitor_node\n" not in cmake
