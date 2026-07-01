from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


CPP_HEADERS = [
    "rgbd_odometry_node.hpp",
    "vo_health_node.hpp",
    "vo_diagnostics_node.hpp",
    "vo_republisher_node.hpp",
    "vo_constants.hpp",
    "vo_types.hpp",
    "tracking_quality.hpp",
    "timestamp_sync.hpp",
    "covariance_builder.hpp",
    "geometry_utils.hpp",
]


CPP_SOURCES = [
    "rgbd_odometry_node.cpp",
    "rgbd_odometry_main.cpp",
    "vo_health_node.cpp",
    "vo_health_main.cpp",
    "vo_diagnostics_node.cpp",
    "vo_diagnostics_main.cpp",
    "vo_republisher_node.cpp",
    "vo_republisher_main.cpp",
    "vo_types.cpp",
    "tracking_quality.cpp",
    "timestamp_sync.cpp",
    "covariance_builder.cpp",
    "geometry_utils.cpp",
]


CPP_EXECUTABLES = [
    "rgbd_odometry_node",
    "vo_health_node",
    "vo_diagnostics_node",
    "vo_republisher_node",
]


PY_FALLBACKS = [
    "rgbd_odometry_node_py",
    "vo_health_node_py",
    "vo_diagnostics_node_py",
    "vo_republisher_node_py",
]


def test_cpp_headers_exist():
    include_dir = PACKAGE_ROOT / "include" / "savo_vo"

    for header in CPP_HEADERS:
        assert (include_dir / header).exists(), f"Missing C++ header: {header}"


def test_cpp_sources_exist():
    src_dir = PACKAGE_ROOT / "src"

    for source in CPP_SOURCES:
        assert (src_dir / source).exists(), f"Missing C++ source: {source}"


def test_cmake_registers_cpp_default_nodes():
    text = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "add_library(savo_vo_core" in text

    for executable in CPP_EXECUTABLES:
        assert f"add_executable({executable}" in text, (
            f"C++ executable not registered: {executable}"
        )


def test_python_fallback_scripts_exist_and_install():
    scripts_dir = PACKAGE_ROOT / "scripts"
    cmake_text = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    for fallback in PY_FALLBACKS:
        script = scripts_dir / fallback

        assert script.exists(), f"Missing Python fallback: {fallback}"
        assert script.stat().st_mode & 0o111, f"Fallback is not executable: {fallback}"
        assert f"scripts/{fallback}" in cmake_text, (
            f"Fallback not installed in CMakeLists.txt: {fallback}"
        )


def test_no_direct_camera_package_dependency_in_cpp_files():
    checked_paths = []

    checked_paths.extend((PACKAGE_ROOT / "include" / "savo_vo").glob("*.hpp"))
    checked_paths.extend((PACKAGE_ROOT / "src").glob("*.cpp"))
    checked_paths.append(PACKAGE_ROOT / "CMakeLists.txt")
    checked_paths.append(PACKAGE_ROOT / "package.xml")

    for path in checked_paths:
        text = path.read_text(encoding="utf-8")
        assert "savo_realsense" not in text, f"Direct dependency found in {path}"
