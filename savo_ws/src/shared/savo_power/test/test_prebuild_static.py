import ast
import os
import py_compile
import re
import stat
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


yaml = pytest.importorskip("yaml")


PACKAGE_NAME = "savo_power"


RUNTIME_DIRS = [
    "include",
    "src",
    "savo_power",
    "config",
    "launch",
    "scripts",
    "resource",
    "test",
]


NO_SHUTDOWN_SCAN_PATHS = [
    "config",
    "launch",
    "scripts",
    "src",
    "include",
    "savo_power",
]


def python_files():
    roots = [
        Path("savo_power"),
        Path("launch"),
        Path("scripts"),
        Path("test"),
    ]

    files = []

    for root in roots:
        if root.exists():
            files.extend(path for path in root.rglob("*.py") if path.is_file())

    for path in Path("scripts").iterdir():
        if path.is_file() and path.suffix == "":
            files.append(path)

    if Path("setup.py").is_file():
        files.append(Path("setup.py"))

    return sorted(set(files))


def yaml_files():
    return sorted(Path("config").rglob("*.yaml"))


def cpp_files():
    files = []

    for root in [Path("src"), Path("include")]:
        if root.exists():
            files.extend(root.rglob("*.cpp"))
            files.extend(root.rglob("*.hpp"))
            files.extend(root.rglob("*.h"))

    return sorted(path for path in files if path.is_file())


def launch_files():
    return sorted(Path("launch").glob("*.launch.py"))


def cmake_text():
    return Path("CMakeLists.txt").read_text()


def package_xml_root():
    return ET.fromstring(Path("package.xml").read_text())


def test_prebuild_root_files_exist():
    for path in [
        "package.xml",
        "setup.py",
        "CMakeLists.txt",
        "resource/savo_power",
    ]:
        assert Path(path).is_file(), path


def test_prebuild_runtime_directories_exist():
    for path in RUNTIME_DIRS:
        assert Path(path).exists(), path


def test_prebuild_python_files_compile():
    files = python_files()

    assert files

    for path in files:
        py_compile.compile(str(path), doraise=True)


def test_prebuild_launch_files_parse_and_define_generate_function():
    files = launch_files()

    assert files

    for path in files:
        tree = ast.parse(path.read_text())

        function_names = {
            node.name
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef)
        }

        assert "generate_launch_description" in function_names, path


def test_prebuild_yaml_configs_parse():
    files = yaml_files()

    assert files

    for path in files:
        data = yaml.safe_load(path.read_text())

        assert isinstance(data, dict), path


def test_prebuild_package_xml_parses_and_matches_package_name():
    root = package_xml_root()

    assert root.findtext("name") == PACKAGE_NAME
    assert root.findtext("version")
    assert root.find("maintainer") is not None
    assert root.findtext("license")


def test_prebuild_cmake_referenced_cpp_sources_exist():
    text = cmake_text()

    sources = sorted(set(re.findall(r"src/[A-Za-z0-9_./-]+\.cpp", text)))

    assert sources

    for source in sources:
        assert Path(source).is_file(), source


def test_prebuild_cmake_referenced_install_dirs_exist():
    text = cmake_text()

    assert "include/" in text
    assert "config" in text
    assert "launch" in text
    assert "scripts" in text

    for path in ["include", "config", "launch", "scripts"]:
        assert Path(path).exists(), path


def test_prebuild_cmake_has_required_targets_and_package_call():
    text = cmake_text()

    for item in [
        "add_library",
        "add_executable",
        "ament_python_install_package",
        "install",
        "ament_package",
    ]:
        assert item in text


def test_prebuild_scripts_are_executable_python_programs():
    scripts = [
        path
        for path in Path("scripts").iterdir()
        if path.is_file()
    ]

    assert scripts

    for path in scripts:
        mode = path.stat().st_mode

        assert mode & stat.S_IXUSR, path
        assert path.read_text().startswith("#!/usr/bin/env python3"), path


def test_prebuild_config_has_no_yaml_null_documents():
    for path in yaml_files():
        data = yaml.safe_load(path.read_text())

        assert data is not None, path


def test_prebuild_package_resource_marker_is_not_directory():
    marker = Path("resource") / PACKAGE_NAME

    assert marker.is_file()


def test_prebuild_python_package_imports_without_ros_runtime_side_effects():
    import savo_power

    assert getattr(savo_power, "__package__", "") == PACKAGE_NAME


def test_prebuild_expected_public_assets_are_non_empty():
    for path in [
        "package.xml",
        "setup.py",
        "CMakeLists.txt",
    ]:
        text = Path(path).read_text().strip()

        assert len(text) > 80, path

    for path in yaml_files() + launch_files():
        text = path.read_text().strip()

        assert len(text) > 20, path


def test_prebuild_cpp_sources_are_non_empty():
    files = cpp_files()

    assert files

    for path in files:
        text = path.read_text().strip()

        assert len(text) > 20, path


def test_prebuild_no_accidental_shutdown_enable_in_runtime_files():
    forbidden = [
        "automatic_shutdown_enabled: true",
        "automatic_shutdown_enabled=True",
        "automatic_shutdown_enabled = True",
        "automatic_shutdown_enabled{true}",
        "automatic_shutdown_enabled(true)",
        "shutdown:=true",
        "request_shutdown(true)",
    ]

    scanned = []

    for root in NO_SHUTDOWN_SCAN_PATHS:
        root_path = Path(root)

        if not root_path.exists():
            continue

        for path in root_path.rglob("*"):
            if not path.is_file():
                continue

            if path.suffix not in {"", ".py", ".yaml", ".cpp", ".hpp", ".h", ".txt"}:
                continue

            scanned.append(path)
            text = path.read_text(errors="ignore")

            for item in forbidden:
                assert item not in text, f"{item} found in {path}"

    assert scanned


def test_prebuild_no_motor_control_terms_in_power_package():
    forbidden = [
        "/cmd_vel",
        "motor_pwm",
        "wheel_speed",
        "pca9685",
        "emergency_recovery",
    ]

    scanned = []

    for root in ["src", "include", "savo_power", "config", "launch"]:
        root_path = Path(root)

        if not root_path.exists():
            continue

        for path in root_path.rglob("*"):
            if not path.is_file():
                continue

            scanned.append(path)
            text = path.read_text(errors="ignore").lower()

            for item in forbidden:
                assert item not in text, f"{item} found in {path}"

    assert scanned


def test_prebuild_no_generated_build_dirs_inside_package():
    forbidden_dirs = [
        "build",
        "install",
        "log",
    ]

    for name in forbidden_dirs:
        assert not Path(name).exists(), name


def test_prebuild_test_files_are_not_empty():
    tests = sorted(Path("test").glob("test_*.py"))

    assert tests

    for path in tests:
        text = path.read_text().strip()

        assert "def test_" in text, path
