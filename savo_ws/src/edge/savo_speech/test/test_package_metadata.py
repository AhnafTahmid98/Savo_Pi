#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Package metadata and hybrid-install contract tests."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

from savo_speech.version import VERSION


ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_package_xml_identity_and_build_type():
    root = ET.fromstring(read_text("package.xml"))

    assert root.findtext("name") == "savo_speech"
    assert root.findtext("version") == VERSION
    assert root.findtext("license") == "Proprietary"

    build_type = root.find("./export/build_type")

    assert build_type is not None
    assert build_type.text == "ament_cmake"


def test_package_xml_core_dependencies():
    root = ET.fromstring(read_text("package.xml"))
    dependencies = {
        element.text
        for element in root.findall("depend")
    }

    required = {
        "rclcpp",
        "rclpy",
        "std_msgs",
        "std_srvs",
        "diagnostic_msgs",
        "builtin_interfaces",
    }

    assert required.issubset(dependencies)


def test_setup_py_matches_hybrid_package_contract():
    text = read_text("setup.py")

    assert 'package_name = "savo_speech"' in text
    assert 'version="0.1.0"' in text
    assert '"console_scripts": []' in text
    assert 'python_requires=">=3.10"' in text


def test_python_support_package_layout_exists():
    required_files = [
        "savo_speech/__init__.py",
        "savo_speech/version.py",
        "savo_speech/constants.py",
        "savo_speech/services/__init__.py",
        "savo_speech/clients/__init__.py",
        "savo_speech/nodes/__init__.py",
        "savo_speech/diagnostics/__init__.py",
        "savo_speech/ros/__init__.py",
        "savo_speech/ros/param_utils.py",
        "savo_speech/ros/launch_config.py",
        "launch/speech_bringup.launch.py",
        "launch/speech_dryrun.launch.py",
        "scripts/speech_dryrun_check.py",
        "include/savo_speech/qos_profiles.hpp",
        "src/respeaker_audio_node.cpp",
        "src/speech_manager_node.cpp",
        "config/audio.yaml",
        "config/speech_manager.yaml",
        "config/wake_word.yaml",
        "config/local_stt.yaml",
        "config/local_tts.yaml",
        "config/savomind.yaml",
        "config/topics.yaml",
    ]

    missing = [
        name
        for name in required_files
        if not (ROOT / name).is_file()
    ]

    assert not missing, f"Missing Python support files: {missing}"


def test_cmake_installs_hybrid_package_resources():
    text = read_text("CMakeLists.txt")

    assert "ament_python_install_package(${PROJECT_NAME})" in text
    assert "install(DIRECTORY include/" in text
    assert "install(DIRECTORY launch/" in text
    assert "install(DIRECTORY config/" in text
    assert "README.md" in text
    assert "package.xml" in text


def test_cmake_registers_phase_zero_tests():
    text = read_text("CMakeLists.txt")

    assert (
        "add_pytest_if_exists(\n"
        "    test_package_identity"
        in text
    )

    assert (
        "add_pytest_if_exists(\n"
        "    test_package_metadata"
        in text
    )
