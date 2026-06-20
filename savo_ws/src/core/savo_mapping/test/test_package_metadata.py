#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Package metadata tests for Robot Savo mapping."""

from __future__ import annotations

import importlib
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


# =============================================================================
# Paths
# =============================================================================
PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _setup_py_version() -> str:
    text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    match = re.search(r'version\s*=\s*["\']([^"\']+)["\']', text)

    if match is None:
        raise AssertionError("setup.py version not found.")

    return match.group(1)


def _package_xml_version() -> str:
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    version = root.findtext("version")

    if version is None:
        raise AssertionError("package.xml version not found.")

    return version.strip()


# =============================================================================
# Python package metadata
# =============================================================================
def test_python_package_imports() -> None:
    import savo_mapping

    assert savo_mapping.__name__ == "savo_mapping"
    assert hasattr(savo_mapping, "__version__")
    assert hasattr(savo_mapping, "get_package_info")


def test_package_version_is_present() -> None:
    import savo_mapping

    assert isinstance(savo_mapping.__version__, str)
    assert savo_mapping.__version__
    assert savo_mapping.__version__ == _package_xml_version()
    assert savo_mapping.__version__ == _setup_py_version()


def test_get_package_info_contains_expected_keys() -> None:
    import savo_mapping

    info = savo_mapping.get_package_info()

    assert info["package"] == "savo_mapping"
    assert info["robot_name"] == "Robot Savo"
    assert info["version"] == savo_mapping.__version__

    assert "node_defaults" in info
    assert "topics" in info
    assert isinstance(info["node_defaults"], dict)
    assert isinstance(info["topics"], dict)


def test_get_package_info_node_defaults() -> None:
    import savo_mapping

    defaults = savo_mapping.get_package_info()["node_defaults"]

    assert defaults["mapping_supervisor"] == "mapping_supervisor_node"
    assert defaults["mapping_mode_manager"] == "mapping_mode_manager_node"
    assert defaults["frontier_explorer"] == "frontier_explorer_node"
    assert defaults["pointcloud_monitor"] == "pointcloud_monitor_node"


def test_get_package_info_topics() -> None:
    import savo_mapping

    topics = savo_mapping.get_package_info()["topics"]

    assert topics["scan"] == "/scan"
    assert topics["odom"] == "/odometry/filtered"
    assert topics["map"] == "/map"
    assert topics["map_metadata"] == "/map_metadata"
    assert topics["realsense_points"] == "/savo_edge/realsense/points"
    assert topics["mapping_ready"] == "/savo_mapping/ready"
    assert topics["mapping_status"] == "/savo_mapping/status"
    assert topics["mapping_mode"] == "/savo_mapping/mode"
    assert topics["mapping_workflow_phase"] == "/savo_mapping/workflow_phase"
    assert topics["semantic_status"] == "/savo_mapping/semantic_status"
    assert topics["exploration_status"] == "/savo_mapping/exploration_status"
    assert topics["mapping_dashboard"] == "/savo_mapping/dashboard"
    assert topics["mapping_dashboard_text"] == "/savo_mapping/dashboard_text"
    assert topics["frontier_explorer_status"] == "/savo_mapping/frontier_explorer/status"
    assert topics["frontier_explorer_goal"] == "/savo_mapping/frontier_explorer/goal"
    assert topics["frontier_explorer_goal_pose"] == "/savo_mapping/frontier_explorer/goal_pose"
    assert topics["location_bridge_status"] == "/savo_mapping/location_bridge/status"
    assert topics["known_locations"] == "/savo_mapping/known_locations"
    assert topics["location_confirmation_command"] == "/savo_mapping/location_confirmation_command"
    assert topics["location_confirmation_status"] == "/savo_mapping/location_confirmation/status"
    assert topics["location_confirmation_result"] == "/savo_mapping/location_confirmation/result"
    assert topics["apriltag_observation"] == "/savo_mapping/apriltag_observation"
    assert topics["apriltag_mapper_status"] == "/savo_mapping/apriltag_mapper/status"
    assert topics["apriltag_mapper_result"] == "/savo_mapping/apriltag_mapper/result"
    assert topics["location_candidate"] == "/savo_mapping/location_candidate"
    assert topics["map_quality"] == "/savo_mapping/map_quality"


def test_version_module_imports() -> None:
    import savo_mapping

    module = importlib.import_module("savo_mapping.version")

    assert module.PACKAGE_NAME == "savo_mapping"
    assert module.ROBOT_NAME == "Robot Savo"
    assert module.__version__ == savo_mapping.__version__
    assert callable(module.get_package_info)


def test_version_module_main_runs_without_error() -> None:
    result = subprocess.run(
        [sys.executable, "-m", "savo_mapping.version"],
        cwd=PACKAGE_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0


# =============================================================================
# Source tree structure
# =============================================================================
def test_expected_top_level_files_exist() -> None:
    expected_files = (
        "package.xml",
        "CMakeLists.txt",
        "setup.py",
        "setup.cfg",
        "savo_mapping/__init__.py",
        "savo_mapping/version.py",
        "savo_mapping/constants.py",
    )

    for relative_path in expected_files:
        assert (PACKAGE_ROOT / relative_path).exists(), relative_path


def test_expected_source_directories_exist() -> None:
    expected_dirs = (
        "savo_mapping",
        "savo_mapping/models",
        "savo_mapping/utils",
        "savo_mapping/ros",
        "savo_mapping/diagnostics",
        "savo_mapping/exploration",
        "savo_mapping/nodes",
        "savo_mapping/semantic",
        "scripts",
        "test",
    )

    for relative_path in expected_dirs:
        path = PACKAGE_ROOT / relative_path

        assert path.exists(), relative_path
        assert path.is_dir(), relative_path


def test_expected_init_files_exist() -> None:
    expected_files = (
        "savo_mapping/__init__.py",
        "savo_mapping/models/__init__.py",
        "savo_mapping/utils/__init__.py",
        "savo_mapping/ros/__init__.py",
        "savo_mapping/diagnostics/__init__.py",
        "savo_mapping/exploration/__init__.py",
        "savo_mapping/nodes/__init__.py",
        "savo_mapping/semantic/__init__.py",
        "scripts/__init__.py",
    )

    for relative_path in expected_files:
        path = PACKAGE_ROOT / relative_path

        assert path.exists(), relative_path
        assert path.is_file(), relative_path


def test_expected_semantic_files_exist() -> None:
    expected_files = [
        "savo_mapping/semantic/__init__.py",
        "savo_mapping/semantic/apriltag_mapper.py",
        "savo_mapping/semantic/apriltag_observation.py",
        "savo_mapping/semantic/human_label_session.py",
        "savo_mapping/semantic/location_bridge.py",
        "savo_mapping/semantic/location_candidate.py",
        "savo_mapping/semantic/location_confirmation.py",
        "savo_mapping/semantic/location_record.py",
        "savo_mapping/semantic/semantic_landmark_store.py",
        "savo_mapping/semantic/tag_database.py",
    ]

    for relative_path in expected_files:
        assert (PACKAGE_ROOT / relative_path).exists()


def test_expected_node_files_exist() -> None:
    expected_files = [
        "savo_mapping/nodes/__init__.py",
        "savo_mapping/nodes/mapping_supervisor_node.py",
        "savo_mapping/nodes/mapping_mode_manager_node.py",
        "savo_mapping/nodes/mapping_dashboard_node.py",
        "savo_mapping/nodes/frontier_explorer_node.py",
        "savo_mapping/nodes/location_bridge_node.py",
        "savo_mapping/nodes/location_confirmation_node.py",
        "savo_mapping/nodes/apriltag_mapper_node.py",
        "savo_mapping/nodes/pointcloud_monitor_node.py",
    ]

    for relative_path in expected_files:
        assert (PACKAGE_ROOT / relative_path).exists()


def test_semantic_package_imports() -> None:
    import savo_mapping.semantic as semantic

    assert semantic.LocationCandidate.__name__ == "LocationCandidate"
    assert semantic.SemanticLandmarkStore.__name__ == "SemanticLandmarkStore"
    assert semantic.TagDatabase.__name__ == "TagDatabase"
    assert callable(semantic.make_apriltag_location_candidate)
    assert callable(semantic.make_apriltag_observation)
    assert callable(semantic.make_tag_record)


def test_mapping_node_modules_import() -> None:
    from savo_mapping.nodes.apriltag_mapper_node import AprilTagMapperNodeStatus
    from savo_mapping.nodes.mapping_dashboard_node import (
        DashboardInput,
        MappingDashboardSnapshot,
    )
    from savo_mapping.nodes.frontier_explorer_node import (
        FrontierExplorerNodeStatus,
    )
    from savo_mapping.nodes.location_bridge_node import LocationBridgeNodeStatus
    from savo_mapping.nodes.location_confirmation_node import (
        LocationConfirmationNodeStatus,
    )
    from savo_mapping.nodes.mapping_mode_manager_node import (
        COMMAND_SEMANTIC_REVIEW,
        PHASE_SEMANTIC_REVIEW,
        ModeSessionState,
    )
    from savo_mapping.nodes.mapping_supervisor_node import (
        ExplorationRuntimeStatus,
        SemanticRuntimeStatus,
    )
    from savo_mapping.nodes.pointcloud_monitor_node import PointcloudMonitorSnapshot

    assert COMMAND_SEMANTIC_REVIEW == "semantic_review"
    assert PHASE_SEMANTIC_REVIEW == "semantic_review"

    state = ModeSessionState(
        mode="manual_mapping",
        active=True,
        workflow_phase=PHASE_SEMANTIC_REVIEW,
        semantic_review_active=True,
        semantic_candidate_key="a201",
    )

    assert state.to_dict()["workflow_phase"] == "semantic_review"
    assert state.to_dict()["semantic_candidate_key"] == "a201"

    semantic_status = SemanticRuntimeStatus(
        enabled=False,
        ok=True,
        message="ok",
    )
    exploration_status = ExplorationRuntimeStatus(
        enabled=True,
        ok=True,
        strategy="frontier",
        supported_strategies=("frontier",),
        message="ok",
    )

    assert semantic_status.to_dict()["message"] == "ok"
    assert exploration_status.to_dict()["strategy"] == "frontier"

    dashboard_input = DashboardInput("/test")
    dashboard_input.update('{"mode":"manual_mapping"}')

    snapshot = MappingDashboardSnapshot(
        ok=True,
        mode="manual_mapping",
        workflow_phase="semantic_review",
        ready=False,
        degraded=True,
        active=True,
        message="ok",
    )

    assert dashboard_input.data["mode"] == "manual_mapping"
    assert snapshot.to_dict()["workflow_phase"] == "semantic_review"
    assert "Robot Savo mapping dashboard" in snapshot.to_text()

    frontier_status = FrontierExplorerNodeStatus(
        enabled=False,
        ok=True,
        decision="disabled",
        message="ok",
        map_ready=False,
        odom_ready=False,
        safety_stop=False,
    )

    assert frontier_status.to_dict()["decision"] == "disabled"

    location_bridge_status = LocationBridgeNodeStatus(
        enabled=False,
        ok=True,
        decision="disabled",
        message="ok",
    )

    assert location_bridge_status.to_dict()["decision"] == "disabled"

    location_confirmation_status = LocationConfirmationNodeStatus(
        enabled=False,
        ok=True,
        decision="disabled",
        message="ok",
    )

    assert location_confirmation_status.to_dict()["decision"] == "disabled"

    apriltag_status = AprilTagMapperNodeStatus(
        enabled=False,
        ok=True,
        decision="disabled",
        message="ok",
    )

    assert apriltag_status.to_dict()["decision"] == "disabled"

    pointcloud_snapshot = PointcloudMonitorSnapshot(
        enabled=False,
        ok=True,
        decision="disabled",
        message="ok",
        topic="/points",
        expected_frame="camera_depth_optical_frame",
    )

    assert pointcloud_snapshot.to_dict()["decision"] == "disabled"


# =============================================================================
# package.xml
# =============================================================================
def test_package_xml_exists_and_parses() -> None:
    package_xml = PACKAGE_ROOT / "package.xml"

    tree = ET.parse(package_xml)
    root = tree.getroot()

    assert root.tag == "package"


def test_package_xml_name_version_description() -> None:
    import savo_mapping

    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()

    name = root.findtext("name")
    version = root.findtext("version")
    description = root.findtext("description")

    assert name == "savo_mapping"
    assert version == savo_mapping.__version__
    assert description is not None
    assert "Robot Savo" in description or "mapping" in description.lower()


def test_package_xml_maintainer_and_license() -> None:
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()

    maintainer = root.find("maintainer")
    license_text = root.findtext("license")

    assert maintainer is not None
    assert maintainer.text is not None
    assert maintainer.text.strip()

    assert "email" in maintainer.attrib
    assert maintainer.attrib["email"]

    assert license_text is not None
    assert license_text.strip()


def test_package_xml_buildtool_dependencies() -> None:
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()

    buildtool_deps = {
        item.text
        for item in root.findall("buildtool_depend")
        if item.text
    }

    assert "ament_cmake" in buildtool_deps
    assert "ament_python" in buildtool_deps


def test_package_xml_runtime_dependencies() -> None:
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()

    deps = {
        item.text
        for item in root.findall("depend")
        if item.text
    }

    expected = {
        "rclcpp",
        "rclpy",
        "std_msgs",
        "geometry_msgs",
        "sensor_msgs",
        "nav_msgs",
        "tf2_msgs",
    }

    missing = expected - deps

    assert not missing, f"Missing package.xml dependencies: {sorted(missing)}"


def test_package_xml_export_build_type() -> None:
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()

    export = root.find("export")

    assert export is not None

    build_type = export.findtext("build_type")

    assert build_type == "ament_cmake"


# =============================================================================
# Build files
# =============================================================================
def test_cmakelists_contains_expected_install_sections() -> None:
    text = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "project(savo_mapping)" in text
    assert "find_package(ament_cmake REQUIRED)" in text
    assert "find_package(ament_cmake_python REQUIRED)" in text
    assert "ament_python_install_package" in text
    assert "install(" in text
    assert "ament_package()" in text


def test_setup_py_contains_expected_package_data() -> None:
    text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert "savo_mapping" in text
    assert "console_scripts" in text
    assert "mapping_readiness_cli" in text
    assert "mapping_smoke_test_cli" in text
    assert "manual_mapping_cli" in text
    assert "autonomous_mapping_cli" in text
    assert "frontier_explorer_node" in text
    assert "location_bridge_node" in text
    assert "location_confirmation_node" in text
    assert "apriltag_mapper_node" in text


def test_setup_cfg_contains_script_paths() -> None:
    text = (PACKAGE_ROOT / "setup.cfg").read_text(encoding="utf-8")

    assert "[develop]" in text
    assert "[install]" in text
    assert "script_dir" in text
    assert "install_scripts" in text


# =============================================================================
# Constants module
# =============================================================================
def test_constants_module_imports() -> None:
    constants = importlib.import_module("savo_mapping.constants")

    assert constants.PACKAGE_NAME == "savo_mapping"
    assert constants.ROBOT_NAME == "Robot Savo"

    assert constants.TOPIC_SCAN == "/scan"
    assert constants.TOPIC_ODOM == "/odometry/filtered"
    assert constants.TOPIC_MAP == "/map"
    assert constants.TOPIC_REALSENSE_POINTS == "/savo_edge/realsense/points"

    assert constants.FRAME_MAP == "map"
    assert constants.FRAME_ODOM == "odom"
    assert constants.FRAME_BASE_LINK == "base_link"
    assert constants.FRAME_LASER == "laser"


def test_constants_timeouts_are_positive() -> None:
    constants = importlib.import_module("savo_mapping.constants")

    assert constants.DEFAULT_PUBLISH_RATE_HZ > 0.0
    assert constants.DEFAULT_SCAN_STALE_TIMEOUT_S > 0.0
    assert constants.DEFAULT_ODOM_STALE_TIMEOUT_S > 0.0
    assert constants.DEFAULT_TF_TIMEOUT_S > 0.0
    assert constants.DEFAULT_MAP_STALE_TIMEOUT_S > 0.0
    assert constants.DEFAULT_POINTCLOUD_STALE_TIMEOUT_S > 0.0
