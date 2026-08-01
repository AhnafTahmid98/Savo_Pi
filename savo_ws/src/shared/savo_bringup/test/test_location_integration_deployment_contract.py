"""Contracts for the production typed location lifecycle launch."""

import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    """Read one package-relative location integration artifact."""
    return (ROOT / relative).read_text(encoding="utf-8")


def test_location_lifecycle_launch_contract() -> None:
    """The shared launch contains every production lifecycle process."""
    launch = read("launch/location_integration.launch.py")

    compile(
        launch,
        str(ROOT / "launch/location_integration.launch.py"),
        "exec",
    )

    required_executables = {
        'executable="savo_locations_node"',
        'executable="supervisor_node"',
        'executable="apriltag_confirm_node"',
        'executable="apriltag_confirmation_action_node"',
        'executable="mapped_location_registration_node"',
        'executable="location_review_gateway_node"',
        'executable="navigate_to_location_node"',
    }
    for token in required_executables:
        assert token in launch

    required_configs = {
        '"locations_node.yaml"',
        '"supervisor.yaml"',
        '"location_authorization.yaml"',
        '"apriltag_semantics.yaml"',
        '"apriltag_confirmation_action.yaml"',
        '"semantic_landmarks.yaml"',
        '"location_navigation.yaml"',
    }
    for token in required_configs:
        assert token in launch

    required_arguments = {
        '"start_review_gateway"',
        '"locations_database_path"',
        '"locations_create_parent_directories"',
        '"supervisor_base_enabled"',
        '"supervisor_base_required"',
        '"supervisor_control_enabled"',
        '"supervisor_control_required"',
        '"supervisor_perception_enabled"',
        '"supervisor_perception_required"',
        '"supervisor_lidar_enabled"',
        '"supervisor_lidar_required"',
        '"supervisor_power_enabled"',
        '"supervisor_power_required"',
        '"supervisor_localization_enabled"',
        '"supervisor_localization_required"',
        '"head_minimum_observations"',
        '"review_operation_timeout_s"',
        '"navigation_action_name"',
        '"arrival_confirmation_timeout_s"',
    }
    for token in required_arguments:
        assert token in launch

    production_supervisor_defaults = {
        '"supervisor_base_enabled",\n            default_value="true"',
        '"supervisor_base_required",\n            default_value="true"',
        '"supervisor_control_enabled",\n            default_value="true"',
        '"supervisor_control_required",\n            default_value="true"',
        '"supervisor_perception_enabled",\n            default_value="true"',
        '"supervisor_perception_required",\n            default_value="true"',
        '"supervisor_lidar_enabled",\n            default_value="true"',
        '"supervisor_lidar_required",\n            default_value="true"',
        '"supervisor_power_enabled",\n            default_value="true"',
        '"supervisor_power_required",\n            default_value="true"',
        '"supervisor_localization_enabled",\n            default_value="true"',
        '"supervisor_localization_required",\n            default_value="true"',
    }
    for token in production_supervisor_defaults:
        assert token in launch

    assert "fake_location_nav2_server_node" not in launch
    assert "ParameterValue" in launch
    assert "_critical_exit_handler" in launch
    assert "OnProcessExit" in launch
    assert "Shutdown" in launch


def test_bringup_is_installable_lifecycle_package() -> None:
    """The hybrid package preserves the lifecycle runtime entry point."""
    package_xml = ROOT / "package.xml"
    cmake = read("CMakeLists.txt")
    wrapper = read("scripts/run_location_lifecycle_runtime")

    tree = ET.parse(package_xml)
    root = tree.getroot()

    assert root.findtext("name") == "savo_bringup"
    assert root.findtext("version") == "0.6.0"
    buildtools = {
        element.text for element in root.findall("buildtool_depend")
    }
    assert {"ament_cmake", "ament_cmake_python"} <= buildtools
    assert root.find("./export/build_type").text == "ament_cmake"

    exec_dependencies = {
        element.text for element in root.findall("exec_depend")
    }
    assert {
        "action_msgs",
        "ament_index_python",
        "geometry_msgs",
        "launch",
        "launch_ros",
        "rclpy",
        "savo_head",
        "savo_locations",
        "savo_mapping",
        "savo_msgs",
        "savo_nav",
        "savo_supervisor",
    }.issubset(exec_dependencies)

    assert "ament_python_install_package" in cmake
    assert "run_location_lifecycle_runtime" in cmake
    assert "location_lifecycle_runtime import main" in wrapper


def test_runtime_uses_only_public_lifecycle_boundaries() -> None:
    """The runtime drives public APIs and the production shared launch."""
    runtime = read("savo_bringup/location_lifecycle_runtime.py")

    required_tokens = {
        "RegisterMappedLocation",
        "ReviewLocationCandidate",
        "ResolveLocation",
        "NavigateToLocation",
        "AuthorizeLocationOperation",
        "ManageSystemState",
        "UpdateMapContext",
        '"/savo_mapping/locations/register"',
        '"/savo_mapping/locations/review"',
        '"/savo_locations/resolve"',
        '"/savo_nav/locations/navigate"',
        '"/savo_supervisor/authorize_location_operation"',
        '"location_integration.launch.py"',
        '"start_head_observer:=false"',
        '"supervisor_allow_degraded_motion:=true"',
        '"/savo_mapping/status"',
        '"/savo_head/status"',
        '"/savo_bridge/state"',
        '"/safety/stop"',
        '"/safety/slowdown_factor"',
        "_publish_clear_safety",
        '"navigation_action_name:=/navigate_to_pose"',
        "candidate.approach_pose",
        "approved.tag_pose_map",
        "arrival_confirmed",
        "wait_supervisor_ready",
        "RESULT_SUPERVISOR_NOT_READY",
        "COMMAND_SET_LIVE_MAPPING",
        "COMMAND_SET_SAVED_RELEASE",
        "COMMAND_ARM",
        "PRAGMA integrity_check",
        "PHASE 2D LOCATION LIFECYCLE RUNTIME: PASS",
    }
    for token in required_tokens:
        assert token in runtime

    forbidden_tokens = {
        "/savo_locations/candidates/approve",
        "/savo_locations/candidates/reject",
        "navigation_goal.pose = location.tag_pose_map",
    }
    for token in forbidden_tokens:
        assert token not in runtime
