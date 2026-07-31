"""Contracts for the production typed location lifecycle launch."""

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
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
        '"supervisor_localization_enabled"',
        '"head_minimum_observations"',
        '"review_operation_timeout_s"',
        '"navigation_action_name"',
        '"arrival_confirmation_timeout_s"',
    }
    for token in required_arguments:
        assert token in launch

    assert "fake_location_nav2_server_node" not in launch
    assert "ParameterValue" in launch
    assert "_critical_exit_handler" in launch
    assert "OnProcessExit" in launch
    assert "Shutdown" in launch


def test_bringup_is_installable_lifecycle_package() -> None:
    """The package installs its launch and runtime entry point."""
    package_xml = ROOT / "package.xml"
    setup_py = read("setup.py")
    setup_cfg = read("setup.cfg")

    tree = ET.parse(package_xml)
    root = tree.getroot()

    assert root.findtext("name") == "savo_bringup"
    assert root.findtext("version") == "0.3.0"
    assert root.findtext("buildtool_depend") == "ament_python"
    assert root.find("./export/build_type").text == "ament_python"

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

    compile(setup_py, str(ROOT / "setup.py"), "exec")
    assert "location_integration.launch.py" in setup_py
    assert "run_location_lifecycle_runtime" in setup_py
    assert "location_lifecycle_runtime:main" in setup_py
    assert "README.md" in setup_py
    assert "script_dir=$base/lib/savo_bringup" in setup_cfg
    assert "install_scripts=$base/lib/savo_bringup" in setup_cfg


def test_runtime_uses_only_public_lifecycle_boundaries() -> None:
    """The runtime drives public APIs and the production shared launch."""
    runtime = read("savo_bringup/location_lifecycle_runtime.py")

    required_tokens = {
        "RegisterMappedLocation",
        "ReviewLocationCandidate",
        "ResolveLocation",
        "NavigateToLocation",
        '"/savo_mapping/locations/register"',
        '"/savo_mapping/locations/review"',
        '"/savo_locations/resolve"',
        '"/savo_nav/locations/navigate"',
        '"location_integration.launch.py"',
        '"start_head_observer:=false"',
        '"navigation_action_name:=/navigate_to_pose"',
        "candidate.approach_pose",
        "approved.tag_pose_map",
        "arrival_confirmed",
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
