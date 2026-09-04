"""Deployment contracts for the autonomous mapping bringup."""

import ast
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    """Read one package-relative contract fixture."""
    return (ROOT / relative).read_text(encoding="utf-8")


def test_autonomous_mapping_launch_composes_all_core_owners() -> None:
    """AM-7 includes each package that owns part of autonomous mapping."""
    launch = read("launch/autonomous_mapping.launch.py")

    compile(
        launch,
        str(ROOT / "launch/autonomous_mapping.launch.py"),
        "exec",
    )

    required_packages = {
        '"savo_base"',
        '"savo_lidar"',
        '"savo_perception"',
        '"savo_control"',
        '"savo_localization"',
        '"savo_power"',
        '"savo_supervisor"',
        '"savo_nav"',
        '"savo_mapping"',
        '"savo_head"',
    }
    for token in required_packages:
        assert token in launch

    required_launches = {
        '"base_bringup.launch.py"',
        '"lidar_mapping_ready.launch.py"',
        '"perception_bringup.launch.py"',
        '"control_bringup.launch.py"',
        '"localization_bringup.launch.py"',
        '"power_core.launch.py"',
        '"supervisor.launch.py"',
        '"live_mapping_navigation.launch.py"',
        '"autonomous_mapping.launch.xml"',
        '"head_bringup.launch.py"',
        '"location_integration.launch.py"',
    }
    for token in required_launches:
        assert token in launch


def test_autonomous_mapping_launch_is_fail_closed_by_default() -> None:
    """Launch starts the stack but never starts motion or a mission."""
    launch = read("launch/autonomous_mapping.launch.py")

    assert '"control_startup_mode"' in launch
    assert 'default_value="STOP"' in launch
    assert '"base_profile"' in launch
    assert 'default_value="real_robot_v1.yaml"' in launch
    assert '"lidar_profile"' in launch
    assert 'default_value="mapping_rplidar_a1.yaml"' in launch
    assert '"localization_use_vo"' in launch
    assert '"localization_use_vo",\n                default_value="true"' in launch
    assert "_MAP_ID_PATTERN" in launch
    assert "OpaqueFunction(function=_validate_arguments)" in launch

    assert "ExecuteProcess" not in launch
    assert "ros2 action send_goal" not in launch
    assert "ActionClient" not in launch
    assert "create_client" not in launch
    assert '"supervisor_auto_arm", default_value="false"' in launch
    assert '"auto_arm": LaunchConfiguration("supervisor_auto_arm")' in launch
    assert '_python_launch("savo_description"' in launch
    assert '"start_description", default_value="true"' in launch
    assert '"require_locked_geometry", default_value="true"' in launch
    assert '"allow_provisional_geometry", default_value="false"' in launch
    assert '"start_head"' in launch
    assert '"head_enable_tf"' in launch
    assert '"head_enable_tf",\n                default_value="true"' in launch
    assert '"head_camera_mode"' in launch
    assert '"head_camera_mode",\n                default_value="ros"' in launch
    assert '"start_location_lifecycle"' in launch
    assert '"start_semantic_interruption"' in launch
    assert (
        '"start_semantic_interruption", default_value="true"'
        in launch
    )
    assert '"locations_database_path"' in launch
    assert '"coverage_enabled"' in launch
    assert '"coverage_execution_handoff_params_file"' in launch
    assert '"coverage_operation_params_file"' in launch
    assert '"final_scan360_required"' in launch
    assert '"final_head_scan_required"' in launch
    assert '"start_head_action": "true"' in launch
    assert '"start_registration": "true"' in launch
    assert '"start_locations": "true"' in launch


def test_canonical_entry_forwards_only_semantic_interruption_control() -> None:
    """Canonical false disables only continuous semantic interruption."""
    robot = read("launch/robot_bringup.launch.py")
    core = read("launch/core_bringup.launch.py")
    autonomous = read("launch/autonomous_mapping.launch.py")
    mapping = (
        ROOT.parents[1]
        / "core"
        / "savo_mapping"
        / "launch"
        / "autonomous_mapping.launch.xml"
    ).read_text(encoding="utf-8")

    for launch in (robot, core, autonomous):
        assert (
            '"start_semantic_interruption", default_value="true"'
            in launch
        )

    core_branch = robot.split(
        'if role in {"core", "all"}:', maxsplit=1
    )[1].split('if role in {"edge", "all"}:', maxsplit=1)[0]
    assert '"start_semantic_interruption",' in core_branch
    assert "core_arguments[name] = LaunchConfiguration(name)" in core_branch
    assert (
        '"start_semantic_interruption": LaunchConfiguration(\n'
        '                        "start_semantic_interruption"\n'
        "                    )"
        in core
    )
    assert (
        '"supervisor_auto_arm": LaunchConfiguration(\n'
        '                        "supervisor_auto_arm"\n'
        "                    )"
        in core
    )
    assert (
        '"supervisor_state_path": LaunchConfiguration(\n'
        '                        "supervisor_state_path"\n'
        "                    )"
        in core
    )
    assert (
        '"semantic_interruption_enabled": LaunchConfiguration(\n'
        '                "start_semantic_interruption"\n'
        "            )"
        in autonomous
    )
    assert (
        '"require_semantic_autonomous_mapping": LaunchConfiguration(\n'
        '                "start_semantic_interruption"\n'
        "            )"
        in autonomous
    )
    assert 'if="$(var semantic_interruption_enabled)"' in mapping
    assert 'exec="semantic_interruption_coordinator_node"' in mapping

    assert '"start_head", default_value="true"' in autonomous
    assert '"start_location_lifecycle", default_value="true"' in autonomous
    assert '"start_supervisor", default_value="true"' in autonomous
    assert 'start_supervisor = True' in core
    assert '"true" if start_locations else "false"' in core


def test_supervisor_preserves_per_request_semantic_strictness() -> None:
    """Map-only policy does not weaken an explicitly semantic request."""
    supervisor_launch = (
        ROOT.parents[1]
        / "shared"
        / "savo_supervisor"
        / "launch"
        / "supervisor.launch.py"
    ).read_text(encoding="utf-8")
    authority = (
        ROOT.parents[1]
        / "shared"
        / "savo_supervisor"
        / "src"
        / "mission_authority.cpp"
    ).read_text(encoding="utf-8")

    assert (
        "'require_semantic_autonomous_mapping',\n"
        "            default_value='true'"
        in supervisor_launch
    )
    assert (
        "'mission_authorization.'\n"
        "                        'require_semantic_autonomous_mapping'"
        in supervisor_launch
    )
    assert (
        "(!policy_.require_semantic_autonomous_mapping || "
        "capabilities.semantic_mapping_ready)"
        in authority
    )
    assert (
        "(!request.require_semantic || capabilities.semantic_mapping_ready)"
        in authority
    )


def test_map_only_release_accepts_zero_semantic_locations() -> None:
    """Map-only release stays valid without weakening semantic sessions."""
    mapping_root = ROOT.parents[1] / "core" / "savo_mapping"
    locations_root = ROOT.parents[1] / "core" / "savo_locations"
    orchestrator_config = (
        mapping_root / "config" / "autonomous_mapping_orchestrator.yaml"
    ).read_text(encoding="utf-8")
    quality_config = (
        mapping_root / "config" / "map_quality.yaml"
    ).read_text(encoding="utf-8")
    orchestrator = (
        mapping_root
        / "src"
        / "nodes"
        / "autonomous_mapping_orchestrator_node.cpp"
    ).read_text(encoding="utf-8")
    release_repository = (
        locations_root / "src" / "location_release_repository.cpp"
    ).read_text(encoding="utf-8")

    assert "require_approved_location: false" in orchestrator_config
    assert "require_semantic_landmarks: false" in quality_config
    assert (
        "if (require_approved_location_ && approved_count == 0U)"
        in orchestrator
    )
    assert (
        "request->require_approved_location = require_approved_location_"
        in orchestrator
    )
    assert (
        "if (request.require_approved_location && locations.empty())"
        in release_repository
    )


def test_core_runner_forwards_semantic_interruption_default_true() -> None:
    """Production runner exposes map-only mode without changing its default."""
    runner = (
        ROOT.parents[3] / "deploy" / "core" / "run_core.sh"
    ).read_text(encoding="utf-8")

    assert (
        'start_semantic_interruption:="'
        '${SAVO_START_SEMANTIC_INTERRUPTION:-true}"'
        in runner
    )


def test_one_launch_wires_complete_am8_release_chain() -> None:
    """One guarded launch owns mapping, review, quality, and joint release."""
    launch = read("launch/autonomous_mapping.launch.py")
    readme = read("README.md")
    mapping_launch = (
        ROOT.parents[1]
        / "core"
        / "savo_mapping"
        / "launch"
        / "autonomous_mapping.launch.xml"
    ).read_text(encoding="utf-8")

    assert '"start_review_gateway": "true"' in launch
    assert '"nav2_live_mapping.yaml"' in launch
    assert '"nav_params_file"' in launch
    assert '"nav_readiness_params"' in launch
    assert '"location_integration.launch.py"' in launch
    assert '"autonomous_mapping.launch.xml"' in launch
    assert "autonomous_mapping_orchestrator.launch.xml" in mapping_launch
    assert "map_session_manager.launch.xml" in mapping_launch
    assert "contract_version: 3" in readme
    assert "authority_generation" in readme
    assert "require_quality_approval: true" in readme


def test_bringup_installs_am4_and_runtime_dependencies() -> None:
    """The hybrid package installs C++ authority and Python launches."""
    tree = ET.parse(ROOT / "package.xml")
    package = tree.getroot()
    cmake = read("CMakeLists.txt")

    assert package.findtext("version") == "0.6.0"
    assert package.find("./export/build_type").text == "ament_cmake"

    dependencies = {
        element.text for element in package.findall("exec_depend")
    }

    # savo_bringup is shared by Core and Edge, so its manifest must stay
    # role-neutral. Role-private packages are selected by the deployment
    # scripts and launch-time host_role logic instead.
    assert {
        "launch_xml",
        "savo_description",
        "savo_msgs",
        "savo_perception",
        "savo_power",
    }.issubset(dependencies)

    role_private_dependencies = {
        "savo_base",
        "savo_bridge",
        "savo_control",
        "savo_head",
        "savo_lidar",
        "savo_localization",
        "savo_locations",
        "savo_mapping",
        "savo_nav",
        "savo_realsense",
        "savo_speech",
        "savo_supervisor",
        "savo_ui",
        "savo_vo",
    }
    assert dependencies.isdisjoint(role_private_dependencies)

    assert "add_executable(bringup_readiness_node" in cmake
    assert "ament_python_install_package" in cmake
    assert "DIRECTORY config launch" in cmake
    assert "scripts/run_location_lifecycle_runtime" in cmake


def test_readme_documents_two_step_motion_authority() -> None:
    """Operator documentation keeps launch and motion as separate actions."""
    readme = read("README.md")

    assert "autonomous_mapping.launch.py" in readme
    assert "defaults the control" in readme
    assert "layer to `STOP`." in readme
    assert "`STOP`." in readme
    assert "/savo_control/mode_cmd" in readme
    assert "/savo_mapping/autonomous/run" in readme
    assert "same map identifier" in readme
    assert "`savo_description` is included" in readme


def test_all_launch_configurations_are_declared() -> None:
    """Every LaunchConfiguration has a matching launch argument."""
    source = read("launch/autonomous_mapping.launch.py")
    tree = ast.parse(source)

    declared = set()
    referenced = set()

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue

        function_name = getattr(node.func, "id", "")
        if not node.args:
            continue

        first = node.args[0]
        if not isinstance(first, ast.Constant) or not isinstance(
            first.value, str
        ):
            continue

        if function_name == "DeclareLaunchArgument":
            declared.add(first.value)
        elif function_name == "LaunchConfiguration":
            referenced.add(first.value)

    assert referenced <= declared
    assert "map_id" in declared
    assert "control_startup_mode" in declared
