"""Deployment contracts for the AM-5 autonomous mapping bringup."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_autonomous_mapping_launch_composes_all_core_owners() -> None:
    """AM-5 includes each package that owns part of autonomous mapping."""
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
    assert 'default_value="false"' in launch
    assert "_MAP_ID_PATTERN" in launch
    assert "OpaqueFunction(function=_validate_arguments)" in launch

    assert "ExecuteProcess" not in launch
    assert "ros2 action send_goal" not in launch
    assert "ActionClient" not in launch
    assert "create_client" not in launch
    assert '_python_launch("savo_description"' not in launch
    assert '"start_head"' in launch
    assert '"head_enable_tf"' in launch
    assert 'default_value="false"' in launch
    assert '"head_camera_mode"' in launch
    assert 'default_value="disabled"' in launch


def test_bringup_installs_am4_and_runtime_dependencies() -> None:
    """The installed bringup package contains AM-5 and its package owners."""
    tree = ET.parse(ROOT / "package.xml")
    package = tree.getroot()
    setup_py = read("setup.py")

    assert package.findtext("version") == "0.5.0"

    dependencies = {
        element.text for element in package.findall("exec_depend")
    }
    assert {
        "launch_xml",
        "savo_base",
        "savo_control",
        "savo_lidar",
        "savo_localization",
        "savo_mapping",
        "savo_head",
        "savo_nav",
        "savo_perception",
        "savo_power",
        "savo_supervisor",
    }.issubset(dependencies)

    compile(setup_py, str(ROOT / "setup.py"), "exec")
    assert "autonomous_mapping.launch.py" in setup_py
    assert "location_integration.launch.py" in setup_py
    assert 'maintainer="Ahnaf Tahmid"' in setup_py
    assert 'license="Proprietary"' in setup_py


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
    assert "savo_description` is intentionally not included" in readme


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
