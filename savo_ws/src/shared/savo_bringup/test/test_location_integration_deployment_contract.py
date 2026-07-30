from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_location_integration_launch_contract() -> None:
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

    assert "fake_location_nav2_server_node" not in launch
    assert "_critical_exit_handler" in launch
    assert "OnProcessExit" in launch
    assert "Shutdown" in launch


def test_bringup_is_installable_ament_python_package() -> None:
    package_xml = ROOT / "package.xml"
    setup_py = read("setup.py")

    tree = ET.parse(package_xml)
    root = tree.getroot()

    assert root.findtext("name") == "savo_bringup"
    assert root.findtext("version") == "0.2.0"
    assert root.findtext("buildtool_depend") == "ament_python"
    assert root.find("./export/build_type").text == "ament_python"

    exec_dependencies = {
        element.text for element in root.findall("exec_depend")
    }
    assert {
        "ament_index_python",
        "launch",
        "launch_ros",
        "savo_head",
        "savo_locations",
        "savo_mapping",
        "savo_msgs",
        "savo_nav",
        "savo_supervisor",
    }.issubset(exec_dependencies)

    compile(setup_py, str(ROOT / "setup.py"), "exec")
    assert "location_integration.launch.py" in setup_py
    assert "resource" in setup_py
