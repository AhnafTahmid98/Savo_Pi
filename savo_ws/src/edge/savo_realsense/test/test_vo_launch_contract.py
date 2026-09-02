# Copyright 2026 Ahnaf Tahmid

"""Contracts for the RealSense VO driver and monitor configuration split."""

import ast
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from savo_realsense.constants import DEFAULT_POINTCLOUD_TOPIC
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read_file(path: str) -> str:
    return (PACKAGE_ROOT / path).read_text(encoding="utf-8")


def load_yaml(path: str) -> dict:
    return yaml.safe_load(read_file(path))


def launch_argument_defaults(path: str) -> dict[str, str]:
    tree = ast.parse(read_file(path), filename=path)
    defaults = {}
    for call in (
        node for node in ast.walk(tree) if isinstance(node, ast.Call)
    ):
        if getattr(call.func, "id", "") != "DeclareLaunchArgument":
            continue
        if not call.args or not isinstance(call.args[0], ast.Constant):
            continue
        default = next(
            (
                keyword.value.value
                for keyword in call.keywords
                if keyword.arg == "default_value"
                and isinstance(keyword.value, ast.Constant)
            ),
            None,
        )
        defaults[call.args[0].value] = default
    return defaults


def node_calls(path: str) -> list[ast.Call]:
    tree = ast.parse(read_file(path), filename=path)
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", "") == "Node"
    ]


def keyword_value(call: ast.Call, name: str) -> ast.expr:
    return next(keyword.value for keyword in call.keywords if keyword.arg == name)


def test_vo_launch_separates_driver_and_monitor_configs() -> None:
    launch_text = read_file("launch/realsense_vo.launch.py")

    assert 'LaunchConfiguration("driver_config_file")' in launch_text
    assert 'LaunchConfiguration("monitor_config_file")' in launch_text
    assert launch_text.count("Node(") == 5
    assert 'package="realsense2_camera"' in launch_text
    assert 'executable="realsense2_camera_node"' in launch_text
    assert 'namespace="camera"' in launch_text
    assert 'name="camera"' in launch_text
    assert "parameters=[driver_config_file]" in launch_text
    assert launch_text.count("parameters=[monitor_config_file]") == 1
    assert '"require_vo_health": False' in launch_text
    assert '"require_obstacle_cloud_health": False' in launch_text
    assert "rs_launch.py" not in launch_text
    assert "IncludeLaunchDescription" not in launch_text
    assert '"realsense_d435_camera.yaml"' in launch_text
    assert '"realsense_vo_driver.yaml"' not in launch_text
    assert '"realsense_vo_profile.yaml"' in launch_text


def test_vo_launch_starts_depth_front_min_from_production_nodes_config() -> None:
    launch_path = "launch/realsense_vo.launch.py"
    launch_text = read_file(launch_path)

    assert launch_argument_defaults(launch_path)["use_depth_front_min"] == "true"
    assert '"realsense_d435_nodes.yaml"' in launch_text
    assert "parameters=[depth_front_min_config_file]" in launch_text

    depth_nodes = [
        call
        for call in node_calls(launch_path)
        if ast.literal_eval(keyword_value(call, "package")) == "savo_realsense"
        and ast.literal_eval(keyword_value(call, "executable"))
        == "depth_front_min_node"
    ]
    assert len(depth_nodes) == 1
    depth_node = depth_nodes[0]
    assert ast.literal_eval(keyword_value(depth_node, "name")) == (
        "depth_front_min_node"
    )
    assert ast.literal_eval(keyword_value(depth_node, "output")) == "screen"
    assert ast.unparse(keyword_value(depth_node, "condition")) == (
        "IfCondition(use_depth_front_min)"
    )


def test_observer_color_relay_is_opt_in_and_hardware_equivalent() -> None:
    assert launch_argument_defaults("launch/realsense_vo.launch.py")[
        "enable_observer_color_relay"
    ] == "false"

    relays = [
        call
        for call in node_calls("launch/realsense_vo.launch.py")
        if ast.literal_eval(keyword_value(call, "package")) == "image_transport"
        and ast.literal_eval(keyword_value(call, "executable")) == "republish"
    ]
    assert len(relays) == 1
    relay = relays[0]

    assert ast.literal_eval(keyword_value(relay, "name")) == (
        "d435_observer_color_republisher"
    )
    assert ast.unparse(keyword_value(relay, "condition")) == (
        "IfCondition(enable_observer_color_relay)"
    )
    assert ast.literal_eval(keyword_value(relay, "parameters")) == [{
        "in_transport": "raw",
        "out_transport": "compressed",
    }]
    assert ast.literal_eval(keyword_value(relay, "remappings")) == [
        ("in", "/camera/camera/color/image_raw"),
        (
            "out/compressed",
            "/savo_observer/d435/color/image_raw/compressed",
        ),
    ]


def test_observer_color_relay_runtime_dependencies_are_declared() -> None:
    package = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    runtime_dependencies = {
        element.text for element in package.findall("exec_depend")
    }
    assert {
        "compressed_image_transport",
        "image_transport",
    }.issubset(runtime_dependencies)


def test_canonical_bringup_starts_one_camera_and_production_color_relay() -> None:
    launch_path = "launch/realsense_bringup.launch.py"
    launch_text = read_file(launch_path)

    assert launch_argument_defaults(launch_path)[
        "enable_observer_color_relay"
    ] == "true"
    assert launch_text.count('executable="realsense2_camera_node"') == 1
    assert launch_text.count("Node(") == 4
    assert '"realsense_d435_camera.yaml"' in launch_text
    assert '"realsense_d435_nodes.yaml"' in launch_text
    assert 'executable="depth_front_min_node"' in launch_text
    assert 'executable="camera_health_node"' in launch_text
    assert 'executable="camera_topic_monitor_node"' not in launch_text

    relays = [
        call
        for call in node_calls(launch_path)
        if ast.literal_eval(keyword_value(call, "package")) == "image_transport"
        and ast.literal_eval(keyword_value(call, "executable")) == "republish"
    ]
    assert len(relays) == 1
    relay = relays[0]
    assert ast.unparse(keyword_value(relay, "condition")) == (
        "IfCondition(enable_observer_color_relay)"
    )
    assert ast.literal_eval(keyword_value(relay, "parameters")) == [{
        "in_transport": "raw",
        "out_transport": "compressed",
    }]
    assert ast.literal_eval(keyword_value(relay, "remappings")) == [
        ("in", "/camera/camera/color/image_raw"),
        (
            "out/compressed",
            "/savo_observer/d435/color/image_raw/compressed",
        ),
    ]


def test_vo_launch_driver_config_is_validated_production_profile() -> None:
    config = load_yaml("config/realsense_d435_camera.yaml")
    params = config["/camera/camera"]["ros__parameters"]

    assert params["camera_name"] == "camera"
    assert params["camera_namespace"] == "camera"
    assert params["serial_no"] == "801212070967"
    assert params["enable_color"] is True
    assert params["enable_depth"] is True
    assert params["depth_module.depth_profile"] == "848x480x30"
    assert params["rgb_camera.color_profile"] == "640x480x30"
    assert params["color_qos"] == "SENSOR_DATA"
    assert params["depth_qos"] == "SENSOR_DATA"
    assert params["align_depth.enable"] is True
    assert params["enable_sync"] is True
    assert params["pointcloud__neon_.enable"] is True
    assert params["pointcloud__neon_.stream_filter"] == 1
    assert params["pointcloud__neon_.stream_index_filter"] == 0
    assert params["pointcloud__neon_.allow_no_texture_points"] is True
    assert params["pointcloud__neon_.ordered_pc"] is False
    assert params["pointcloud__neon_.pointcloud_qos"] == "SENSOR_DATA"
    assert "pointcloud.enable" not in params
    assert "pointcloud.stream_filter" not in params
    assert "pointcloud.stream_index_filter" not in params


def test_vo_launch_driver_disables_tf_infrared_and_motion_streams() -> None:
    config = load_yaml("config/realsense_d435_camera.yaml")
    params = config["/camera/camera"]["ros__parameters"]

    assert params["publish_tf"] is False
    assert params["tf_publish_rate"] == 0.0
    assert params["enable_infra"] is False
    assert params["enable_infra1"] is False
    assert params["enable_infra2"] is False
    assert params["enable_gyro"] is False
    assert params["enable_accel"] is False


def test_legacy_vo_driver_matches_validated_production_driver() -> None:
    legacy = load_yaml("config/realsense_vo_driver.yaml")
    production = load_yaml("config/realsense_d435_camera.yaml")

    assert legacy == production


def test_vo_monitor_profile_keeps_diagnostics_and_lightweight_health_split() -> None:
    config = load_yaml("config/realsense_vo_profile.yaml")

    assert set(config) == {
        "camera_topic_monitor_node",
        "camera_health_node",
    }
    monitor = config["camera_topic_monitor_node"]["ros__parameters"]
    health = config["camera_health_node"]["ros__parameters"]

    assert monitor["expected_pointcloud_hz"] == 30.0
    assert monitor["require_pointcloud"] is True
    assert monitor["expected_aligned_depth_hz"] == 30.0
    assert monitor["require_aligned_depth"] is True
    assert health["depth_signal_topic"] == "/depth/min_front_m"
    assert health["vo_health_topic"] == "/vo/health"
    assert health["require_depth_signal"] is True
    assert health["require_vo_health"] is False
    assert health["require_obstacle_cloud_health"] is False


def test_vo_pointcloud_topic_contract_remains_raw_filter_input() -> None:
    assert DEFAULT_POINTCLOUD_TOPIC == "/camera/camera/depth/color/points"


def test_cmake_installs_vo_launch_and_driver_config_directories() -> None:
    cmake = read_file("CMakeLists.txt")

    assert (PACKAGE_ROOT / "launch" / "realsense_vo.launch.py").is_file()
    assert (PACKAGE_ROOT / "config" / "realsense_vo_driver.yaml").is_file()
    assert re.search(
        r"install\(DIRECTORY\s+launch/\s+"
        r"DESTINATION share/\$\{PROJECT_NAME\}/launch",
        cmake,
    )
    assert re.search(
        r"install\(DIRECTORY\s+config/\s+"
        r"DESTINATION share/\$\{PROJECT_NAME\}/config",
        cmake,
    )
