"""Contracts for the canonical staged Core startup."""

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = PACKAGE_ROOT.parents[3]
CORE_LAUNCH = PACKAGE_ROOT / "launch" / "core_bringup.launch.py"
ROBOT_LAUNCH = PACKAGE_ROOT / "launch" / "robot_bringup.launch.py"
EDGE_LAUNCH = PACKAGE_ROOT / "launch" / "edge_bringup.launch.py"
AUTONOMOUS_LAUNCH = PACKAGE_ROOT / "launch" / "autonomous_mapping.launch.py"

CORE_STAGE_DEFAULTS = {
    "description_start_delay_s": "0.0",
    "base_start_delay_s": "3.0",
    "lidar_start_delay_s": "6.0",
    "perception_start_delay_s": "9.0",
    "control_start_delay_s": "12.0",
    "localization_start_delay_s": "17.0",
    "power_start_delay_s": "22.0",
    "head_start_delay_s": "27.0",
    "supervisor_start_delay_s": "33.0",
    "location_lifecycle_start_delay_s": "37.0",
    "manual_mapping_start_delay_s": "40.0",
    "navigation_start_delay_s": "40.0",
    "readiness_start_delay_s": "45.0",
}

AUTONOMOUS_STAGE_DEFAULTS = {
    "description_start_delay_s": "0.0",
    "base_start_delay_s": "3.0",
    "lidar_start_delay_s": "6.0",
    "perception_start_delay_s": "9.0",
    "control_start_delay_s": "12.0",
    "localization_start_delay_s": "17.0",
    "power_start_delay_s": "22.0",
    "head_start_delay_s": "27.0",
    "supervisor_start_delay_s": "33.0",
    "location_lifecycle_start_delay_s": "37.0",
    "navigation_start_delay_s": "40.0",
    "mapping_start_delay_s": "45.0",
}

EDGE_STAGE_DEFAULTS = {
    "realsense_start_delay_s": "0.0",
    "camera_support_start_delay_s": "7.0",
    "vo_start_delay_s": "14.0",
    "obstacle_cloud_start_delay_s": "22.0",
    "observer_relay_start_delay_s": "28.0",
    "speech_start_delay_s": "34.0",
    "ui_start_delay_s": "40.0",
    "bridge_start_delay_s": "46.0",
    "readiness_start_delay_s": "52.0",
}


def read(path: Path) -> str:
    """Read one contract artifact."""
    return path.read_text(encoding="utf-8")


def launch_defaults(path: Path) -> dict[str, str]:
    """Extract literal launch argument defaults without importing ROS."""
    defaults = {}
    for node in ast.walk(ast.parse(read(path), filename=str(path))):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", "") != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        default = next(
            (
                keyword.value.value
                for keyword in node.keywords
                if keyword.arg == "default_value"
                and isinstance(keyword.value, ast.Constant)
                and isinstance(keyword.value.value, str)
            ),
            None,
        )
        if default is not None:
            defaults[node.args[0].value] = default
    return defaults


def launch_argument_names(path: Path) -> tuple[set[str], set[str]]:
    """Return declared and referenced launch argument names."""
    declared = set()
    referenced = set()
    for node in ast.walk(ast.parse(read(path), filename=str(path))):
        if not isinstance(node, ast.Call):
            continue
        function = getattr(node.func, "id", "")
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        if not isinstance(node.args[0].value, str):
            continue
        if function == "DeclareLaunchArgument":
            declared.add(node.args[0].value)
        elif function == "LaunchConfiguration":
            referenced.add(node.args[0].value)
    return declared, referenced


def staged_actions(path: Path) -> dict[str, str]:
    """Return action-variable to delay-argument bindings from `_stage` calls."""
    stages = {}
    for node in ast.walk(ast.parse(read(path), filename=str(path))):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", "") != "_stage" or len(node.args) != 2:
            continue
        delay, action = node.args
        if not isinstance(delay, ast.Constant) or not isinstance(action, ast.Name):
            continue
        stages[action.id] = delay.value
    return stages


def autonomous_wrapper_include(path: Path) -> tuple[ast.Call, dict[str, str]]:
    """Return the parent call and delay forwarding for the mapping include."""
    tree = ast.parse(read(path), filename=str(path))
    parents = {
        child: parent
        for parent in ast.walk(tree)
        for child in ast.iter_child_nodes(parent)
    }
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", "") != "IncludeLaunchDescription":
            continue
        if not any(
            isinstance(child, ast.Constant)
            and child.value == "autonomous_mapping.launch.py"
            for child in ast.walk(node)
        ):
            continue

        forwarding = {}
        for keyword in node.keywords:
            if keyword.arg != "launch_arguments":
                continue
            value = keyword.value
            if not (
                isinstance(value, ast.Call)
                and isinstance(value.func, ast.Attribute)
                and isinstance(value.func.value, ast.Dict)
            ):
                continue
            for key, item in zip(value.func.value.keys, value.func.value.values):
                if not (
                    isinstance(key, ast.Constant)
                    and isinstance(key.value, str)
                    and key.value.endswith("_start_delay_s")
                    and isinstance(item, ast.Call)
                    and getattr(item.func, "id", "") == "LaunchConfiguration"
                    and item.args
                    and isinstance(item.args[0], ast.Constant)
                ):
                    continue
                forwarding[key.value] = item.args[0].value

        parent = parents[node]
        assert isinstance(parent, ast.Call)
        return parent, forwarding
    raise AssertionError("Core launch does not include autonomous mapping")


def test_core_stage_defaults_are_dependency_ordered() -> None:
    """Core Pi defaults preserve the requested dependency-safe timeline."""
    defaults = launch_defaults(CORE_LAUNCH)
    assert {
        name: defaults[name] for name in CORE_STAGE_DEFAULTS
    } == CORE_STAGE_DEFAULTS

    foundation = list(CORE_STAGE_DEFAULTS)[:10]
    foundation_values = [float(defaults[name]) for name in foundation]
    assert foundation_values == sorted(foundation_values)
    assert len(set(foundation_values)) == len(foundation_values)
    assert float(defaults["manual_mapping_start_delay_s"]) == 40.0
    assert float(defaults["navigation_start_delay_s"]) == 40.0
    assert float(defaults["readiness_start_delay_s"]) > 40.0


def test_autonomous_mapping_uses_dependency_ordered_core_offsets() -> None:
    """The dedicated mapping launch staggers heavy stacks through TimerAction."""
    defaults = launch_defaults(AUTONOMOUS_LAUNCH)
    assert {
        name: defaults[name] for name in AUTONOMOUS_STAGE_DEFAULTS
    } == AUTONOMOUS_STAGE_DEFAULTS

    expected_bindings = {
        "description_launch": "description_start_delay_s",
        "base_launch": "base_start_delay_s",
        "lidar_launch": "lidar_start_delay_s",
        "perception_launch": "perception_start_delay_s",
        "control_launch": "control_start_delay_s",
        "localization_launch": "localization_start_delay_s",
        "power_launch": "power_start_delay_s",
        "head_launch": "head_start_delay_s",
        "supervisor_launch": "supervisor_start_delay_s",
        "location_lifecycle_launch": "location_lifecycle_start_delay_s",
        "navigation_launch": "navigation_start_delay_s",
        "mapping_launch": "mapping_start_delay_s",
    }
    assert staged_actions(AUTONOMOUS_LAUNCH) == expected_bindings

    required_order = (
        "description_launch",
        "base_launch",
        "lidar_launch",
        "perception_launch",
        "control_launch",
        "localization_launch",
        "power_launch",
        "supervisor_launch",
        "navigation_launch",
        "mapping_launch",
    )
    delays = [
        float(defaults[expected_bindings[action]]) for action in required_order
    ]
    assert delays == sorted(delays)
    assert len(delays) == len(set(delays))
    assert defaults["localization_start_delay_s"] != defaults[
        "navigation_start_delay_s"
    ]


def test_core_autonomous_entry_forwards_its_canonical_stage_offsets() -> None:
    """The Core wrapper and direct mapping entry share one timing contract."""
    parent, forwarding = autonomous_wrapper_include(CORE_LAUNCH)
    assert isinstance(parent.func, ast.Attribute)
    assert isinstance(parent.func.value, ast.Name)
    assert parent.func.value.id == "actions"
    assert parent.func.attr == "append"

    expected_forwarding = {
        name: (
            "readiness_start_delay_s"
            if name == "mapping_start_delay_s"
            else name
        )
        for name in AUTONOMOUS_STAGE_DEFAULTS
    }
    assert forwarding == expected_forwarding

    core_defaults = launch_defaults(CORE_LAUNCH)
    effective_offsets = {
        child_name: core_defaults[parent_name]
        for child_name, parent_name in forwarding.items()
    }
    assert effective_offsets == AUTONOMOUS_STAGE_DEFAULTS


def test_autonomous_timers_stagger_only_and_cannot_authorize_motion() -> None:
    """Timer stages schedule includes without creating readiness or authority."""
    launch = read(AUTONOMOUS_LAUNCH)

    assert "TimerAction" in launch
    assert "period=LaunchConfiguration(delay_argument)" in launch
    assert "cancel_on_shutdown=True" in launch
    assert "StartupStageGroup" not in launch
    assert "build_staged_sequence" not in launch
    assert "startup_stage_gate_node" not in launch
    assert "bringup_readiness_node" not in launch
    assert "ExecuteProcess" not in launch
    assert "ActionClient" not in launch
    assert "async_send_goal" not in launch
    assert "ros2 action send_goal" not in launch


def test_dedicated_autonomous_defaults_are_lightweight_and_head_is_optional() -> None:
    """Direct room mapping is Core-only while every optional flag remains exposed."""
    defaults = launch_defaults(AUTONOMOUS_LAUNCH)
    assert {
        name: defaults[name]
        for name in (
            "perception_use_ultrasonic",
            "localization_use_vo",
            "start_head",
            "start_location_lifecycle",
            "start_semantic_interruption",
            "coverage_enabled",
            "initial_scan360_required",
            "initial_head_scan_required",
            "final_scan360_required",
            "final_head_scan_required",
        )
    } == {
        "perception_use_ultrasonic": "false",
        "localization_use_vo": "false",
        "start_head": "false",
        "start_location_lifecycle": "false",
        "start_semantic_interruption": "false",
        "coverage_enabled": "false",
        "initial_scan360_required": "false",
        "initial_head_scan_required": "false",
        "final_scan360_required": "false",
        "final_head_scan_required": "false",
    }

    launch = read(AUTONOMOUS_LAUNCH)
    assert 'condition=IfCondition(LaunchConfiguration("start_head"))' in launch
    assert '"backend": LaunchConfiguration("head_backend")' in launch
    assert '"enable_scan": "true"' in launch


def test_core_uses_simple_bounded_launch_offsets_without_global_gates() -> None:
    """Core production has no Phase-3 process-release coordinator."""
    launch = read(CORE_LAUNCH)

    assert "TimerAction" in launch
    assert "StartupStageGroup" not in launch
    assert "build_staged_sequence" not in launch
    assert "startup_stage_gate_node" not in launch
    assert "bringup_readiness_node" not in launch
    assert '"complete"' not in launch
    assert "ExecuteProcess" not in launch
    assert "subprocess" not in launch
    assert "time.sleep" not in launch


def test_every_core_launch_configuration_is_declared() -> None:
    """Core staging does not leave unresolved launch substitutions."""
    for path in (CORE_LAUNCH, ROBOT_LAUNCH):
        declared, referenced = launch_argument_names(path)
        assert referenced <= declared, (path, referenced - declared)


def test_feature_flags_gate_delayed_core_components() -> None:
    """Disabled Core features never create their delayed include."""
    launch = read(CORE_LAUNCH)
    expected = {
        "start_description": "description_start_delay_s",
        "start_base": "base_start_delay_s",
        "start_lidar": "lidar_start_delay_s",
        "start_perception": "perception_start_delay_s",
        "start_control": "control_start_delay_s",
        "start_localization": "localization_start_delay_s",
        "start_power": "power_start_delay_s",
        "start_head": "head_start_delay_s",
        "start_supervisor": "supervisor_start_delay_s",
        "start_locations": "location_lifecycle_start_delay_s",
    }
    for guard, delay in expected.items():
        guard_offset = launch.index(f"if {guard}:")
        delay_offset = launch.index(f'"{delay}"', guard_offset)
        assert delay_offset > guard_offset

    assert 'if mode == "manual_mapping":' in launch
    assert '"manual_mapping_start_delay_s"' in launch
    assert 'elif mode == "saved_map_navigation":' in launch
    assert '"navigation_start_delay_s"' in launch


def test_robot_forwards_core_delays_only_through_core_branch() -> None:
    """The role selector keeps Core and Edge staging independent."""
    robot = read(ROBOT_LAUNCH)
    defaults = launch_defaults(ROBOT_LAUNCH)
    core_branch = robot.split('if role in {"core", "all"}:', maxsplit=1)[1]
    core_branch, edge_branch = core_branch.split(
        'if role in {"edge", "all"}:', maxsplit=1
    )
    edge_branch = edge_branch.split("\n    return actions", maxsplit=1)[0]

    for name, default in CORE_STAGE_DEFAULTS.items():
        if name == "readiness_start_delay_s":
            assert defaults["core_readiness_start_delay_s"] == default
            assert 'core_arguments["readiness_start_delay_s"]' in core_branch
            assert '"core_readiness_start_delay_s"' in core_branch
            assert '"core_readiness_start_delay_s"' not in edge_branch
        else:
            assert defaults[name] == default
            assert f'"{name}"' in core_branch
            assert f'"{name}"' not in edge_branch


def test_edge_staging_defaults_remain_isolated_from_core() -> None:
    """Core timing adds no coupling to the complete Edge timeline."""
    edge_defaults = launch_defaults(EDGE_LAUNCH)
    robot_defaults = launch_defaults(ROBOT_LAUNCH)
    assert {
        name: edge_defaults[name] for name in EDGE_STAGE_DEFAULTS
    } == EDGE_STAGE_DEFAULTS
    assert {
        name: robot_defaults[name] for name in EDGE_STAGE_DEFAULTS
    } == EDGE_STAGE_DEFAULTS


def test_safe_startup_policy_and_head_behavior_are_unchanged() -> None:
    """Staging changes time only: Core remains stopped and unarmed."""
    core = read(CORE_LAUNCH)
    head = read(
        PROJECT_ROOT / "savo_ws/src/core/savo_head/launch/head_bringup.launch.py"
    )
    defaults = launch_defaults(CORE_LAUNCH)

    assert defaults["robot_mode"] == "safe_idle"
    assert defaults["control_startup_mode"] == "STOP"
    assert defaults["supervisor_auto_arm"] == "false"
    assert defaults["head_camera_mode"] == "ros"
    assert '"auto_arm": LaunchConfiguration("supervisor_auto_arm")' in core
    assert '"center_on_start": "false"' in core
    assert '"auto_start": False' in head
    assert '"camera_mode": LaunchConfiguration("head_camera_mode")' in core


def test_modes_start_only_their_owned_mapping_or_navigation_stack() -> None:
    """Safe idle cannot enter either delayed mode-specific branch."""
    launch = read(CORE_LAUNCH)
    manual_guard = launch.index('if mode == "manual_mapping":')
    manual_include = launch.index("manual_mapping.launch.xml", manual_guard)
    navigation_guard = launch.index('elif mode == "saved_map_navigation":')
    navigation_include = launch.index(
        "production_navigation.launch.py", navigation_guard
    )

    assert manual_include > manual_guard
    assert navigation_include > navigation_guard
    assert "manual_mapping.launch.xml" not in launch[:manual_guard]
    assert "production_navigation.launch.py" not in launch[:navigation_guard]


def test_core_composition_keeps_single_component_owners() -> None:
    """Simple bringup includes each component stack once."""
    core = read(CORE_LAUNCH)
    autonomous = read(AUTONOMOUS_LAUNCH)

    for marker in (
        '_python_launch("savo_description", "description.launch.py")',
        '_python_launch("savo_base", "base_bringup.launch.py")',
        '_python_launch("savo_supervisor", "supervisor.launch.py")',
    ):
        assert core.count(marker) == 1

    assert core.count('"savo_localization", "localization_bringup.launch.py"') == 1
    assert '"use_imu": "true"' in core
    assert '"use_wheel_odom": "true"' in core
    assert '"use_ekf": "true"' in core

    assert autonomous.count('"savo_description", "description.launch.py"') == 1
    assert autonomous.count('"savo_base", "base_bringup.launch.py"') == 1
    assert autonomous.count('"localization_bringup.launch.py"') == 1
    assert '"use_imu": "true"' in autonomous
    assert '"use_wheel_odom": "true"' in autonomous
    assert '"use_ekf": "true"' in autonomous
    assert autonomous.count('"savo_supervisor", "supervisor.launch.py"') == 1

    description = read(
        PROJECT_ROOT
        / "savo_ws/src/shared/savo_description/launch/description.launch.py"
    )
    assert description.count('package="robot_state_publisher"') == 1


def test_global_readiness_is_not_a_production_launch_authority() -> None:
    """Package health and Supervisor remain independent of global bringup."""
    launch = read(CORE_LAUNCH)
    readiness_source = read(
        PACKAGE_ROOT / "src/nodes/bringup_readiness_node.cpp"
    )

    assert "bringup_readiness_node" not in launch
    assert "startup_stage_gate_node" not in launch
    assert '"supervisor_heartbeat", "supervisor_heartbeat_topic"' in readiness_source
    assert "/savo_bringup/core/ready" not in read(
        PROJECT_ROOT / "savo_ws/src/shared/savo_supervisor/config/supervisor.yaml"
    )


def test_tf_control_and_power_authorities_are_unchanged() -> None:
    """Launch-only staging preserves the validated authority contracts."""
    localization = yaml.safe_load(
        read(PROJECT_ROOT / "savo_ws/src/core/savo_localization/config/ekf_odom.yaml")
    )["ekf_filter_node"]["ros__parameters"]
    wheel = yaml.safe_load(
        read(
            PROJECT_ROOT
            / "savo_ws/src/core/savo_localization/config/wheel_odom.yaml"
        )
    )["wheel_odom_node"]["ros__parameters"]
    base = yaml.safe_load(
        read(
            PROJECT_ROOT
            / "savo_ws/src/core/savo_base/config/profiles/real_robot_v1.yaml"
        )
    )["/base_driver_node"]["ros__parameters"]
    core = read(CORE_LAUNCH)

    assert localization["publish_tf"] is True
    assert localization["world_frame"] == "odom"
    assert localization["base_link_frame"] == "base_footprint"
    assert wheel["publish_tf"] is False
    assert base["cmd_topic"] == "/cmd_vel_safe"
    assert '"edge_ups_expected", default_value="false"' in core


def test_autonomous_mapping_authority_path_remains_explicit() -> None:
    """Staging does not turn launch timing into mission authority."""
    core = read(CORE_LAUNCH)
    autonomous = read(AUTONOMOUS_LAUNCH)

    assert 'if mode == "autonomous_mapping":' in core
    assert '"autonomous_mapping.launch.py"' in core
    assert "StartupStageGroup" not in autonomous
    assert "TimerAction" in autonomous
    assert 'default_value="STOP"' in autonomous
    assert "typed RunAutonomousMapping action only after readiness" in autonomous
