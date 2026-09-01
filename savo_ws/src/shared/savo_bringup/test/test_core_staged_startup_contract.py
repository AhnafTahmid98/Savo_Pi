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

EDGE_STAGE_DEFAULTS = {
    "realsense_start_delay_s": "0.0",
    "camera_support_start_delay_s": "7.0",
    "vo_start_delay_s": "14.0",
    "obstacle_cloud_start_delay_s": "22.0",
    "observer_relay_start_delay_s": "28.0",
    "bridge_start_delay_s": "34.0",
    "readiness_start_delay_s": "40.0",
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


def test_core_stages_use_nonblocking_shutdown_canceling_timers() -> None:
    """Every Core stage uses one launch-native helper with clean shutdown."""
    launch = read(CORE_LAUNCH)
    tree = ast.parse(launch, filename=str(CORE_LAUNCH))

    assert "from launch.actions import TimerAction" in launch
    timers = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", "") == "TimerAction"
    ]
    assert len(timers) == 1
    cancel_value = next(
        keyword.value
        for keyword in timers[0].keywords
        if keyword.arg == "cancel_on_shutdown"
    )
    assert isinstance(cancel_value, ast.Constant)
    assert cancel_value.value is True
    assert "ExecuteProcess" not in launch
    assert "subprocess" not in launch
    assert "time.sleep" not in launch

    for name in CORE_STAGE_DEFAULTS:
        assert f'_stage(\n                    "{name}"' in launch or (
            name == "readiness_start_delay_s"
            and f'_stage(\n            "{name}"' in launch
        )


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


def test_edge_staging_defaults_remain_unchanged() -> None:
    """Core timing adds no coupling to the validated Edge timeline."""
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
    """Staging wraps existing includes instead of duplicating stacks."""
    core = read(CORE_LAUNCH)
    autonomous = read(AUTONOMOUS_LAUNCH)

    for marker in (
        '_python_launch("savo_description", "description.launch.py")',
        '_python_launch("savo_base", "base_bringup.launch.py")',
        '"savo_localization", "localization_bringup.launch.py"',
        '_python_launch("savo_supervisor", "supervisor.launch.py")',
    ):
        assert core.count(marker) == 1

    assert autonomous.count('"savo_description", "description.launch.py"') == 1
    assert autonomous.count('"savo_base", "base_bringup.launch.py"') == 1
    assert autonomous.count('"localization_bringup.launch.py"') == 1
    assert autonomous.count('"savo_supervisor", "supervisor.launch.py"') == 1

    description = read(
        PROJECT_ROOT
        / "savo_ws/src/shared/savo_description/launch/description.launch.py"
    )
    assert description.count('package="robot_state_publisher"') == 1


def test_readiness_starts_last_without_weakening_requirements() -> None:
    """Readiness observes the settled system with its existing gates."""
    launch = read(CORE_LAUNCH)
    readiness_source = read(
        PACKAGE_ROOT / "src/nodes/bringup_readiness_node.cpp"
    )

    readiness_offset = launch.index('"readiness_start_delay_s"')
    assert readiness_offset > launch.index('"navigation_start_delay_s"')
    for gate in (
        '"require_base": start_base',
        '"require_control": start_control',
        '"require_safety": start_perception',
        '"require_lidar": start_lidar',
        '"require_localization": start_localization',
        '"require_power": start_power',
        '"require_mapping": requirements.require_mapping',
        '"require_navigation": requirements.require_navigation',
    ):
        assert gate in launch
    assert 'SubscribeString("supervisor_heartbeat"' in readiness_source
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


def test_autonomous_mapping_authority_path_is_intentionally_unchanged() -> None:
    """The dedicated autonomous composition remains an explicit exception."""
    core = read(CORE_LAUNCH)
    autonomous = read(AUTONOMOUS_LAUNCH)

    assert 'if mode == "autonomous_mapping":' in core
    assert '"autonomous_mapping.launch.py"' in core
    assert "TimerAction" not in autonomous
    assert 'default_value="STOP"' in autonomous
    assert "typed RunAutonomousMapping action only after readiness" in autonomous
