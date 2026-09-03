"""Contracts for the canonical staged Edge startup."""

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = PACKAGE_ROOT.parents[3]
EDGE_LAUNCH = PACKAGE_ROOT / "launch" / "edge_bringup.launch.py"
ROBOT_LAUNCH = PACKAGE_ROOT / "launch" / "robot_bringup.launch.py"
REALSENSE_LAUNCH = (
    PROJECT_ROOT
    / "savo_ws/src/edge/savo_realsense/launch/realsense_bringup.launch.py"
)
BRIDGE_CONFIG = (
    PROJECT_ROOT
    / "savo_ws/src/shared/savo_bridge/config/savo_bridge.edge.yaml"
)

STAGE_DEFAULTS = {
    "realsense_start_delay_s": "0.0",
    "camera_support_start_delay_s": "7.0",
    "vo_start_delay_s": "14.0",
    "obstacle_cloud_start_delay_s": "22.0",
    "observer_relay_start_delay_s": "28.0",
    "bridge_start_delay_s": "34.0",
    "readiness_start_delay_s": "40.0",
}


def read(path: Path) -> str:
    """Read one startup contract artifact."""
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


def test_default_edge_stage_delays_exist_and_are_monotonic() -> None:
    """The Pi 5 production defaults follow the required startup order."""
    defaults = launch_defaults(EDGE_LAUNCH)

    assert {name: defaults[name] for name in STAGE_DEFAULTS} == STAGE_DEFAULTS
    values = [float(defaults[name]) for name in STAGE_DEFAULTS]
    assert values == sorted(values)
    assert len(set(values)) == len(values)


def test_every_staged_launch_configuration_is_declared() -> None:
    """The edited launch files remain self-contained launch descriptions."""
    for path in (EDGE_LAUNCH, ROBOT_LAUNCH, REALSENSE_LAUNCH):
        declared, referenced = launch_argument_names(path)
        assert referenced <= declared, (path, referenced - declared)


def test_edge_stages_use_launch_timers_with_zero_delay_support() -> None:
    """Every heavy stage uses launch-native timing and accepts zero."""
    edge = read(EDGE_LAUNCH)
    realsense = read(REALSENSE_LAUNCH)

    assert "from launch.actions import TimerAction" in edge
    assert "from launch.actions import TimerAction" in realsense
    for name in (
        "vo_start_delay_s",
        "obstacle_cloud_start_delay_s",
        "bridge_start_delay_s",
        "readiness_start_delay_s",
    ):
        assert f'period=LaunchConfiguration("{name}")' in edge
    for name in (
        "realsense_start_delay_s",
        "camera_support_start_delay_s",
        "observer_relay_start_delay_s",
    ):
        assert f'LaunchConfiguration(\n        "{name}"' in realsense
        assert f"period={name}" in realsense

    # TimerAction accepts a 0.0 period; no validation or shell sleep changes
    # the semantics of an explicit zero-delay override.
    assert "ExecuteProcess" not in edge
    assert "sleep" not in edge.lower()
    assert "ExecuteProcess" not in realsense
    assert "sleep" not in realsense.lower()


def test_pending_stages_are_canceled_on_launch_shutdown() -> None:
    """Ctrl+C cannot allow a pending staged action to fire afterward."""
    expected_timer_counts = {
        EDGE_LAUNCH: 4,
        REALSENSE_LAUNCH: 3,
    }
    for path, expected_count in expected_timer_counts.items():
        tree = ast.parse(read(path), filename=str(path))
        timers = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and getattr(node.func, "id", "") == "TimerAction"
        ]
        assert len(timers) == expected_count
        for timer in timers:
            cancel_value = next(
                keyword.value
                for keyword in timer.keywords
                if keyword.arg == "cancel_on_shutdown"
            )
            assert isinstance(cancel_value, ast.Constant)
            assert cancel_value.value is True


def test_camera_stage_is_one_driver_then_support_then_observer_relay() -> None:
    """Staging must not duplicate the D435 or split its stream producers."""
    launch = read(REALSENSE_LAUNCH)

    assert launch.count('executable="realsense2_camera_node"') == 1
    assert "actions=[realsense_node]" in launch
    assert "actions=[health_node, depth_front_min_node]" in launch
    assert 'executable="camera_topic_monitor_node"' not in launch
    assert "actions=[observer_color_relay]" in launch
    assert 'condition=IfCondition(use_depth_front_min)' in launch
    assert 'condition=IfCondition(enable_observer_color_relay)' in launch


def test_camera_health_requirements_follow_enabled_edge_consumers() -> None:
    """Optional VO/cloud producers only gate health when they are started."""
    edge = read(EDGE_LAUNCH)
    realsense = read(REALSENSE_LAUNCH)

    assert '"require_vo_health": (' in edge
    assert '"true" if start_vo else "false"' in edge
    assert '"require_obstacle_cloud_health": (' in edge
    assert '"true" if start_obstacle_cloud else "false"' in edge
    assert '"require_depth_signal": ParameterValue(' in realsense
    assert '"require_vo_health": ParameterValue(' in realsense
    assert '"require_obstacle_cloud_health": ParameterValue(' in realsense
    assert 'default_value="false"' in realsense
    assert '"require_obstacle_cloud": start_obstacle_cloud' in edge


def test_feature_flags_still_gate_all_delayed_edge_components() -> None:
    """A timer is only created after its existing feature gate passes."""
    launch = read(EDGE_LAUNCH)
    expected_guards = {
        "if start_realsense:": "realsense_bringup.launch.py",
        "if start_vo:": 'period=LaunchConfiguration("vo_start_delay_s")',
        "if start_obstacle_cloud:": (
            'period=LaunchConfiguration("obstacle_cloud_start_delay_s")'
        ),
        "if start_bridge:": 'period=LaunchConfiguration("bridge_start_delay_s")',
        "if start_power:": "power_edge.launch.py",
    }
    for guard, delayed_action in expected_guards.items():
        guard_offset = launch.index(guard)
        action_offset = launch.index(delayed_action, guard_offset)
        assert action_offset > guard_offset

    assert "and explicit_obstacle_cloud" in launch
    assert "and obstacle_cloud_requested" in launch
    assert 'if start_vo and not start_realsense:' in launch
    assert 'if explicit_obstacle_cloud and not start_realsense:' in launch
    assert '"enable_observer_color_relay": (' in launch


def test_edge_power_remains_optional_for_readiness() -> None:
    """Broken Edge UPS hardware can be disabled without blocking readiness."""
    launch = read(EDGE_LAUNCH)

    assert 'start_power = as_bool(_value(context, "start_power"))' in launch
    assert "if start_power:" in launch
    assert '"require_power": start_power' in launch
    assert 'DeclareLaunchArgument("start_power", default_value="true")' in launch


def test_bridge_edge_evidence_does_not_require_edge_ups() -> None:
    """Bridge graph presence is independent of UPS and Edge readiness."""
    parameters = yaml.safe_load(read(BRIDGE_CONFIG))[
        "/savo_bridge/savo_bridge_node"
    ]["ros__parameters"]

    assert "core_evidence_nodes" not in parameters
    assert parameters.get("core_evidence_nodes", []) == []
    assert parameters["core_evidence_topics"] == [
        "/savo_control/mode_state"
    ]
    assert "edge_evidence_nodes" not in parameters
    assert parameters.get("edge_evidence_nodes", []) == []
    assert "edge_evidence_topics" not in parameters
    assert parameters.get("edge_evidence_topics", []) == []
    config = read(BRIDGE_CONFIG)
    assert "/edge_ups_node" not in config
    assert "/edge_bringup_readiness_node" not in config


def test_bridge_readiness_cycle_is_removed_without_weakening_health() -> None:
    """Local DDS proves Edge presence while all other bridge gates remain."""
    bridge_source = read(
        PROJECT_ROOT / "savo_ws/src/shared/savo_bridge/src/bridge_node.cpp"
    )
    readiness_source = read(
        PACKAGE_ROOT / "src/nodes/bringup_readiness_node.cpp"
    )

    assert 'evidence.core_evidence_configured;' in bridge_source
    assert '!evidence.edge_evidence_configured ||' in bridge_source
    assert 'evidence.dds_active &&' in bridge_source
    assert 'evidence.core_visible &&' in bridge_source
    assert 'health.required_topics_ready &&' in bridge_source
    assert 'snapshot_enabled_ &&' in bridge_source
    assert 'SubscribeBool("bridge"' in readiness_source
    assert 'SubscribeCounter("bridge_heartbeat"' in readiness_source


def test_core_launch_does_not_receive_edge_stage_controls() -> None:
    """Edge staging does not alter the Core launch interface or composition."""
    core = read(PACKAGE_ROOT / "launch" / "core_bringup.launch.py")
    robot = read(ROBOT_LAUNCH)

    for name in STAGE_DEFAULTS.keys() - {"readiness_start_delay_s"}:
        assert name not in core
    assert launch_defaults(PACKAGE_ROOT / "launch" / "core_bringup.launch.py")[
        "readiness_start_delay_s"
    ] == "45.0"
    core_branch = robot.split('if role in {"core", "all"}:', maxsplit=1)[1]
    core_branch = core_branch.split('if role in {"edge", "all"}:', maxsplit=1)[0]
    for name in STAGE_DEFAULTS.keys() - {"readiness_start_delay_s"}:
        assert name not in core_branch
    assert '"core_readiness_start_delay_s"' in core_branch
    assert 'core_arguments["readiness_start_delay_s"]' in core_branch


def test_edge_bringup_does_not_introduce_nav2_or_motion() -> None:
    """Staging changes process timing only, not Edge authority."""
    launch = read(EDGE_LAUNCH).lower()

    for forbidden in (
        "nav2",
        "voxel_layer",
        "navigate_to_pose",
        "cmd_vel",
        "autonomous motion",
    ):
        assert forbidden not in launch
    assert 'default_value="safe_idle"' in launch


def test_existing_production_profiles_and_topics_remain_selected() -> None:
    """The staged actions retain validated camera, VO, and cloud inputs."""
    edge = read(EDGE_LAUNCH)
    camera = read(
        PROJECT_ROOT
        / "savo_ws/src/edge/savo_realsense/config/realsense_d435_camera.yaml"
    )
    obstacle = read(
        PROJECT_ROOT
        / (
            "savo_ws/src/shared/savo_perception/config/edge/"
            "obstacle_cloud_filter.yaml"
        )
    )
    vo = read(
        PROJECT_ROOT
        / "savo_ws/src/edge/savo_vo/config/profiles/real_robot_v1.yaml"
    )

    assert '"realsense_d435_camera.yaml"' in edge
    assert '"realsense_d435_nodes.yaml"' in edge
    assert '"implementation": "cpp"' in edge
    assert 'default_value="real_robot_v1"' in edge
    assert "depth_module.depth_profile: \"848x480x30\"" in camera
    assert "rgb_camera.color_profile: \"640x480x30\"" in camera
    assert "align_depth.enable: true" in camera
    assert "enable_sync: true" in camera
    assert "pointcloud__neon_.enable: true" in camera
    assert "publish_tf: false" in camera
    assert "color_image_topic: /camera/camera/color/image_raw" in vo
    assert (
        "depth_image_topic: /camera/camera/aligned_depth_to_color/image_raw"
        in vo
    )
    assert "odom_topic: /vo/odom/raw" in vo
    assert "odom_raw_topic: /vo/odom/raw" in vo
    assert "odom_topic: /vo/odom" in vo
    assert "base_frame: base_footprint" in vo
    assert "input_topic: /camera/camera/depth/color/points" in obstacle
    assert "output_topic: /savo_perception/obstacles/points" in obstacle
    assert "output_frame: base_link" in obstacle
    assert "voxel_size_m: 0.05" in obstacle
    assert "health_topic: /savo_perception/obstacle_cloud/health" in obstacle
    assert "status_topic: /savo_perception/obstacle_cloud/status" in obstacle
    assert (
        "heartbeat_topic: /savo_perception/obstacle_cloud/heartbeat"
        in obstacle
    )


def test_robot_bringup_forwards_every_edge_stage_delay() -> None:
    """The canonical role selector exposes and passes each delay unchanged."""
    robot = read(ROBOT_LAUNCH)
    defaults = launch_defaults(ROBOT_LAUNCH)

    assert {name: defaults[name] for name in STAGE_DEFAULTS} == STAGE_DEFAULTS
    for name in STAGE_DEFAULTS:
        assert robot.count(f'"{name}"') >= 2
    assert "edge_arguments[key] = LaunchConfiguration(name)" in robot
