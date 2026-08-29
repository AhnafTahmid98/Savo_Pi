"""Contracts for the complete distributed Robot Savo bringup."""

import ast
import grp
import os
import pwd
import re
import subprocess
from pathlib import Path

from savo_bringup.launch_contract import resolve_host_role
from savo_bringup.launch_contract import resolve_requirements
from savo_bringup.launch_contract import should_start_obstacle_cloud
from savo_bringup.launch_contract import validate_selection

import yaml


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    """Read one package-relative bringup artifact."""
    return (ROOT / relative).read_text(encoding="utf-8")


def launch_string_defaults(relative: str) -> dict[str, str]:
    """Return literal string defaults from one Python launch file."""
    tree = ast.parse(read(relative), filename=relative)
    defaults = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", "") != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        for keyword in node.keywords:
            if keyword.arg != "default_value":
                continue
            if isinstance(keyword.value, ast.Constant) and isinstance(
                keyword.value.value, str
            ):
                defaults[node.args[0].value] = keyword.value.value
    return defaults


def test_required_launch_files_are_nonempty_and_parseable() -> None:
    """Every canonical and compatibility launch must parse."""
    required = {
        "core_bringup.launch.py",
        "edge_bringup.launch.py",
        "robot_bringup.launch.py",
        "bringup.launch.py",
        "savo_full.launch.py",
        "manual_mapping.launch.py",
        "saved_map_navigation.launch.py",
        "live_mapping_navigation.launch.py",
        "diagnostics_bringup.launch.py",
        "sensors_bringup.launch.py",
        "localization_bringup.launch.py",
        "demo_ui.launch.py",
    }
    for name in required:
        path = ROOT / "launch" / name
        source = path.read_text(encoding="utf-8")
        assert source.strip(), name
        ast.parse(source, filename=str(path))


def test_core_bringup_owns_every_core_package_and_mode_boundary() -> None:
    """Core launch composition must retain core-only ownership."""
    launch = read("launch/core_bringup.launch.py")
    for package in {
        "savo_description",
        "savo_base",
        "savo_lidar",
        "savo_perception",
        "savo_localization",
        "savo_control",
        "savo_head",
        "savo_power",
        "savo_mapping",
        "savo_nav",
        "savo_supervisor",
    }:
        assert f'"{package}"' in launch

    assert 'mode == "autonomous_mapping"' in launch
    assert 'mode == "manual_mapping"' in launch
    assert 'mode == "saved_map_navigation"' in launch
    assert "location_integration.launch.py" in launch
    assert "production_navigation.launch.py" in launch
    assert "manual_mapping.launch.xml" in launch
    assert "autonomous_mapping.launch.py" in launch
    assert "bringup_readiness_node" in launch


def test_edge_bringup_uses_cpp_production_implementations() -> None:
    """Edge launch must select production C++ implementations."""
    launch = read("launch/edge_bringup.launch.py")
    assert "realsense_bringup.launch.py" in launch
    assert '"implementation": "cpp"' in launch
    assert "obstacle_cloud_filter.launch.py" in launch
    assert "speech_bringup.launch.xml" in launch
    assert "ui_bringup.launch.py" in launch
    assert "edge_bridge.launch.py" in launch
    assert "power_edge.launch.py" in launch
    assert "should_start_obstacle_cloud(" in launch
    assert '"robot_mode": mode' in launch


def test_edge_realsense_and_obstacle_cloud_profile_selection() -> None:
    """Run the optional cloud helper in navigation or by override."""
    launch = read("launch/edge_bringup.launch.py")
    common = {
        "host_role": "edge",
        "start_bridge": True,
        "start_realsense": True,
        "start_vo": True,
        "start_speech": False,
    }
    for mode in (
        "safe_idle",
        "manual_mapping",
        "autonomous_mapping",
        "saved_map_navigation",
    ):
        baseline = resolve_requirements(
            robot_mode=mode,
            bringup_profile="lidar_only",
            **common,
        )
        assert baseline.voxel_layer_enabled is False

    voxel = resolve_requirements(
        robot_mode="safe_idle",
        bringup_profile="lidar_d435_voxel",
        **common,
    )
    assert voxel.voxel_layer_enabled is True
    assert '"realsense_pointcloud_camera.yaml"' in launch
    assert 'else "realsense_d435_camera.yaml"' in launch
    assert '"realsense_pointcloud_nodes.yaml"' in launch
    assert 'else "realsense_d435_nodes.yaml"' in launch

    for mode in ("safe_idle", "manual_mapping"):
        assert should_start_obstacle_cloud(
            mode,
            "lidar_d435_voxel",
            d435_voxel_validated=True,
            explicit_start=False,
        ) is False
    for mode in ("autonomous_mapping", "saved_map_navigation"):
        for selected_profile in ("lidar_only", "lidar_d435_voxel"):
            for validated in (False, True):
                assert should_start_obstacle_cloud(
                    mode,
                    selected_profile,
                    d435_voxel_validated=validated,
                    explicit_start=False,
                ) is True

    assert should_start_obstacle_cloud(
        "safe_idle",
        "lidar_only",
        d435_voxel_validated=False,
        explicit_start=True,
    ) is True
    assert should_start_obstacle_cloud(
        "autonomous_mapping",
        "lidar_only",
        d435_voxel_validated=False,
        explicit_start=False,
    ) is True
    assert '"require_obstacle_cloud": False' in launch
    assert (
        'start_obstacle_cloud = start_realsense and '
        'obstacle_cloud_requested'
    ) in launch
    assert 'if explicit_obstacle_cloud and not start_realsense' in launch
    assert (
        '"realsense_pointcloud_camera.yaml"\n'
        '                    if start_obstacle_cloud'
    ) in launch
    assert (
        '"realsense_pointcloud_nodes.yaml"\n'
        '                    if start_obstacle_cloud'
    ) in launch


def test_supervisor_and_bridge_receive_the_existing_robot_mode() -> None:
    """Existing robot_mode must select one matching Supervisor/Bridge policy."""
    core = read("launch/core_bringup.launch.py")
    edge = read("launch/edge_bringup.launch.py")
    autonomous = read("launch/autonomous_mapping.launch.py")
    assert '"robot_mode": mode' in core
    assert '"robot_mode": mode' in edge
    assert '"robot_mode": "autonomous_mapping"' in autonomous

    project_root = ROOT.parents[3]
    supported = {
        "safe_idle",
        "manual_mapping",
        "autonomous_mapping",
        "saved_map_navigation",
    }
    for relative in (
        "savo_ws/src/shared/savo_supervisor/launch/supervisor.launch.py",
        "savo_ws/src/shared/savo_bridge/launch/edge_bridge.launch.py",
    ):
        tree = ast.parse((project_root / relative).read_text())
        assignments = [
            node for node in tree.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "_MODE_POLICY_FILES"
                for target in node.targets
            )
        ]
        assert len(assignments) == 1
        policies = ast.literal_eval(assignments[0].value)
        assert set(policies) == supported
        assert len(set(policies.values())) == len(supported)


def test_edge_ups_expectation_reaches_every_core_power_launch() -> None:
    """Canonical bringup keeps Edge optional but allows strict override."""
    robot = read("launch/robot_bringup.launch.py")
    core = read("launch/core_bringup.launch.py")
    autonomous = read("launch/autonomous_mapping.launch.py")

    for launch in (robot, core, autonomous):
        assert '"edge_ups_expected"' in launch
        assert 'default_value="false"' in launch

    assert '"edge_ups_expected": LaunchConfiguration(' in core
    assert '"edge_ups_expected": LaunchConfiguration(' in autonomous


def test_full_entry_point_keeps_core_and_edge_roles_explicit() -> None:
    """The full launch must keep distributed roles explicit."""
    launch = read("launch/robot_bringup.launch.py")
    assert 'role in {"core", "all"}' in launch
    assert 'role in {"edge", "all"}' in launch
    assert "host_role:=all is reserved" in launch
    assert "core_bringup.launch.py" in launch
    assert "edge_bringup.launch.py" in launch


def test_auto_host_role_resolves_only_known_robot_hosts() -> None:
    """Automatic role selection must use the established hostname contract."""
    for hostname in ("core", "savo-core", "core.robot.local"):
        assert resolve_host_role("auto", hostname) == "core"
    for hostname in ("edge", "savo-edge", "edge.robot.local"):
        assert resolve_host_role("auto", hostname) == "edge"


def test_explicit_host_roles_remain_supported() -> None:
    """Explicit role selection remains available for deployment and tests."""
    assert resolve_host_role("core", "developer-mac") == "core"
    assert resolve_host_role("edge", "developer-mac") == "edge"
    assert resolve_host_role("all", "developer-mac") == "all"


def test_auto_host_role_rejects_unknown_or_conflicting_identity() -> None:
    """Unknown hosts and hostname/environment conflicts must fail closed."""
    try:
        resolve_host_role("auto", "developer-mac")
    except RuntimeError as error:
        assert "cannot identify this host" in str(error)
    else:
        raise AssertionError("unknown automatic host role was accepted")

    try:
        resolve_host_role("auto", "savo-core", "edge")
    except RuntimeError as error:
        assert "SAVO_ROLE does not match" in str(error)
    else:
        raise AssertionError("hostname/SAVO_ROLE mismatch was accepted")


def test_all_host_role_remains_bench_only() -> None:
    """Automatic-role support must not broaden the single-host bench role."""
    validate_selection(
        resolve_host_role("all", "developer-mac"),
        "safe_idle",
        "bench",
        d435_voxel_validated=False,
        require_locked_geometry=False,
        allow_provisional_geometry=False,
    )
    try:
        validate_selection(
            "all",
            "safe_idle",
            "lidar_only",
            d435_voxel_validated=False,
            require_locked_geometry=True,
            allow_provisional_geometry=False,
        )
    except RuntimeError as error:
        assert "reserved for bench use" in str(error)
    else:
        raise AssertionError("host_role:=all escaped its bench-only gate")


def test_validated_normal_runtime_defaults_are_synchronized() -> None:
    """Canonical and nested production launches must use validated defaults."""
    expected = {
        "localization_use_vo": "true",
        "head_enable_tf": "true",
        "head_camera_mode": "ros",
        "control_startup_mode": "STOP",
        "control_use_backup_escape": "false",
        "control_use_stuck_detector": "false",
    }
    for launch in (
        "launch/robot_bringup.launch.py",
        "launch/core_bringup.launch.py",
        "launch/autonomous_mapping.launch.py",
    ):
        defaults = launch_string_defaults(launch)
        for name, value in expected.items():
            assert defaults[name] == value, f"{launch}: {name}"

    top = launch_string_defaults("launch/robot_bringup.launch.py")
    assert top["host_role"] == "auto"
    assert top["supervisor_auto_arm"] == "false"
    assert top["start_obstacle_cloud"] == "false"
    assert top["start_speech"] == "false"
    assert top["start_ui"] == "false"
    assert top["d435_voxel_validated"] == "false"
    assert top["speech_params_file"] == "edge_real_robot_v1.yaml"
    assert top["active_map_id"] == ""
    assert top["active_map_revision"] == "0"

    edge = launch_string_defaults("launch/edge_bringup.launch.py")
    assert edge["start_vo"] == "true"
    assert edge["start_obstacle_cloud"] == "false"
    assert edge["d435_voxel_validated"] == "false"
    assert edge["vo_profile"] == "real_robot_v1"
    assert edge["active_map_id"] == ""
    assert edge["active_map_revision"] == "0"

    for launch in (
        "launch/manual_mapping.launch.py",
        "launch/mapping.launch.py",
    ):
        assert (
            launch_string_defaults(launch)["localization_use_vo"]
            == "true"
        )


def test_all_production_vo_entrypoints_and_wrapper_are_synchronized() -> None:
    """Default every normal entrypoint to VO while preserving explicit false."""
    project_root = ROOT.parents[3]
    localization_launch = (
        project_root
        / "savo_ws/src/core/savo_localization/launch/localization_bringup.launch.py"
    )
    localization_tree = ast.parse(localization_launch.read_text(encoding="utf-8"))
    standalone_defaults = {}
    for node in ast.walk(localization_tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", "") != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value" and isinstance(
                keyword.value, ast.Constant
            ):
                standalone_defaults[node.args[0].value] = keyword.value.value

    assert standalone_defaults["use_vo"] == "true"
    assert launch_string_defaults("launch/core_bringup.launch.py")[
        "localization_use_vo"
    ] == "true"
    edge_defaults = launch_string_defaults("launch/edge_bringup.launch.py")
    assert edge_defaults["start_vo"] == "true"
    assert edge_defaults["start_realsense"] == "true"

    core_run = (project_root / "deploy/core/run_core.sh").read_text(
        encoding="utf-8"
    )
    match = re.search(
        r'localization_use_vo:=("\$\{SAVO_LOCALIZATION_USE_VO:-[^}]+\}")',
        core_run,
    )
    assert match is not None
    expansion = match.group(1)
    override_cases = ((None, "true"), ("false", "false"), ("true", "true"))
    for override, expected in override_cases:
        environment = os.environ.copy()
        if override is None:
            environment.pop("SAVO_LOCALIZATION_USE_VO", None)
        else:
            environment["SAVO_LOCALIZATION_USE_VO"] = override
        resolved = subprocess.run(
            ["bash", "-c", f'printf "%s" {expansion}'],
            check=True,
            capture_output=True,
            text=True,
            env=environment,
        )
        assert resolved.stdout == expected


def test_motion_modes_fail_closed_on_geometry_and_voxel_validation() -> None:
    """Unsafe geometry and voxel selections must be rejected."""
    contract = read("savo_bringup/launch_contract.py")
    assert "motion profiles require locked geometry validation" in contract
    assert "d435_voxel_validated:=true" in contract
    assert "production profile cannot allow provisional geometry" in contract
    assert "production profile requires locked geometry" in contract
    assert "host_role:=all is reserved for bench use" in contract

    for mode in ("autonomous_mapping", "saved_map_navigation"):
        try:
            validate_selection(
                "core",
                mode,
                "lidar_d435_voxel",
                d435_voxel_validated=False,
                require_locked_geometry=True,
                allow_provisional_geometry=False,
            )
        except RuntimeError as error:
            assert "d435_voxel_validated:=true" in str(error)
        else:
            raise AssertionError("unvalidated D435 voxel mode was accepted")


def test_navigation_mode_and_profile_selection_matrix() -> None:
    """Select Nav2 only for autonomous modes and their sensor profile."""
    common = {
        "host_role": "core",
        "start_bridge": False,
        "start_realsense": False,
        "start_vo": False,
        "start_speech": False,
    }
    assert resolve_requirements(
        robot_mode="safe_idle",
        bringup_profile="lidar_only",
        **common,
    ).require_navigation is False
    assert resolve_requirements(
        robot_mode="manual_mapping",
        bringup_profile="lidar_only",
        **common,
    ).require_navigation is False
    for mode in ("autonomous_mapping", "saved_map_navigation"):
        assert resolve_requirements(
            robot_mode=mode,
            bringup_profile="lidar_only",
            **common,
        ).require_navigation is True

    core = read("launch/core_bringup.launch.py")
    assert '"nav2_live_mapping_voxel.yaml"' in core
    assert 'else "nav2_live_mapping.yaml"' in core
    assert '"nav2_saved_map_voxel.yaml"' in core
    assert 'else "nav2_saved_map.yaml"' in core
    assert '"nav_params_file": nav_params' in core
    assert '"nav_readiness_params": readiness_params' in core


def test_readiness_authority_is_cpp_and_publishes_operator_contract() -> None:
    """Readiness must remain a C++ aggregator with stable outputs."""
    cmake = read("CMakeLists.txt")
    node = read("src/nodes/bringup_readiness_node.cpp")
    assert "add_executable(bringup_readiness_node" in cmake
    assert 'output_namespace_ + "/state"' in node
    assert 'output_namespace_ + "/ready"' in node
    assert 'output_namespace_ + "/heartbeat"' in node
    assert 'output_namespace_ + "/diagnostics"' in node
    assert "startup_timeout_missing=" in node
    assert "all_required_bringup_dependencies_ready" in node
    for dependency in {
        "base",
        "control",
        "safety_state",
        "lidar_heartbeat",
        "perception_heartbeat",
        "localization",
        "active_release",
        "map_context",
        "goal_admission",
        "power",
        "obstacle_cloud",
    }:
        assert f'"{dependency}"' in node


def test_profiles_are_versioned_and_keep_voxel_disabled_by_default() -> None:
    """Profiles must be versioned with LiDAR-only defaults."""
    profiles = {}
    for name in ["bench", "lidar_only", "lidar_d435_voxel", "production"]:
        path = ROOT / "config" / "profiles" / f"{name}.yaml"
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        profile = document["robot_savo_bringup_profile"]
        assert profile["schema_version"] == 1
        assert profile["profile"] == name
        profiles[name] = profile

    assert profiles["lidar_only"]["d435_voxel_validated"] is False
    assert profiles["lidar_only"]["navigation_profile"] == "lidar_only"
    assert profiles["lidar_d435_voxel"]["d435_voxel_validated"] is False
    assert profiles["production"]["allow_provisional_geometry"] is False


def test_saved_navigation_preserves_am8_release_gate() -> None:
    """Saved navigation must retain the package-owned AM-8 gate."""
    core = read("launch/core_bringup.launch.py")
    wrapper = read("launch/saved_map_navigation.launch.py")
    assert "production_navigation.launch.py" in core
    assert "production_map_root" in core
    assert "active_map_contract" in core
    assert "nav2_saved_map.yaml" in core
    assert "nav2_saved_map_voxel.yaml" in core
    assert '"robot_mode": "saved_map_navigation"' in wrapper


def test_autonomous_mapping_still_uses_review_and_quality_contract_v2() -> None:
    """Autonomous mapping must retain review and quality contract v2."""
    launch = read("launch/autonomous_mapping.launch.py")
    readme = read("README.md")
    assert '"start_review_gateway": "true"' in launch
    assert "contract_version: 2" in readme
    assert "require_quality_approval: true" in readme


def test_core_and_edge_readiness_topics_do_not_collide() -> None:
    """Each host must publish readiness in a distinct namespace."""
    core = yaml.safe_load(read("config/core_real_robot.yaml"))
    edge = yaml.safe_load(read("config/edge_real_robot.yaml"))
    core_ns = core["bringup_readiness_node"]["ros__parameters"][
        "output_namespace"
    ]
    edge_ns = edge["bringup_readiness_node"]["ros__parameters"][
        "output_namespace"
    ]
    assert core_ns == "/savo_bringup/core"
    assert edge_ns == "/savo_bringup/edge"
    assert core_ns != edge_ns


def test_deployment_entry_points_use_real_packages_and_launches() -> None:
    """Deployment entry points must select valid roles and paths."""
    project_root = ROOT.parents[3]
    core_env = (project_root / "deploy/core/env_core.sh").read_text()
    edge_env = (project_root / "deploy/edge/env_edge.sh").read_text()
    core_run = (project_root / "deploy/core/run_core.sh").read_text()
    edge_run = (project_root / "deploy/edge/run_edge.sh").read_text()
    storage = (
        project_root / "deploy/core/prepare_runtime_storage.sh"
    ).read_text()

    assert "savo_dashboard" not in core_env + edge_env
    assert "savo_intent" not in core_env + edge_env
    assert "robot_bringup.launch.py" in core_run
    assert "robot_bringup.launch.py" in edge_run
    assert "host_role:=core" in core_run
    assert "host_role:=edge" in edge_run
    assert '"${STATE_ROOT}/maps/production"' in storage
    assert '"${STATE_ROOT}/locations/releases"' in storage
    assert '"${STATE_ROOT}/localization"' in storage
    assert '"${STATE_ROOT}/localization"' in core_run
    assert "map_output_root" in core_run
    assert "locations_database_path" in core_run
    assert "supervisor_state_path" in core_run
    assert "role=core mode=%s profile=%s startup=STOP" in core_run
    assert "role=edge mode=%s profile=%s startup=STOP" in edge_run


def test_runtime_storage_is_idempotent_without_root(tmp_path: Path) -> None:
    """Custom-root storage preparation must be safe and idempotent."""
    project_root = ROOT.parents[3]
    script = project_root / "deploy/core/prepare_runtime_storage.sh"
    state_root = tmp_path / "state"
    log_root = tmp_path / "log"
    owner = pwd.getpwuid(os.getuid()).pw_name
    group = grp.getgrgid(os.getgid()).gr_name
    command = [
        str(script),
        "--owner",
        owner,
        "--group",
        group,
        "--state-root",
        str(state_root),
        "--log-root",
        str(log_root),
    ]
    subprocess.run(command, check=True)
    subprocess.run(command, check=True)
    for path in {
        state_root / "maps/sessions",
        state_root / "maps/production",
        state_root / "maps/release_transactions",
        state_root / "locations/releases",
        state_root / "localization",
        state_root / "supervisor",
        log_root,
    }:
        assert path.is_dir()
        assert path.stat().st_mode & 0o002 == 0
