"""Contracts for the complete distributed Robot Savo bringup."""

import ast
import grp
import os
import pwd
import subprocess
from pathlib import Path

import yaml

from savo_bringup.launch_contract import resolve_host_role
from savo_bringup.launch_contract import validate_selection


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
    assert 'or profile == "lidar_d435_voxel"' in launch


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


def test_motion_modes_fail_closed_on_geometry_and_voxel_validation() -> None:
    """Unsafe geometry and voxel selections must be rejected."""
    contract = read("savo_bringup/launch_contract.py")
    assert "motion profiles require locked geometry validation" in contract
    assert "d435_voxel_validated:=true" in contract
    assert "production profile cannot allow provisional geometry" in contract
    assert "production profile requires locked geometry" in contract
    assert "host_role:=all is reserved for bench use" in contract


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
    assert profiles["lidar_d435_voxel"]["d435_voxel_validated"] is True
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
        state_root / "supervisor",
        log_root,
    }:
        assert path.is_dir()
        assert path.stat().st_mode & 0o002 == 0
