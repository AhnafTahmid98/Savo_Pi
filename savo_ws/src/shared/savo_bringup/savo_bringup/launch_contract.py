"""Shared validation and policy helpers for Robot Savo launch files."""

from dataclasses import dataclass


HOST_ROLES = {"core", "edge", "all"}
ROBOT_MODES = {
    "safe_idle",
    "manual",
    "manual_mapping",
    "autonomous_mapping",
    "saved_map_navigation",
    "diagnostics",
}
BRINGUP_PROFILES = {
    "bench",
    "lidar_only",
    "lidar_d435_voxel",
    "production",
}


@dataclass(frozen=True)
class LaunchRequirements:
    """Resolved component requirements for one host and robot mode."""

    start_core: bool
    start_edge: bool
    require_supervisor: bool
    require_navigation: bool
    require_mapping: bool
    require_bridge: bool
    require_realsense: bool
    require_vo: bool
    require_speech: bool
    require_locked_geometry: bool
    voxel_layer_enabled: bool


def as_bool(value: str) -> bool:
    """Interpret the common ROS launch boolean spellings."""
    return value.strip().lower() in {"1", "true", "yes", "on"}


def validate_selection(
    host_role: str,
    robot_mode: str,
    bringup_profile: str,
    *,
    d435_voxel_validated: bool,
    require_locked_geometry: bool,
    allow_provisional_geometry: bool,
) -> None:
    """Fail closed for impossible or unsafe launch combinations."""
    if host_role not in HOST_ROLES:
        raise RuntimeError(f"unsupported host_role: {host_role}")
    if robot_mode not in ROBOT_MODES:
        raise RuntimeError(f"unsupported robot_mode: {robot_mode}")
    if bringup_profile not in BRINGUP_PROFILES:
        raise RuntimeError(f"unsupported bringup_profile: {bringup_profile}")

    if bringup_profile == "lidar_d435_voxel" and not d435_voxel_validated:
        raise RuntimeError(
            "lidar_d435_voxel requires d435_voxel_validated:=true"
        )

    if host_role == "all" and bringup_profile != "bench":
        raise RuntimeError("host_role:=all is reserved for bench use")

    if bringup_profile == "production" and not require_locked_geometry:
        raise RuntimeError("production profile requires locked geometry")

    motion_mode = robot_mode in {
        "manual",
        "manual_mapping",
        "autonomous_mapping",
        "saved_map_navigation",
    }
    if motion_mode and bringup_profile != "bench" and not require_locked_geometry:
        raise RuntimeError("motion profiles require locked geometry validation")

    if require_locked_geometry and allow_provisional_geometry:
        raise RuntimeError(
            "require_locked_geometry and allow_provisional_geometry conflict"
        )
    if bringup_profile == "production" and allow_provisional_geometry:
        raise RuntimeError("production profile cannot allow provisional geometry")


def resolve_requirements(
    host_role: str,
    robot_mode: str,
    bringup_profile: str,
    *,
    start_bridge: bool,
    start_realsense: bool,
    start_vo: bool,
    start_speech: bool,
) -> LaunchRequirements:
    """Resolve the readiness requirements used by launch and C++ authority."""
    start_core = host_role in {"core", "all"}
    start_edge = host_role in {"edge", "all"}
    return LaunchRequirements(
        start_core=start_core,
        start_edge=start_edge,
        require_supervisor=start_core and robot_mode != "diagnostics",
        require_navigation=start_core
        and robot_mode in {"autonomous_mapping", "saved_map_navigation"},
        require_mapping=start_core
        and robot_mode in {"manual_mapping", "autonomous_mapping"},
        require_bridge=start_edge and start_bridge,
        require_realsense=start_edge and start_realsense,
        require_vo=start_edge and start_vo,
        require_speech=start_edge and start_speech,
        require_locked_geometry=(
            start_core
            and robot_mode
            in {
                "manual",
                "manual_mapping",
                "autonomous_mapping",
                "saved_map_navigation",
            }
            and bringup_profile != "bench"
        ),
        voxel_layer_enabled=bringup_profile == "lidar_d435_voxel",
    )
