"""Shared validation and policy helpers for Robot Savo launch files."""

from dataclasses import dataclass


HOST_ROLES = {"core", "edge", "all"}
AUTO_HOST_ROLE = "auto"
HOSTNAME_ROLES = {
    "core": "core",
    "savo-core": "core",
    "edge": "edge",
    "savo-edge": "edge",
}
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


def should_start_obstacle_cloud(
    robot_mode: str,
    bringup_profile: str,
    *,
    d435_voxel_validated: bool,
    explicit_start: bool,
) -> bool:
    """Run the validated D435 helper for selected production use."""
    return d435_voxel_validated and (
        explicit_start
        or bringup_profile in {"lidar_d435_voxel", "production"}
        or robot_mode in {"autonomous_mapping", "saved_map_navigation"}
    )


def resolve_host_role(
    requested_role: str,
    hostname: str,
    environment_role: str | None = None,
) -> str:
    """Resolve the canonical host role without guessing on unknown hosts."""
    requested = requested_role.strip().lower()
    if requested not in HOST_ROLES | {AUTO_HOST_ROLE}:
        raise RuntimeError(f"unsupported host_role: {requested_role}")

    short_hostname = hostname.strip().lower().split(".", maxsplit=1)[0]
    detected_role = HOSTNAME_ROLES.get(short_hostname)

    configured_role = (environment_role or "").strip().lower()
    if configured_role:
        if configured_role not in {"core", "edge"}:
            raise RuntimeError(
                "SAVO_ROLE must be exactly 'core' or 'edge' when set"
            )
        if detected_role is not None and configured_role != detected_role:
            raise RuntimeError(
                "SAVO_ROLE does not match Robot Savo hostname: "
                f"hostname={short_hostname}, SAVO_ROLE={configured_role}"
            )

    if requested != AUTO_HOST_ROLE:
        return requested
    if detected_role is None:
        raise RuntimeError(
            "host_role:=auto cannot identify this host: "
            f"hostname={short_hostname or '<empty>'}; expected "
            "core, savo-core, edge, or savo-edge"
        )
    return detected_role


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
