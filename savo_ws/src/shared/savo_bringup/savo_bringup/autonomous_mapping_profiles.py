"""Allowlisted asset pairs for dedicated autonomous mapping."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AutonomousMappingAssets:
    """Perception and Nav2 files selected as one safety profile."""

    perception_config_filename: str
    nav_params_filename: str


_PROFILES = {
    "production": AutonomousMappingAssets(
        perception_config_filename="core_real_robot_v1.yaml",
        nav_params_filename="nav2_live_mapping.yaml",
    ),
    "core_lidar_mapping_degraded": AutonomousMappingAssets(
        perception_config_filename="core_lidar_mapping_degraded.yaml",
        nav_params_filename="nav2_live_mapping_core_lidar_degraded.yaml",
    ),
}


def validate_startup_scan360_disabled(value: str) -> None:
    """Reject any attempt to enable startup Scan360 for dedicated mapping."""
    if value.strip().lower() not in {"false", "0", "no", "off"}:
        raise ValueError(
            "dedicated autonomous mapping requires "
            "initial_scan360_required must remain false"
        )


def resolve_autonomous_mapping_profile(
    profile_name: str,
    *,
    perception_config_file: str = "",
    nav_params_file: str = "",
) -> AutonomousMappingAssets:
    """Resolve one approved pair and reject direct asset overrides."""
    if perception_config_file.strip() or nav_params_file.strip():
        raise ValueError(
            "autonomous mapping perception and Nav2 assets are selected atomically; "
            "direct perception_config_file/nav_params_file overrides are forbidden"
        )

    normalized_name = profile_name.strip()
    try:
        return _PROFILES[normalized_name]
    except KeyError as error:
        allowed = ", ".join(_PROFILES)
        raise ValueError(
            "unsupported autonomous mapping profile "
            f"{normalized_name!r}; allowed values: {allowed}"
        ) from error


__all__ = [
    "AutonomousMappingAssets",
    "resolve_autonomous_mapping_profile",
    "validate_startup_scan360_disabled",
]
