#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Mapping mode model. No ROS imports."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Final, Iterable


# =============================================================================
# Mapping modes
# =============================================================================
class MappingMode(str, Enum):
    IDLE = "idle"
    MANUAL_MAPPING = "manual_mapping"
    AUTONOMOUS_MAPPING = "autonomous_mapping"
    LOCALIZATION_CHECK = "localization_check"
    MAP_SAVING = "map_saving"

    @classmethod
    def values(cls) -> tuple[str, ...]:
        return tuple(mode.value for mode in cls)

    @classmethod
    def from_string(cls, value: str) -> "MappingMode":
        normalized = normalize_mapping_mode(value)

        for mode in cls:
            if mode.value == normalized:
                return mode

        raise ValueError(f"Unsupported mapping mode: {value!r}")


VALID_MAPPING_MODES: Final[tuple[str, ...]] = MappingMode.values()


# =============================================================================
# Helpers
# =============================================================================
def normalize_mapping_mode(value: str) -> str:
    mode = str(value).strip().lower().replace("-", "_").replace(" ", "_")

    if not mode:
        raise ValueError("Mapping mode cannot be empty.")

    return mode


def is_valid_mapping_mode(value: str) -> bool:
    try:
        MappingMode.from_string(value)
        return True
    except ValueError:
        return False


def require_valid_mapping_mode(value: str) -> str:
    return MappingMode.from_string(value).value


def list_mapping_modes() -> tuple[str, ...]:
    return VALID_MAPPING_MODES


def validate_mapping_modes(values: Iterable[str]) -> None:
    for value in values:
        require_valid_mapping_mode(value)


# =============================================================================
# Mode capabilities
# =============================================================================
@dataclass(frozen=True)
class MappingModeProfile:
    mode: MappingMode
    use_slam_toolbox: bool
    use_nav2: bool
    use_frontier_explorer: bool
    use_realsense_pointcloud: bool
    allow_teleop: bool
    allow_teleop_override: bool
    allow_map_save: bool

    def to_dict(self) -> dict:
        return {
            "mode": self.mode.value,
            "use_slam_toolbox": self.use_slam_toolbox,
            "use_nav2": self.use_nav2,
            "use_frontier_explorer": self.use_frontier_explorer,
            "use_realsense_pointcloud": self.use_realsense_pointcloud,
            "allow_teleop": self.allow_teleop,
            "allow_teleop_override": self.allow_teleop_override,
            "allow_map_save": self.allow_map_save,
        }


MODE_PROFILES: Final[dict[MappingMode, MappingModeProfile]] = {
    MappingMode.IDLE: MappingModeProfile(
        mode=MappingMode.IDLE,
        use_slam_toolbox=False,
        use_nav2=False,
        use_frontier_explorer=False,
        use_realsense_pointcloud=False,
        allow_teleop=False,
        allow_teleop_override=False,
        allow_map_save=False,
    ),
    MappingMode.MANUAL_MAPPING: MappingModeProfile(
        mode=MappingMode.MANUAL_MAPPING,
        use_slam_toolbox=True,
        use_nav2=False,
        use_frontier_explorer=False,
        use_realsense_pointcloud=False,
        allow_teleop=True,
        allow_teleop_override=False,
        allow_map_save=True,
    ),
    MappingMode.AUTONOMOUS_MAPPING: MappingModeProfile(
        mode=MappingMode.AUTONOMOUS_MAPPING,
        use_slam_toolbox=True,
        use_nav2=True,
        use_frontier_explorer=True,
        use_realsense_pointcloud=True,
        allow_teleop=False,
        allow_teleop_override=True,
        allow_map_save=True,
    ),
    MappingMode.LOCALIZATION_CHECK: MappingModeProfile(
        mode=MappingMode.LOCALIZATION_CHECK,
        use_slam_toolbox=False,
        use_nav2=False,
        use_frontier_explorer=False,
        use_realsense_pointcloud=False,
        allow_teleop=False,
        allow_teleop_override=False,
        allow_map_save=False,
    ),
    MappingMode.MAP_SAVING: MappingModeProfile(
        mode=MappingMode.MAP_SAVING,
        use_slam_toolbox=False,
        use_nav2=False,
        use_frontier_explorer=False,
        use_realsense_pointcloud=False,
        allow_teleop=False,
        allow_teleop_override=False,
        allow_map_save=True,
    ),
}


def get_mode_profile(value: str | MappingMode) -> MappingModeProfile:
    mode = value if isinstance(value, MappingMode) else MappingMode.from_string(value)
    return MODE_PROFILES[mode]


def mode_uses_nav2(value: str | MappingMode) -> bool:
    return get_mode_profile(value).use_nav2


def mode_uses_frontier_explorer(value: str | MappingMode) -> bool:
    return get_mode_profile(value).use_frontier_explorer


def mode_allows_teleop(value: str | MappingMode) -> bool:
    return get_mode_profile(value).allow_teleop


def mode_allows_map_save(value: str | MappingMode) -> bool:
    return get_mode_profile(value).allow_map_save


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    print("Robot Savo mapping modes:")

    for mode in MappingMode:
        profile = get_mode_profile(mode)
        print(f"- {mode.value}: {profile.to_dict()}")


if __name__ == "__main__":
    main()


__all__ = [
    "MappingMode",
    "VALID_MAPPING_MODES",
    "MappingModeProfile",
    "MODE_PROFILES",
    "normalize_mapping_mode",
    "is_valid_mapping_mode",
    "require_valid_mapping_mode",
    "list_mapping_modes",
    "validate_mapping_modes",
    "get_mode_profile",
    "mode_uses_nav2",
    "mode_uses_frontier_explorer",
    "mode_allows_teleop",
    "mode_allows_map_save",
    "main",
]