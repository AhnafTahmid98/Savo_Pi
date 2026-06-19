#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Mapping readiness state model. No ROS imports."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional


# =============================================================================
# Individual checks
# =============================================================================
@dataclass(frozen=True)
class ReadinessCheck:
    name: str
    ok: bool
    required: bool = True
    enabled: bool = True
    message: str = ""
    age_s: Optional[float] = None

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "ok": self.ok,
            "required": self.required,
            "enabled": self.enabled,
            "message": self.message,
            "age_s": self.age_s,
        }


# =============================================================================
# Full mapping readiness
# =============================================================================
@dataclass(frozen=True)
class MappingReadinessState:
    checks: Dict[str, ReadinessCheck] = field(default_factory=dict)

    @property
    def ready(self) -> bool:
        return all(
            check.ok
            for check in self.checks.values()
            if check.required and check.enabled
        )

    @property
    def degraded(self) -> bool:
        return any(
            not check.ok
            for check in self.checks.values()
            if not check.required and check.enabled
        )

    @property
    def failed_required_checks(self) -> tuple[str, ...]:
        return tuple(
            name
            for name, check in self.checks.items()
            if check.required and check.enabled and not check.ok
        )

    @property
    def failed_optional_checks(self) -> tuple[str, ...]:
        return tuple(
            name
            for name, check in self.checks.items()
            if not check.required and check.enabled and not check.ok
        )

    def get(self, name: str) -> Optional[ReadinessCheck]:
        return self.checks.get(name)

    def with_check(self, check: ReadinessCheck) -> "MappingReadinessState":
        updated = dict(self.checks)
        updated[check.name] = check
        return MappingReadinessState(checks=updated)

    def to_dict(self) -> dict:
        return {
            "ready": self.ready,
            "degraded": self.degraded,
            "failed_required_checks": list(self.failed_required_checks),
            "failed_optional_checks": list(self.failed_optional_checks),
            "checks": {
                name: check.to_dict()
                for name, check in self.checks.items()
            },
        }


# =============================================================================
# Builders
# =============================================================================
def make_check(
    name: str,
    ok: bool,
    required: bool = True,
    message: str = "",
    age_s: Optional[float] = None,
    enabled: bool = True,
) -> ReadinessCheck:
    return ReadinessCheck(
        name=str(name).strip(),
        ok=bool(ok),
        required=bool(required),
        enabled=bool(enabled),
        message=str(message),
        age_s=age_s,
    )


def build_readiness_state(checks: Iterable[ReadinessCheck]) -> MappingReadinessState:
    state = MappingReadinessState()

    for check in checks:
        state = state.with_check(check)

    return state


def build_idle_readiness_state() -> MappingReadinessState:
    return build_readiness_state(
        (
            make_check("scan", False, True, "Mapping idle.", enabled=False),
            make_check("odom", False, True, "Mapping idle.", enabled=False),
            make_check("tf", False, True, "Mapping idle.", enabled=False),
            make_check("map", False, False, "Mapping idle.", enabled=False),
            make_check("slam_toolbox", False, False, "Mapping idle.", enabled=False),
            make_check("pointcloud", False, False, "Mapping idle.", enabled=False),
            make_check("apriltag", False, False, "Mapping idle.", enabled=False),
        )
    )


def build_default_not_ready_state() -> MappingReadinessState:
    return build_readiness_state(
        (
            make_check("scan", False, True, "Waiting for LiDAR scan."),
            make_check("odom", False, True, "Waiting for filtered odometry."),
            make_check("tf", False, True, "Waiting for TF tree."),
            make_check("map", False, False, "Waiting for map topic.", enabled=False),
            make_check("slam_toolbox", False, False, "Waiting for slam_toolbox.", enabled=False),
            make_check("pointcloud", False, False, "RealSense pointcloud disabled.", enabled=False),
            make_check("apriltag", False, False, "AprilTag bridge disabled.", enabled=False),
        )
    )


def build_manual_mapping_ready_state() -> MappingReadinessState:
    return build_readiness_state(
        (
            make_check("scan", True, True, "LiDAR scan is ready."),
            make_check("odom", True, True, "Filtered odometry is ready."),
            make_check("tf", True, True, "TF tree is ready."),
            make_check("map", True, False, "Map topic is publishing."),
            make_check("slam_toolbox", True, False, "slam_toolbox is active."),
            make_check("pointcloud", False, False, "RealSense pointcloud not used.", enabled=False),
            make_check("apriltag", False, False, "AprilTag bridge disabled.", enabled=False),
        )
    )


def build_autonomous_mapping_ready_state(
    use_pointcloud: bool = True,
) -> MappingReadinessState:
    return build_readiness_state(
        (
            make_check("scan", True, True, "LiDAR scan is ready."),
            make_check("odom", True, True, "Filtered odometry is ready."),
            make_check("tf", True, True, "TF tree is ready."),
            make_check("map", True, True, "Map topic is publishing."),
            make_check("slam_toolbox", True, True, "slam_toolbox is active."),
            make_check("nav2", True, True, "Nav2 mapping stack is ready."),
            make_check(
                "pointcloud",
                use_pointcloud,
                False,
                "RealSense pointcloud assist enabled." if use_pointcloud
                else "RealSense pointcloud assist disabled.",
                enabled=use_pointcloud,
            ),
            make_check("apriltag", False, False, "AprilTag bridge disabled.", enabled=False),
        )
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    state = build_default_not_ready_state()

    print("Robot Savo mapping readiness:")
    print(state.to_dict())


if __name__ == "__main__":
    main()


__all__ = [
    "ReadinessCheck",
    "MappingReadinessState",
    "make_check",
    "build_readiness_state",
    "build_idle_readiness_state",
    "build_default_not_ready_state",
    "build_manual_mapping_ready_state",
    "build_autonomous_mapping_ready_state",
    "main",
]
