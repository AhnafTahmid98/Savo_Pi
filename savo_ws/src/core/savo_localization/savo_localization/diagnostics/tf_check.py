#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""TF health checks for Robot Savo localization. No ROS imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

from savo_localization.constants import (
    DEFAULT_EKF_SENSOR_TIMEOUT_S,
    FRAME_BASE_LINK,
    FRAME_IMU,
    FRAME_ODOM,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_WARN,
)
from savo_localization.utils.frames import normalize_frame_id


@dataclass(frozen=True)
class TfEdge:
    parent_frame: str
    child_frame: str

    def normalized(self) -> "TfEdge":
        return TfEdge(
            parent_frame=normalize_frame_id(self.parent_frame),
            child_frame=normalize_frame_id(self.child_frame),
        )

    def validate(self) -> None:
        parent = normalize_frame_id(self.parent_frame)
        child = normalize_frame_id(self.child_frame)

        if parent == child:
            raise ValueError(f"TF parent and child cannot be the same: {parent}")

    def label(self) -> str:
        edge = self.normalized()
        return f"{edge.parent_frame} -> {edge.child_frame}"

    def to_dict(self) -> dict[str, str]:
        edge = self.normalized()
        return {
            "parent_frame": edge.parent_frame,
            "child_frame": edge.child_frame,
            "label": edge.label(),
        }


@dataclass(frozen=True)
class TfCheckResult:
    edge: TfEdge
    status: str
    ok: bool
    message: str

    available: bool = False
    fresh: bool = False
    age_s: float | None = None
    timeout_s: float = DEFAULT_EKF_SENSOR_TIMEOUT_S

    reasons: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "edge": self.edge.to_dict(),
            "status": self.status,
            "ok": bool(self.ok),
            "message": self.message,
            "available": bool(self.available),
            "fresh": bool(self.fresh),
            "age_s": self.age_s,
            "timeout_s": float(self.timeout_s),
            "reasons": list(self.reasons),
        }


@dataclass(frozen=True)
class TfTreeCheckResult:
    status: str
    ok: bool
    message: str
    reasons: list[str] = field(default_factory=list)
    results: dict[str, TfCheckResult] = field(default_factory=dict)

    @property
    def available_count(self) -> int:
        return sum(1 for result in self.results.values() if result.available)

    @property
    def healthy_count(self) -> int:
        return sum(1 for result in self.results.values() if result.ok)

    @property
    def checked_count(self) -> int:
        return len(self.results)

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "ok": bool(self.ok),
            "message": self.message,
            "reasons": list(self.reasons),
            "available_count": self.available_count,
            "healthy_count": self.healthy_count,
            "checked_count": self.checked_count,
            "results": {
                name: result.to_dict()
                for name, result in self.results.items()
            },
        }


def check_tf_edge(
    *,
    parent_frame: str,
    child_frame: str,
    available: bool,
    age_s: float | None = None,
    timeout_s: float = DEFAULT_EKF_SENSOR_TIMEOUT_S,
) -> TfCheckResult:
    edge = TfEdge(parent_frame=parent_frame, child_frame=child_frame)

    try:
        edge.validate()
    except ValueError as exc:
        return TfCheckResult(
            edge=edge,
            status=STATUS_ERROR,
            ok=False,
            message="TF edge invalid",
            available=False,
            fresh=False,
            age_s=age_s,
            timeout_s=timeout_s,
            reasons=[str(exc)],
        )

    if timeout_s <= 0.0:
        raise ValueError(f"timeout_s must be > 0.0, got {timeout_s}")

    reasons: list[str] = []

    if not available:
        reasons.append(f"transform not available: {edge.label()}")

        return TfCheckResult(
            edge=edge,
            status=STATUS_STALE,
            ok=False,
            message="TF transform missing",
            available=False,
            fresh=False,
            age_s=age_s,
            timeout_s=timeout_s,
            reasons=reasons,
        )

    if age_s is None:
        reasons.append("transform age is not available")

        return TfCheckResult(
            edge=edge,
            status=STATUS_WARN,
            ok=True,
            message="TF transform available but age unknown",
            available=True,
            fresh=False,
            age_s=None,
            timeout_s=timeout_s,
            reasons=reasons,
        )

    age = max(0.0, float(age_s))
    fresh = age <= float(timeout_s)

    if not fresh:
        reasons.append(f"transform stale: age_s={age:.3f} > timeout_s={timeout_s:.3f}")

        return TfCheckResult(
            edge=edge,
            status=STATUS_STALE,
            ok=False,
            message="TF transform stale",
            available=True,
            fresh=False,
            age_s=age,
            timeout_s=timeout_s,
            reasons=reasons,
        )

    return TfCheckResult(
        edge=edge,
        status=STATUS_OK,
        ok=True,
        message="TF transform healthy",
        available=True,
        fresh=True,
        age_s=age,
        timeout_s=timeout_s,
        reasons=[],
    )


def check_odom_to_base_tf(
    *,
    available: bool,
    age_s: float | None = None,
    timeout_s: float = DEFAULT_EKF_SENSOR_TIMEOUT_S,
    odom_frame: str = FRAME_ODOM,
    base_frame: str = FRAME_BASE_LINK,
) -> TfCheckResult:
    return check_tf_edge(
        parent_frame=odom_frame,
        child_frame=base_frame,
        available=available,
        age_s=age_s,
        timeout_s=timeout_s,
    )


def check_base_to_imu_tf(
    *,
    available: bool,
    age_s: float | None = None,
    timeout_s: float = DEFAULT_EKF_SENSOR_TIMEOUT_S,
    base_frame: str = FRAME_BASE_LINK,
    imu_frame: str = FRAME_IMU,
) -> TfCheckResult:
    return check_tf_edge(
        parent_frame=base_frame,
        child_frame=imu_frame,
        available=available,
        age_s=age_s,
        timeout_s=timeout_s,
    )


def check_localization_tf_tree(
    *,
    odom_to_base_available: bool,
    base_to_imu_available: bool,
    odom_to_base_age_s: float | None = None,
    base_to_imu_age_s: float | None = None,
    timeout_s: float = DEFAULT_EKF_SENSOR_TIMEOUT_S,
    odom_frame: str = FRAME_ODOM,
    base_frame: str = FRAME_BASE_LINK,
    imu_frame: str = FRAME_IMU,
) -> TfTreeCheckResult:
    results = {
        "odom_to_base": check_odom_to_base_tf(
            available=odom_to_base_available,
            age_s=odom_to_base_age_s,
            timeout_s=timeout_s,
            odom_frame=odom_frame,
            base_frame=base_frame,
        ),
        "base_to_imu": check_base_to_imu_tf(
            available=base_to_imu_available,
            age_s=base_to_imu_age_s,
            timeout_s=timeout_s,
            base_frame=base_frame,
            imu_frame=imu_frame,
        ),
    }

    reasons: list[str] = []
    for name, result in results.items():
        if not result.ok:
            reasons.extend(f"{name}: {reason}" for reason in result.reasons)

    if not reasons:
        return TfTreeCheckResult(
            status=STATUS_OK,
            ok=True,
            message="localization TF tree healthy",
            reasons=[],
            results=results,
        )

    if any(result.status == STATUS_ERROR for result in results.values()):
        return TfTreeCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="localization TF tree invalid",
            reasons=_unique_preserve_order(reasons),
            results=results,
        )

    return TfTreeCheckResult(
        status=STATUS_STALE,
        ok=False,
        message="localization TF tree not ready",
        reasons=_unique_preserve_order(reasons),
        results=results,
    )


def tf_edge_from_dict(data: dict[str, Any]) -> TfEdge:
    return TfEdge(
        parent_frame=str(data.get("parent_frame", "")),
        child_frame=str(data.get("child_frame", "")),
    )


def tf_edge_label(parent_frame: str, child_frame: str) -> str:
    return TfEdge(
        parent_frame=parent_frame,
        child_frame=child_frame,
    ).label()


def expected_localization_edges(
    *,
    odom_frame: str = FRAME_ODOM,
    base_frame: str = FRAME_BASE_LINK,
    imu_frame: str = FRAME_IMU,
) -> dict[str, TfEdge]:
    return {
        "odom_to_base": TfEdge(odom_frame, base_frame).normalized(),
        "base_to_imu": TfEdge(base_frame, imu_frame).normalized(),
    }


def validate_expected_localization_edges(
    *,
    odom_frame: str = FRAME_ODOM,
    base_frame: str = FRAME_BASE_LINK,
    imu_frame: str = FRAME_IMU,
) -> None:
    for edge in expected_localization_edges(
        odom_frame=odom_frame,
        base_frame=base_frame,
        imu_frame=imu_frame,
    ).values():
        edge.validate()


def _unique_preserve_order(values: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []

    for value in values:
        if value in seen:
            continue

        seen.add(value)
        result.append(value)

    return result