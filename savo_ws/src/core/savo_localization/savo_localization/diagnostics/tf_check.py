#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""TF health checks for Robot Savo localization. No ROS imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Iterable

from savo_localization.constants import (
    FRAME_BASE_LINK,
    FRAME_IMU,
    FRAME_MAP,
    FRAME_ODOM,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_WARN,
)
from savo_localization.utils.frames import make_tf_edge_label, require_frame_pair


@dataclass(frozen=True)
class TfEdgeCheckResult:
    parent_frame: str = ""
    child_frame: str = ""

    available: bool = False
    fresh: bool = False
    age_s: float | None = None

    status: str = STATUS_UNKNOWN
    message: str = ""
    reasons: list[str] = field(default_factory=list)

    @property
    def label(self) -> str:
        if not self.parent_frame or not self.child_frame:
            return ""

        return make_tf_edge_label(self.parent_frame, self.child_frame)

    @property
    def ok(self) -> bool:
        return self.available and self.fresh and self.status == STATUS_OK

    def to_dict(self) -> dict[str, object]:
        data = asdict(self)
        data["label"] = self.label
        data["ok"] = self.ok
        return data


@dataclass(frozen=True)
class TfChainCheckResult:
    status: str = STATUS_UNKNOWN
    ok: bool = False
    message: str = ""
    reasons: list[str] = field(default_factory=list)
    edges: dict[str, TfEdgeCheckResult] = field(default_factory=dict)

    @property
    def checked_count(self) -> int:
        return len(self.edges)

    @property
    def healthy_count(self) -> int:
        return sum(1 for edge in self.edges.values() if edge.ok)

    @property
    def available_count(self) -> int:
        return sum(1 for edge in self.edges.values() if edge.available)

    def to_dict(self) -> dict[str, object]:
        return {
            "status": self.status,
            "ok": self.ok,
            "message": self.message,
            "reasons": list(self.reasons),
            "checked_count": self.checked_count,
            "healthy_count": self.healthy_count,
            "available_count": self.available_count,
            "edges": {
                name: edge.to_dict()
                for name, edge in self.edges.items()
            },
        }


def tf_edge_label(parent_frame: str, child_frame: str) -> str:
    return make_tf_edge_label(parent_frame, child_frame)


def make_tf_edge_result(
    *,
    parent_frame: str,
    child_frame: str,
    available: bool,
    age_s: float | None,
    max_age_s: float,
    message: str = "",
) -> TfEdgeCheckResult:
    if max_age_s <= 0.0:
        raise ValueError("max_age_s must be > 0.0")

    parent, child = require_frame_pair(parent_frame, child_frame)

    reasons: list[str] = []

    if not available:
        reasons.append(f"transform not available: {parent} -> {child}")
        return TfEdgeCheckResult(
            parent_frame=parent,
            child_frame=child,
            available=False,
            fresh=False,
            age_s=age_s,
            status=STATUS_STALE,
            message=message or "TF missing",
            reasons=_unique_preserve_order(reasons),
        )

    if age_s is None:
        reasons.append("TF age is not available")
    elif age_s > max_age_s:
        reasons.append(
            f"TF stale: age_s={age_s:.3f} > max_age_s={max_age_s:.3f}"
        )

    fresh = not reasons

    return TfEdgeCheckResult(
        parent_frame=parent,
        child_frame=child,
        available=True,
        fresh=fresh,
        age_s=age_s,
        status=STATUS_OK if fresh else STATUS_STALE,
        message=message or ("TF healthy" if fresh else "TF stale"),
        reasons=_unique_preserve_order(reasons),
    )


def check_tf_edge(edge: TfEdgeCheckResult) -> TfEdgeCheckResult:
    if edge.ok:
        return edge

    reasons = list(edge.reasons)

    if not edge.available:
        reasons.append(f"transform not available: {edge.label}")

    if not edge.fresh:
        reasons.append(f"transform stale: {edge.label}")

    return TfEdgeCheckResult(
        parent_frame=edge.parent_frame,
        child_frame=edge.child_frame,
        available=edge.available,
        fresh=edge.fresh,
        age_s=edge.age_s,
        status=STATUS_STALE,
        message=edge.message or "TF edge not healthy",
        reasons=_unique_preserve_order(reasons),
    )


def check_tf_chain(
    edges: dict[str, TfEdgeCheckResult],
) -> TfChainCheckResult:
    if not edges:
        return TfChainCheckResult(
            status=STATUS_WARN,
            ok=False,
            message="No TF edges checked",
            reasons=["empty TF chain"],
            edges={},
        )

    checked_edges = {
        name: check_tf_edge(edge)
        for name, edge in edges.items()
    }

    reasons: list[str] = []

    for edge in checked_edges.values():
        if edge.ok:
            continue

        if edge.reasons:
            reasons.extend(f"{edge.label}: {reason}" for reason in edge.reasons)
        else:
            reasons.append(f"{edge.label}: TF edge not healthy")

    if not reasons:
        return TfChainCheckResult(
            status=STATUS_OK,
            ok=True,
            message="TF chain healthy",
            reasons=[],
            edges=checked_edges,
        )

    return TfChainCheckResult(
        status=STATUS_STALE,
        ok=False,
        message="TF chain not healthy",
        reasons=_unique_preserve_order(reasons),
        edges=checked_edges,
    )


def tf_chain_summary(
    edges: dict[str, TfEdgeCheckResult],
) -> dict[str, object]:
    return check_tf_chain(edges).to_dict()


TfEdge = TfEdgeCheckResult
TfCheckResult = TfEdgeCheckResult
TfTreeCheckResult = TfChainCheckResult


def tf_edge_from_dict(data: dict[str, object]) -> TfEdgeCheckResult:
    return TfEdgeCheckResult(
        parent_frame=str(data.get("parent_frame", "")),
        child_frame=str(data.get("child_frame", "")),
        available=bool(data.get("available", False)),
        fresh=bool(data.get("fresh", False)),
        age_s=data.get("age_s"),  # type: ignore[arg-type]
        status=str(data.get("status", STATUS_UNKNOWN)),
        message=str(data.get("message", "")),
        reasons=list(data.get("reasons", [])),  # type: ignore[arg-type]
    )


def check_odom_to_base_tf(
    *,
    available: bool,
    age_s: float | None,
    max_age_s: float,
    message: str = "",
) -> TfEdgeCheckResult:
    return make_tf_edge_result(
        parent_frame=FRAME_ODOM,
        child_frame=FRAME_BASE_LINK,
        available=available,
        age_s=age_s,
        max_age_s=max_age_s,
        message=message,
    )


def check_base_to_imu_tf(
    *,
    available: bool,
    age_s: float | None,
    max_age_s: float,
    message: str = "",
) -> TfEdgeCheckResult:
    return make_tf_edge_result(
        parent_frame=FRAME_BASE_LINK,
        child_frame=FRAME_IMU,
        available=available,
        age_s=age_s,
        max_age_s=max_age_s,
        message=message,
    )


def expected_localization_edges(
    *,
    include_map_to_odom: bool = False,
    include_base_to_imu: bool = True,
) -> dict[str, tuple[str, str]]:
    edges: dict[str, tuple[str, str]] = {}

    if include_map_to_odom:
        edges["map_to_odom"] = (FRAME_MAP, FRAME_ODOM)

    edges["odom_to_base"] = (FRAME_ODOM, FRAME_BASE_LINK)

    if include_base_to_imu:
        edges["base_to_imu"] = (FRAME_BASE_LINK, FRAME_IMU)

    return edges


def validate_expected_localization_edges(
    edges: dict[str, TfEdgeCheckResult],
    *,
    include_map_to_odom: bool = False,
    include_base_to_imu: bool = True,
) -> TfChainCheckResult:
    expected = expected_localization_edges(
        include_map_to_odom=include_map_to_odom,
        include_base_to_imu=include_base_to_imu,
    )

    checked = dict(edges)
    for name, (parent, child) in expected.items():
        if name not in checked:
            checked[name] = TfEdgeCheckResult(
                parent_frame=parent,
                child_frame=child,
                available=False,
                fresh=False,
                status=STATUS_STALE,
                message="TF missing",
                reasons=[f"transform not available: {parent} -> {child}"],
            )

    return check_tf_chain(checked)


def check_localization_tf_tree(
    edges: dict[str, TfEdgeCheckResult],
    *,
    include_map_to_odom: bool = False,
    include_base_to_imu: bool = True,
) -> TfChainCheckResult:
    return validate_expected_localization_edges(
        edges,
        include_map_to_odom=include_map_to_odom,
        include_base_to_imu=include_base_to_imu,
    )


def _unique_preserve_order(values: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []

    for value in values:
        if value in seen:
            continue

        seen.add(value)
        result.append(value)

    return result


__all__ = [
    "TfCheckResult",
    "TfEdge",
    "TfEdgeCheckResult",
    "TfChainCheckResult",
    "TfTreeCheckResult",
    "tf_edge_label",
    "tf_edge_from_dict",
    "make_tf_edge_result",
    "check_tf_edge",
    "check_tf_chain",
    "check_odom_to_base_tf",
    "check_base_to_imu_tf",
    "expected_localization_edges",
    "validate_expected_localization_edges",
    "check_localization_tf_tree",
    "tf_chain_summary",
]
