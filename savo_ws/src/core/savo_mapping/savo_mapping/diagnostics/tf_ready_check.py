#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""TF readiness check. No ROS runtime required."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional

from savo_mapping.constants import (
    DEFAULT_TF_TIMEOUT_S,
    FRAME_BASE_LINK,
    FRAME_LASER,
    FRAME_MAP,
    FRAME_ODOM,
)
from savo_mapping.utils.diagnostics import (
    DiagnosticItem,
    make_error,
    make_ok,
    make_stale,
    make_warn,
)


# =============================================================================
# TF edge result
# =============================================================================
@dataclass(frozen=True)
class TfEdgeResult:
    parent_frame: str
    child_frame: str
    ok: bool
    stale: bool = True
    age_s: Optional[float] = None
    message: str = ""

    @property
    def key(self) -> str:
        return f"{self.parent_frame}->{self.child_frame}"

    def to_dict(self) -> dict:
        return {
            "key": self.key,
            "parent_frame": self.parent_frame,
            "child_frame": self.child_frame,
            "ok": self.ok,
            "stale": self.stale,
            "age_s": self.age_s,
            "message": self.message,
        }


# =============================================================================
# TF readiness result
# =============================================================================
@dataclass(frozen=True)
class TfReadyResult:
    ok: bool
    stale: bool = True
    required_edges: tuple[TfEdgeResult, ...] = field(default_factory=tuple)
    frame_count: int = 0
    message: str = "Waiting for TF tree."
    timestamp_s: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "stale": self.stale,
            "frame_count": self.frame_count,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "required_edges": [
                edge.to_dict()
                for edge in self.required_edges
            ],
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Check logic
# =============================================================================
def make_tf_edge_result(
    parent_frame: str,
    child_frame: str,
    available: bool,
    age_s: Optional[float],
    timeout_s: float = DEFAULT_TF_TIMEOUT_S,
) -> TfEdgeResult:
    parent = str(parent_frame).strip()
    child = str(child_frame).strip()

    stale = True if age_s is None else float(age_s) > max(0.0, float(timeout_s))
    ok = bool(available) and not stale and bool(parent) and bool(child)

    failures: list[str] = []

    if not parent:
        failures.append("missing_parent_frame")

    if not child:
        failures.append("missing_child_frame")

    if not available:
        failures.append("not_available")

    if stale:
        failures.append("stale")

    if ok:
        message = "TF edge ready."
    else:
        message = f"TF edge not ready: {', '.join(failures)}."

    return TfEdgeResult(
        parent_frame=parent,
        child_frame=child,
        ok=ok,
        stale=stale,
        age_s=age_s,
        message=message,
    )


def evaluate_tf_ready(
    edge_results: Iterable[TfEdgeResult],
    frame_count: int = 0,
) -> TfReadyResult:
    edges = tuple(edge_results)

    if not edges:
        return TfReadyResult(
            ok=False,
            stale=True,
            required_edges=(),
            frame_count=max(0, int(frame_count)),
            message="TF tree not ready: no required edges configured.",
        )

    stale = any(edge.stale for edge in edges)
    ok = all(edge.ok for edge in edges)

    if ok:
        message = "TF tree ready."
    else:
        failed = [edge.key for edge in edges if not edge.ok]
        message = f"TF tree not ready: {', '.join(failed)}."

    return TfReadyResult(
        ok=ok,
        stale=stale,
        required_edges=edges,
        frame_count=max(0, int(frame_count)),
        message=message,
    )


def evaluate_default_tf_ready(
    edge_ages_s: Optional[Dict[str, Optional[float]]] = None,
    available_edges: Optional[Dict[str, bool]] = None,
    frame_count: int = 0,
    timeout_s: float = DEFAULT_TF_TIMEOUT_S,
) -> TfReadyResult:
    ages = edge_ages_s or {}
    available = available_edges or {}

    required_pairs = (
        (FRAME_MAP, FRAME_ODOM),
        (FRAME_ODOM, FRAME_BASE_LINK),
        (FRAME_BASE_LINK, FRAME_LASER),
    )

    edges = []

    for parent, child in required_pairs:
        key = f"{parent}->{child}"
        edges.append(
            make_tf_edge_result(
                parent_frame=parent,
                child_frame=child,
                available=bool(available.get(key, False)),
                age_s=ages.get(key),
                timeout_s=timeout_s,
            )
        )

    return evaluate_tf_ready(edges, frame_count=frame_count)


def tf_result_to_diagnostic(
    result: TfReadyResult,
    required: bool = True,
) -> DiagnosticItem:
    values = result.to_dict()

    if result.ok:
        return make_ok(
            "tf",
            result.message,
            required=required,
            values=values,
        )

    if result.stale:
        return make_stale(
            "tf",
            result.message,
            required=required,
            values=values,
        )

    if not result.required_edges:
        return make_error(
            "tf",
            result.message,
            required=required,
            values=values,
        )

    return make_warn(
        "tf",
        result.message,
        required=required,
        values=values,
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    result = evaluate_default_tf_ready(
        edge_ages_s={
            "map->odom": 0.05,
            "odom->base_link": 0.05,
            "base_link->laser": 0.0,
        },
        available_edges={
            "map->odom": True,
            "odom->base_link": True,
            "base_link->laser": True,
        },
        frame_count=4,
    )

    print(result.to_json(indent=2))
    print(tf_result_to_diagnostic(result).to_dict())


if __name__ == "__main__":
    main()


__all__ = [
    "TfEdgeResult",
    "TfReadyResult",
    "make_tf_edge_result",
    "evaluate_tf_ready",
    "evaluate_default_tf_ready",
    "tf_result_to_diagnostic",
    "main",
]