#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Frame helpers for Robot Savo localization."""

from __future__ import annotations

from dataclasses import dataclass

from savo_localization.constants import (
    FRAME_BASE_LINK,
    FRAME_FILTERED_ODOM,
    FRAME_IMU,
    FRAME_MAP,
    FRAME_ODOM,
    FRAME_VO_ODOM,
    FRAME_WHEEL_ODOM,
)


@dataclass(frozen=True)
class FramePair:
    parent: str
    child: str

    @property
    def parent_frame(self) -> str:
        return self.parent

    @property
    def child_frame(self) -> str:
        return self.child

    @property
    def label(self) -> str:
        return make_tf_edge_label(self.parent, self.child)

    def to_dict(self) -> dict[str, str]:
        return {
            "parent": self.parent,
            "child": self.child,
            "parent_frame": self.parent,
            "child_frame": self.child,
            "label": self.label,
        }


@dataclass(frozen=True)
class LocalizationFrames:
    map_frame: str = FRAME_MAP
    odom_frame: str = FRAME_ODOM
    base_frame: str = FRAME_BASE_LINK
    imu_frame: str = FRAME_IMU
    wheel_odom_frame: str = FRAME_WHEEL_ODOM
    filtered_odom_frame: str = FRAME_FILTERED_ODOM
    vo_odom_frame: str = FRAME_VO_ODOM

    @property
    def map_frame_id(self) -> str:
        return self.map_frame

    @property
    def odom_frame_id(self) -> str:
        return self.odom_frame

    @property
    def base_frame_id(self) -> str:
        return self.base_frame

    @property
    def imu_frame_id(self) -> str:
        return self.imu_frame

    def to_dict(self) -> dict[str, str]:
        return {
            "map_frame": self.map_frame,
            "odom_frame": self.odom_frame,
            "base_frame": self.base_frame,
            "imu_frame": self.imu_frame,
            "wheel_odom_frame": self.wheel_odom_frame,
            "filtered_odom_frame": self.filtered_odom_frame,
            "vo_odom_frame": self.vo_odom_frame,
        }


def normalize_frame_id(frame_id: str | None) -> str:
    if frame_id is None:
        return ""

    return str(frame_id).strip().lstrip("/")


def frame_id_valid(frame_id: str | None) -> bool:
    return bool(normalize_frame_id(frame_id))


def require_frame_id(frame_id: str | None) -> str:
    normalized = normalize_frame_id(frame_id)

    if not normalized:
        raise ValueError("frame_id cannot be empty")

    return normalized


def frame_pair_valid(parent_frame: str | None, child_frame: str | None) -> bool:
    parent = normalize_frame_id(parent_frame)
    child = normalize_frame_id(child_frame)

    return bool(parent and child and parent != child)


def require_frame_pair(
    parent_frame: str | None,
    child_frame: str | None,
) -> tuple[str, str]:
    parent = require_frame_id(parent_frame)
    child = require_frame_id(child_frame)

    if parent == child:
        raise ValueError("parent_frame and child_frame cannot be the same")

    return parent, child


def make_tf_edge_label(parent_frame: str | None, child_frame: str | None) -> str:
    parent, child = require_frame_pair(parent_frame, child_frame)
    return f"{parent} -> {child}"


def tf_edge_label(parent_frame: str | None, child_frame: str | None) -> str:
    return make_tf_edge_label(parent_frame, child_frame)


def frame_pair(parent_frame: str | None, child_frame: str | None) -> FramePair:
    parent, child = require_frame_pair(parent_frame, child_frame)
    return FramePair(parent=parent, child=child)


def make_frame_pair(parent_frame: str | None, child_frame: str | None) -> FramePair:
    return frame_pair(parent_frame, child_frame)


def map_to_odom_pair() -> FramePair:
    return frame_pair(FRAME_MAP, FRAME_ODOM)


def odom_to_base_pair() -> FramePair:
    return frame_pair(FRAME_ODOM, FRAME_BASE_LINK)


def base_to_imu_pair() -> FramePair:
    return frame_pair(FRAME_BASE_LINK, FRAME_IMU)


def wheel_odom_to_base_pair() -> FramePair:
    return frame_pair(FRAME_WHEEL_ODOM, FRAME_BASE_LINK)


def filtered_odom_to_base_pair() -> FramePair:
    return frame_pair(FRAME_FILTERED_ODOM, FRAME_BASE_LINK)


def vo_odom_to_base_pair() -> FramePair:
    return frame_pair(FRAME_VO_ODOM, FRAME_BASE_LINK)


def is_expected_odom_frame(frame_id: str | None) -> bool:
    return normalize_frame_id(frame_id) == FRAME_ODOM


def is_expected_base_frame(frame_id: str | None) -> bool:
    return normalize_frame_id(frame_id) == FRAME_BASE_LINK


def is_expected_imu_frame(frame_id: str | None) -> bool:
    return normalize_frame_id(frame_id) == FRAME_IMU


def require_expected_frame(
    frame_id: str | None,
    expected_frame_id: str,
    *,
    name: str = "frame_id",
) -> str:
    normalized = require_frame_id(frame_id)
    expected = require_frame_id(expected_frame_id)

    if normalized != expected:
        raise ValueError(f"{name} must be {expected}, got {normalized}")

    return normalized


def make_localization_frames(
    *,
    map_frame: str = FRAME_MAP,
    odom_frame: str = FRAME_ODOM,
    base_frame: str = FRAME_BASE_LINK,
    imu_frame: str = FRAME_IMU,
    wheel_odom_frame: str = FRAME_WHEEL_ODOM,
    filtered_odom_frame: str = FRAME_FILTERED_ODOM,
    vo_odom_frame: str = FRAME_VO_ODOM,
) -> LocalizationFrames:
    frames = LocalizationFrames(
        map_frame=normalize_frame_id(map_frame),
        odom_frame=normalize_frame_id(odom_frame),
        base_frame=normalize_frame_id(base_frame),
        imu_frame=normalize_frame_id(imu_frame),
        wheel_odom_frame=normalize_frame_id(wheel_odom_frame),
        filtered_odom_frame=normalize_frame_id(filtered_odom_frame),
        vo_odom_frame=normalize_frame_id(vo_odom_frame),
    )

    validate_localization_frames(frames)
    return frames


def default_localization_frames() -> LocalizationFrames:
    return make_localization_frames()


def validate_localization_frames(frames: LocalizationFrames) -> bool:
    require_frame_id(frames.map_frame)
    require_frame_id(frames.odom_frame)
    require_frame_id(frames.base_frame)
    require_frame_id(frames.imu_frame)

    require_frame_pair(frames.map_frame, frames.odom_frame)
    require_frame_pair(frames.odom_frame, frames.base_frame)
    require_frame_pair(frames.base_frame, frames.imu_frame)

    return True


def localization_tf_edges(
    *,
    include_map_to_odom: bool = False,
    include_base_to_imu: bool = True,
) -> tuple[FramePair, ...]:
    edges: list[FramePair] = []

    if include_map_to_odom:
        edges.append(map_to_odom_pair())

    edges.append(odom_to_base_pair())

    if include_base_to_imu:
        edges.append(base_to_imu_pair())

    return tuple(edges)


def frame_chain_labels(
    *,
    include_map_to_odom: bool = False,
    include_base_to_imu: bool = True,
) -> tuple[str, ...]:
    return tuple(
        edge.label
        for edge in localization_tf_edges(
            include_map_to_odom=include_map_to_odom,
            include_base_to_imu=include_base_to_imu,
        )
    )


__all__ = [
    "FramePair",
    "LocalizationFrames",
    "normalize_frame_id",
    "frame_id_valid",
    "require_frame_id",
    "frame_pair_valid",
    "require_frame_pair",
    "make_tf_edge_label",
    "tf_edge_label",
    "frame_pair",
    "make_frame_pair",
    "map_to_odom_pair",
    "odom_to_base_pair",
    "base_to_imu_pair",
    "wheel_odom_to_base_pair",
    "filtered_odom_to_base_pair",
    "vo_odom_to_base_pair",
    "is_expected_odom_frame",
    "is_expected_base_frame",
    "is_expected_imu_frame",
    "require_expected_frame",
    "make_localization_frames",
    "default_localization_frames",
    "validate_localization_frames",
    "localization_tf_edges",
    "frame_chain_labels",
]
