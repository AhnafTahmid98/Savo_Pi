#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Topic-name helpers for Robot Savo mapping. No ROS imports."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Final, Iterable, Mapping


# =============================================================================
# Optional bridge to package constants
# =============================================================================
try:
    from savo_mapping.constants import (
        TOPIC_APRILTAG_DETECTIONS,
        TOPIC_APRILTAG_STATUS,
        TOPIC_DEPTH_FRONT,
        TOPIC_EXPLORATION_STATUS,
        TOPIC_LOCATION_BRIDGE_STATUS,
        TOPIC_MAP,
        TOPIC_MAP_METADATA,
        TOPIC_MAP_QUALITY,
        TOPIC_MAPPING_MODE,
        TOPIC_MAPPING_READY,
        TOPIC_MAPPING_STATUS,
        TOPIC_NEXT_GOAL,
        TOPIC_ODOM,
        TOPIC_POINTCLOUD_STATUS,
        TOPIC_REALSENSE_POINTS,
        TOPIC_REALSENSE_STATUS,
        TOPIC_SCAN,
        TOPIC_SEMANTIC_LANDMARKS,
        TOPIC_SESSION_STATE,
        TOPIC_TF,
        TOPIC_TF_STATIC,
    )

    _HAS_PACKAGE_CONSTANTS: Final[bool] = True
except Exception:
    _HAS_PACKAGE_CONSTANTS = False

    TOPIC_SCAN = "/scan"
    TOPIC_ODOM = "/odometry/filtered"
    TOPIC_MAP = "/map"
    TOPIC_MAP_METADATA = "/map_metadata"
    TOPIC_TF = "/tf"
    TOPIC_TF_STATIC = "/tf_static"

    TOPIC_REALSENSE_POINTS = "/savo_edge/realsense/points"
    TOPIC_REALSENSE_STATUS = "/realsense/status"
    TOPIC_DEPTH_FRONT = "/depth/min_front_m"

    TOPIC_MAPPING_READY = "/savo_mapping/ready"
    TOPIC_MAPPING_STATUS = "/savo_mapping/status"
    TOPIC_MAPPING_MODE = "/savo_mapping/mode"
    TOPIC_SESSION_STATE = "/savo_mapping/session_state"
    TOPIC_MAP_QUALITY = "/savo_mapping/map_quality"
    TOPIC_EXPLORATION_STATUS = "/savo_mapping/exploration_status"
    TOPIC_NEXT_GOAL = "/savo_mapping/next_goal"
    TOPIC_POINTCLOUD_STATUS = "/savo_mapping/pointcloud_status"

    TOPIC_APRILTAG_DETECTIONS = "/apriltag/detections"
    TOPIC_APRILTAG_STATUS = "/savo_mapping/apriltag_status"
    TOPIC_SEMANTIC_LANDMARKS = "/savo_mapping/semantic_landmarks"
    TOPIC_LOCATION_BRIDGE_STATUS = "/savo_mapping/location_bridge_status"


# =============================================================================
# Low-level helpers
# =============================================================================
def is_absolute_topic(topic: str) -> bool:
    """Return True when topic starts from the ROS root namespace."""
    return str(topic).strip().startswith("/")


def is_private_topic(topic: str) -> bool:
    """Return True when topic is a ROS private topic."""
    value = str(topic).strip()
    return value.startswith("~") or value.startswith("~/")


def normalize_topic(topic: str) -> str:
    """Normalize a ROS topic name."""
    value = str(topic).strip()

    if not value:
        raise ValueError("Topic name cannot be empty.")

    if is_private_topic(value):
        while "//" in value:
            value = value.replace("//", "/")
        return value.rstrip("/") if len(value) > 1 else value

    if not value.startswith("/"):
        value = f"/{value}"

    while "//" in value:
        value = value.replace("//", "/")

    if len(value) > 1:
        value = value.rstrip("/")

    return value


def join_topic(*parts: str) -> str:
    """Join topic fragments safely."""
    clean_parts: list[str] = []

    for part in parts:
        value = str(part).strip().strip("/")
        if value:
            clean_parts.append(value)

    if not clean_parts:
        raise ValueError("Topic parts cannot be empty.")

    return normalize_topic("/".join(clean_parts))


def topic_basename(topic: str) -> str:
    """Return the final name part of a topic."""
    value = normalize_topic(topic)

    if value in ("/", "~"):
        return ""

    return value.rsplit("/", maxsplit=1)[-1]


def topic_namespace(topic: str) -> str:
    """Return the namespace part of a topic."""
    value = normalize_topic(topic)

    if value in ("/", "~"):
        return value

    parts = value.rsplit("/", maxsplit=1)

    if len(parts) == 1 or not parts[0]:
        return "/"

    return parts[0]


def ensure_topic_namespace(topic: str, namespace: str) -> str:
    """Place topic under namespace when it is not already there."""
    value = normalize_topic(topic)
    ns = normalize_topic(namespace)

    if value == ns or value.startswith(f"{ns}/"):
        return value

    return join_topic(ns, topic_basename(value))


def normalize_topic_map(topics: Mapping[str, str]) -> Dict[str, str]:
    """Normalize a name-to-topic mapping."""
    return {str(key): normalize_topic(value) for key, value in topics.items()}


def validate_topics(topics: Iterable[str]) -> None:
    """Raise ValueError if any topic in the list is invalid."""
    for topic in topics:
        normalize_topic(topic)


# =============================================================================
# Canonical topic groups
# =============================================================================
@dataclass(frozen=True)
class MappingTopicGroups:
    core: Mapping[str, str]
    transforms: Mapping[str, str]
    edge_realsense: Mapping[str, str]
    outputs: Mapping[str, str]
    semantic: Mapping[str, str]
    nav2: Mapping[str, str]

    def grouped(self) -> Dict[str, Mapping[str, str]]:
        return {
            "core": self.core,
            "transforms": self.transforms,
            "edge_realsense": self.edge_realsense,
            "outputs": self.outputs,
            "semantic": self.semantic,
            "nav2": self.nav2,
        }

    def all_topics(self) -> Dict[str, str]:
        topics: Dict[str, str] = {}
        key_counts: Dict[str, int] = {}

        for group in self.grouped().values():
            for key in group:
                key_counts[key] = key_counts.get(key, 0) + 1

        for group_name, group in self.grouped().items():
            for key, topic in group.items():
                topics[f"{group_name}.{key}"] = topic

                if key_counts[key] == 1:
                    topics[key] = topic

        return topics

    def all_topic_values(self) -> set[str]:
        values: set[str] = set()

        for group in self.grouped().values():
            values.update(group.values())

        return values

    def normalized(self) -> "MappingTopicGroups":
        return MappingTopicGroups(
            core=normalize_topic_map(self.core),
            transforms=normalize_topic_map(self.transforms),
            edge_realsense=normalize_topic_map(self.edge_realsense),
            outputs=normalize_topic_map(self.outputs),
            semantic=normalize_topic_map(self.semantic),
            nav2=normalize_topic_map(self.nav2),
        )


CORE_TOPICS: Final[Mapping[str, str]] = {
    "scan": TOPIC_SCAN,
    "odom": TOPIC_ODOM,
    "map": TOPIC_MAP,
    "map_metadata": TOPIC_MAP_METADATA,
}

TRANSFORM_TOPICS: Final[Mapping[str, str]] = {
    "tf": TOPIC_TF,
    "tf_static": TOPIC_TF_STATIC,
}

EDGE_REALSENSE_TOPICS: Final[Mapping[str, str]] = {
    "points": TOPIC_REALSENSE_POINTS,
    "status": TOPIC_REALSENSE_STATUS,
    "depth_front": TOPIC_DEPTH_FRONT,
}

MAPPING_OUTPUT_TOPICS: Final[Mapping[str, str]] = {
    "ready": TOPIC_MAPPING_READY,
    "status": TOPIC_MAPPING_STATUS,
    "mode": TOPIC_MAPPING_MODE,
    "session_state": TOPIC_SESSION_STATE,
    "map_quality": TOPIC_MAP_QUALITY,
    "exploration_status": TOPIC_EXPLORATION_STATUS,
    "next_goal": TOPIC_NEXT_GOAL,
    "pointcloud_status": TOPIC_POINTCLOUD_STATUS,
}

SEMANTIC_TOPICS: Final[Mapping[str, str]] = {
    "apriltag_detections": TOPIC_APRILTAG_DETECTIONS,
    "apriltag_status": TOPIC_APRILTAG_STATUS,
    "semantic_landmarks": TOPIC_SEMANTIC_LANDMARKS,
    "location_bridge_status": TOPIC_LOCATION_BRIDGE_STATUS,
}

NAV2_TOPICS: Final[Mapping[str, str]] = {
    "navigate_to_pose": "/navigate_to_pose",
    "global_costmap": "/global_costmap/costmap",
    "local_costmap": "/local_costmap/costmap",
    "global_costmap_updates": "/global_costmap/costmap_updates",
    "local_costmap_updates": "/local_costmap/costmap_updates",
    "plan": "/plan",
}


DEFAULT_TOPIC_GROUPS: Final[MappingTopicGroups] = MappingTopicGroups(
    core=CORE_TOPICS,
    transforms=TRANSFORM_TOPICS,
    edge_realsense=EDGE_REALSENSE_TOPICS,
    outputs=MAPPING_OUTPUT_TOPICS,
    semantic=SEMANTIC_TOPICS,
    nav2=NAV2_TOPICS,
).normalized()


def get_mapping_topic_groups() -> MappingTopicGroups:
    """Return canonical Robot Savo mapping topic groups."""
    return DEFAULT_TOPIC_GROUPS


def get_all_mapping_topics() -> Dict[str, str]:
    """Return all canonical mapping topics as a flat dictionary."""
    return DEFAULT_TOPIC_GROUPS.all_topics()


def get_topic(name: str) -> str:
    """Return one canonical topic by logical name."""
    key = str(name).strip()

    if not key:
        raise ValueError("Topic key cannot be empty.")

    topics = get_all_mapping_topics()

    if key not in topics:
        raise KeyError(f"Unknown mapping topic key: {key}")

    return topics[key]


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    """Print canonical topic groups for quick checks."""
    print("Robot Savo mapping topic contract:")

    groups = get_mapping_topic_groups()

    for group_name in (
        "core",
        "transforms",
        "edge_realsense",
        "outputs",
        "semantic",
        "nav2",
    ):
        group = getattr(groups, group_name)
        print(f"\n[{group_name}]")
        for key, topic in group.items():
            print(f"{key}: {topic}")


if __name__ == "__main__":
    main()


__all__ = [
    "is_absolute_topic",
    "is_private_topic",
    "normalize_topic",
    "join_topic",
    "topic_basename",
    "topic_namespace",
    "ensure_topic_namespace",
    "normalize_topic_map",
    "validate_topics",
    "MappingTopicGroups",
    "CORE_TOPICS",
    "TRANSFORM_TOPICS",
    "EDGE_REALSENSE_TOPICS",
    "MAPPING_OUTPUT_TOPICS",
    "SEMANTIC_TOPICS",
    "NAV2_TOPICS",
    "DEFAULT_TOPIC_GROUPS",
    "get_mapping_topic_groups",
    "get_all_mapping_topics",
    "get_topic",
    "main",
]
