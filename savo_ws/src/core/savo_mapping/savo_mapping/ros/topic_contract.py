#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Robot Savo mapping topic contract. No ROS runtime required."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Final, Mapping

from savo_mapping.utils.topic_names import (
    DEFAULT_TOPIC_GROUPS,
    MappingTopicGroups,
    normalize_topic,
)


# =============================================================================
# Message type names
# =============================================================================
MSG_LASER_SCAN: Final[str] = "sensor_msgs/msg/LaserScan"
MSG_ODOMETRY: Final[str] = "nav_msgs/msg/Odometry"
MSG_OCCUPANCY_GRID: Final[str] = "nav_msgs/msg/OccupancyGrid"
MSG_MAP_METADATA: Final[str] = "nav_msgs/msg/MapMetaData"
MSG_POINTCLOUD2: Final[str] = "sensor_msgs/msg/PointCloud2"
MSG_BOOL: Final[str] = "std_msgs/msg/Bool"
MSG_STRING: Final[str] = "std_msgs/msg/String"
MSG_POSE_STAMPED: Final[str] = "geometry_msgs/msg/PoseStamped"
MSG_TF: Final[str] = "tf2_msgs/msg/TFMessage"


# =============================================================================
# Contract models
# =============================================================================
@dataclass(frozen=True)
class TopicSpec:
    key: str
    topic: str
    msg_type: str
    required: bool = True
    description: str = ""

    def normalized(self) -> "TopicSpec":
        return TopicSpec(
            key=self.key,
            topic=normalize_topic(self.topic),
            msg_type=self.msg_type,
            required=self.required,
            description=self.description,
        )

    def to_dict(self) -> dict:
        return {
            "key": self.key,
            "topic": self.topic,
            "msg_type": self.msg_type,
            "required": self.required,
            "description": self.description,
        }


@dataclass(frozen=True)
class MappingTopicContract:
    subscriptions: Mapping[str, TopicSpec] = field(default_factory=dict)
    publications: Mapping[str, TopicSpec] = field(default_factory=dict)
    optional_subscriptions: Mapping[str, TopicSpec] = field(default_factory=dict)
    action_clients: Mapping[str, str] = field(default_factory=dict)

    def all_topics(self) -> Dict[str, TopicSpec]:
        topics: Dict[str, TopicSpec] = {}
        topics.update(self.subscriptions)
        topics.update(self.optional_subscriptions)
        topics.update(self.publications)
        return topics

    def required_topics(self) -> Dict[str, TopicSpec]:
        return {
            key: spec
            for key, spec in self.all_topics().items()
            if spec.required
        }

    def optional_topics(self) -> Dict[str, TopicSpec]:
        return {
            key: spec
            for key, spec in self.all_topics().items()
            if not spec.required
        }

    def normalized(self) -> "MappingTopicContract":
        return MappingTopicContract(
            subscriptions={
                key: spec.normalized()
                for key, spec in self.subscriptions.items()
            },
            publications={
                key: spec.normalized()
                for key, spec in self.publications.items()
            },
            optional_subscriptions={
                key: spec.normalized()
                for key, spec in self.optional_subscriptions.items()
            },
            action_clients=dict(self.action_clients),
        )

    def to_dict(self) -> dict:
        return {
            "subscriptions": {
                key: spec.to_dict()
                for key, spec in self.subscriptions.items()
            },
            "optional_subscriptions": {
                key: spec.to_dict()
                for key, spec in self.optional_subscriptions.items()
            },
            "publications": {
                key: spec.to_dict()
                for key, spec in self.publications.items()
            },
            "action_clients": dict(self.action_clients),
        }


# =============================================================================
# Default contract
# =============================================================================
def build_default_topic_contract(
    groups: MappingTopicGroups = DEFAULT_TOPIC_GROUPS,
) -> MappingTopicContract:
    return MappingTopicContract(
        subscriptions={
            "scan": TopicSpec(
                key="scan",
                topic=groups.core["scan"],
                msg_type=MSG_LASER_SCAN,
                required=True,
                description="Filtered or raw LiDAR scan used by slam_toolbox.",
            ),
            "odom": TopicSpec(
                key="odom",
                topic=groups.core["odom"],
                msg_type=MSG_ODOMETRY,
                required=True,
                description="Filtered odometry from Robot Savo localization.",
            ),
            "map": TopicSpec(
                key="map",
                topic=groups.core["map"],
                msg_type=MSG_OCCUPANCY_GRID,
                required=False,
                description="Occupancy grid produced by SLAM.",
            ),
            "map_metadata": TopicSpec(
                key="map_metadata",
                topic=groups.core["map_metadata"],
                msg_type=MSG_MAP_METADATA,
                required=False,
                description="Metadata for the active occupancy grid.",
            ),
            "tf": TopicSpec(
                key="tf",
                topic=groups.transforms["tf"],
                msg_type=MSG_TF,
                required=True,
                description="Live transform tree.",
            ),
            "tf_static": TopicSpec(
                key="tf_static",
                topic=groups.transforms["tf_static"],
                msg_type=MSG_TF,
                required=True,
                description="Static transform tree.",
            ),
        },
        optional_subscriptions={
            "realsense_points": TopicSpec(
                key="realsense_points",
                topic=groups.edge_realsense["points"],
                msg_type=MSG_POINTCLOUD2,
                required=False,
                description="PointCloud2 from savo-edge for 3D obstacle assist.",
            ),
            "realsense_status": TopicSpec(
                key="realsense_status",
                topic=groups.edge_realsense["status"],
                msg_type=MSG_STRING,
                required=False,
                description="RealSense status from edge camera package.",
            ),
            "depth_front": TopicSpec(
                key="depth_front",
                topic=groups.edge_realsense["depth_front"],
                msg_type=MSG_STRING,
                required=False,
                description="Front depth summary from RealSense pipeline.",
            ),
            "apriltag_detections": TopicSpec(
                key="apriltag_detections",
                topic=groups.semantic["apriltag_detections"],
                msg_type=MSG_STRING,
                required=False,
                description="Future AprilTag observations for semantic mapping.",
            ),
        },
        publications={
            "ready": TopicSpec(
                key="ready",
                topic=groups.outputs["ready"],
                msg_type=MSG_BOOL,
                required=True,
                description="Mapping readiness flag.",
            ),
            "status": TopicSpec(
                key="status",
                topic=groups.outputs["status"],
                msg_type=MSG_STRING,
                required=True,
                description="Mapping status JSON.",
            ),
            "mode": TopicSpec(
                key="mode",
                topic=groups.outputs["mode"],
                msg_type=MSG_STRING,
                required=True,
                description="Current mapping mode.",
            ),
            "session_state": TopicSpec(
                key="session_state",
                topic=groups.outputs["session_state"],
                msg_type=MSG_STRING,
                required=False,
                description="Current mapping session state JSON.",
            ),
            "map_quality": TopicSpec(
                key="map_quality",
                topic=groups.outputs["map_quality"],
                msg_type=MSG_STRING,
                required=False,
                description="Map quality summary JSON.",
            ),
            "exploration_status": TopicSpec(
                key="exploration_status",
                topic=groups.outputs["exploration_status"],
                msg_type=MSG_STRING,
                required=False,
                description="Autonomous exploration status JSON.",
            ),
            "next_goal": TopicSpec(
                key="next_goal",
                topic=groups.outputs["next_goal"],
                msg_type=MSG_POSE_STAMPED,
                required=False,
                description="Selected autonomous exploration goal.",
            ),
            "pointcloud_status": TopicSpec(
                key="pointcloud_status",
                topic=groups.outputs["pointcloud_status"],
                msg_type=MSG_STRING,
                required=False,
                description="RealSense pointcloud health/status JSON.",
            ),
            "apriltag_status": TopicSpec(
                key="apriltag_status",
                topic=groups.semantic["apriltag_status"],
                msg_type=MSG_STRING,
                required=False,
                description="Future AprilTag mapping status JSON.",
            ),
            "semantic_landmarks": TopicSpec(
                key="semantic_landmarks",
                topic=groups.semantic["semantic_landmarks"],
                msg_type=MSG_STRING,
                required=False,
                description="Future semantic landmark observations JSON.",
            ),
            "location_bridge_status": TopicSpec(
                key="location_bridge_status",
                topic=groups.semantic["location_bridge_status"],
                msg_type=MSG_STRING,
                required=False,
                description="Future bridge status to savo_location.",
            ),
        },
        action_clients={
            "navigate_to_pose": "nav2_msgs/action/NavigateToPose",
        },
    ).normalized()


DEFAULT_TOPIC_CONTRACT: Final[MappingTopicContract] = build_default_topic_contract()


def get_mapping_topic_contract() -> MappingTopicContract:
    """Return the default Robot Savo mapping topic contract."""
    return DEFAULT_TOPIC_CONTRACT


def get_required_topic_names() -> Dict[str, str]:
    """Return required topic names keyed by contract key."""
    return {
        key: spec.topic
        for key, spec in DEFAULT_TOPIC_CONTRACT.required_topics().items()
    }


def get_optional_topic_names() -> Dict[str, str]:
    """Return optional topic names keyed by contract key."""
    return {
        key: spec.topic
        for key, spec in DEFAULT_TOPIC_CONTRACT.optional_topics().items()
    }


def get_publication_topic_names() -> Dict[str, str]:
    """Return publication topic names keyed by contract key."""
    return {
        key: spec.topic
        for key, spec in DEFAULT_TOPIC_CONTRACT.publications.items()
    }


def get_subscription_topic_names() -> Dict[str, str]:
    """Return subscription topic names keyed by contract key."""
    return {
        key: spec.topic
        for key, spec in DEFAULT_TOPIC_CONTRACT.subscriptions.items()
    }


def validate_contract_topics() -> None:
    """Validate the default topic contract."""
    all_group_topics = DEFAULT_TOPIC_GROUPS.all_topic_values()

    for spec in DEFAULT_TOPIC_CONTRACT.all_topics().values():
        normalize_topic(spec.topic)

    for spec in DEFAULT_TOPIC_CONTRACT.all_topics().values():
        if spec.topic not in all_group_topics:
            raise ValueError(f"Topic not present in topic groups: {spec.topic}")


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    """Print the mapping topic contract."""
    contract = get_mapping_topic_contract()

    print("Robot Savo mapping topic contract")

    print("\n[subscriptions]")
    for key, spec in contract.subscriptions.items():
        print(f"{key}: {spec.topic} ({spec.msg_type}) required={spec.required}")

    print("\n[optional_subscriptions]")
    for key, spec in contract.optional_subscriptions.items():
        print(f"{key}: {spec.topic} ({spec.msg_type}) required={spec.required}")

    print("\n[publications]")
    for key, spec in contract.publications.items():
        print(f"{key}: {spec.topic} ({spec.msg_type}) required={spec.required}")

    print("\n[action_clients]")
    for key, action_type in contract.action_clients.items():
        print(f"{key}: {action_type}")


if __name__ == "__main__":
    main()


__all__ = [
    "MSG_LASER_SCAN",
    "MSG_ODOMETRY",
    "MSG_OCCUPANCY_GRID",
    "MSG_MAP_METADATA",
    "MSG_POINTCLOUD2",
    "MSG_BOOL",
    "MSG_STRING",
    "MSG_POSE_STAMPED",
    "MSG_TF",
    "TopicSpec",
    "MappingTopicContract",
    "build_default_topic_contract",
    "DEFAULT_TOPIC_CONTRACT",
    "get_mapping_topic_contract",
    "get_required_topic_names",
    "get_optional_topic_names",
    "get_publication_topic_names",
    "get_subscription_topic_names",
    "validate_contract_topics",
    "main",
]
