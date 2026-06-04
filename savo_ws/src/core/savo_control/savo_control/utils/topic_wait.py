#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Wait helpers for confirming required ROS topics are visible before starting a test.

    from savo_control.utils.topic_wait import wait_for_topics

    ok = wait_for_topics(node, ["/cmd_vel_safe", "/safety/stop"], timeout_s=5.0)
    if not ok:
        node.get_logger().error("Required topics missing; refusing to start test.")
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Iterable, Optional, Sequence


@dataclass
class TopicWaitResult:
    """Result of waiting for a set of topics."""

    ok: bool
    required_topics: list[str] = field(default_factory=list)
    found_topics: list[str] = field(default_factory=list)
    missing_topics: list[str] = field(default_factory=list)
    elapsed_s: float = 0.0

    def summary(self) -> str:
        """Return compact human-readable summary."""
        found = ",".join(self.found_topics) if self.found_topics else "none"
        missing = ",".join(self.missing_topics) if self.missing_topics else "none"

        return (
            f"ok={self.ok}; "
            f"elapsed_s={self.elapsed_s:.2f}; "
            f"found=[{found}]; "
            f"missing=[{missing}]"
        )


@dataclass
class TopicTypeWaitResult:
    """Result of waiting for topics with expected message types."""

    ok: bool
    required: dict[str, str] = field(default_factory=dict)
    found: dict[str, list[str]] = field(default_factory=dict)
    missing: dict[str, str] = field(default_factory=dict)
    mismatched: dict[str, list[str]] = field(default_factory=dict)
    elapsed_s: float = 0.0

    def summary(self) -> str:
        """Return compact human-readable summary."""
        missing_text = (
            ",".join(f"{topic}:{msg_type}" for topic, msg_type in self.missing.items())
            if self.missing
            else "none"
        )

        mismatch_text = (
            ",".join(
                f"{topic}:expected={self.required.get(topic, '?')},actual={types}"
                for topic, types in self.mismatched.items()
            )
            if self.mismatched
            else "none"
        )

        return (
            f"ok={self.ok}; "
            f"elapsed_s={self.elapsed_s:.2f}; "
            f"missing=[{missing_text}]; "
            f"mismatched=[{mismatch_text}]"
        )


def normalize_topic_name(topic: str) -> str:
    """
    Normalize a ROS topic name for simple graph comparison.

    This does not perform full ROS name resolution. It only ensures a leading slash
    and removes repeated trailing slashes for common project use.
    """
    topic = str(topic).strip()

    if not topic:
        return ""

    if not topic.startswith("/"):
        topic = "/" + topic

    while len(topic) > 1 and topic.endswith("/"):
        topic = topic[:-1]

    return topic


def normalize_topic_list(topics: Iterable[str]) -> list[str]:
    """Normalize and de-duplicate topic list while preserving order."""
    result: list[str] = []
    seen: set[str] = set()

    for topic in topics:
        normalized = normalize_topic_name(topic)
        if not normalized:
            continue

        if normalized in seen:
            continue

        seen.add(normalized)
        result.append(normalized)

    return result


def get_current_topic_names(node) -> set[str]:
    """
    Return current topic names visible to a ROS 2 node.

    node:
        rclpy Node-like object.
    """
    names_and_types = node.get_topic_names_and_types()
    return {normalize_topic_name(name) for name, _types in names_and_types}


def get_current_topic_types(node) -> dict[str, list[str]]:
    """
    Return mapping of topic name -> list of visible type names.

    Example type:
        "geometry_msgs/msg/Twist"
    """
    names_and_types = node.get_topic_names_and_types()
    result: dict[str, list[str]] = {}

    for name, types in names_and_types:
        normalized = normalize_topic_name(name)
        result[normalized] = list(types)

    return result


def topic_exists(node, topic: str) -> bool:
    """Return True if topic exists in current ROS graph."""
    normalized = normalize_topic_name(topic)
    if not normalized:
        return False

    return normalized in get_current_topic_names(node)


def topic_has_type(node, topic: str, expected_type: str) -> bool:
    """
    Return True if topic exists with expected ROS message type.

    expected_type example:
        "geometry_msgs/msg/Twist"
        "std_msgs/msg/Bool"
        "nav_msgs/msg/Odometry"
    """
    normalized = normalize_topic_name(topic)
    expected = str(expected_type).strip()

    if not normalized or not expected:
        return False

    topic_types = get_current_topic_types(node)
    actual_types = topic_types.get(normalized, [])

    return expected in actual_types


def wait_for_topics(
    node,
    required_topics: Sequence[str],
    timeout_s: float = 5.0,
    poll_period_s: float = 0.10,
    log_progress: bool = True,
) -> TopicWaitResult:
    """
    Wait until all required topics are visible in the ROS graph.

    Returns:
        TopicWaitResult

    This function does not spin the node. It only checks the graph repeatedly.
    For graph discovery this is normally enough, but if your program depends on
    callbacks, spin separately.
    """
    required = normalize_topic_list(required_topics)
    timeout_s = max(0.0, float(timeout_s))
    poll_period_s = max(0.01, float(poll_period_s))

    start = time.monotonic()
    last_log = 0.0

    found: list[str] = []
    missing: list[str] = list(required)

    while True:
        visible = get_current_topic_names(node)

        found = [topic for topic in required if topic in visible]
        missing = [topic for topic in required if topic not in visible]

        if not missing:
            elapsed = time.monotonic() - start
            if log_progress:
                node.get_logger().info(
                    f"All required topics available: {','.join(required)}"
                )
            return TopicWaitResult(
                ok=True,
                required_topics=required,
                found_topics=found,
                missing_topics=[],
                elapsed_s=elapsed,
            )

        elapsed = time.monotonic() - start
        if elapsed >= timeout_s:
            if log_progress:
                node.get_logger().warning(
                    "Timeout waiting for topics | "
                    f"missing={missing} | found={found} | elapsed={elapsed:.2f}s"
                )
            return TopicWaitResult(
                ok=False,
                required_topics=required,
                found_topics=found,
                missing_topics=missing,
                elapsed_s=elapsed,
            )

        now = time.monotonic()
        if log_progress and (now - last_log) >= 1.0:
            last_log = now
            node.get_logger().info(
                "Waiting for topics | "
                f"missing={missing} | found={found}"
            )

        time.sleep(poll_period_s)


def wait_for_topic_types(
    node,
    required_topic_types: dict[str, str],
    timeout_s: float = 5.0,
    poll_period_s: float = 0.10,
    log_progress: bool = True,
) -> TopicTypeWaitResult:
    """
    Wait until required topics exist with expected message types.

    Args:
        required_topic_types:
            Dict mapping topic -> expected message type.

            Example:
                {
                    "/cmd_vel": "geometry_msgs/msg/Twist",
                    "/safety/stop": "std_msgs/msg/Bool",
                }

    Returns:
        TopicTypeWaitResult
    """
    required = {
        normalize_topic_name(topic): str(msg_type).strip()
        for topic, msg_type in required_topic_types.items()
        if normalize_topic_name(topic) and str(msg_type).strip()
    }

    timeout_s = max(0.0, float(timeout_s))
    poll_period_s = max(0.01, float(poll_period_s))

    start = time.monotonic()
    last_log = 0.0

    found: dict[str, list[str]] = {}
    missing: dict[str, str] = {}
    mismatched: dict[str, list[str]] = {}

    while True:
        visible_types = get_current_topic_types(node)

        found.clear()
        missing.clear()
        mismatched.clear()

        for topic, expected_type in required.items():
            actual_types = visible_types.get(topic)

            if actual_types is None:
                missing[topic] = expected_type
                continue

            found[topic] = actual_types

            if expected_type not in actual_types:
                mismatched[topic] = actual_types

        ok = not missing and not mismatched
        elapsed = time.monotonic() - start

        if ok:
            if log_progress:
                node.get_logger().info(
                    f"All required topic types available: {required}"
                )
            return TopicTypeWaitResult(
                ok=True,
                required=required,
                found=dict(found),
                missing={},
                mismatched={},
                elapsed_s=elapsed,
            )

        if elapsed >= timeout_s:
            if log_progress:
                node.get_logger().warning(
                    "Timeout waiting for topic types | "
                    f"missing={missing} | mismatched={mismatched} | "
                    f"elapsed={elapsed:.2f}s"
                )
            return TopicTypeWaitResult(
                ok=False,
                required=required,
                found=dict(found),
                missing=dict(missing),
                mismatched=dict(mismatched),
                elapsed_s=elapsed,
            )

        now = time.monotonic()
        if log_progress and (now - last_log) >= 1.0:
            last_log = now
            node.get_logger().info(
                "Waiting for topic types | "
                f"missing={missing} | mismatched={mismatched}"
            )

        time.sleep(poll_period_s)


def wait_for_control_chain_topics(
    node,
    timeout_s: float = 5.0,
    include_safe: bool = True,
    include_safety_stop: bool = True,
    include_odom: bool = False,
    include_depth: bool = False,
) -> TopicWaitResult:
    """
    Wait for common Robot Savo control-chain topics.

    Base chain:
        /cmd_vel_mux
        /cmd_vel

    Optional:
        /cmd_vel_safe
        /safety/stop
        /odometry/filtered
        /depth/min_front_m
    """
    topics = [
        "/cmd_vel_mux",
        "/cmd_vel",
    ]

    if include_safe:
        topics.append("/cmd_vel_safe")

    if include_safety_stop:
        topics.append("/safety/stop")

    if include_odom:
        topics.append("/odometry/filtered")

    if include_depth:
        topics.append("/depth/min_front_m")

    return wait_for_topics(
        node,
        topics,
        timeout_s=timeout_s,
        poll_period_s=0.10,
        log_progress=True,
    )


def wait_for_manual_mapping_topics(
    node,
    timeout_s: float = 5.0,
) -> TopicWaitResult:
    """
    Wait for topics needed for manual mapping teleop chain.

    Expected:
        keyboard -> /cmd_vel_manual
        twist_mux -> /cmd_vel_mux
        shaper -> /cmd_vel
        safety gate -> /cmd_vel_safe
        perception -> /safety/stop
    """
    return wait_for_topics(
        node,
        [
            "/cmd_vel_manual",
            "/cmd_vel_mux",
            "/cmd_vel",
            "/cmd_vel_safe",
            "/safety/stop",
        ],
        timeout_s=timeout_s,
        poll_period_s=0.10,
        log_progress=True,
    )


def wait_for_recovery_topics(
    node,
    timeout_s: float = 5.0,
) -> TopicWaitResult:
    """
    Wait for topics needed for recovery testing.

    Expected new recovery contract:
        /savo_control/recovery_request
        /savo_control/recovery_active
        /cmd_vel_recovery
        /cmd_vel_mux
        /cmd_vel
        /cmd_vel_safe
        /safety/stop
    """
    return wait_for_topics(
        node,
        [
            "/savo_control/recovery_request",
            "/savo_control/recovery_active",
            "/cmd_vel_recovery",
            "/cmd_vel_mux",
            "/cmd_vel",
            "/cmd_vel_safe",
            "/safety/stop",
        ],
        timeout_s=timeout_s,
        poll_period_s=0.10,
        log_progress=True,
    )


def wait_for_localization_topics(
    node,
    timeout_s: float = 5.0,
    include_vo: bool = False,
) -> TopicWaitResult:
    """
    Wait for localization topics used by control.

    For Robot Savo:
        /odometry/filtered is the only topic control should normally consume.

    Optional:
        /vo/odom from savo-edge, used by localization stack, not directly by
        most control nodes.
    """
    topics = ["/odometry/filtered"]

    if include_vo:
        topics.append("/vo/odom")

    return wait_for_topics(
        node,
        topics,
        timeout_s=timeout_s,
        poll_period_s=0.10,
        log_progress=True,
    )


def required_type_contract_control_chain() -> dict[str, str]:
    """
    Return expected message type contract for common control chain topics.
    """
    return {
        "/cmd_vel_manual": "geometry_msgs/msg/Twist",
        "/cmd_vel_auto": "geometry_msgs/msg/Twist",
        "/cmd_vel_nav": "geometry_msgs/msg/Twist",
        "/cmd_vel_recovery": "geometry_msgs/msg/Twist",
        "/cmd_vel_mux": "geometry_msgs/msg/Twist",
        "/cmd_vel": "geometry_msgs/msg/Twist",
        "/cmd_vel_safe": "geometry_msgs/msg/Twist",
        "/safety/stop": "std_msgs/msg/Bool",
        "/safety/slowdown_factor": "std_msgs/msg/Float32",
        "/savo_control/mode_cmd": "std_msgs/msg/String",
        "/savo_control/mode_state": "std_msgs/msg/String",
        "/savo_control/recovery_request": "std_msgs/msg/Bool",
        "/savo_control/recovery_active": "std_msgs/msg/Bool",
        "/odometry/filtered": "nav_msgs/msg/Odometry",
        "/depth/min_front_m": "std_msgs/msg/Float32",
    }


def wait_for_control_chain_topic_types(
    node,
    timeout_s: float = 5.0,
    include_optional: bool = False,
) -> TopicTypeWaitResult:
    """
    Wait for expected message types of core control topics.

    If include_optional=False, only checks core always-needed topics.
    If include_optional=True, includes odometry and depth topics too.
    """
    contract = required_type_contract_control_chain()

    required = {
        "/cmd_vel_mux": contract["/cmd_vel_mux"],
        "/cmd_vel": contract["/cmd_vel"],
        "/cmd_vel_safe": contract["/cmd_vel_safe"],
        "/safety/stop": contract["/safety/stop"],
        "/savo_control/mode_cmd": contract["/savo_control/mode_cmd"],
        "/savo_control/mode_state": contract["/savo_control/mode_state"],
    }

    if include_optional:
        required["/odometry/filtered"] = contract["/odometry/filtered"]
        required["/depth/min_front_m"] = contract["/depth/min_front_m"]

    return wait_for_topic_types(
        node,
        required,
        timeout_s=timeout_s,
        poll_period_s=0.10,
        log_progress=True,
    )