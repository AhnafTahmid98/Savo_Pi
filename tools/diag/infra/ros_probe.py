#!/usr/bin/env python3
"""Read-only ROS 2 topic sampling for Robot SAVO diagnostics."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Any


@dataclass
class Probe:
    topic: str
    message_type: str
    publisher_count: int
    sample_count: int
    elapsed_s: float
    rate_hz: float
    samples: list[dict[str, Any]]


def collect(
    topic: str,
    message_type: str,
    *,
    duration_s: float = 3.0,
    minimum_samples: int = 1,
    reliable: bool = False,
    maximum_samples: int = 100,
) -> Probe:
    """Collect bounded samples without creating any publisher/client."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    if not topic.startswith("/"):
        raise ValueError("topic must be absolute")
    if duration_s <= 0.0 or minimum_samples < 1 or maximum_samples < minimum_samples:
        raise ValueError("invalid bounded probe configuration")

    rclpy.init(args=None)
    node = Node("robot_savo_read_only_probe")
    samples: list[dict[str, Any]] = []
    message_class = get_message(message_type)
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=(ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT),
        durability=DurabilityPolicy.VOLATILE,
    )

    def callback(message: Any) -> None:
        if len(samples) < maximum_samples:
            samples.append(dict(message_to_ordereddict(message)))

    subscription = node.create_subscription(message_class, topic, callback, qos)
    del subscription
    start = time.monotonic()
    deadline = start + duration_s
    try:
        while time.monotonic() < deadline and len(samples) < maximum_samples:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
            if len(samples) >= minimum_samples and duration_s <= 0.5:
                break
        elapsed = max(time.monotonic() - start, 1.0e-6)
        publishers = len(node.get_publishers_info_by_topic(topic))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    rate = 0.0 if len(samples) < 2 else (len(samples) - 1) / elapsed
    return Probe(topic, message_type, publishers, len(samples), elapsed, rate, samples)


def nested(data: dict[str, Any], path: str, default: Any = None) -> Any:
    value: Any = data
    for token in path.split("."):
        if not isinstance(value, dict) or token not in value:
            return default
        value = value[token]
    return value


def quaternion_yaw(orientation: dict[str, Any]) -> float:
    x = float(orientation.get("x", 0.0))
    y = float(orientation.get("y", 0.0))
    z = float(orientation.get("z", 0.0))
    w = float(orientation.get("w", 1.0))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def compact(probe: Probe) -> dict[str, Any]:
    return {
        "topic": probe.topic,
        "message_type": probe.message_type,
        "publisher_count": probe.publisher_count,
        "sample_count": probe.sample_count,
        "elapsed_s": round(probe.elapsed_s, 3),
        "rate_hz": round(probe.rate_hz, 3),
        "first_sample": probe.samples[0] if probe.samples else None,
        "last_sample": probe.samples[-1] if probe.samples else None,
    }
