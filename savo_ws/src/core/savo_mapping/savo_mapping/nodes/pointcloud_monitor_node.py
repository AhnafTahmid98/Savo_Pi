#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Pointcloud monitor node for Robot Savo mapping."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from savo_mapping.constants import (
    DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
    DEFAULT_PUBLISH_RATE_HZ,
    FRAME_CAMERA_DEPTH,
    TOPIC_REALSENSE_POINTS,
)
from savo_mapping.diagnostics.pointcloud_ready_check import (
    evaluate_pointcloud_msg,
    pointcloud_result_to_diagnostic,
)
from savo_mapping.ros.adapters import json_msg
from savo_mapping.ros.qos_profiles import get_topic_qos_profile, status_qos
from savo_mapping.utils.timing import RateTracker, age_s, now_s


DECISION_DISABLED = "disabled"
DECISION_WAITING = "waiting"
DECISION_READY = "ready"
DECISION_DEGRADED = "degraded"


@dataclass
class PointcloudRuntime:
    latest_msg: Optional[PointCloud2] = None
    msg_count: int = 0
    last_wall_s: Optional[float] = None

    def __post_init__(self) -> None:
        self.rate_tracker = RateTracker()

    @property
    def rate_hz(self) -> float:
        return self.rate_tracker.rate_hz

    @property
    def sample_age_s(self) -> Optional[float]:
        return age_s(self.last_wall_s)

    @property
    def frame_id(self) -> str:
        if self.latest_msg is None:
            return ""

        return str(self.latest_msg.header.frame_id)

    def update(self, msg: PointCloud2) -> None:
        stamp = now_s()

        self.latest_msg = msg
        self.msg_count += 1
        self.last_wall_s = stamp
        self.rate_tracker.tick(stamp)


@dataclass(frozen=True)
class PointcloudMonitorSnapshot:
    enabled: bool
    ok: bool
    decision: str
    message: str
    topic: str
    expected_frame: str
    received_frame: str = ""
    point_count: int = 0
    msg_count: int = 0
    rate_hz: float = 0.0
    age_s: Optional[float] = None
    stale: bool = False
    diagnostic: dict[str, Any] = field(default_factory=dict)
    timestamp_s: float = field(default_factory=now_s)

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "ok": self.ok,
            "decision": self.decision,
            "message": self.message,
            "topic": self.topic,
            "expected_frame": self.expected_frame,
            "received_frame": self.received_frame,
            "point_count": self.point_count,
            "msg_count": self.msg_count,
            "rate_hz": self.rate_hz,
            "age_s": self.age_s,
            "stale": self.stale,
            "diagnostic": dict(self.diagnostic),
            "timestamp_s": self.timestamp_s,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True)


class PointcloudMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_monitor_node")

        self._declare_parameters()
        self._load_parameters()

        self._runtime = PointcloudRuntime()
        self._last_log_state: Optional[tuple[bool, str, bool, int]] = None

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            status_qos(),
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._on_pointcloud,
            get_topic_qos_profile(self.pointcloud_topic),
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._on_timer,
        )

        self.get_logger().info(
            "Pointcloud monitor started: "
            f"enabled={self.enabled} topic={self.pointcloud_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("enabled", False)
        self.declare_parameter("pointcloud_topic", TOPIC_REALSENSE_POINTS)
        self.declare_parameter("status_topic", "/savo_mapping/pointcloud_status")
        self.declare_parameter("expected_frame", FRAME_CAMERA_DEPTH)

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter("min_rate_hz", 3.0)
        self.declare_parameter("min_points", 100)
        self.declare_parameter("stale_timeout_s", DEFAULT_POINTCLOUD_STALE_TIMEOUT_S)

        self.declare_parameter("verbose_status_log", False)

    def _load_parameters(self) -> None:
        self.enabled = bool(self.get_parameter("enabled").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.expected_frame = str(self.get_parameter("expected_frame").value)

        self.publish_rate_hz = self._positive_float_parameter(
            "publish_rate_hz",
            DEFAULT_PUBLISH_RATE_HZ,
        )
        self.min_rate_hz = self._non_negative_float_parameter("min_rate_hz", 3.0)
        self.min_points = max(0, int(self.get_parameter("min_points").value))
        self.stale_timeout_s = self._positive_float_parameter(
            "stale_timeout_s",
            DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
        )

        self.verbose_status_log = bool(
            self.get_parameter("verbose_status_log").value
        )

    def _positive_float_parameter(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)

        if value <= 0.0:
            self.get_logger().warning(
                f"Parameter {name} must be positive. Using {default}."
            )
            return float(default)

        return value

    def _non_negative_float_parameter(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)

        if value < 0.0:
            self.get_logger().warning(
                f"Parameter {name} must be non-negative. Using {default}."
            )
            return float(default)

        return value

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        self._runtime.update(msg)

    def _on_timer(self) -> None:
        snapshot = self._build_snapshot()

        self.status_pub.publish(json_msg(snapshot.to_dict()))

        log_state = (
            bool(snapshot.ok),
            str(snapshot.decision),
            bool(snapshot.stale),
            int(snapshot.msg_count),
        )

        if self.verbose_status_log or log_state != self._last_log_state:
            self.get_logger().info(
                "Pointcloud status: "
                f"enabled={snapshot.enabled} ok={snapshot.ok} "
                f"decision={snapshot.decision} stale={snapshot.stale} "
                f"count={snapshot.msg_count} points={snapshot.point_count}"
            )

        self._last_log_state = log_state

    def _build_snapshot(self) -> PointcloudMonitorSnapshot:
        result = self._evaluate()
        diagnostic = pointcloud_result_to_diagnostic(result, required=self.enabled)

        return PointcloudMonitorSnapshot(
            enabled=bool(result.enabled),
            ok=bool(result.ok),
            decision=_decision_from_result(result),
            message=_message_from_result(result),
            topic=self.pointcloud_topic,
            expected_frame=self.expected_frame,
            received_frame=self._runtime.frame_id,
            point_count=int(result.point_count),
            msg_count=int(result.msg_count),
            rate_hz=float(result.rate_hz),
            age_s=result.age_s,
            stale=bool(result.stale),
            diagnostic=diagnostic.to_dict(),
        )

    def _evaluate(self):
        if self._runtime.latest_msg is None:
            return evaluate_pointcloud_msg(
                _empty_pointcloud_msg(self.expected_frame),
                enabled=self.enabled,
                msg_count=0,
                rate_hz=0.0,
                age_s=None,
                topic=self.pointcloud_topic,
                min_rate_hz=self.min_rate_hz,
                min_points=self.min_points,
                stale_timeout_s=self.stale_timeout_s,
            )

        return evaluate_pointcloud_msg(
            self._runtime.latest_msg,
            enabled=self.enabled,
            msg_count=self._runtime.msg_count,
            rate_hz=self._runtime.rate_hz,
            age_s=self._runtime.sample_age_s,
            topic=self.pointcloud_topic,
            min_rate_hz=self.min_rate_hz,
            min_points=self.min_points,
            stale_timeout_s=self.stale_timeout_s,
        )


def _decision_from_result(result: Any) -> str:
    if not bool(result.enabled):
        return DECISION_DISABLED

    if int(result.msg_count) <= 0:
        return DECISION_WAITING

    if bool(result.ok):
        return DECISION_READY

    return DECISION_DEGRADED


def _message_from_result(result: Any) -> str:
    if not bool(result.enabled):
        return "Pointcloud monitor disabled."

    if int(result.msg_count) <= 0:
        return "Waiting for pointcloud messages."

    if bool(result.ok):
        return "Pointcloud stream ready."

    if bool(result.stale):
        return "Pointcloud stream is stale."

    return "Pointcloud stream is degraded."


def _empty_pointcloud_msg(frame_id: str) -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = str(frame_id)
    return msg


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)

    node = PointcloudMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Pointcloud monitor stopped.")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())