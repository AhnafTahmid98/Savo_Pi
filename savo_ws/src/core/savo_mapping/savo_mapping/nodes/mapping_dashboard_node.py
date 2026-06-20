#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Lightweight dashboard node for Robot Savo mapping."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from savo_mapping.constants import DEFAULT_PUBLISH_RATE_HZ
from savo_mapping.ros.qos_profiles import status_qos
from savo_mapping.utils.timing import age_s, now_s


@dataclass
class TopicSnapshot:
    name: str
    latest: Any = None
    msg_count: int = 0
    last_wall_s: Optional[float] = None

    @property
    def sample_age_s(self) -> Optional[float]:
        return age_s(self.last_wall_s)

    @property
    def seen(self) -> bool:
        return self.msg_count > 0

    def update(self, value: Any) -> None:
        self.latest = value
        self.msg_count += 1
        self.last_wall_s = now_s()


class MappingDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("mapping_dashboard_node")

        self._declare_parameters()
        self._load_parameters()

        self.ready = TopicSnapshot("ready")
        self.mode = TopicSnapshot("mode")
        self.status = TopicSnapshot("status")
        self.map_quality = TopicSnapshot("map_quality")
        self.exploration = TopicSnapshot("exploration")
        self.pointcloud = TopicSnapshot("pointcloud")
        self.apriltag = TopicSnapshot("apriltag")
        self.location_bridge = TopicSnapshot("location_bridge")

        self._last_log_text = ""

        self.dashboard_pub = self.create_publisher(
            String,
            self.dashboard_topic,
            status_qos(),
        )

        self._create_subscriptions()

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._on_timer,
        )

        self.get_logger().info(
            "Mapping dashboard started: "
            f"status={self.status_topic} ready={self.ready_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("ready_topic", "/savo_mapping/ready")
        self.declare_parameter("mode_topic", "/savo_mapping/mode")
        self.declare_parameter("status_topic", "/savo_mapping/status")
        self.declare_parameter("map_quality_topic", "/savo_mapping/map_quality")
        self.declare_parameter(
            "exploration_status_topic",
            "/savo_mapping/exploration_status",
        )
        self.declare_parameter(
            "pointcloud_status_topic",
            "/savo_mapping/pointcloud_status",
        )
        self.declare_parameter(
            "apriltag_status_topic",
            "/savo_mapping/apriltag_status",
        )
        self.declare_parameter(
            "location_bridge_status_topic",
            "/savo_mapping/location_bridge_status",
        )
        self.declare_parameter("dashboard_topic", "/savo_mapping/dashboard")

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter("stale_timeout_s", 2.0)
        self.declare_parameter("print_to_console", True)
        self.declare_parameter("show_optional", True)
        self.declare_parameter("verbose_status_log", False)

    def _load_parameters(self) -> None:
        self.ready_topic = str(self.get_parameter("ready_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.map_quality_topic = str(self.get_parameter("map_quality_topic").value)
        self.exploration_status_topic = str(
            self.get_parameter("exploration_status_topic").value
        )
        self.pointcloud_status_topic = str(
            self.get_parameter("pointcloud_status_topic").value
        )
        self.apriltag_status_topic = str(
            self.get_parameter("apriltag_status_topic").value
        )
        self.location_bridge_status_topic = str(
            self.get_parameter("location_bridge_status_topic").value
        )
        self.dashboard_topic = str(self.get_parameter("dashboard_topic").value)

        self.publish_rate_hz = self._positive_float_parameter(
            "publish_rate_hz",
            DEFAULT_PUBLISH_RATE_HZ,
        )
        self.stale_timeout_s = self._positive_float_parameter(
            "stale_timeout_s",
            2.0,
        )

        self.print_to_console = bool(self.get_parameter("print_to_console").value)
        self.show_optional = bool(self.get_parameter("show_optional").value)
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

    def _create_subscriptions(self) -> None:
        self.ready_sub = self.create_subscription(
            Bool,
            self.ready_topic,
            self._on_ready,
            status_qos(),
        )
        self.mode_sub = self.create_subscription(
            String,
            self.mode_topic,
            self._on_mode,
            status_qos(),
        )
        self.status_sub = self.create_subscription(
            String,
            self.status_topic,
            self._on_status,
            status_qos(),
        )
        self.map_quality_sub = self.create_subscription(
            String,
            self.map_quality_topic,
            self._on_map_quality,
            status_qos(),
        )
        self.exploration_sub = self.create_subscription(
            String,
            self.exploration_status_topic,
            self._on_exploration,
            status_qos(),
        )
        self.pointcloud_sub = self.create_subscription(
            String,
            self.pointcloud_status_topic,
            self._on_pointcloud,
            status_qos(),
        )
        self.apriltag_sub = self.create_subscription(
            String,
            self.apriltag_status_topic,
            self._on_apriltag,
            status_qos(),
        )
        self.location_bridge_sub = self.create_subscription(
            String,
            self.location_bridge_status_topic,
            self._on_location_bridge,
            status_qos(),
        )

    def _on_ready(self, msg: Bool) -> None:
        self.ready.update(bool(msg.data))

    def _on_mode(self, msg: String) -> None:
        self.mode.update(str(msg.data))

    def _on_status(self, msg: String) -> None:
        self.status.update(_parse_json_or_text(msg.data))

    def _on_map_quality(self, msg: String) -> None:
        self.map_quality.update(_parse_json_or_text(msg.data))

    def _on_exploration(self, msg: String) -> None:
        self.exploration.update(_parse_json_or_text(msg.data))

    def _on_pointcloud(self, msg: String) -> None:
        self.pointcloud.update(_parse_json_or_text(msg.data))

    def _on_apriltag(self, msg: String) -> None:
        self.apriltag.update(_parse_json_or_text(msg.data))

    def _on_location_bridge(self, msg: String) -> None:
        self.location_bridge.update(_parse_json_or_text(msg.data))

    def _on_timer(self) -> None:
        data = self._build_dashboard()
        text = self._format_dashboard(data)

        self.dashboard_pub.publish(String(data=json.dumps(data, sort_keys=True)))

        if self.print_to_console and (
            self.verbose_status_log or text != self._last_log_text
        ):
            self.get_logger().info(text)

        self._last_log_text = text

    def _build_dashboard(self) -> dict[str, Any]:
        status_data = self.status.latest if isinstance(self.status.latest, dict) else {}
        readiness = status_data.get("readiness", {}) if status_data else {}

        dashboard = {
            "timestamp_s": time.time(),
            "ready": bool(self.ready.latest) if self.ready.seen else False,
            "mode": self.mode.latest if self.mode.seen else _safe_get(status_data, "mode", "unknown"),
            "active": bool(_safe_get(status_data, "active", False)),
            "degraded": bool(_safe_get(status_data, "degraded", True)),
            "map_name": _safe_get(status_data, "map_name", None),
            "session_id": _safe_get(status_data, "session_id", None),
            "message": _safe_get(status_data, "message", "Waiting for mapping status."),
            "topics": {
                "ready": self._topic_state(self.ready),
                "mode": self._topic_state(self.mode),
                "status": self._topic_state(self.status),
                "map_quality": self._topic_state(self.map_quality),
                "exploration": self._topic_state(self.exploration),
                "pointcloud": self._topic_state(self.pointcloud),
                "apriltag": self._topic_state(self.apriltag),
                "location_bridge": self._topic_state(self.location_bridge),
            },
            "failed_required_checks": tuple(
                readiness.get("failed_required_checks", ())
            ),
            "failed_optional_checks": tuple(
                readiness.get("failed_optional_checks", ())
            ),
        }

        if self.show_optional:
            dashboard["optional"] = {
                "map_quality": self.map_quality.latest,
                "exploration": self.exploration.latest,
                "pointcloud": self.pointcloud.latest,
                "apriltag": self.apriltag.latest,
                "location_bridge": self.location_bridge.latest,
            }

        return dashboard

    def _topic_state(self, snapshot: TopicSnapshot) -> dict[str, Any]:
        topic_age = snapshot.sample_age_s

        return {
            "seen": snapshot.seen,
            "msg_count": snapshot.msg_count,
            "age_s": topic_age,
            "stale": topic_age is None or topic_age > self.stale_timeout_s,
        }

    def _format_dashboard(self, data: dict[str, Any]) -> str:
        failed_required = data.get("failed_required_checks", ())
        failed_optional = data.get("failed_optional_checks", ())

        lines = [
            "Mapping dashboard: "
            f"ready={data['ready']} mode={data['mode']} "
            f"active={data['active']} degraded={data['degraded']}",
            f"message={data['message']}",
        ]

        if data.get("map_name"):
            lines.append(f"map={data['map_name']}")

        if data.get("session_id"):
            lines.append(f"session={data['session_id']}")

        if failed_required:
            lines.append(f"required_waiting={list(failed_required)}")

        if failed_optional and self.show_optional:
            lines.append(f"optional_waiting={list(failed_optional)}")

        return " | ".join(lines)


def _parse_json_or_text(raw: str) -> Any:
    text = str(raw).strip()

    if not text:
        return ""

    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return text


def _safe_get(data: dict[str, Any], key: str, default: Any) -> Any:
    if not isinstance(data, dict):
        return default

    return data.get(key, default)


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)

    node = MappingDashboardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Mapping dashboard stopped.")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())