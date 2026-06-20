#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Map event logger node for Robot Savo mapping."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from savo_mapping.constants import DEFAULT_PUBLISH_RATE_HZ
from savo_mapping.ros.adapters import json_msg
from savo_mapping.ros.qos_profiles import status_qos


DECISION_DISABLED = "disabled"
DECISION_READY = "ready"
DECISION_ERROR = "error"

DEFAULT_EVENT_TOPICS = (
    "/savo_mapping/mode_command",
    "/savo_mapping/frontier_explorer/goal",
    "/savo_mapping/location_confirmation/result",
    "/savo_mapping/semantic_landmark_recorder/result",
    "/savo_mapping/apriltag_mapper/result",
    "/savo_mapping/location_bridge/status",
)


@dataclass(frozen=True)
class MapEventRecord:
    topic: str
    payload_raw: str
    payload: dict[str, Any] = field(default_factory=dict)
    event_type: str = "ros_message"
    timestamp_s: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp_s": self.timestamp_s,
            "event_type": self.event_type,
            "topic": self.topic,
            "payload": dict(self.payload),
            "payload_raw": self.payload_raw,
        }

    def to_json_line(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True)


@dataclass(frozen=True)
class MapEventLoggerStatus:
    enabled: bool
    ok: bool
    decision: str
    message: str
    log_path: str = ""
    event_topics: tuple[str, ...] = ()
    event_count: int = 0
    written_count: int = 0
    rejected_count: int = 0
    last_topic: str = ""
    last_event_type: str = ""
    last_error: str = ""
    timestamp_s: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "ok": self.ok,
            "decision": self.decision,
            "message": self.message,
            "log_path": self.log_path,
            "event_topics": list(self.event_topics),
            "event_count": self.event_count,
            "written_count": self.written_count,
            "rejected_count": self.rejected_count,
            "last_topic": self.last_topic,
            "last_event_type": self.last_event_type,
            "last_error": self.last_error,
            "timestamp_s": self.timestamp_s,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True)


class MapEventLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("map_event_logger_node")

        self._declare_parameters()
        self._load_parameters()

        self.event_count = 0
        self.written_count = 0
        self.rejected_count = 0
        self.last_topic = ""
        self.last_event_type = ""
        self.last_error = ""
        self._last_status_text = ""

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            status_qos(),
        )
        self.event_pub = self.create_publisher(
            String,
            self.event_topic,
            status_qos(),
        )

        self._subscriptions = [
            self.create_subscription(
                String,
                topic,
                self._make_event_callback(topic),
                status_qos(),
            )
            for topic in self.event_topics
        ]

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._on_timer,
        )

        self.get_logger().info(
            "Map event logger started: "
            f"enabled={self.enabled} log_path={self.log_path}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("enabled", False)
        self.declare_parameter("log_path", "/tmp/savo_mapping/map_events.jsonl")

        self.declare_parameter(
            "status_topic",
            "/savo_mapping/map_event_logger/status",
        )
        self.declare_parameter(
            "event_topic",
            "/savo_mapping/map_event_logger/event",
        )
        self.declare_parameter("event_topics", list(DEFAULT_EVENT_TOPICS))

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter("max_payload_chars", 20000)
        self.declare_parameter("publish_logged_event", True)
        self.declare_parameter("verbose_status_log", False)

    def _load_parameters(self) -> None:
        self.enabled = bool(self.get_parameter("enabled").value)
        self.log_path = str(self.get_parameter("log_path").value or "")

        self.status_topic = str(self.get_parameter("status_topic").value)
        self.event_topic = str(self.get_parameter("event_topic").value)

        raw_topics = self.get_parameter("event_topics").value
        self.event_topics = tuple(
            topic for topic in (str(item).strip() for item in raw_topics) if topic
        )

        self.publish_rate_hz = self._positive_float_parameter(
            "publish_rate_hz",
            DEFAULT_PUBLISH_RATE_HZ,
        )
        self.max_payload_chars = max(
            256,
            int(self.get_parameter("max_payload_chars").value),
        )
        self.publish_logged_event = bool(
            self.get_parameter("publish_logged_event").value
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

    def _make_event_callback(self, topic: str):
        def _callback(msg: String) -> None:
            self._on_event(topic, msg)

        return _callback

    def _on_event(self, topic: str, msg: String) -> None:
        self.event_count += 1

        if not self.enabled:
            self.last_topic = topic
            self.last_event_type = DECISION_DISABLED
            return

        try:
            record = self._make_record(topic, msg.data)
            self._write_record(record)
            self.written_count += 1
            self.last_topic = topic
            self.last_event_type = record.event_type
            self.last_error = ""

            if self.publish_logged_event:
                self.event_pub.publish(json_msg(record.to_dict()))

        except Exception as exc:
            self.rejected_count += 1
            self.last_topic = topic
            self.last_event_type = DECISION_ERROR
            self.last_error = str(exc)
            self.get_logger().warning(f"Map event logging failed: {exc}")

    def _make_record(self, topic: str, raw_payload: str) -> MapEventRecord:
        payload_raw = str(raw_payload)

        if len(payload_raw) > self.max_payload_chars:
            payload_raw = payload_raw[: self.max_payload_chars]
            payload_raw += "...[truncated]"

        payload = _parse_json_object(payload_raw)
        event_type = _event_type_from_topic(topic)

        return MapEventRecord(
            topic=topic,
            payload_raw=payload_raw,
            payload=payload,
            event_type=event_type,
        )

    def _write_record(self, record: MapEventRecord) -> None:
        if not self.log_path:
            raise ValueError("log_path parameter is empty")

        output_path = Path(self.log_path).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with output_path.open("a", encoding="utf-8") as handle:
            handle.write(record.to_json_line() + "\n")

    def _on_timer(self) -> None:
        status = self._build_status()

        self.status_pub.publish(json_msg(status.to_dict()))

        status_text = (
            f"Map event logger: enabled={status.enabled} ok={status.ok} "
            f"decision={status.decision} events={status.event_count} "
            f"written={status.written_count} rejected={status.rejected_count}"
        )

        if self.verbose_status_log or status_text != self._last_status_text:
            self.get_logger().info(status_text)

        self._last_status_text = status_text

    def _build_status(self) -> MapEventLoggerStatus:
        if not self.enabled:
            return MapEventLoggerStatus(
                enabled=False,
                ok=True,
                decision=DECISION_DISABLED,
                message="Map event logger disabled.",
                log_path=self.log_path,
                event_topics=self.event_topics,
                event_count=self.event_count,
                written_count=self.written_count,
                rejected_count=self.rejected_count,
                last_topic=self.last_topic,
                last_event_type=self.last_event_type,
                last_error=self.last_error,
            )

        if self.last_error:
            return MapEventLoggerStatus(
                enabled=True,
                ok=False,
                decision=DECISION_ERROR,
                message="Map event logger has an error.",
                log_path=self.log_path,
                event_topics=self.event_topics,
                event_count=self.event_count,
                written_count=self.written_count,
                rejected_count=self.rejected_count,
                last_topic=self.last_topic,
                last_event_type=self.last_event_type,
                last_error=self.last_error,
            )

        return MapEventLoggerStatus(
            enabled=True,
            ok=True,
            decision=DECISION_READY,
            message="Map event logger ready.",
            log_path=self.log_path,
            event_topics=self.event_topics,
            event_count=self.event_count,
            written_count=self.written_count,
            rejected_count=self.rejected_count,
            last_topic=self.last_topic,
            last_event_type=self.last_event_type,
            last_error=self.last_error,
        )


def _parse_json_object(raw: str) -> dict[str, Any]:
    text = str(raw).strip()

    if not text.startswith("{"):
        return {}

    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return {}

    if not isinstance(data, dict):
        return {}

    return data


def _event_type_from_topic(topic: str) -> str:
    clean = str(topic).strip("/").replace("/", ".")

    return clean or "ros_message"


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)

    node = MapEventLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Map event logger stopped.")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())