#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Text/JSON dashboard node for Robot Savo mapping."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from savo_mapping.constants import DEFAULT_PUBLISH_RATE_HZ
from savo_mapping.ros.adapters import json_msg
from savo_mapping.ros.qos_profiles import status_qos
from savo_mapping.utils.timing import age_s


@dataclass
class DashboardInput:
    topic: str
    raw: str = ""
    data: dict[str, Any] = field(default_factory=dict)
    msg_count: int = 0
    last_wall_s: Optional[float] = None

    @property
    def sample_age_s(self) -> Optional[float]:
        return age_s(self.last_wall_s)

    def update(self, raw: str) -> None:
        self.raw = str(raw)
        self.data = _parse_json_object(self.raw)
        self.msg_count += 1
        self.last_wall_s = time.time()


@dataclass(frozen=True)
class MappingDashboardSnapshot:
    ok: bool
    mode: str
    workflow_phase: str
    ready: bool
    degraded: bool
    active: bool
    message: str
    map_name: Optional[str] = None
    session_id: Optional[str] = None
    semantic_enabled: bool = False
    semantic_ok: bool = True
    semantic_message: str = ""
    landmark_count: int = 0
    candidate_count: int = 0
    pending_candidate_count: int = 0
    known_location_count: int = 0
    exploration_enabled: bool = False
    exploration_ok: bool = True
    exploration_strategy: str = "frontier"
    exploration_message: str = ""
    map_quality_level: str = ""
    map_quality_message: str = ""
    input_ages_s: dict[str, Optional[float]] = field(default_factory=dict)
    input_counts: dict[str, int] = field(default_factory=dict)
    timestamp_s: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "mode": self.mode,
            "workflow_phase": self.workflow_phase,
            "ready": self.ready,
            "degraded": self.degraded,
            "active": self.active,
            "message": self.message,
            "map_name": self.map_name,
            "session_id": self.session_id,
            "semantic": {
                "enabled": self.semantic_enabled,
                "ok": self.semantic_ok,
                "message": self.semantic_message,
                "landmark_count": self.landmark_count,
                "candidate_count": self.candidate_count,
                "pending_candidate_count": self.pending_candidate_count,
                "known_location_count": self.known_location_count,
            },
            "exploration": {
                "enabled": self.exploration_enabled,
                "ok": self.exploration_ok,
                "strategy": self.exploration_strategy,
                "message": self.exploration_message,
            },
            "map_quality": {
                "level": self.map_quality_level,
                "message": self.map_quality_message,
            },
            "input_ages_s": dict(self.input_ages_s),
            "input_counts": dict(self.input_counts),
            "timestamp_s": self.timestamp_s,
        }

    def to_text(self) -> str:
        semantic_text = (
            f"semantic={self.landmark_count} landmarks, "
            f"{self.pending_candidate_count} pending"
            if self.semantic_enabled
            else "semantic=disabled"
        )
        exploration_text = (
            f"exploration={self.exploration_strategy}"
            if self.exploration_enabled
            else "exploration=idle"
        )

        return (
            "Robot Savo mapping dashboard | "
            f"mode={self.mode} phase={self.workflow_phase} "
            f"ready={self.ready} degraded={self.degraded} | "
            f"{semantic_text} | {exploration_text}"
        )

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True)


class MappingDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("mapping_dashboard_node")

        self._declare_parameters()
        self._load_parameters()

        self.status_input = DashboardInput(self.status_topic)
        self.mode_input = DashboardInput(self.mode_topic)
        self.workflow_phase_input = DashboardInput(self.workflow_phase_topic)
        self.session_state_input = DashboardInput(self.session_state_topic)
        self.semantic_status_input = DashboardInput(self.semantic_status_topic)
        self.exploration_status_input = DashboardInput(self.exploration_status_topic)
        self.map_quality_input = DashboardInput(self.map_quality_topic)

        self.dashboard_pub = self.create_publisher(
            String,
            self.dashboard_topic,
            status_qos(),
        )
        self.dashboard_text_pub = self.create_publisher(
            String,
            self.dashboard_text_topic,
            status_qos(),
        )

        self._create_subscriptions()

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._on_timer,
        )

        self._last_text = ""

        self.get_logger().info(
            "Mapping dashboard started: "
            f"status={self.status_topic} output={self.dashboard_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("status_topic", "/savo_mapping/status")
        self.declare_parameter("mode_topic", "/savo_mapping/mode")
        self.declare_parameter("workflow_phase_topic", "/savo_mapping/workflow_phase")
        self.declare_parameter("session_state_topic", "/savo_mapping/session_state")
        self.declare_parameter("semantic_status_topic", "/savo_mapping/semantic_status")
        self.declare_parameter(
            "exploration_status_topic",
            "/savo_mapping/exploration_status",
        )
        self.declare_parameter("map_quality_topic", "/savo_mapping/map_quality")

        self.declare_parameter("dashboard_topic", "/savo_mapping/dashboard")
        self.declare_parameter("dashboard_text_topic", "/savo_mapping/dashboard_text")

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter("verbose_status_log", False)

    def _load_parameters(self) -> None:
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.workflow_phase_topic = str(
            self.get_parameter("workflow_phase_topic").value
        )
        self.session_state_topic = str(
            self.get_parameter("session_state_topic").value
        )
        self.semantic_status_topic = str(
            self.get_parameter("semantic_status_topic").value
        )
        self.exploration_status_topic = str(
            self.get_parameter("exploration_status_topic").value
        )
        self.map_quality_topic = str(self.get_parameter("map_quality_topic").value)

        self.dashboard_topic = str(self.get_parameter("dashboard_topic").value)
        self.dashboard_text_topic = str(
            self.get_parameter("dashboard_text_topic").value
        )

        self.publish_rate_hz = self._positive_float_parameter(
            "publish_rate_hz",
            DEFAULT_PUBLISH_RATE_HZ,
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

    def _create_subscriptions(self) -> None:
        self.create_subscription(
            String,
            self.status_topic,
            lambda msg: self.status_input.update(msg.data),
            status_qos(),
        )
        self.create_subscription(
            String,
            self.mode_topic,
            lambda msg: self.mode_input.update(msg.data),
            status_qos(),
        )
        self.create_subscription(
            String,
            self.workflow_phase_topic,
            lambda msg: self.workflow_phase_input.update(msg.data),
            status_qos(),
        )
        self.create_subscription(
            String,
            self.session_state_topic,
            lambda msg: self.session_state_input.update(msg.data),
            status_qos(),
        )
        self.create_subscription(
            String,
            self.semantic_status_topic,
            lambda msg: self.semantic_status_input.update(msg.data),
            status_qos(),
        )
        self.create_subscription(
            String,
            self.exploration_status_topic,
            lambda msg: self.exploration_status_input.update(msg.data),
            status_qos(),
        )
        self.create_subscription(
            String,
            self.map_quality_topic,
            lambda msg: self.map_quality_input.update(msg.data),
            status_qos(),
        )

    def _on_timer(self) -> None:
        snapshot = self._build_snapshot()
        text = snapshot.to_text()

        self.dashboard_pub.publish(json_msg(snapshot.to_dict()))
        self.dashboard_text_pub.publish(String(data=text))

        if self.verbose_status_log or text != self._last_text:
            self.get_logger().info(text)

        self._last_text = text

    def _build_snapshot(self) -> MappingDashboardSnapshot:
        status = self.status_input.data
        session = self.session_state_input.data
        semantic = self.semantic_status_input.data
        exploration = self.exploration_status_input.data
        map_quality = self.map_quality_input.data

        mode = _first_non_empty(
            self.mode_input.raw,
            session.get("mode"),
            status.get("mode"),
            "unknown",
        )
        workflow_phase = _first_non_empty(
            self.workflow_phase_input.raw,
            session.get("workflow_phase"),
            "unknown",
        )

        ready = bool(status.get("ready", False))
        degraded = bool(status.get("degraded", True))
        active = bool(status.get("active", session.get("active", False)))

        semantic_ok = bool(semantic.get("ok", True))
        exploration_ok = bool(exploration.get("ok", True))

        message = _first_non_empty(
            status.get("message"),
            session.get("message"),
            "Mapping dashboard waiting for status.",
        )

        return MappingDashboardSnapshot(
            ok=semantic_ok and exploration_ok,
            mode=str(mode),
            workflow_phase=str(workflow_phase),
            ready=ready,
            degraded=degraded,
            active=active,
            message=str(message),
            map_name=_optional_string(status.get("map_name") or session.get("map_name")),
            session_id=_optional_string(
                status.get("session_id") or session.get("session_id")
            ),
            semantic_enabled=bool(semantic.get("enabled", False)),
            semantic_ok=semantic_ok,
            semantic_message=str(semantic.get("message", "")),
            landmark_count=_int_value(semantic.get("landmark_count")),
            candidate_count=_int_value(semantic.get("candidate_count")),
            pending_candidate_count=_int_value(
                semantic.get("pending_candidate_count")
            ),
            known_location_count=_int_value(semantic.get("known_location_count")),
            exploration_enabled=bool(exploration.get("enabled", False)),
            exploration_ok=exploration_ok,
            exploration_strategy=str(exploration.get("strategy", "frontier")),
            exploration_message=str(exploration.get("message", "")),
            map_quality_level=str(map_quality.get("level", "")),
            map_quality_message=str(map_quality.get("message", "")),
            input_ages_s=self._input_ages(),
            input_counts=self._input_counts(),
        )

    def _input_ages(self) -> dict[str, Optional[float]]:
        return {
            "status": self.status_input.sample_age_s,
            "mode": self.mode_input.sample_age_s,
            "workflow_phase": self.workflow_phase_input.sample_age_s,
            "session_state": self.session_state_input.sample_age_s,
            "semantic_status": self.semantic_status_input.sample_age_s,
            "exploration_status": self.exploration_status_input.sample_age_s,
            "map_quality": self.map_quality_input.sample_age_s,
        }

    def _input_counts(self) -> dict[str, int]:
        return {
            "status": self.status_input.msg_count,
            "mode": self.mode_input.msg_count,
            "workflow_phase": self.workflow_phase_input.msg_count,
            "session_state": self.session_state_input.msg_count,
            "semantic_status": self.semantic_status_input.msg_count,
            "exploration_status": self.exploration_status_input.msg_count,
            "map_quality": self.map_quality_input.msg_count,
        }


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


def _first_non_empty(*items: Any) -> str:
    for item in items:
        if item is None:
            continue

        text = str(item).strip()

        if text:
            return text

    return ""


def _optional_string(value: Any) -> Optional[str]:
    if value is None:
        return None

    text = str(value).strip()

    return text or None


def _int_value(value: Any) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


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