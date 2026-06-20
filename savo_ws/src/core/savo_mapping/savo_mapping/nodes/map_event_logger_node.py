#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Semantic landmark recorder node for Robot Savo mapping."""

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
from savo_mapping.semantic.location_candidate import (
    SOURCE_HUMAN,
    LocationPose,
    make_location_candidate,
)
from savo_mapping.semantic.semantic_landmark_store import (
    SemanticLandmarkStore,
    load_semantic_landmark_store,
    save_semantic_landmark_store,
)


DECISION_DISABLED = "disabled"
DECISION_WAITING = "waiting"
DECISION_READY = "ready"
DECISION_RECORDED = "recorded"
DECISION_ERROR = "error"


@dataclass(frozen=True)
class LandmarkRecordCommand:
    label: str
    pose: LocationPose
    source: str = SOURCE_HUMAN
    area_type: str = "manual_location"
    confidence: float = 1.0
    actor: str = "operator"
    notes: str = ""
    confirm: bool = False
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.label.strip():
            raise ValueError("Landmark label cannot be empty.")

        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("Landmark confidence must be between 0 and 1.")


@dataclass(frozen=True)
class SemanticLandmarkRecorderStatus:
    enabled: bool
    ok: bool
    decision: str
    message: str
    semantic_store_path: str = ""
    command_topic: str = ""
    command_count: int = 0
    recorded_count: int = 0
    rejected_count: int = 0
    candidate_count: int = 0
    landmark_count: int = 0
    last_candidate_key: str = ""
    last_label: str = ""
    last_source: str = ""
    last_confirmed: bool = False
    error: str = ""
    timestamp_s: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "ok": self.ok,
            "decision": self.decision,
            "message": self.message,
            "semantic_store_path": self.semantic_store_path,
            "command_topic": self.command_topic,
            "command_count": self.command_count,
            "recorded_count": self.recorded_count,
            "rejected_count": self.rejected_count,
            "candidate_count": self.candidate_count,
            "landmark_count": self.landmark_count,
            "last_candidate_key": self.last_candidate_key,
            "last_label": self.last_label,
            "last_source": self.last_source,
            "last_confirmed": self.last_confirmed,
            "error": self.error,
            "timestamp_s": self.timestamp_s,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True)


class SemanticLandmarkRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_landmark_recorder_node")

        self._declare_parameters()
        self._load_parameters()

        self.command_count = 0
        self.recorded_count = 0
        self.rejected_count = 0
        self.last_candidate_key = ""
        self.last_label = ""
        self.last_source = ""
        self.last_confirmed = False
        self.last_message = "Semantic landmark recorder ready."
        self.last_error = ""
        self._last_status_text = ""

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            status_qos(),
        )
        self.result_pub = self.create_publisher(
            String,
            self.result_topic,
            status_qos(),
        )
        self.candidate_pub = self.create_publisher(
            String,
            self.candidate_topic,
            status_qos(),
        )

        self.command_sub = self.create_subscription(
            String,
            self.command_topic,
            self._on_command,
            status_qos(),
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._on_timer,
        )

        self.get_logger().info(
            "Semantic landmark recorder started: "
            f"enabled={self.enabled} store={self.semantic_store_path or '<none>'}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("enabled", False)

        self.declare_parameter("semantic_store_path", "")

        self.declare_parameter(
            "command_topic",
            "/savo_mapping/semantic_landmark_record",
        )
        self.declare_parameter(
            "status_topic",
            "/savo_mapping/semantic_landmark_recorder/status",
        )
        self.declare_parameter(
            "result_topic",
            "/savo_mapping/semantic_landmark_recorder/result",
        )
        self.declare_parameter(
            "candidate_topic",
            "/savo_mapping/location_candidate",
        )

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter("require_semantic_store_path", False)
        self.declare_parameter("default_source", SOURCE_HUMAN)
        self.declare_parameter("default_area_type", "manual_location")
        self.declare_parameter("default_actor", "operator")
        self.declare_parameter("default_confirm", False)
        self.declare_parameter("verbose_status_log", False)

    def _load_parameters(self) -> None:
        self.enabled = bool(self.get_parameter("enabled").value)
        self.semantic_store_path = str(
            self.get_parameter("semantic_store_path").value or ""
        )

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.result_topic = str(self.get_parameter("result_topic").value)
        self.candidate_topic = str(self.get_parameter("candidate_topic").value)

        self.publish_rate_hz = self._positive_float_parameter(
            "publish_rate_hz",
            DEFAULT_PUBLISH_RATE_HZ,
        )
        self.require_semantic_store_path = bool(
            self.get_parameter("require_semantic_store_path").value
        )
        self.default_source = str(
            self.get_parameter("default_source").value or SOURCE_HUMAN
        )
        self.default_area_type = str(
            self.get_parameter("default_area_type").value or "manual_location"
        )
        self.default_actor = str(
            self.get_parameter("default_actor").value or "operator"
        )
        self.default_confirm = bool(self.get_parameter("default_confirm").value)
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

    def _on_command(self, msg: String) -> None:
        self.command_count += 1

        if not self.enabled:
            self.rejected_count += 1
            self.last_message = "Semantic landmark command ignored: node disabled."
            self.last_error = ""
            self.get_logger().warning(self.last_message)
            return

        try:
            command = self._parse_command(msg.data)
            result = self._record_landmark(command)

            self.recorded_count += 1
            self.last_candidate_key = str(result["candidate"]["key"])
            self.last_label = str(result["candidate"]["label"])
            self.last_source = str(result["candidate"]["source"])
            self.last_confirmed = bool(result["candidate"]["confirmed"])
            self.last_message = "Semantic landmark recorded."
            self.last_error = ""

            self.result_pub.publish(json_msg(result))
            self.candidate_pub.publish(json_msg(result["candidate"]))

            self.get_logger().info(
                "Semantic landmark recorded: "
                f"label={self.last_label} key={self.last_candidate_key} "
                f"confirmed={self.last_confirmed}"
            )

        except Exception as exc:
            self.rejected_count += 1
            self.last_message = "Semantic landmark command rejected."
            self.last_error = str(exc)
            self.get_logger().warning(f"{self.last_message} {exc}")

    def _parse_command(self, raw: str) -> LandmarkRecordCommand:
        text = str(raw).strip()

        if not text:
            raise ValueError("empty semantic landmark command")

        data = json.loads(text)

        if not isinstance(data, dict):
            raise ValueError("semantic landmark command must be a JSON object")

        pose_data = data.get("pose", {})

        if "x" in data or "y" in data:
            pose_data = {
                "x": data.get("x", 0.0),
                "y": data.get("y", 0.0),
                "yaw": data.get("yaw", 0.0),
                "frame_id": data.get("frame_id", "map"),
            }

        if not isinstance(pose_data, dict):
            raise ValueError("pose must be an object")

        return LandmarkRecordCommand(
            label=str(data["label"]),
            pose=LocationPose(
                x=float(pose_data.get("x", 0.0)),
                y=float(pose_data.get("y", 0.0)),
                yaw=float(pose_data.get("yaw", 0.0)),
                frame_id=str(pose_data.get("frame_id", "map")),
            ),
            source=str(data.get("source") or self.default_source),
            area_type=str(data.get("area_type") or self.default_area_type),
            confidence=float(data.get("confidence", 1.0)),
            actor=str(data.get("actor") or self.default_actor),
            notes=str(data.get("notes", "")),
            confirm=bool(data.get("confirm", self.default_confirm)),
            metadata=dict(data.get("metadata", {})),
        )

    def _record_landmark(self, command: LandmarkRecordCommand) -> dict[str, Any]:
        if not self.semantic_store_path:
            if self.require_semantic_store_path:
                raise ValueError("semantic_store_path parameter is required")

            raise ValueError("semantic_store_path is empty; refusing to record")

        store_path = Path(self.semantic_store_path).expanduser()
        store = load_semantic_landmark_store(store_path)

        candidate = make_location_candidate(
            label=command.label,
            x=command.pose.x,
            y=command.pose.y,
            yaw=command.pose.yaw,
            frame_id=command.pose.frame_id,
            source=command.source,
            confidence=command.confidence,
            area_type=command.area_type,
            notes=command.notes,
            metadata={
                **command.metadata,
                "recorded_by": command.actor,
                "recorded_source": command.source,
            },
        )

        store = store.upsert_candidate(candidate)

        if command.confirm:
            store = store.confirm_candidate(candidate.key, confirmed_by=command.actor)
            candidate = store.find_candidate(candidate.key)

            if candidate is None:
                raise RuntimeError("confirmed candidate disappeared from store")

        save_semantic_landmark_store(store_path, store)

        return {
            "ok": True,
            "message": "Semantic landmark recorded.",
            "confirmed": bool(candidate.confirmed),
            "candidate": candidate.to_dict(),
            "semantic_store_path": str(store_path),
            "candidate_count": store.candidate_count,
            "landmark_count": store.landmark_count,
            "timestamp_s": time.time(),
        }

    def _on_timer(self) -> None:
        status = self._build_status()

        self.status_pub.publish(json_msg(status.to_dict()))

        status_text = (
            f"Semantic landmark recorder: enabled={status.enabled} ok={status.ok} "
            f"decision={status.decision} commands={status.command_count} "
            f"recorded={status.recorded_count} rejected={status.rejected_count}"
        )

        if self.verbose_status_log or status_text != self._last_status_text:
            self.get_logger().info(status_text)

        self._last_status_text = status_text

    def _build_status(self) -> SemanticLandmarkRecorderStatus:
        if not self.enabled:
            return SemanticLandmarkRecorderStatus(
                enabled=False,
                ok=True,
                decision=DECISION_DISABLED,
                message="Semantic landmark recorder disabled.",
                semantic_store_path=self.semantic_store_path,
                command_topic=self.command_topic,
                command_count=self.command_count,
                recorded_count=self.recorded_count,
                rejected_count=self.rejected_count,
                last_candidate_key=self.last_candidate_key,
                last_label=self.last_label,
                last_source=self.last_source,
                last_confirmed=self.last_confirmed,
            )

        if not self.semantic_store_path:
            return SemanticLandmarkRecorderStatus(
                enabled=True,
                ok=False,
                decision=DECISION_WAITING,
                message="Waiting for semantic_store_path parameter.",
                semantic_store_path="",
                command_topic=self.command_topic,
                command_count=self.command_count,
                recorded_count=self.recorded_count,
                rejected_count=self.rejected_count,
                last_candidate_key=self.last_candidate_key,
                last_label=self.last_label,
                last_source=self.last_source,
                last_confirmed=self.last_confirmed,
                error=self.last_error,
            )

        try:
            store = load_semantic_landmark_store(
                Path(self.semantic_store_path).expanduser()
            )
        except Exception as exc:
            return SemanticLandmarkRecorderStatus(
                enabled=True,
                ok=False,
                decision=DECISION_ERROR,
                message="Failed to load semantic store.",
                semantic_store_path=self.semantic_store_path,
                command_topic=self.command_topic,
                command_count=self.command_count,
                recorded_count=self.recorded_count,
                rejected_count=self.rejected_count,
                last_candidate_key=self.last_candidate_key,
                last_label=self.last_label,
                last_source=self.last_source,
                last_confirmed=self.last_confirmed,
                error=str(exc),
            )

        decision = DECISION_RECORDED if self.recorded_count > 0 else DECISION_READY

        return SemanticLandmarkRecorderStatus(
            enabled=True,
            ok=self.last_error == "",
            decision=decision,
            message=self.last_message,
            semantic_store_path=self.semantic_store_path,
            command_topic=self.command_topic,
            command_count=self.command_count,
            recorded_count=self.recorded_count,
            rejected_count=self.rejected_count,
            candidate_count=store.candidate_count,
            landmark_count=store.landmark_count,
            last_candidate_key=self.last_candidate_key,
            last_label=self.last_label,
            last_source=self.last_source,
            last_confirmed=self.last_confirmed,
            error=self.last_error,
        )


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)

    node = SemanticLandmarkRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Semantic landmark recorder stopped.")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())