#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""AprilTag mapper node for Robot Savo semantic mapping."""

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
from savo_mapping.semantic.apriltag_mapper import (
    AprilTagMapper,
    AprilTagMappingResult,
)
from savo_mapping.semantic.apriltag_observation import (
    AprilTagObservation,
    apriltag_observation_from_dict,
)
from savo_mapping.semantic.semantic_landmark_store import (
    SemanticLandmarkStore,
    load_semantic_landmark_store,
    save_semantic_landmark_store,
)
from savo_mapping.semantic.tag_database import load_tag_database


DECISION_DISABLED = "disabled"
DECISION_WAITING = "waiting"
DECISION_MAPPED = "mapped"
DECISION_REJECTED = "rejected"
DECISION_ERROR = "error"


@dataclass(frozen=True)
class AprilTagMapperNodeStatus:
    enabled: bool
    ok: bool
    decision: str
    message: str
    observation_topic: str = ""
    tag_database_path: str = ""
    semantic_store_path: str = ""
    observation_count: int = 0
    mapped_count: int = 0
    rejected_count: int = 0
    candidate_count: int = 0
    landmark_count: int = 0
    last_tag_id: Optional[int] = None
    last_candidate_key: str = ""
    last_label: str = ""
    error: str = ""
    timestamp_s: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "ok": self.ok,
            "decision": self.decision,
            "message": self.message,
            "observation_topic": self.observation_topic,
            "tag_database_path": self.tag_database_path,
            "semantic_store_path": self.semantic_store_path,
            "observation_count": self.observation_count,
            "mapped_count": self.mapped_count,
            "rejected_count": self.rejected_count,
            "candidate_count": self.candidate_count,
            "landmark_count": self.landmark_count,
            "last_tag_id": self.last_tag_id,
            "last_candidate_key": self.last_candidate_key,
            "last_label": self.last_label,
            "error": self.error,
            "timestamp_s": self.timestamp_s,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True)


class AprilTagMapperNode(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_mapper_node")

        self._declare_parameters()
        self._load_parameters()

        self.observation_count = 0
        self.mapped_count = 0
        self.rejected_count = 0
        self.last_tag_id: Optional[int] = None
        self.last_candidate_key = ""
        self.last_label = ""
        self.last_message = "AprilTag mapper node ready."
        self.last_error = ""
        self.last_result: Optional[AprilTagMappingResult] = None
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

        self.observation_sub = self.create_subscription(
            String,
            self.observation_topic,
            self._on_observation,
            status_qos(),
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._on_timer,
        )

        self.get_logger().info(
            "AprilTag mapper started: "
            f"enabled={self.enabled} observation_topic={self.observation_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("enabled", False)

        self.declare_parameter(
            "observation_topic",
            "/savo_mapping/apriltag_observation",
        )
        self.declare_parameter(
            "status_topic",
            "/savo_mapping/apriltag_mapper/status",
        )
        self.declare_parameter(
            "result_topic",
            "/savo_mapping/apriltag_mapper/result",
        )
        self.declare_parameter(
            "candidate_topic",
            "/savo_mapping/location_candidate",
        )

        self.declare_parameter("tag_database_path", "")
        self.declare_parameter("semantic_store_path", "")

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter("min_confidence", 0.30)
        self.declare_parameter("allow_unknown_tags", False)
        self.declare_parameter("unknown_label_prefix", "Tag")
        self.declare_parameter("write_semantic_store", True)
        self.declare_parameter("require_tag_database", False)
        self.declare_parameter("require_semantic_store_path", False)
        self.declare_parameter("verbose_status_log", False)

    def _load_parameters(self) -> None:
        self.enabled = bool(self.get_parameter("enabled").value)

        self.observation_topic = str(self.get_parameter("observation_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.result_topic = str(self.get_parameter("result_topic").value)
        self.candidate_topic = str(self.get_parameter("candidate_topic").value)

        self.tag_database_path = str(
            self.get_parameter("tag_database_path").value or ""
        )
        self.semantic_store_path = str(
            self.get_parameter("semantic_store_path").value or ""
        )

        self.publish_rate_hz = self._positive_float_parameter(
            "publish_rate_hz",
            DEFAULT_PUBLISH_RATE_HZ,
        )
        self.min_confidence = self._confidence_parameter("min_confidence", 0.30)
        self.allow_unknown_tags = bool(
            self.get_parameter("allow_unknown_tags").value
        )
        self.unknown_label_prefix = str(
            self.get_parameter("unknown_label_prefix").value or "Tag"
        )
        self.write_semantic_store = bool(
            self.get_parameter("write_semantic_store").value
        )
        self.require_tag_database = bool(
            self.get_parameter("require_tag_database").value
        )
        self.require_semantic_store_path = bool(
            self.get_parameter("require_semantic_store_path").value
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

    def _confidence_parameter(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)

        if not 0.0 <= value <= 1.0:
            self.get_logger().warning(
                f"Parameter {name} must be between 0 and 1. Using {default}."
            )
            return float(default)

        return value

    def _on_observation(self, msg: String) -> None:
        self.observation_count += 1

        if not self.enabled:
            self.rejected_count += 1
            self.last_message = "AprilTag observation ignored: node disabled."
            self.last_error = ""
            self.get_logger().warning(self.last_message)
            return

        try:
            observation = self._parse_observation(msg.data)
            self._map_observation(observation)

        except Exception as exc:
            self.rejected_count += 1
            self.last_message = "AprilTag observation rejected."
            self.last_error = str(exc)
            self.get_logger().warning(f"{self.last_message} {exc}")

    def _parse_observation(self, raw: str) -> AprilTagObservation:
        text = str(raw).strip()

        if not text:
            raise ValueError("empty AprilTag observation")

        data = json.loads(text)

        if not isinstance(data, dict):
            raise ValueError("AprilTag observation must be a JSON object")

        return apriltag_observation_from_dict(data)

    def _map_observation(self, observation: AprilTagObservation) -> None:
        mapper = self._load_mapper()
        result = mapper.map_observation(observation)

        self.last_tag_id = observation.tag_id
        self.last_result = result

        if result.ok and result.candidate is not None:
            self.mapped_count += 1
            self.last_candidate_key = result.candidate.key
            self.last_label = result.candidate.label
            self.last_message = result.message
            self.last_error = ""

            self.candidate_pub.publish(json_msg(result.candidate.to_dict()))
            self._maybe_write_candidate(result.candidate)

            self.get_logger().info(
                "AprilTag mapped: "
                f"tag_id={observation.tag_id} label={result.candidate.label}"
            )
        else:
            self.rejected_count += 1
            self.last_candidate_key = ""
            self.last_label = ""
            self.last_message = result.message
            self.last_error = result.message

            self.get_logger().warning(
                "AprilTag mapping failed: "
                f"tag_id={observation.tag_id} message={result.message}"
            )

        self.result_pub.publish(json_msg(result.to_dict()))

    def _load_mapper(self) -> AprilTagMapper:
        if self.tag_database_path:
            database = load_tag_database(Path(self.tag_database_path).expanduser())
            mapper = database.to_apriltag_mapper(
                min_confidence=self.min_confidence
            )

            if self.allow_unknown_tags:
                return AprilTagMapper(
                    labels=mapper.labels,
                    min_confidence=self.min_confidence,
                    allow_unknown_tags=True,
                    unknown_label_prefix=self.unknown_label_prefix,
                )

            return mapper

        if self.require_tag_database and not self.allow_unknown_tags:
            raise ValueError("tag_database_path is required")

        return AprilTagMapper(
            labels=(),
            min_confidence=self.min_confidence,
            allow_unknown_tags=self.allow_unknown_tags,
            unknown_label_prefix=self.unknown_label_prefix,
        )

    def _maybe_write_candidate(self, candidate) -> None:
        if not self.write_semantic_store:
            return

        if not self.semantic_store_path:
            if self.require_semantic_store_path:
                raise ValueError("semantic_store_path is required")

            return

        store_path = Path(self.semantic_store_path).expanduser()
        store = load_semantic_landmark_store(store_path)
        store = store.upsert_candidate(candidate)
        save_semantic_landmark_store(store_path, store)

    def _on_timer(self) -> None:
        status = self._build_status()

        self.status_pub.publish(json_msg(status.to_dict()))

        status_text = (
            f"AprilTag mapper: enabled={status.enabled} ok={status.ok} "
            f"decision={status.decision} observations={status.observation_count} "
            f"mapped={status.mapped_count} rejected={status.rejected_count}"
        )

        if self.verbose_status_log or status_text != self._last_status_text:
            self.get_logger().info(status_text)

        self._last_status_text = status_text

    def _build_status(self) -> AprilTagMapperNodeStatus:
        if not self.enabled:
            return AprilTagMapperNodeStatus(
                enabled=False,
                ok=True,
                decision=DECISION_DISABLED,
                message="AprilTag mapper disabled.",
                observation_topic=self.observation_topic,
                tag_database_path=self.tag_database_path,
                semantic_store_path=self.semantic_store_path,
                observation_count=self.observation_count,
                mapped_count=self.mapped_count,
                rejected_count=self.rejected_count,
                last_tag_id=self.last_tag_id,
                last_candidate_key=self.last_candidate_key,
                last_label=self.last_label,
            )

        if self.require_tag_database and not self.tag_database_path:
            return AprilTagMapperNodeStatus(
                enabled=True,
                ok=False,
                decision=DECISION_WAITING,
                message="Waiting for tag_database_path parameter.",
                observation_topic=self.observation_topic,
                tag_database_path=self.tag_database_path,
                semantic_store_path=self.semantic_store_path,
                observation_count=self.observation_count,
                mapped_count=self.mapped_count,
                rejected_count=self.rejected_count,
                error=self.last_error,
            )

        candidate_count = 0
        landmark_count = 0

        if self.semantic_store_path:
            try:
                store = load_semantic_landmark_store(
                    Path(self.semantic_store_path).expanduser()
                )
                candidate_count = store.candidate_count
                landmark_count = store.landmark_count
            except Exception as exc:
                return AprilTagMapperNodeStatus(
                    enabled=True,
                    ok=False,
                    decision=DECISION_ERROR,
                    message="Failed to load semantic store.",
                    observation_topic=self.observation_topic,
                    tag_database_path=self.tag_database_path,
                    semantic_store_path=self.semantic_store_path,
                    observation_count=self.observation_count,
                    mapped_count=self.mapped_count,
                    rejected_count=self.rejected_count,
                    last_tag_id=self.last_tag_id,
                    last_candidate_key=self.last_candidate_key,
                    last_label=self.last_label,
                    error=str(exc),
                )

        decision = DECISION_WAITING
        ok = self.last_error == ""

        if self.last_result is not None and self.last_result.ok:
            decision = DECISION_MAPPED
        elif self.last_result is not None and not self.last_result.ok:
            decision = DECISION_REJECTED

        return AprilTagMapperNodeStatus(
            enabled=True,
            ok=ok,
            decision=decision,
            message=self.last_message,
            observation_topic=self.observation_topic,
            tag_database_path=self.tag_database_path,
            semantic_store_path=self.semantic_store_path,
            observation_count=self.observation_count,
            mapped_count=self.mapped_count,
            rejected_count=self.rejected_count,
            candidate_count=candidate_count,
            landmark_count=landmark_count,
            last_tag_id=self.last_tag_id,
            last_candidate_key=self.last_candidate_key,
            last_label=self.last_label,
            error=self.last_error,
        )


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)

    node = AprilTagMapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("AprilTag mapper stopped.")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())