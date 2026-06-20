#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Location confirmation node for Robot Savo mapping."""

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
from savo_mapping.semantic.location_confirmation import (
    LocationConfirmation,
    LocationConfirmationResult,
    apply_confirmation_to_store,
    location_confirmation_from_dict,
)
from savo_mapping.semantic.semantic_landmark_store import (
    SemanticLandmarkStore,
    load_semantic_landmark_store,
    save_semantic_landmark_store,
)


DECISION_DISABLED = "disabled"
DECISION_WAITING = "waiting"
DECISION_READY = "ready"
DECISION_APPLIED = "applied"
DECISION_REJECTED = "rejected"
DECISION_ERROR = "error"


@dataclass(frozen=True)
class LocationConfirmationNodeStatus:
    enabled: bool
    ok: bool
    decision: str
    message: str
    semantic_store_path: str = ""
    command_topic: str = ""
    command_count: int = 0
    applied_count: int = 0
    rejected_count: int = 0
    last_action: str = ""
    last_candidate_key: str = ""
    candidate_count: int = 0
    landmark_count: int = 0
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
            "applied_count": self.applied_count,
            "rejected_count": self.rejected_count,
            "last_action": self.last_action,
            "last_candidate_key": self.last_candidate_key,
            "candidate_count": self.candidate_count,
            "landmark_count": self.landmark_count,
            "error": self.error,
            "timestamp_s": self.timestamp_s,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True)


class LocationConfirmationNode(Node):
    def __init__(self) -> None:
        super().__init__("location_confirmation_node")

        self._declare_parameters()
        self._load_parameters()

        self.command_count = 0
        self.applied_count = 0
        self.rejected_count = 0
        self.last_action = ""
        self.last_candidate_key = ""
        self.last_message = "Location confirmation node ready."
        self.last_error = ""
        self.last_result: Optional[LocationConfirmationResult] = None
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
            "Location confirmation started: "
            f"enabled={self.enabled} store={self.semantic_store_path or '<none>'}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("enabled", False)

        self.declare_parameter("semantic_store_path", "")

        self.declare_parameter(
            "command_topic",
            "/savo_mapping/location_confirmation_command",
        )
        self.declare_parameter(
            "status_topic",
            "/savo_mapping/location_confirmation/status",
        )
        self.declare_parameter(
            "result_topic",
            "/savo_mapping/location_confirmation/result",
        )

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter("require_semantic_store", False)
        self.declare_parameter("verbose_status_log", False)

    def _load_parameters(self) -> None:
        self.enabled = bool(self.get_parameter("enabled").value)
        self.semantic_store_path = str(
            self.get_parameter("semantic_store_path").value or ""
        )

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.result_topic = str(self.get_parameter("result_topic").value)

        self.publish_rate_hz = self._positive_float_parameter(
            "publish_rate_hz",
            DEFAULT_PUBLISH_RATE_HZ,
        )
        self.require_semantic_store = bool(
            self.get_parameter("require_semantic_store").value
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

    def _on_command(self, msg: String) -> None:
        self.command_count += 1

        if not self.enabled:
            self.rejected_count += 1
            self.last_message = "Location confirmation command ignored: node disabled."
            self.last_error = ""
            self.get_logger().warning(self.last_message)
            return

        try:
            confirmation = self._parse_command(msg.data)
            self._apply_confirmation(confirmation)

        except Exception as exc:
            self.rejected_count += 1
            self.last_message = "Location confirmation command rejected."
            self.last_error = str(exc)
            self.get_logger().warning(f"{self.last_message} {exc}")

    def _parse_command(self, raw: str) -> LocationConfirmation:
        text = str(raw).strip()

        if not text:
            raise ValueError("empty confirmation command")

        data = json.loads(text)

        if not isinstance(data, dict):
            raise ValueError("confirmation command must be a JSON object")

        return location_confirmation_from_dict(data)

    def _apply_confirmation(self, confirmation: LocationConfirmation) -> None:
        if not self.semantic_store_path:
            raise ValueError("semantic_store_path parameter is required")

        store_path = Path(self.semantic_store_path).expanduser()

        if self.require_semantic_store and not store_path.exists():
            raise FileNotFoundError(f"semantic store not found: {store_path}")

        store = load_semantic_landmark_store(store_path)
        updated_store, result = apply_confirmation_to_store(store, confirmation)

        if result.ok:
            save_semantic_landmark_store(store_path, updated_store)
            self.applied_count += 1
            self.last_message = result.message
            self.last_error = ""
        else:
            self.rejected_count += 1
            self.last_message = result.message
            self.last_error = result.message

        self.last_action = confirmation.action
        self.last_candidate_key = confirmation.clean_candidate_key
        self.last_result = result

        self.result_pub.publish(json_msg(result.to_dict()))

        if result.ok:
            self.get_logger().info(
                "Location confirmation applied: "
                f"action={confirmation.action} key={confirmation.clean_candidate_key}"
            )
        else:
            self.get_logger().warning(
                "Location confirmation failed: "
                f"action={confirmation.action} key={confirmation.clean_candidate_key} "
                f"message={result.message}"
            )

    def _on_timer(self) -> None:
        status = self._build_status()

        self.status_pub.publish(json_msg(status.to_dict()))

        status_text = (
            f"Location confirmation: enabled={status.enabled} ok={status.ok} "
            f"decision={status.decision} commands={status.command_count} "
            f"applied={status.applied_count} rejected={status.rejected_count}"
        )

        if self.verbose_status_log or status_text != self._last_status_text:
            self.get_logger().info(status_text)

        self._last_status_text = status_text

    def _build_status(self) -> LocationConfirmationNodeStatus:
        if not self.enabled:
            return LocationConfirmationNodeStatus(
                enabled=False,
                ok=True,
                decision=DECISION_DISABLED,
                message="Location confirmation disabled.",
                semantic_store_path=self.semantic_store_path,
                command_topic=self.command_topic,
                command_count=self.command_count,
                applied_count=self.applied_count,
                rejected_count=self.rejected_count,
                last_action=self.last_action,
                last_candidate_key=self.last_candidate_key,
            )

        if not self.semantic_store_path:
            return LocationConfirmationNodeStatus(
                enabled=True,
                ok=not self.require_semantic_store,
                decision=DECISION_WAITING,
                message="Waiting for semantic_store_path parameter.",
                command_topic=self.command_topic,
                command_count=self.command_count,
                applied_count=self.applied_count,
                rejected_count=self.rejected_count,
                error=self.last_error,
            )

        store_path = Path(self.semantic_store_path).expanduser()

        if self.require_semantic_store and not store_path.exists():
            return LocationConfirmationNodeStatus(
                enabled=True,
                ok=False,
                decision=DECISION_WAITING,
                message="Semantic store file does not exist yet.",
                semantic_store_path=str(store_path),
                command_topic=self.command_topic,
                command_count=self.command_count,
                applied_count=self.applied_count,
                rejected_count=self.rejected_count,
                error=self.last_error,
            )

        try:
            store = load_semantic_landmark_store(store_path)
        except Exception as exc:
            return LocationConfirmationNodeStatus(
                enabled=True,
                ok=False,
                decision=DECISION_ERROR,
                message="Failed to load semantic store.",
                semantic_store_path=str(store_path),
                command_topic=self.command_topic,
                command_count=self.command_count,
                applied_count=self.applied_count,
                rejected_count=self.rejected_count,
                last_action=self.last_action,
                last_candidate_key=self.last_candidate_key,
                error=str(exc),
            )

        decision = DECISION_READY

        if self.last_result is not None and self.last_result.ok:
            decision = DECISION_APPLIED
        elif self.last_result is not None and not self.last_result.ok:
            decision = DECISION_REJECTED

        return LocationConfirmationNodeStatus(
            enabled=True,
            ok=self.last_error == "",
            decision=decision,
            message=self.last_message,
            semantic_store_path=str(store_path),
            command_topic=self.command_topic,
            command_count=self.command_count,
            applied_count=self.applied_count,
            rejected_count=self.rejected_count,
            last_action=self.last_action,
            last_candidate_key=self.last_candidate_key,
            candidate_count=store.candidate_count,
            landmark_count=store.landmark_count,
            error=self.last_error,
        )


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)

    node = LocationConfirmationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Location confirmation stopped.")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())