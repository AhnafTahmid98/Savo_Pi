#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Location bridge node for Robot Savo mapping."""

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
from savo_mapping.semantic.location_bridge import (
    LocationBridgeResult,
    bridge_store_to_locations,
    save_known_locations,
)
from savo_mapping.semantic.semantic_landmark_store import (
    SemanticLandmarkStore,
    load_semantic_landmark_store,
)


@dataclass(frozen=True)
class LocationBridgeNodeStatus:
    enabled: bool
    ok: bool
    decision: str
    message: str
    semantic_store_path: str = ""
    known_locations_path: str = ""
    landmark_count: int = 0
    location_count: int = 0
    known_location_keys: tuple[str, ...] = ()
    wrote_known_locations: bool = False
    error: str = ""
    timestamp_s: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "ok": self.ok,
            "decision": self.decision,
            "message": self.message,
            "semantic_store_path": self.semantic_store_path,
            "known_locations_path": self.known_locations_path,
            "landmark_count": self.landmark_count,
            "location_count": self.location_count,
            "known_location_keys": list(self.known_location_keys),
            "wrote_known_locations": self.wrote_known_locations,
            "error": self.error,
            "timestamp_s": self.timestamp_s,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True)


class LocationBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("location_bridge_node")

        self._declare_parameters()
        self._load_parameters()

        self._last_status_text = ""
        self._last_known_locations_json = ""

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            status_qos(),
        )
        self.known_locations_pub = self.create_publisher(
            String,
            self.known_locations_topic,
            status_qos(),
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._on_timer,
        )

        self.get_logger().info(
            "Location bridge started: "
            f"enabled={self.enabled} store={self.semantic_store_path or '<none>'}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("enabled", False)

        self.declare_parameter("semantic_store_path", "")
        self.declare_parameter("known_locations_path", "")

        self.declare_parameter(
            "status_topic",
            "/savo_mapping/location_bridge/status",
        )
        self.declare_parameter(
            "known_locations_topic",
            "/savo_mapping/known_locations",
        )

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter("write_known_locations", True)
        self.declare_parameter("require_semantic_store", False)
        self.declare_parameter("verbose_status_log", False)

    def _load_parameters(self) -> None:
        self.enabled = bool(self.get_parameter("enabled").value)

        self.semantic_store_path = str(
            self.get_parameter("semantic_store_path").value or ""
        )
        self.known_locations_path = str(
            self.get_parameter("known_locations_path").value or ""
        )

        self.status_topic = str(self.get_parameter("status_topic").value)
        self.known_locations_topic = str(
            self.get_parameter("known_locations_topic").value
        )

        self.publish_rate_hz = self._positive_float_parameter(
            "publish_rate_hz",
            DEFAULT_PUBLISH_RATE_HZ,
        )
        self.write_known_locations = bool(
            self.get_parameter("write_known_locations").value
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

    def _on_timer(self) -> None:
        status, known_locations = self._run_bridge()

        self.status_pub.publish(json_msg(status.to_dict()))

        if known_locations is not None:
            known_locations_json = json.dumps(known_locations, sort_keys=True)

            if known_locations_json != self._last_known_locations_json:
                self.known_locations_pub.publish(json_msg(known_locations))
                self._last_known_locations_json = known_locations_json

        status_text = (
            f"Location bridge: enabled={status.enabled} ok={status.ok} "
            f"decision={status.decision} locations={status.location_count} "
            f"wrote={status.wrote_known_locations}"
        )

        if self.verbose_status_log or status_text != self._last_status_text:
            self.get_logger().info(status_text)

        self._last_status_text = status_text

    def _run_bridge(self) -> tuple[LocationBridgeNodeStatus, Optional[dict[str, Any]]]:
        if not self.enabled:
            return (
                LocationBridgeNodeStatus(
                    enabled=False,
                    ok=True,
                    decision="disabled",
                    message="Location bridge disabled.",
                    semantic_store_path=self.semantic_store_path,
                    known_locations_path=self.known_locations_path,
                ),
                None,
            )

        if not self.semantic_store_path:
            return (
                LocationBridgeNodeStatus(
                    enabled=True,
                    ok=not self.require_semantic_store,
                    decision="waiting",
                    message="Waiting for semantic_store_path parameter.",
                    semantic_store_path="",
                    known_locations_path=self.known_locations_path,
                ),
                None,
            )

        store_path = Path(self.semantic_store_path).expanduser()

        if self.require_semantic_store and not store_path.exists():
            return (
                LocationBridgeNodeStatus(
                    enabled=True,
                    ok=False,
                    decision="waiting",
                    message="Semantic landmark store file does not exist yet.",
                    semantic_store_path=str(store_path),
                    known_locations_path=self.known_locations_path,
                ),
                None,
            )

        try:
            store = load_semantic_landmark_store(store_path)
            result = bridge_store_to_locations(store)
            known_locations = result.to_known_locations_dict()
            wrote = self._maybe_write_known_locations(known_locations)

            status = self._status_from_result(
                store=store,
                result=result,
                known_locations=known_locations,
                wrote_known_locations=wrote,
            )

            return status, known_locations

        except Exception as exc:
            return (
                LocationBridgeNodeStatus(
                    enabled=True,
                    ok=False,
                    decision="error",
                    message="Location bridge failed.",
                    semantic_store_path=str(store_path),
                    known_locations_path=self.known_locations_path,
                    error=str(exc),
                ),
                None,
            )

    def _maybe_write_known_locations(self, data: dict[str, Any]) -> bool:
        if not self.write_known_locations:
            return False

        if not self.known_locations_path:
            return False

        save_known_locations(
            Path(self.known_locations_path).expanduser(),
            data,
        )

        return True

    def _status_from_result(
        self,
        *,
        store: SemanticLandmarkStore,
        result: LocationBridgeResult,
        known_locations: dict[str, Any],
        wrote_known_locations: bool,
    ) -> LocationBridgeNodeStatus:
        locations = known_locations.get("locations", {})

        if isinstance(locations, dict):
            keys = tuple(sorted(str(key) for key in locations.keys()))
        else:
            keys = ()

        return LocationBridgeNodeStatus(
            enabled=True,
            ok=result.ok,
            decision="bridged",
            message=result.message,
            semantic_store_path=self.semantic_store_path,
            known_locations_path=self.known_locations_path,
            landmark_count=store.landmark_count,
            location_count=result.location_count,
            known_location_keys=keys,
            wrote_known_locations=wrote_known_locations,
        )


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)

    node = LocationBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Location bridge stopped.")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())