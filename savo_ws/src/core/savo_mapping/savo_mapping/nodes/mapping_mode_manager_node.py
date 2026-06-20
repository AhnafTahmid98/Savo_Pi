#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Mode manager node for Robot Savo mapping."""

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
from savo_mapping.models.mapping_mode import MappingMode, require_valid_mapping_mode
from savo_mapping.models.mapping_status import make_mapping_status
from savo_mapping.models.readiness_state import build_readiness_state, make_check
from savo_mapping.ros.adapters import json_msg
from savo_mapping.ros.qos_profiles import status_qos


@dataclass(frozen=True)
class ModeSessionState:
    mode: str
    active: bool
    map_name: Optional[str] = None
    session_id: Optional[str] = None
    previous_mode: Optional[str] = None
    command_count: int = 0
    last_command_s: Optional[float] = None
    message: str = "Mapping mode manager ready."
    timestamp_s: float = field(default_factory=time.time)
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "active": self.active,
            "map_name": self.map_name,
            "session_id": self.session_id,
            "previous_mode": self.previous_mode,
            "command_count": self.command_count,
            "last_command_s": self.last_command_s,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra),
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True)


class MappingModeManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("mapping_mode_manager_node")

        self._declare_parameters()
        self._load_parameters()

        self.mode = self.default_mode
        self.previous_mode: Optional[str] = None
        self.command_count = 0
        self.last_command_s: Optional[float] = None
        self.last_message = "Mapping mode manager ready."
        self._last_log_state: Optional[tuple[str, bool, Optional[str]]] = None

        self.mode_pub = self.create_publisher(
            String,
            self.mode_topic,
            status_qos(),
        )
        self.session_state_pub = self.create_publisher(
            String,
            self.session_state_topic,
            status_qos(),
        )
        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            status_qos(),
        )

        self.command_sub = self.create_subscription(
            String,
            self.command_topic,
            self._on_mode_command,
            status_qos(),
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._on_timer,
        )

        self.get_logger().info(
            "Mapping mode manager started: "
            f"mode={self.mode} command_topic={self.command_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("default_mode", MappingMode.IDLE.value)
        self.declare_parameter("map_name", "")
        self.declare_parameter("session_id", "")

        self.declare_parameter("command_topic", "/savo_mapping/mode_command")
        self.declare_parameter("mode_topic", "/savo_mapping/mode")
        self.declare_parameter("session_state_topic", "/savo_mapping/session_state")
        self.declare_parameter("status_topic", "/savo_mapping/status")

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)

        self.declare_parameter("allow_idle", True)
        self.declare_parameter("allow_manual_mapping", True)
        self.declare_parameter("allow_autonomous_mapping", False)
        self.declare_parameter("allow_map_saving", True)

        self.declare_parameter("verbose_status_log", False)

    def _load_parameters(self) -> None:
        self.default_mode = require_valid_mapping_mode(
            self.get_parameter("default_mode").value
        )

        self.map_name = str(self.get_parameter("map_name").value or "")
        self.session_id = str(self.get_parameter("session_id").value or "") or None

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.session_state_topic = str(
            self.get_parameter("session_state_topic").value
        )
        self.status_topic = str(self.get_parameter("status_topic").value)

        self.publish_rate_hz = self._positive_float_parameter(
            "publish_rate_hz",
            DEFAULT_PUBLISH_RATE_HZ,
        )

        self.allow_idle = bool(self.get_parameter("allow_idle").value)
        self.allow_manual_mapping = bool(
            self.get_parameter("allow_manual_mapping").value
        )
        self.allow_autonomous_mapping = bool(
            self.get_parameter("allow_autonomous_mapping").value
        )
        self.allow_map_saving = bool(
            self.get_parameter("allow_map_saving").value
        )
        self.verbose_status_log = bool(
            self.get_parameter("verbose_status_log").value
        )

        if not self._mode_allowed(self.default_mode):
            self.get_logger().warning(
                f"default_mode={self.default_mode} is not allowed. Falling back to idle."
            )
            self.default_mode = MappingMode.IDLE.value

    def _positive_float_parameter(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)

        if value <= 0.0:
            self.get_logger().warning(
                f"Parameter {name} must be positive. Using {default}."
            )
            return float(default)

        return value

    def _on_mode_command(self, msg: String) -> None:
        try:
            command = self._parse_command(msg.data)
            requested_mode = require_valid_mapping_mode(command["mode"])
        except (KeyError, TypeError, ValueError) as exc:
            self.last_message = f"Rejected mode command: {exc}"
            self.get_logger().warning(self.last_message)
            return

        if not self._mode_allowed(requested_mode):
            self.last_message = f"Rejected mode command: {requested_mode} is disabled."
            self.get_logger().warning(self.last_message)
            return

        self.previous_mode = self.mode
        self.mode = requested_mode
        self.command_count += 1
        self.last_command_s = time.time()

        if "map_name" in command:
            self.map_name = str(command["map_name"] or "")

        if "session_id" in command:
            self.session_id = str(command["session_id"] or "") or None

        if self.mode == MappingMode.IDLE.value:
            self.session_id = None

        if self.session_id is None and self.mode != MappingMode.IDLE.value:
            self.session_id = self._make_session_id(self.mode)

        self.last_message = f"Mapping mode changed to {self.mode}."
        self.get_logger().info(self.last_message)

    def _parse_command(self, raw: str) -> dict[str, Any]:
        text = str(raw).strip()

        if not text:
            raise ValueError("empty command")

        if text.startswith("{"):
            data = json.loads(text)

            if not isinstance(data, dict):
                raise ValueError("JSON command must be an object")

            return data

        return {"mode": text}

    def _mode_allowed(self, mode: str) -> bool:
        if mode == MappingMode.IDLE.value:
            return self.allow_idle

        if mode == MappingMode.MANUAL_MAPPING.value:
            return self.allow_manual_mapping

        if mode == MappingMode.AUTONOMOUS_MAPPING.value:
            return self.allow_autonomous_mapping

        if mode == MappingMode.MAP_SAVING.value:
            return self.allow_map_saving

        return False

    def _make_session_id(self, mode: str) -> str:
        safe_mode = str(mode).replace(" ", "_")
        return f"{safe_mode}_{int(time.time())}"

    def _on_timer(self) -> None:
        state = self._build_session_state()
        status = self._build_status(state)

        self.mode_pub.publish(String(data=state.mode))
        self.session_state_pub.publish(json_msg(state.to_dict()))
        self.status_pub.publish(json_msg(status.to_dict()))

        log_state = (state.mode, state.active, state.session_id)

        if self.verbose_status_log or log_state != self._last_log_state:
            self.get_logger().info(
                "Mapping mode state: "
                f"mode={state.mode} active={state.active} session_id={state.session_id}"
            )

        self._last_log_state = log_state

    def _build_session_state(self) -> ModeSessionState:
        return ModeSessionState(
            mode=self.mode,
            active=self.mode != MappingMode.IDLE.value,
            map_name=self.map_name or None,
            session_id=self.session_id,
            previous_mode=self.previous_mode,
            command_count=self.command_count,
            last_command_s=self.last_command_s,
            message=self.last_message,
            extra={
                "allowed_modes": self._allowed_modes(),
                "command_topic": self.command_topic,
            },
        )

    def _build_status(self, state: ModeSessionState):
        readiness = build_readiness_state(
            (
                make_check(
                    name="mode_manager",
                    ok=True,
                    required=True,
                    enabled=True,
                    message="Mode manager is running.",
                ),
            )
        )

        return make_mapping_status(
            mode=state.mode,
            readiness=readiness,
            active=state.active,
            message=state.message,
            map_name=state.map_name,
            session_id=state.session_id,
            extra={
                "mode_manager": state.to_dict(),
            },
        )

    def _allowed_modes(self) -> list[str]:
        modes: list[str] = []

        if self.allow_idle:
            modes.append(MappingMode.IDLE.value)

        if self.allow_manual_mapping:
            modes.append(MappingMode.MANUAL_MAPPING.value)

        if self.allow_autonomous_mapping:
            modes.append(MappingMode.AUTONOMOUS_MAPPING.value)

        if self.allow_map_saving:
            modes.append(MappingMode.MAP_SAVING.value)

        return modes


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)

    node = MappingModeManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Mapping mode manager stopped.")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())