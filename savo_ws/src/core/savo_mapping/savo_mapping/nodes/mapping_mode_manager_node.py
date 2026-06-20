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


COMMAND_SEMANTIC_REVIEW = "semantic_review"

PHASE_IDLE = "idle"
PHASE_MAPPING = "mapping"
PHASE_MAP_SAVING = "map_saving"
PHASE_SEMANTIC_REVIEW = "semantic_review"

VALID_WORKFLOW_PHASES = (
    PHASE_IDLE,
    PHASE_MAPPING,
    PHASE_MAP_SAVING,
    PHASE_SEMANTIC_REVIEW,
)


@dataclass(frozen=True)
class ModeSessionState:
    mode: str
    active: bool
    workflow_phase: str = PHASE_IDLE
    map_name: Optional[str] = None
    session_id: Optional[str] = None
    previous_mode: Optional[str] = None
    previous_workflow_phase: Optional[str] = None
    command_count: int = 0
    last_command_s: Optional[float] = None
    semantic_review_active: bool = False
    semantic_candidate_key: str = ""
    semantic_actor: str = "operator"
    semantic_source: str = "cli"
    message: str = "Mapping mode manager ready."
    timestamp_s: float = field(default_factory=time.time)
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "active": self.active,
            "workflow_phase": self.workflow_phase,
            "map_name": self.map_name,
            "session_id": self.session_id,
            "previous_mode": self.previous_mode,
            "previous_workflow_phase": self.previous_workflow_phase,
            "command_count": self.command_count,
            "last_command_s": self.last_command_s,
            "semantic_review_active": self.semantic_review_active,
            "semantic_candidate_key": self.semantic_candidate_key,
            "semantic_actor": self.semantic_actor,
            "semantic_source": self.semantic_source,
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
        self.workflow_phase = self._phase_for_mode(self.mode)
        self.previous_mode: Optional[str] = None
        self.previous_workflow_phase: Optional[str] = None
        self.command_count = 0
        self.last_command_s: Optional[float] = None
        self.semantic_review_active = False
        self.semantic_candidate_key = ""
        self.semantic_actor = self.default_semantic_actor
        self.semantic_source = self.default_semantic_source
        self.last_message = "Mapping mode manager ready."
        self._last_log_state: Optional[tuple[str, str, bool, Optional[str]]] = None

        self.mode_pub = self.create_publisher(
            String,
            self.mode_topic,
            status_qos(),
        )
        self.workflow_phase_pub = self.create_publisher(
            String,
            self.workflow_phase_topic,
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
        self.declare_parameter("workflow_phase_topic", "/savo_mapping/workflow_phase")
        self.declare_parameter("session_state_topic", "/savo_mapping/session_state")
        self.declare_parameter("status_topic", "/savo_mapping/status")

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)

        self.declare_parameter("allow_idle", True)
        self.declare_parameter("allow_manual_mapping", True)
        self.declare_parameter("allow_autonomous_mapping", False)
        self.declare_parameter("allow_map_saving", True)
        self.declare_parameter("allow_semantic_review", True)

        self.declare_parameter("default_semantic_actor", "operator")
        self.declare_parameter("default_semantic_source", "cli")

        self.declare_parameter("verbose_status_log", False)

    def _load_parameters(self) -> None:
        self.default_mode = require_valid_mapping_mode(
            self.get_parameter("default_mode").value
        )

        self.map_name = str(self.get_parameter("map_name").value or "")
        self.session_id = str(self.get_parameter("session_id").value or "") or None

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.workflow_phase_topic = str(
            self.get_parameter("workflow_phase_topic").value
        )
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
        self.allow_semantic_review = bool(
            self.get_parameter("allow_semantic_review").value
        )

        self.default_semantic_actor = str(
            self.get_parameter("default_semantic_actor").value or "operator"
        )
        self.default_semantic_source = str(
            self.get_parameter("default_semantic_source").value or "cli"
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
            requested_mode, requested_phase = self._resolve_command(command)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.last_message = f"Rejected mode command: {exc}"
            self.get_logger().warning(self.last_message)
            return

        if not self._mode_allowed(requested_mode):
            self.last_message = f"Rejected mode command: {requested_mode} is disabled."
            self.get_logger().warning(self.last_message)
            return

        self.previous_mode = self.mode
        self.previous_workflow_phase = self.workflow_phase
        self.mode = requested_mode
        self.workflow_phase = requested_phase
        self.command_count += 1
        self.last_command_s = time.time()

        if "map_name" in command:
            self.map_name = str(command["map_name"] or "")

        if "session_id" in command:
            self.session_id = str(command["session_id"] or "") or None

        self._update_semantic_review_state(command)

        if self.mode == MappingMode.IDLE.value:
            self._clear_session_state()

        if self.session_id is None and self.mode != MappingMode.IDLE.value:
            self.session_id = self._make_session_id(self.mode)

        self.last_message = self._message_for_state()
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

    def _resolve_command(self, command: dict[str, Any]) -> tuple[str, str]:
        raw_mode = str(command.get("mode", "")).strip()

        if not raw_mode:
            raise ValueError("mode command is missing")

        if raw_mode == COMMAND_SEMANTIC_REVIEW:
            return self._semantic_review_mode_and_phase()

        mode = require_valid_mapping_mode(raw_mode)
        phase = str(command.get("workflow_phase") or self._phase_for_mode(mode)).strip()

        if phase not in VALID_WORKFLOW_PHASES:
            raise ValueError(f"invalid workflow_phase: {phase}")

        if phase == PHASE_SEMANTIC_REVIEW and not self.allow_semantic_review:
            raise ValueError("semantic_review phase is disabled")

        if mode == MappingMode.IDLE.value:
            phase = PHASE_IDLE

        return mode, phase

    def _semantic_review_mode_and_phase(self) -> tuple[str, str]:
        if not self.allow_semantic_review:
            raise ValueError("semantic_review command is disabled")

        if self.mode != MappingMode.IDLE.value:
            return self.mode, PHASE_SEMANTIC_REVIEW

        if self.allow_manual_mapping:
            return MappingMode.MANUAL_MAPPING.value, PHASE_SEMANTIC_REVIEW

        raise ValueError("semantic_review needs manual_mapping to be allowed")

    def _update_semantic_review_state(self, command: dict[str, Any]) -> None:
        if self.workflow_phase != PHASE_SEMANTIC_REVIEW:
            self.semantic_review_active = False
            self.semantic_candidate_key = ""
            self.semantic_actor = self.default_semantic_actor
            self.semantic_source = self.default_semantic_source
            return

        self.semantic_review_active = True
        self.semantic_candidate_key = str(
            command.get("candidate_key") or self.semantic_candidate_key or ""
        )
        self.semantic_actor = str(
            command.get("actor") or self.default_semantic_actor
        )
        self.semantic_source = str(
            command.get("source") or self.default_semantic_source
        )

    def _clear_session_state(self) -> None:
        self.session_id = None
        self.workflow_phase = PHASE_IDLE
        self.semantic_review_active = False
        self.semantic_candidate_key = ""
        self.semantic_actor = self.default_semantic_actor
        self.semantic_source = self.default_semantic_source

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

    def _phase_for_mode(self, mode: str) -> str:
        if mode == MappingMode.IDLE.value:
            return PHASE_IDLE

        if mode == MappingMode.MAP_SAVING.value:
            return PHASE_MAP_SAVING

        return PHASE_MAPPING

    def _message_for_state(self) -> str:
        if self.workflow_phase == PHASE_SEMANTIC_REVIEW:
            if self.semantic_candidate_key:
                return (
                    "Semantic review started for candidate "
                    f"{self.semantic_candidate_key}."
                )

            return "Semantic review started."

        return f"Mapping mode changed to {self.mode}."

    def _make_session_id(self, mode: str) -> str:
        safe_mode = str(mode).replace(" ", "_")
        return f"{safe_mode}_{int(time.time())}"

    def _on_timer(self) -> None:
        state = self._build_session_state()
        status = self._build_status(state)

        self.mode_pub.publish(String(data=state.mode))
        self.workflow_phase_pub.publish(String(data=state.workflow_phase))
        self.session_state_pub.publish(json_msg(state.to_dict()))
        self.status_pub.publish(json_msg(status.to_dict()))

        log_state = (
            state.mode,
            state.workflow_phase,
            state.active,
            state.session_id,
        )

        if self.verbose_status_log or log_state != self._last_log_state:
            self.get_logger().info(
                "Mapping mode state: "
                f"mode={state.mode} phase={state.workflow_phase} "
                f"active={state.active} session_id={state.session_id}"
            )

        self._last_log_state = log_state

    def _build_session_state(self) -> ModeSessionState:
        active = (
            self.mode != MappingMode.IDLE.value
            or self.workflow_phase == PHASE_SEMANTIC_REVIEW
        )

        return ModeSessionState(
            mode=self.mode,
            active=active,
            workflow_phase=self.workflow_phase,
            map_name=self.map_name or None,
            session_id=self.session_id,
            previous_mode=self.previous_mode,
            previous_workflow_phase=self.previous_workflow_phase,
            command_count=self.command_count,
            last_command_s=self.last_command_s,
            semantic_review_active=self.semantic_review_active,
            semantic_candidate_key=self.semantic_candidate_key,
            semantic_actor=self.semantic_actor,
            semantic_source=self.semantic_source,
            message=self.last_message,
            extra={
                "allowed_modes": self._allowed_modes(),
                "allowed_workflow_phases": self._allowed_workflow_phases(),
                "command_topic": self.command_topic,
                "workflow_phase_topic": self.workflow_phase_topic,
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

        if self.allow_semantic_review:
            modes.append(COMMAND_SEMANTIC_REVIEW)

        return modes

    def _allowed_workflow_phases(self) -> list[str]:
        phases = [PHASE_IDLE, PHASE_MAPPING, PHASE_MAP_SAVING]

        if self.allow_semantic_review:
            phases.append(PHASE_SEMANTIC_REVIEW)

        return phases


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