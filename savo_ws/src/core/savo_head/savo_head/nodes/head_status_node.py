# -*- coding: utf-8 -*-

"""Python fallback status aggregator for Robot Savo head."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from savo_head.constants import (
    STATUS_DRYRUN,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
)
from savo_head.contracts.topic_names import (
    CAMERA_STATUS,
    DASHBOARD_TEXT,
    PAN_TILT_STATE,
    SCAN_STATE,
    SEMANTIC_CONFIRMATIONS,
    STATUS,
)


@dataclass
class TopicWatch:
    name: str
    last_stamp_s: float = 0.0
    last_text: str = ""
    seen: bool = False

    def update(self, stamp_s: float, text: str = "") -> None:
        self.last_stamp_s = float(stamp_s)
        self.last_text = str(text)
        self.seen = True

    def age_s(self, now_s: float) -> float:
        if not self.seen or self.last_stamp_s <= 0.0:
            return float("inf")
        return max(0.0, float(now_s) - float(self.last_stamp_s))

    def state(self, now_s: float, stale_timeout_s: float, required: bool) -> str:
        if not self.seen:
            return STATUS_STALE if required else STATUS_DRYRUN
        if self.age_s(now_s) > float(stale_timeout_s):
            return STATUS_STALE if required else STATUS_DRYRUN
        return STATUS_OK


class HeadStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("head_status_node_py")

        self._declare_parameters()

        self._pan_tilt = TopicWatch("pan_tilt")
        self._scan = TopicWatch("scan")
        self._camera = TopicWatch("camera")
        self._semantic = TopicWatch("semantic")
        self._last_joint: Optional[JointState] = None
        self._last_error = ""

        self._pan_tilt_sub = self.create_subscription(
            JointState,
            self.get_parameter("pan_tilt_state_topic").value,
            self._on_pan_tilt_state,
            10,
        )
        self._scan_sub = self.create_subscription(
            String,
            self.get_parameter("scan_state_topic").value,
            self._on_scan_state,
            10,
        )
        self._camera_sub = self.create_subscription(
            String,
            self.get_parameter("camera_status_topic").value,
            self._on_camera_status,
            10,
        )
        self._semantic_sub = self.create_subscription(
            String,
            self.get_parameter("semantic_confirmations_topic").value,
            self._on_semantic_confirmation,
            10,
        )

        self._status_pub = self.create_publisher(
            DiagnosticArray,
            self.get_parameter("status_topic").value,
            10,
        )
        self._dashboard_pub = self.create_publisher(
            String,
            self.get_parameter("dashboard_text_topic").value,
            10,
        )

        self._health_srv = self.create_service(
            Trigger,
            self.get_parameter("health_check_service").value,
            self._on_health_service,
        )

        period = 1.0 / max(0.1, float(self.get_parameter("status_publish_hz").value))
        self._timer = self.create_timer(period, self._publish_status)

        self.get_logger().info("head status fallback node started")

    def _declare_parameters(self) -> None:
        self.declare_parameter("pan_tilt_state_topic", PAN_TILT_STATE)
        self.declare_parameter("scan_state_topic", SCAN_STATE)
        self.declare_parameter("camera_status_topic", CAMERA_STATUS)
        self.declare_parameter("semantic_confirmations_topic", SEMANTIC_CONFIRMATIONS)

        self.declare_parameter("status_topic", STATUS)
        self.declare_parameter("dashboard_text_topic", DASHBOARD_TEXT)
        self.declare_parameter("health_check_service", "/savo_head/health_check_status")

        self.declare_parameter("status_publish_hz", 2.0)
        self.declare_parameter("state_stale_timeout_s", 0.50)
        self.declare_parameter("scan_stale_timeout_s", 1.00)
        self.declare_parameter("camera_stream_stale_timeout_s", 2.00)
        self.declare_parameter("apriltag_detection_stale_timeout_s", 1.00)

        self.declare_parameter("require_pan_tilt_for_ok", True)
        self.declare_parameter("require_scan_for_ok", False)
        self.declare_parameter("require_camera_for_ok", False)
        self.declare_parameter("require_apriltag_for_ok", False)

    def _on_pan_tilt_state(self, msg: JointState) -> None:
        self._last_joint = msg
        text = self._joint_text(msg)
        self._pan_tilt.update(self._now_s(), text)

    def _on_scan_state(self, msg: String) -> None:
        self._scan.update(self._now_s(), msg.data)

    def _on_camera_status(self, msg: String) -> None:
        self._camera.update(self._now_s(), msg.data)

    def _on_semantic_confirmation(self, msg: String) -> None:
        self._semantic.update(self._now_s(), msg.data)

    def _on_health_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        health = self._compute_health()

        response.success = health["ok"]
        response.message = health["message"]

        return response

    def _publish_status(self) -> None:
        try:
            health = self._compute_health()

            status = DiagnosticStatus()
            status.name = "savo_head.head_status_py"
            status.hardware_id = "savo_head"
            status.level = DiagnosticStatus.OK if health["ok"] else DiagnosticStatus.WARN
            status.message = health["message"]

            status.values = [
                KeyValue(key="pan_tilt_state", value=health["pan_tilt_state"]),
                KeyValue(key="scan_state", value=health["scan_state"]),
                KeyValue(key="camera_state", value=health["camera_state"]),
                KeyValue(key="semantic_state", value=health["semantic_state"]),
                KeyValue(key="pan_tilt_age_s", value=f"{health['pan_tilt_age_s']:.3f}"),
                KeyValue(key="scan_age_s", value=f"{health['scan_age_s']:.3f}"),
                KeyValue(key="camera_age_s", value=f"{health['camera_age_s']:.3f}"),
                KeyValue(key="semantic_age_s", value=f"{health['semantic_age_s']:.3f}"),
                KeyValue(key="pan_tilt_text", value=self._pan_tilt.last_text),
                KeyValue(key="scan_text", value=self._scan.last_text),
                KeyValue(key="camera_text", value=self._camera.last_text),
                KeyValue(key="semantic_text", value=self._semantic.last_text),
                KeyValue(key="last_error", value=self._last_error),
            ]

            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            array.status = [status]

            self._status_pub.publish(array)

            dashboard = String()
            dashboard.data = self._dashboard_text(health)
            self._dashboard_pub.publish(dashboard)

        except Exception as exc:
            self._last_error = str(exc)
            self.get_logger().error(f"status publish failed: {exc}")

    def _compute_health(self) -> dict:
        now_s = self._now_s()

        pan_state = self._pan_tilt.state(
            now_s,
            float(self.get_parameter("state_stale_timeout_s").value),
            bool(self.get_parameter("require_pan_tilt_for_ok").value),
        )
        scan_state = self._scan.state(
            now_s,
            float(self.get_parameter("scan_stale_timeout_s").value),
            bool(self.get_parameter("require_scan_for_ok").value),
        )
        camera_state = self._camera.state(
            now_s,
            float(self.get_parameter("camera_stream_stale_timeout_s").value),
            bool(self.get_parameter("require_camera_for_ok").value),
        )
        semantic_state = self._semantic.state(
            now_s,
            float(self.get_parameter("apriltag_detection_stale_timeout_s").value),
            bool(self.get_parameter("require_apriltag_for_ok").value),
        )

        required_states = []
        if bool(self.get_parameter("require_pan_tilt_for_ok").value):
            required_states.append(pan_state)
        if bool(self.get_parameter("require_scan_for_ok").value):
            required_states.append(scan_state)
        if bool(self.get_parameter("require_camera_for_ok").value):
            required_states.append(camera_state)
        if bool(self.get_parameter("require_apriltag_for_ok").value):
            required_states.append(semantic_state)

        ok = not self._last_error and all(state == STATUS_OK for state in required_states)

        if self._last_error:
            message = STATUS_ERROR
        elif ok:
            message = STATUS_OK
        else:
            message = STATUS_STALE

        return {
            "ok": ok,
            "message": message,
            "pan_tilt_state": pan_state,
            "scan_state": scan_state,
            "camera_state": camera_state,
            "semantic_state": semantic_state,
            "pan_tilt_age_s": self._pan_tilt.age_s(now_s),
            "scan_age_s": self._scan.age_s(now_s),
            "camera_age_s": self._camera.age_s(now_s),
            "semantic_age_s": self._semantic.age_s(now_s),
        }

    def _dashboard_text(self, health: dict) -> str:
        return (
            f"savo_head status={health['message']} "
            f"pan_tilt={health['pan_tilt_state']} "
            f"scan={health['scan_state']} "
            f"camera={health['camera_state']} "
            f"semantic={health['semantic_state']}"
        )

    def _joint_text(self, msg: JointState) -> str:
        if len(msg.position) >= 2:
            return f"pan_rad={msg.position[0]:.3f};tilt_rad={msg.position[1]:.3f}"
        if len(msg.position) == 1:
            return f"pan_rad={msg.position[0]:.3f}"
        return "joint_state_empty"

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    node = HeadStatusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
