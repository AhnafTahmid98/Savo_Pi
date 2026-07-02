# -*- coding: utf-8 -*-

"""Python fallback head scan node."""

from __future__ import annotations

from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from savo_head.constants import (
    SCAN_STEP_DELAY_S_DEFAULT,
    STATUS_DRYRUN,
    STATUS_ERROR,
    STATUS_OK,
)
from savo_head.contracts.topic_names import (
    DASHBOARD_TEXT,
    PAN_TILT_CMD,
    SCAN_CMD,
    SCAN_STATE,
    STATUS,
)
from savo_head.core.scan_pattern import ScanRuntime, make_scan_runtime, profile_from_params
from savo_head.models.scan_status import (
    SCAN_STATE_DONE,
    SCAN_STATE_ERROR,
    SCAN_STATE_IDLE,
    SCAN_STATE_PAUSED,
    SCAN_STATE_RUNNING,
)


CMD_ABSOLUTE = 0.0
CMD_CENTER = 2.0
CMD_HOLD = 3.0
CMD_STOP = 4.0

SCAN_CMD_START = "start"
SCAN_CMD_STOP = "stop"
SCAN_CMD_PAUSE = "pause"
SCAN_CMD_RESUME = "resume"
SCAN_CMD_TOGGLE = "toggle"
SCAN_CMD_CENTER = "center"


class HeadScanNode(Node):
    def __init__(self) -> None:
        super().__init__("head_scan_node_py")

        self._declare_parameters()

        self._runtime = make_scan_runtime(
            profile_from_params(self._scan_params_from_ros())
        )
        self._last_error = ""

        self._pan_tilt_pub = self.create_publisher(
            Vector3,
            self.get_parameter("pan_tilt_cmd_topic").value,
            10,
        )
        self._scan_state_pub = self.create_publisher(
            String,
            self.get_parameter("scan_state_topic").value,
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

        self._scan_cmd_sub = self.create_subscription(
            String,
            self.get_parameter("scan_cmd_topic").value,
            self._on_scan_cmd,
            10,
        )

        self._start_srv = self.create_service(
            Trigger,
            self.get_parameter("start_scan_service").value,
            self._on_start_service,
        )
        self._stop_srv = self.create_service(
            Trigger,
            self.get_parameter("stop_scan_service").value,
            self._on_stop_service,
        )
        self._pause_srv = self.create_service(
            Trigger,
            self.get_parameter("pause_scan_service").value,
            self._on_pause_service,
        )
        self._resume_srv = self.create_service(
            Trigger,
            self.get_parameter("resume_scan_service").value,
            self._on_resume_service,
        )

        tick_period = max(0.01, float(self.get_parameter("semantic_scan_step_delay_s").value))
        status_period = 1.0 / max(0.1, float(self.get_parameter("status_hz").value))

        self._scan_timer = self.create_timer(tick_period, self._scan_tick)
        self._status_timer = self.create_timer(status_period, self._publish_status)

        if bool(self.get_parameter("auto_start").value):
            self._start_scan("auto_start")

        self.get_logger().info("head scan fallback node started")

    def _declare_parameters(self) -> None:
        self.declare_parameter("pan_tilt_cmd_topic", PAN_TILT_CMD)
        self.declare_parameter("scan_cmd_topic", SCAN_CMD)
        self.declare_parameter("scan_state_topic", SCAN_STATE)
        self.declare_parameter("status_topic", STATUS)
        self.declare_parameter("dashboard_text_topic", DASHBOARD_TEXT)

        self.declare_parameter("start_scan_service", "/savo_head/start_scan")
        self.declare_parameter("stop_scan_service", "/savo_head/stop_scan")
        self.declare_parameter("pause_scan_service", "/savo_head/pause_scan")
        self.declare_parameter("resume_scan_service", "/savo_head/resume_scan")

        self.declare_parameter("auto_start", False)
        self.declare_parameter("status_hz", 2.0)

        self.declare_parameter("default_scan_profile", "semantic_scan")
        self.declare_parameter("semantic_scan_enabled", True)
        self.declare_parameter("semantic_scan_mode", "staged")

        self.declare_parameter("semantic_scan_pan_min_deg", 0)
        self.declare_parameter("semantic_scan_pan_center_deg", 72)
        self.declare_parameter("semantic_scan_pan_max_deg", 170)

        self.declare_parameter("semantic_scan_tilt_min_deg", 45)
        self.declare_parameter("semantic_scan_tilt_max_deg", 130)

        self.declare_parameter("semantic_scan_pan_step_deg", 2)
        self.declare_parameter("semantic_scan_tilt_step_deg", 2)
        self.declare_parameter("semantic_scan_step_delay_s", SCAN_STEP_DELAY_S_DEFAULT)

        self.declare_parameter("semantic_scan_start_pan_deg", 0)
        self.declare_parameter("semantic_scan_start_tilt_deg", 45)

        self.declare_parameter("semantic_scan_pan_targets_deg", [72, 170, 72, 0])
        self.declare_parameter("semantic_scan_tilt_sweep_pan_targets_deg", [72])

        self.declare_parameter("semantic_scan_hold_at_pan_target_s", 0.10)
        self.declare_parameter("semantic_scan_hold_after_tilt_sweep_s", 0.15)
        self.declare_parameter("semantic_scan_pause_on_manual_command", True)
        self.declare_parameter("semantic_scan_resume_after_manual_s", 0.0)
        self.declare_parameter("semantic_scan_center_on_stop", True)

    def _scan_params_from_ros(self) -> dict:
        keys = (
            "default_scan_profile",
            "semantic_scan_enabled",
            "semantic_scan_mode",
            "semantic_scan_pan_min_deg",
            "semantic_scan_pan_center_deg",
            "semantic_scan_pan_max_deg",
            "semantic_scan_tilt_min_deg",
            "semantic_scan_tilt_max_deg",
            "semantic_scan_pan_step_deg",
            "semantic_scan_tilt_step_deg",
            "semantic_scan_step_delay_s",
            "semantic_scan_start_pan_deg",
            "semantic_scan_start_tilt_deg",
            "semantic_scan_pan_targets_deg",
            "semantic_scan_tilt_sweep_pan_targets_deg",
            "semantic_scan_hold_at_pan_target_s",
            "semantic_scan_hold_after_tilt_sweep_s",
            "semantic_scan_pause_on_manual_command",
            "semantic_scan_resume_after_manual_s",
            "semantic_scan_center_on_stop",
        )

        return {key: self.get_parameter(key).value for key in keys}

    def _on_scan_cmd(self, msg: String) -> None:
        command = msg.data.strip().lower()

        if command == SCAN_CMD_START:
            self._start_scan("topic")
        elif command == SCAN_CMD_STOP:
            self._stop_scan("topic")
        elif command == SCAN_CMD_PAUSE:
            self._pause_scan("topic")
        elif command == SCAN_CMD_RESUME:
            self._resume_scan("topic")
        elif command == SCAN_CMD_TOGGLE:
            self._toggle_scan("topic")
        elif command == SCAN_CMD_CENTER:
            self._publish_center_command("topic")
        else:
            self._last_error = f"unknown scan command: {command!r}"
            self.get_logger().warn(self._last_error)

    def _on_start_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._start_scan("service")
        response.success = self._runtime.status.state == SCAN_STATE_RUNNING
        response.message = self._state_text()
        return response

    def _on_stop_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._stop_scan("service")
        response.success = self._runtime.status.state in (SCAN_STATE_DONE, SCAN_STATE_IDLE)
        response.message = self._state_text()
        return response

    def _on_pause_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._pause_scan("service")
        response.success = self._runtime.status.state == SCAN_STATE_PAUSED
        response.message = self._state_text()
        return response

    def _on_resume_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._resume_scan("service")
        response.success = self._runtime.status.state == SCAN_STATE_RUNNING
        response.message = self._state_text()
        return response

    def _start_scan(self, source: str) -> None:
        del source
        now_s = self._now_s()
        self._runtime = make_scan_runtime(
            profile_from_params(self._scan_params_from_ros())
        ).start(stamp_s=now_s)
        self._last_error = ""
        self._publish_scan_state()

    def _stop_scan(self, source: str) -> None:
        del source
        now_s = self._now_s()

        if bool(self.get_parameter("semantic_scan_center_on_stop").value):
            self._publish_center_command("stop")

        self._runtime = self._runtime.stop(stamp_s=now_s)
        self._publish_scan_state()

    def _pause_scan(self, source: str) -> None:
        del source
        now_s = self._now_s()
        self._runtime = self._runtime.pause(stamp_s=now_s)
        self._publish_hold_command("pause")
        self._publish_scan_state()

    def _resume_scan(self, source: str) -> None:
        del source
        now_s = self._now_s()
        self._runtime = self._runtime.resume(stamp_s=now_s)
        self._publish_scan_state()

    def _toggle_scan(self, source: str) -> None:
        if self._runtime.status.state == SCAN_STATE_RUNNING:
            self._pause_scan(source)
        elif self._runtime.status.state == SCAN_STATE_PAUSED:
            self._resume_scan(source)
        else:
            self._start_scan(source)

    def _scan_tick(self) -> None:
        if self._runtime.status.state != SCAN_STATE_RUNNING:
            return

        try:
            self._runtime, result = self._runtime.step(stamp_s=self._now_s())

            msg = Vector3()
            msg.x = float(result.command.pan_deg or 0)
            msg.y = float(result.command.tilt_deg or 0)
            msg.z = CMD_ABSOLUTE
            self._pan_tilt_pub.publish(msg)

            self._last_error = ""
            self._publish_scan_state()

        except Exception as exc:
            self._last_error = str(exc)
            self._runtime = self._runtime.stop(stamp_s=self._now_s(), error=str(exc))
            self.get_logger().error(f"scan step failed: {exc}")
            self._publish_scan_state()

    def _publish_center_command(self, source: str) -> None:
        del source
        msg = Vector3()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = CMD_CENTER
        self._pan_tilt_pub.publish(msg)

    def _publish_hold_command(self, source: str) -> None:
        del source
        msg = Vector3()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = CMD_HOLD
        self._pan_tilt_pub.publish(msg)

    def _publish_stop_command(self) -> None:
        msg = Vector3()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = CMD_STOP
        self._pan_tilt_pub.publish(msg)

    def _publish_scan_state(self) -> None:
        msg = String()
        msg.data = self._state_text()
        self._scan_state_pub.publish(msg)

    def _publish_status(self) -> None:
        status = DiagnosticStatus()
        status.name = "savo_head.head_scan_py"
        status.hardware_id = "savo_head"

        if self._last_error:
            status.level = DiagnosticStatus.ERROR
            status.message = STATUS_ERROR
        elif self._runtime.status.state == SCAN_STATE_ERROR:
            status.level = DiagnosticStatus.ERROR
            status.message = STATUS_ERROR
        elif self._runtime.status.state == SCAN_STATE_RUNNING:
            status.level = DiagnosticStatus.OK
            status.message = STATUS_OK
        else:
            status.level = DiagnosticStatus.WARN
            status.message = STATUS_DRYRUN

        status.values = [
            KeyValue(key="state", value=self._runtime.status.state),
            KeyValue(key="phase", value=self._runtime.status.phase),
            KeyValue(key="pan_deg", value=str(self._runtime.status.pan_deg)),
            KeyValue(key="tilt_deg", value=str(self._runtime.status.tilt_deg)),
            KeyValue(key="target_deg", value=str(self._runtime.status.current_pan_target_deg)),
            KeyValue(key="cycle_count", value=str(self._runtime.status.cycle_count)),
            KeyValue(key="step_count", value=str(self._runtime.status.step_count)),
            KeyValue(key="last_error", value=self._last_error),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._status_pub.publish(array)

        dashboard = String()
        dashboard.data = (
            f"savo_head scan={self._runtime.status.state} "
            f"phase={self._runtime.status.phase} "
            f"pan={self._runtime.status.pan_deg} "
            f"tilt={self._runtime.status.tilt_deg}"
        )
        self._dashboard_pub.publish(dashboard)

    def _state_text(self) -> str:
        status = self._runtime.status
        return (
            f"state={status.state};phase={status.phase};"
            f"pan={status.pan_deg};tilt={status.tilt_deg};"
            f"target={status.current_pan_target_deg};"
            f"cycle={status.cycle_count};step={status.step_count};"
            f"error={self._last_error}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    node = HeadScanNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop_command()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
