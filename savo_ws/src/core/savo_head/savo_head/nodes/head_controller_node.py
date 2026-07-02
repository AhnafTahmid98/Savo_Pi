# -*- coding: utf-8 -*-

"""Python fallback head controller node."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from savo_head.constants import (
    HEAD_CONTROL_HZ_DEFAULT,
    HEAD_STATUS_HZ_DEFAULT,
    HEAD_WATCHDOG_TIMEOUT_S_DEFAULT,
    PAN_CENTER_DEG_DEFAULT,
    STATUS_DRYRUN,
    STATUS_ERROR,
    STATUS_OK,
    TILT_CENTER_DEG_DEFAULT,
)
from savo_head.contracts.frame_names import HEAD_PAN_JOINT, HEAD_TILT_JOINT
from savo_head.contracts.topic_names import (
    DASHBOARD_TEXT,
    DIAGNOSTICS,
    EMERGENCY_CENTER,
    PAN_TILT_CMD,
    PAN_TILT_STATE,
    STATUS,
)
from savo_head.drivers.pantilt_driver import (
    BACKEND_DRYRUN,
    BACKEND_PCA9685,
    PanTiltDriver,
    PanTiltDriverConfig,
)
from savo_head.models.pantilt_command import (
    absolute_command,
    center_command,
    delta_command,
    hold_command,
    stop_command,
)


CMD_ABSOLUTE = 0.0
CMD_DELTA = 1.0
CMD_CENTER = 2.0
CMD_HOLD = 3.0
CMD_STOP = 4.0


class HeadControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("head_controller_node_py")

        self._declare_parameters()
        self._driver = self._make_driver()
        self._last_cmd_time_s = self.get_clock().now().nanoseconds * 1e-9
        self._last_error = ""

        try:
            self._driver.open()
            self.get_logger().info(
                f"head controller fallback started with backend={self._driver.config.backend}"
            )
        except Exception as exc:
            self._last_error = str(exc)
            self.get_logger().error(f"failed to open head driver: {exc}")

        self._cmd_sub = self.create_subscription(
            Vector3,
            self.get_parameter("pan_tilt_cmd_topic").value,
            self._on_pan_tilt_cmd,
            10,
        )
        self._emergency_center_sub = self.create_subscription(
            String,
            self.get_parameter("emergency_center_topic").value,
            self._on_emergency_center,
            10,
        )

        self._joint_pub = self.create_publisher(
            JointState,
            self.get_parameter("pan_tilt_state_topic").value,
            10,
        )
        self._status_pub = self.create_publisher(
            DiagnosticArray,
            self.get_parameter("status_topic").value,
            10,
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            self.get_parameter("diagnostics_topic").value,
            10,
        )
        self._dashboard_pub = self.create_publisher(
            String,
            self.get_parameter("dashboard_text_topic").value,
            10,
        )

        self._center_srv = self.create_service(
            Trigger,
            self.get_parameter("center_service").value,
            self._on_center_service,
        )
        self._health_srv = self.create_service(
            Trigger,
            self.get_parameter("health_check_service").value,
            self._on_health_service,
        )

        control_period = 1.0 / float(self.get_parameter("control_hz").value)
        status_period = 1.0 / float(self.get_parameter("status_hz").value)

        self._control_timer = self.create_timer(control_period, self._publish_state)
        self._status_timer = self.create_timer(status_period, self._publish_status)

    def destroy_node(self) -> bool:
        try:
            if bool(self.get_parameter("center_on_shutdown").value):
                now_s = self.get_clock().now().nanoseconds * 1e-9
                self._driver.apply_command(center_command(source="shutdown", stamp_s=now_s))
        except Exception as exc:
            self.get_logger().warn(f"could not center head on shutdown: {exc}")

        try:
            self._driver.close()
        except Exception as exc:
            self.get_logger().warn(f"could not close head driver: {exc}")

        return super().destroy_node()

    def _declare_parameters(self) -> None:
        self.declare_parameter("backend", BACKEND_DRYRUN)
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("pca9685_address", 0x40)
        self.declare_parameter("pwm_frequency_hz", 50.0)

        self.declare_parameter("center_on_start", False)
        self.declare_parameter("center_on_shutdown", True)
        self.declare_parameter("stop_on_watchdog_timeout", True)

        self.declare_parameter("control_hz", HEAD_CONTROL_HZ_DEFAULT)
        self.declare_parameter("status_hz", HEAD_STATUS_HZ_DEFAULT)
        self.declare_parameter("watchdog_timeout_s", HEAD_WATCHDOG_TIMEOUT_S_DEFAULT)

        self.declare_parameter("pan_tilt_cmd_topic", PAN_TILT_CMD)
        self.declare_parameter("pan_tilt_state_topic", PAN_TILT_STATE)
        self.declare_parameter("status_topic", STATUS)
        self.declare_parameter("diagnostics_topic", DIAGNOSTICS)
        self.declare_parameter("dashboard_text_topic", DASHBOARD_TEXT)
        self.declare_parameter("emergency_center_topic", EMERGENCY_CENTER)
        self.declare_parameter("center_service", "/savo_head/center")
        self.declare_parameter("health_check_service", "/savo_head/health_check")

    def _make_driver(self) -> PanTiltDriver:
        backend = str(self.get_parameter("backend").value).strip().lower()
        if backend not in (BACKEND_PCA9685, BACKEND_DRYRUN):
            self.get_logger().warn(f"invalid backend={backend!r}; using dryrun")
            backend = BACKEND_DRYRUN

        return PanTiltDriver(
            PanTiltDriverConfig(
                backend=backend,
                i2c_bus=int(self.get_parameter("i2c_bus").value),
                pca9685_address=int(self.get_parameter("pca9685_address").value),
                pwm_frequency_hz=float(self.get_parameter("pwm_frequency_hz").value),
                center_on_open=bool(self.get_parameter("center_on_start").value),
                center_on_close=False,
            )
        )

    def _on_pan_tilt_cmd(self, msg: Vector3) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        self._last_cmd_time_s = now_s

        try:
            command = self._vector_to_command(msg, now_s)
            self._driver.apply_command(command)
            self._last_error = ""
        except Exception as exc:
            self._last_error = str(exc)
            self.get_logger().error(f"pan/tilt command failed: {exc}")

    def _on_emergency_center(self, msg: String) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        reason = msg.data.strip() or "emergency_center"

        try:
            self._driver.apply_command(center_command(source="emergency", stamp_s=now_s, reason=reason))
            self._last_cmd_time_s = now_s
            self._last_error = ""
        except Exception as exc:
            self._last_error = str(exc)
            self.get_logger().error(f"emergency center failed: {exc}")

    def _on_center_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        now_s = self.get_clock().now().nanoseconds * 1e-9

        try:
            state = self._driver.apply_command(center_command(source="service", stamp_s=now_s))
            response.success = True
            response.message = f"centered pan={state.pan_deg} tilt={state.tilt_deg}"
            self._last_error = ""
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._last_error = str(exc)

        return response

    def _on_health_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        response.success = not bool(self._last_error)
        response.message = self._health_message()
        return response

    def _vector_to_command(self, msg: Vector3, stamp_s: float):
        mode = float(msg.z)

        if math.isclose(mode, CMD_DELTA):
            return delta_command(
                pan_delta_deg=int(round(msg.x)),
                tilt_delta_deg=int(round(msg.y)),
                source="topic",
                stamp_s=stamp_s,
                reason="vector_delta",
            )

        if math.isclose(mode, CMD_CENTER):
            return center_command(source="topic", stamp_s=stamp_s, reason="vector_center")

        if math.isclose(mode, CMD_HOLD):
            return hold_command(source="topic", stamp_s=stamp_s, reason="vector_hold")

        if math.isclose(mode, CMD_STOP):
            return stop_command(source="topic", stamp_s=stamp_s, reason="vector_stop")

        return absolute_command(
            pan_deg=int(round(msg.x)),
            tilt_deg=int(round(msg.y)),
            source="topic",
            stamp_s=stamp_s,
            reason="vector_absolute",
        )

    def _publish_state(self) -> None:
        self._watchdog_check()

        state = self._driver.state
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [HEAD_PAN_JOINT, HEAD_TILT_JOINT]
        msg.position = [math.radians(float(state.pan_deg)), math.radians(float(state.tilt_deg))]
        msg.velocity = []
        msg.effort = []

        self._joint_pub.publish(msg)

    def _watchdog_check(self) -> None:
        if not bool(self.get_parameter("stop_on_watchdog_timeout").value):
            return

        now_s = self.get_clock().now().nanoseconds * 1e-9
        timeout_s = float(self.get_parameter("watchdog_timeout_s").value)

        if now_s - self._last_cmd_time_s <= timeout_s:
            return

        if self._driver.state.source == "watchdog":
            return

        try:
            self._driver.apply_command(hold_command(source="watchdog", stamp_s=now_s))
        except Exception as exc:
            self._last_error = str(exc)

    def _publish_status(self) -> None:
        status = DiagnosticStatus()
        status.name = "savo_head.head_controller_py"
        status.hardware_id = "savo_head"

        if self._last_error:
            status.level = DiagnosticStatus.ERROR
            status.message = STATUS_ERROR
        elif self._driver.config.dryrun():
            status.level = DiagnosticStatus.WARN
            status.message = STATUS_DRYRUN
        else:
            status.level = DiagnosticStatus.OK
            status.message = STATUS_OK

        state = self._driver.state
        status.values = [
            KeyValue(key="backend", value=str(self._driver.config.backend)),
            KeyValue(key="opened", value=str(self._driver.opened)),
            KeyValue(key="pan_deg", value=str(state.pan_deg)),
            KeyValue(key="tilt_deg", value=str(state.tilt_deg)),
            KeyValue(key="mode", value=str(state.mode)),
            KeyValue(key="source", value=str(state.source)),
            KeyValue(key="last_error", value=self._last_error),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]

        self._status_pub.publish(array)
        self._diagnostics_pub.publish(array)

        text = String()
        text.data = (
            f"savo_head backend={self._driver.config.backend} "
            f"pan={state.pan_deg} tilt={state.tilt_deg} "
            f"status={status.message}"
        )
        self._dashboard_pub.publish(text)

    def _health_message(self) -> str:
        state = self._driver.state
        if self._last_error:
            return f"ERROR: {self._last_error}"
        return (
            f"OK backend={self._driver.config.backend} opened={self._driver.opened} "
            f"pan={state.pan_deg} tilt={state.tilt_deg}"
        )


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    node = HeadControllerNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
