# -*- coding: utf-8 -*-

"""Python fallback dynamic TF node for Robot Savo head."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from savo_head.constants import STATUS_ERROR, STATUS_OK, STATUS_STALE
from savo_head.contracts.frame_names import (
    BASE_LINK,
    HEAD_PAN_JOINT,
    HEAD_TILT_JOINT,
    PANTILT_PAN_LINK,
    PANTILT_TILT_LINK,
    PI_CAMERA_LINK,
    PI_CAMERA_OPTICAL_FRAME,
)
from savo_head.contracts.topic_names import (
    DASHBOARD_TEXT,
    PAN_TILT_STATE,
    STATUS,
)


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_multiply(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b

    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_from_axis_angle(axis: str, angle_rad: float) -> tuple[float, float, float, float]:
    half = angle_rad * 0.5
    s = math.sin(half)
    c = math.cos(half)
    axis_norm = str(axis).strip().lower()

    if axis_norm == "x":
        return (s, 0.0, 0.0, c)
    if axis_norm == "y":
        return (0.0, s, 0.0, c)
    if axis_norm == "z":
        return (0.0, 0.0, s, c)

    raise ValueError(f"unsupported axis: {axis!r}")


def combined_rotation(
    base_rpy: list[float],
    axis: str,
    joint_angle_rad: float,
) -> tuple[float, float, float, float]:
    base = quat_from_rpy(float(base_rpy[0]), float(base_rpy[1]), float(base_rpy[2]))
    joint = quat_from_axis_angle(axis, joint_angle_rad)
    return quat_multiply(base, joint)


def make_transform(
    *,
    stamp,
    parent: str,
    child: str,
    xyz: list[float],
    quat_xyzw: tuple[float, float, float, float],
) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child

    msg.transform.translation.x = float(xyz[0])
    msg.transform.translation.y = float(xyz[1])
    msg.transform.translation.z = float(xyz[2])

    msg.transform.rotation.x = float(quat_xyzw[0])
    msg.transform.rotation.y = float(quat_xyzw[1])
    msg.transform.rotation.z = float(quat_xyzw[2])
    msg.transform.rotation.w = float(quat_xyzw[3])

    return msg


class HeadTfNode(Node):
    def __init__(self) -> None:
        super().__init__("head_tf_node_py")

        self._declare_parameters()

        self._tf_broadcaster = TransformBroadcaster(self)
        self._last_joint_state: Optional[JointState] = None
        self._last_state_s = 0.0
        self._last_error = ""

        self._state_sub = self.create_subscription(
            JointState,
            self.get_parameter("pan_tilt_state_topic").value,
            self._on_joint_state,
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

        tf_period = 1.0 / max(1.0, float(self.get_parameter("tf_rate_hz").value))
        status_period = 1.0 / max(0.1, float(self.get_parameter("status_hz").value))

        self._tf_timer = self.create_timer(tf_period, self._publish_tf)
        self._status_timer = self.create_timer(status_period, self._publish_status)

        self.get_logger().info("head TF fallback node started")

    def _declare_parameters(self) -> None:
        self.declare_parameter("pan_tilt_state_topic", PAN_TILT_STATE)
        self.declare_parameter("status_topic", STATUS)
        self.declare_parameter("dashboard_text_topic", DASHBOARD_TEXT)

        self.declare_parameter("publish_tf", True)
        self.declare_parameter("tf_rate_hz", 30.0)
        self.declare_parameter("status_hz", 2.0)

        self.declare_parameter("base_frame", BASE_LINK)
        self.declare_parameter("pan_frame", PANTILT_PAN_LINK)
        self.declare_parameter("tilt_frame", PANTILT_TILT_LINK)
        self.declare_parameter("camera_frame", PI_CAMERA_LINK)
        self.declare_parameter("camera_optical_frame", PI_CAMERA_OPTICAL_FRAME)

        self.declare_parameter("pan_joint_name", HEAD_PAN_JOINT)
        self.declare_parameter("tilt_joint_name", HEAD_TILT_JOINT)

        self.declare_parameter("pan_axis", "z")
        self.declare_parameter("tilt_axis", "y")
        self.declare_parameter("pan_sign", 1.0)
        self.declare_parameter("tilt_sign", 1.0)

        self.declare_parameter("pan_zero_deg", 72.0)
        self.declare_parameter("tilt_zero_deg", 55.0)

        self.declare_parameter("base_to_pan_xyz_m", [0.0, 0.0, 0.0])
        self.declare_parameter("base_to_pan_rpy_rad", [0.0, 0.0, 0.0])

        self.declare_parameter("pan_to_tilt_xyz_m", [0.0, 0.0, 0.0])
        self.declare_parameter("pan_to_tilt_rpy_rad", [0.0, 0.0, 0.0])

        self.declare_parameter("tilt_to_camera_xyz_m", [0.0, 0.0, 0.0])
        self.declare_parameter("tilt_to_camera_rpy_rad", [0.0, 0.0, 0.0])

        self.declare_parameter("camera_to_optical_xyz_m", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "camera_to_optical_rpy_rad",
            [-1.57079632679, 0.0, -1.57079632679],
        )

        self.declare_parameter("require_valid_pan_tilt_state", True)
        self.declare_parameter("stale_state_timeout_s", 0.50)

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_joint_state = msg
        self._last_state_s = self._now_s()
        self._last_error = ""

    def _publish_tf(self) -> None:
        if not bool(self.get_parameter("publish_tf").value):
            return

        try:
            pan_rad, tilt_rad = self._current_joint_angles()
            stamp = self.get_clock().now().to_msg()

            transforms = [
                self._base_to_pan(stamp, pan_rad),
                self._pan_to_tilt(stamp, tilt_rad),
                self._tilt_to_camera(stamp),
                self._camera_to_optical(stamp),
            ]

            self._tf_broadcaster.sendTransform(transforms)

        except Exception as exc:
            self._last_error = str(exc)

    def _current_joint_angles(self) -> tuple[float, float]:
        msg = self._last_joint_state

        if msg is None:
            if bool(self.get_parameter("require_valid_pan_tilt_state").value):
                raise RuntimeError("no pan-tilt joint state received")

            return (0.0, 0.0)

        timeout_s = float(self.get_parameter("stale_state_timeout_s").value)
        if self._now_s() - self._last_state_s > timeout_s:
            if bool(self.get_parameter("require_valid_pan_tilt_state").value):
                raise RuntimeError("pan-tilt joint state is stale")

        pan_abs = self._joint_position(msg, str(self.get_parameter("pan_joint_name").value), 0)
        tilt_abs = self._joint_position(msg, str(self.get_parameter("tilt_joint_name").value), 1)

        pan_zero = math.radians(float(self.get_parameter("pan_zero_deg").value))
        tilt_zero = math.radians(float(self.get_parameter("tilt_zero_deg").value))

        pan = float(self.get_parameter("pan_sign").value) * (pan_abs - pan_zero)
        tilt = float(self.get_parameter("tilt_sign").value) * (tilt_abs - tilt_zero)

        return pan, tilt

    def _joint_position(self, msg: JointState, name: str, fallback_index: int) -> float:
        if name in msg.name:
            index = msg.name.index(name)
            if index < len(msg.position):
                return float(msg.position[index])

        if fallback_index < len(msg.position):
            return float(msg.position[fallback_index])

        raise RuntimeError(f"joint position unavailable for {name!r}")

    def _base_to_pan(self, stamp, pan_rad: float) -> TransformStamped:
        quat = combined_rotation(
            list(self.get_parameter("base_to_pan_rpy_rad").value),
            str(self.get_parameter("pan_axis").value),
            pan_rad,
        )

        return make_transform(
            stamp=stamp,
            parent=str(self.get_parameter("base_frame").value),
            child=str(self.get_parameter("pan_frame").value),
            xyz=list(self.get_parameter("base_to_pan_xyz_m").value),
            quat_xyzw=quat,
        )

    def _pan_to_tilt(self, stamp, tilt_rad: float) -> TransformStamped:
        quat = combined_rotation(
            list(self.get_parameter("pan_to_tilt_rpy_rad").value),
            str(self.get_parameter("tilt_axis").value),
            tilt_rad,
        )

        return make_transform(
            stamp=stamp,
            parent=str(self.get_parameter("pan_frame").value),
            child=str(self.get_parameter("tilt_frame").value),
            xyz=list(self.get_parameter("pan_to_tilt_xyz_m").value),
            quat_xyzw=quat,
        )

    def _tilt_to_camera(self, stamp) -> TransformStamped:
        quat = quat_from_rpy(*list(self.get_parameter("tilt_to_camera_rpy_rad").value))

        return make_transform(
            stamp=stamp,
            parent=str(self.get_parameter("tilt_frame").value),
            child=str(self.get_parameter("camera_frame").value),
            xyz=list(self.get_parameter("tilt_to_camera_xyz_m").value),
            quat_xyzw=quat,
        )

    def _camera_to_optical(self, stamp) -> TransformStamped:
        quat = quat_from_rpy(*list(self.get_parameter("camera_to_optical_rpy_rad").value))

        return make_transform(
            stamp=stamp,
            parent=str(self.get_parameter("camera_frame").value),
            child=str(self.get_parameter("camera_optical_frame").value),
            xyz=list(self.get_parameter("camera_to_optical_xyz_m").value),
            quat_xyzw=quat,
        )

    def _publish_status(self) -> None:
        status = DiagnosticStatus()
        status.name = "savo_head.head_tf_py"
        status.hardware_id = "savo_head"

        if self._last_error:
            status.level = DiagnosticStatus.ERROR
            status.message = STATUS_ERROR
        elif self._is_state_stale():
            status.level = DiagnosticStatus.WARN
            status.message = STATUS_STALE
        else:
            status.level = DiagnosticStatus.OK
            status.message = STATUS_OK

        status.values = [
            KeyValue(key="base_frame", value=str(self.get_parameter("base_frame").value)),
            KeyValue(key="pan_frame", value=str(self.get_parameter("pan_frame").value)),
            KeyValue(key="tilt_frame", value=str(self.get_parameter("tilt_frame").value)),
            KeyValue(key="camera_frame", value=str(self.get_parameter("camera_frame").value)),
            KeyValue(
                key="camera_optical_frame",
                value=str(self.get_parameter("camera_optical_frame").value),
            ),
            KeyValue(key="last_error", value=self._last_error),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._status_pub.publish(array)

        dashboard = String()
        dashboard.data = (
            f"savo_head tf={status.message} "
            f"{self.get_parameter('base_frame').value}->"
            f"{self.get_parameter('pan_frame').value}->"
            f"{self.get_parameter('tilt_frame').value}->"
            f"{self.get_parameter('camera_frame').value}->"
            f"{self.get_parameter('camera_optical_frame').value}"
        )
        self._dashboard_pub.publish(dashboard)

    def _is_state_stale(self) -> bool:
        if self._last_joint_state is None:
            return True

        timeout_s = float(self.get_parameter("stale_state_timeout_s").value)
        return self._now_s() - self._last_state_s > timeout_s

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    node = HeadTfNode()

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
