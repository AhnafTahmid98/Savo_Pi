#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot SAVO — imu_node.py (ROS 2 Jazzy)
-------------------------------------
Publishes sensor_msgs/Imu on /imu/data using BNO055 via ImuApi.

Design goals (professional defaults):
- IMU-only mode (acc + gyro) by default (no magnetometer / no fused orientation)
- Address (0x28/0x29) is a ROS parameter so you can switch without code edits
- Robust: retry reads in ImuApi; publish at fixed rate; safe shutdown

Parameters:
- bus (int)                : I2C bus number (default: 1)
- addr (int)               : I2C address (default: 0x28). Accepts 40 or 0x28 style.
- imu_only (bool)          : True = acc+gyro only (recommended). False = NDOF (not recommended indoors)
- frame_id (string)        : TF frame id for IMU message (default: imu_link)
- topic (string)           : output topic (default: /imu/data)
- rate_hz (double)         : publish rate (default: 25.0)
- publish_orientation (bool): publish quaternion if available (default: False)
- publish_status (bool)    : log status periodically (default: True)
- status_every_s (double)  : status log period (default: 5.0)

Notes:
- If imu_only=True, orientation is not published (set to 0,0,0,1).
- Covariances are set to "unknown" (-1) by default to avoid misleading EKF.
  Later, we can tune covariances based on real data.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

# Import your API (put imu_api.py under savo_localization/sensors_api/imu_api.py)
from savo_localization.sensors_api.imu_api import ImuApi


def _to_int_addr(v) -> int:
    """
    Allow:
      - 40         (decimal)
      - "40"
      - "0x28"
    """
    if isinstance(v, int):
        return v
    s = str(v).strip().lower()
    if s.startswith("0x"):
        return int(s, 16)
    return int(s, 10)


class ImuNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_node")

        # ---------------- Parameters ----------------
        self.declare_parameter("bus", 1)
        self.declare_parameter("addr", 0x28)  # 0x28 or 0x29
        self.declare_parameter("imu_only", True)

        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("topic", "/imu/data")
        self.declare_parameter("rate_hz", 25.0)

        # Orientation is optional; for IMU-only we keep it disabled by default.
        self.declare_parameter("publish_orientation", False)

        # Optional periodic status logging (I2C extra traffic)
        self.declare_parameter("publish_status", True)
        self.declare_parameter("status_every_s", 5.0)

        bus = int(self.get_parameter("bus").value)
        addr_raw = self.get_parameter("addr").value
        imu_only = bool(self.get_parameter("imu_only").value)

        self.frame_id = str(self.get_parameter("frame_id").value)
        topic = str(self.get_parameter("topic").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.publish_orientation = bool(self.get_parameter("publish_orientation").value)

        self.publish_status = bool(self.get_parameter("publish_status").value)
        self.status_every_s = float(self.get_parameter("status_every_s").value)

        self.addr = _to_int_addr(addr_raw)

        if self.rate_hz <= 0.0:
            self.rate_hz = 25.0

        # ---------------- Publisher ----------------
        self.pub = self.create_publisher(Imu, topic, 10)

        # ---------------- IMU API ----------------
        # Professional default: imu_only=True (no magnetometer)
        self.api = ImuApi(
            bus=bus,
            addr=self.addr,
            imu_only=imu_only,
            read_retries=2,
            retry_delay_s=0.002,
            remap_fn=None,  # add later if TF axis mapping needs it
        )

        self.get_logger().info(
            f"[imu_node] Starting BNO055 IMU (bus={bus}, addr=0x{self.addr:02X}, imu_only={imu_only}) "
            f"topic={topic} frame_id={self.frame_id} rate={self.rate_hz:.1f}Hz"
        )

        # Start device
        self.api.start(do_reset=True)

        # Timer loop
        self.period_s = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(self.period_s, self._on_timer)

        # Status timer (optional)
        self._last_status_t = time.monotonic()
        if self.publish_status:
            # Log immediately (one status read)
            self._log_status_once()

    def destroy_node(self):
        # Clean shutdown
        try:
            self.api.stop()
        except Exception:
            pass
        super().destroy_node()

    def _log_status_once(self) -> None:
        try:
            st = self.api.read_status()
            self.get_logger().info(
                f"[imu_node] status: chip_ok={st.chip_ok} chip_id=0x{st.chip_id:02X} "
                f"sys_status={st.sys_status} sys_err={st.sys_err} "
                f"calib(sys,gyr,acc,mag)=({st.calib_sys},{st.calib_gyr},{st.calib_acc},{st.calib_mag}) "
                f"temp={st.temp_c:.1f}C"
            )
        except Exception as e:
            self.get_logger().warn(f"[imu_node] status read failed: {e}")

    def _fill_covariances_unknown(self, msg: Imu) -> None:
        # -1 means "covariance unknown" (ROS convention)
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

    def _on_timer(self) -> None:
        # Periodic status log
        if self.publish_status and self.status_every_s > 0.0:
            now = time.monotonic()
            if (now - self._last_status_t) >= self.status_every_s:
                self._last_status_t = now
                self._log_status_once()

        # Read sample + publish
        try:
            s = self.api.read_sample(with_status=False)
        except Exception as e:
            # Don't spam logs too hard; warn and keep running
            self.get_logger().warn(f"[imu_node] IMU read failed: {e}")
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Linear acceleration (m/s^2)
        msg.linear_acceleration.x = float(s.ax)
        msg.linear_acceleration.y = float(s.ay)
        msg.linear_acceleration.z = float(s.az)

        # Angular velocity (rad/s)
        msg.angular_velocity.x = float(s.gx)
        msg.angular_velocity.y = float(s.gy)
        msg.angular_velocity.z = float(s.gz)

        # Orientation handling (optional)
        if self.publish_orientation and (s.qw is not None):
            msg.orientation.x = float(s.qx or 0.0)
            msg.orientation.y = float(s.qy or 0.0)
            msg.orientation.z = float(s.qz or 0.0)
            msg.orientation.w = float(s.qw or 1.0)
        else:
            # Identity quaternion (not providing orientation)
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0

        # Covariances: unknown by default (safe)
        self._fill_covariances_unknown(msg)

        self.pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()