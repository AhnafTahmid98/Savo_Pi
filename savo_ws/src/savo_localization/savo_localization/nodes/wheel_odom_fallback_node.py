#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot SAVO — wheel_odom_fallback_node.py (ROS2 Jazzy, Python)
-------------------------------------------------------------
Fallback wheel odometry publisher using EncodersApi (lgpio polling).

Primary wheel odom should be the C++ node for best timing/performance.
This node is for:
- diagnostics / bringup
- emergency fallback if C++ node is disabled
- validating wiring, invert flags, and scale on real hardware

Publishes:
- /wheel/odom   (nav_msgs/Odometry)

TF:
- Does NOT publish TF by default (recommended when EKF publishes TF).
- Set publish_tf:=true only if you explicitly want odom->base_link TF from this node.

Notes:
- With only two rear encoders on a mecanum robot, this odometry is an approximation.
  It estimates vx (forward) and wz (yaw rate) from left/right wheels (diff-style).
  vy is set to 0.0 (unknown without additional sensing).
- Fuse with IMU + LiDAR SLAM/AMCL via robot_localization for navigation.

Dependencies:
- rclpy, nav_msgs, geometry_msgs, tf2_ros
- EncodersApi + EncoderKinematics in savo_localization/sensors_api/encoders_api.py
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from savo_localization.sensors_api.encoders_api import EncodersApi, EncoderKinematics


def yaw_to_quat(yaw_rad: float) -> Quaternion:
    """Planar yaw -> quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class WheelOdomFallbackNode(Node):
    def __init__(self) -> None:
        super().__init__("wheel_odom_fallback_node")

        # ---------------- Parameters ----------------
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", False)

        # Publishing + polling
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("poll_s", 0.001)           # one poll step duration (s)
        self.declare_parameter("poll_burst_ms", 10.0)     # how long to poll before each read()

        # Encoder pins
        self.declare_parameter("l_a", 21)
        self.declare_parameter("l_b", 20)
        self.declare_parameter("r_a", 12)
        self.declare_parameter("r_b", 26)

        # GPIO / debounce
        self.declare_parameter("chip", -1)  # -1 = auto
        self.declare_parameter("pullup", True)
        self.declare_parameter("use_hw_debounce", True)
        self.declare_parameter("debounce_s", 0.0003)

        # Direction (your typical wiring expects True/True)
        self.declare_parameter("invert_left", True)
        self.declare_parameter("invert_right", True)

        # Kinematics
        self.declare_parameter("wheel_dia_m", 0.065)
        self.declare_parameter("cpr", 20)
        self.declare_parameter("decoding", 4)   # 1/2/4
        self.declare_parameter("gear", 1.0)
        self.declare_parameter("track_m", 0.165)

        # Clamp (safety against spikes)
        self.declare_parameter("max_abs_v_mps", 2.0)
        self.declare_parameter("max_abs_omega_rad_s", 8.0)

        # Covariances (conservative; vy is unknown)
        self.declare_parameter("pose_cov_x", 0.05)
        self.declare_parameter("pose_cov_y", 0.05)
        self.declare_parameter("pose_cov_yaw", 0.20)
        self.declare_parameter("twist_cov_vx", 0.20)
        self.declare_parameter("twist_cov_wz", 0.40)

        # ---------------- Read parameters ----------------
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        publish_hz = float(self.get_parameter("publish_hz").value)
        self.poll_s = float(self.get_parameter("poll_s").value)
        self.poll_burst_ms = float(self.get_parameter("poll_burst_ms").value)

        l_a = int(self.get_parameter("l_a").value)
        l_b = int(self.get_parameter("l_b").value)
        r_a = int(self.get_parameter("r_a").value)
        r_b = int(self.get_parameter("r_b").value)

        chip = int(self.get_parameter("chip").value)
        chip_opt: Optional[int] = None if chip < 0 else chip

        pullup = bool(self.get_parameter("pullup").value)
        use_hw_debounce = bool(self.get_parameter("use_hw_debounce").value)
        debounce_s = float(self.get_parameter("debounce_s").value)

        invert_left = bool(self.get_parameter("invert_left").value)
        invert_right = bool(self.get_parameter("invert_right").value)

        kin = EncoderKinematics(
            wheel_dia_m=float(self.get_parameter("wheel_dia_m").value),
            cpr=int(self.get_parameter("cpr").value),
            decoding=int(self.get_parameter("decoding").value),
            gear=float(self.get_parameter("gear").value),
            track_m=float(self.get_parameter("track_m").value),
        )

        self.max_abs_v = float(self.get_parameter("max_abs_v_mps").value)
        self.max_abs_omega = float(self.get_parameter("max_abs_omega_rad_s").value)

        self.pose_cov_x = float(self.get_parameter("pose_cov_x").value)
        self.pose_cov_y = float(self.get_parameter("pose_cov_y").value)
        self.pose_cov_yaw = float(self.get_parameter("pose_cov_yaw").value)
        self.twist_cov_vx = float(self.get_parameter("twist_cov_vx").value)
        self.twist_cov_wz = float(self.get_parameter("twist_cov_wz").value)

        # ---------------- Encoders API ----------------
        self.enc = EncodersApi(
            l_a=l_a, l_b=l_b, r_a=r_a, r_b=r_b,
            chip=chip_opt,
            pullup=pullup,
            use_hw_debounce=use_hw_debounce,
            debounce_s=debounce_s,
            invert_left=invert_left,
            invert_right=invert_right,
            kin=kin,
        )

        try:
            self.enc.start()
            self.get_logger().info(
                f"[wheel_odom_fallback] Encoders started (gpiochip={self.enc.chip_used}) "
                f"L({l_a},{l_b}) R({r_a},{r_b}) invert_left={invert_left} invert_right={invert_right}"
            )
        except Exception as e:
            self.get_logger().error(f"[wheel_odom_fallback] Failed to start encoders: {e}")
            raise

        # ---------------- ROS IO ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.pub_odom = self.create_publisher(Odometry, "/wheel/odom", qos)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # Integrator state (planar)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Timer
        self.publish_hz = max(1.0, publish_hz)
        self.period = 1.0 / self.publish_hz
        self.timer = self.create_timer(self.period, self._on_timer)

        self.get_logger().warn(
            "[wheel_odom_fallback] mecanum + rear-only encoders: vy=0.0 (unknown). "
            "Fuse with IMU + SLAM/AMCL in EKF."
        )
        self.get_logger().info(
            f"[wheel_odom_fallback] Publishing /wheel/odom @ {self.publish_hz:.1f} Hz "
            f"(poll_burst_ms={self.poll_burst_ms:.1f}, poll_s={self.poll_s*1e6:.0f} us, publish_tf={self.publish_tf})"
        )

    def destroy_node(self):
        try:
            self.enc.stop()
        except Exception:
            pass
        super().destroy_node()

    @staticmethod
    def _clamp(v: float, lim: float) -> float:
        if v > lim:
            return lim
        if v < -lim:
            return -lim
        return v

    def _on_timer(self) -> None:
        """
        Poll for a short burst to capture edges, then read() once (EncodersApi.read()) and publish odom.
        """
        try:
            burst_s = max(0.0, self.poll_burst_ms / 1000.0)
            if burst_s > 0.0:
                # Poll in small steps so we don't busy-spin forever
                t_end = self.get_clock().now().nanoseconds / 1e9 + burst_s
                while (self.get_clock().now().nanoseconds / 1e9) < t_end:
                    self.enc.poll_for(self.poll_s)
            sample = self.enc.read()  # EncodersSample
        except Exception as e:
            self.get_logger().warn(f"[wheel_odom_fallback] Encoder read failed: {e}")
            return

        # Differential-style estimate (approx on mecanum)
        v = self._clamp(sample.v_mps, self.max_abs_v)
        omega = self._clamp(sample.omega_rad_s, self.max_abs_omega)

        # Integrate pose (planar, assume vy ≈ 0)
        dt = max(1e-6, float(sample.dt_s))
        self.yaw = wrap_pi(self.yaw + omega * dt)
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        # Build Odometry
        now_msg = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = now_msg
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quat(self.yaw)

        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = float(omega)

        # Covariances (conservative; vy and roll/pitch are unknown here)
        msg.pose.covariance = [0.0] * 36
        msg.twist.covariance = [0.0] * 36

        # Pose: x,y,yaw
        msg.pose.covariance[0] = self.pose_cov_x
        msg.pose.covariance[7] = self.pose_cov_y
        msg.pose.covariance[35] = self.pose_cov_yaw
        # Large for z/roll/pitch
        msg.pose.covariance[14] = 1e6
        msg.pose.covariance[21] = 1e6
        msg.pose.covariance[28] = 1e6

        # Twist: vx,wz, and huge for vy/vz/rollrate/pitchrate
        msg.twist.covariance[0] = self.twist_cov_vx
        msg.twist.covariance[35] = self.twist_cov_wz
        msg.twist.covariance[7] = 1e6   # vy unknown
        msg.twist.covariance[14] = 1e6  # vz unused
        msg.twist.covariance[21] = 1e6  # wx unused
        msg.twist.covariance[28] = 1e6  # wy unused

        self.pub_odom.publish(msg)

        # Optional TF odom->base_link
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now_msg
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelOdomFallbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()