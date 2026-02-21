#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — Localization Dashboard (ROS2 Jazzy) [v2.1 Professional]
---------------------------------------------------------------------
Terminal dashboard for localization bringup/debug on the Pi.

Shows live status for:
- /imu/data                 (sensor_msgs/Imu)
- /wheel/odom               (nav_msgs/Odometry)
- /odometry/filtered        (nav_msgs/Odometry, EKF)
- /cmd_vel_safe             (geometry_msgs/Twist) [optional]
- /safety/stop              (std_msgs/Bool)       [optional]

Features:
- Curses terminal UI
- Stale detection per topic
- Message rate estimate (dashboard sampling rate)
- IMU / Odom live summaries
- IMU inferred motion state (STATIONARY / MOVING / ROTATING)
- Motion mode classification (IDLE / FORWARD / REVERSE / STRAFE_LEFT / STRAFE_RIGHT / TURN_CW / TURN_CCW / MIXED)
  using odom/cmd_vel/IMU fallbacks
- Per-motion-mode IMU statistics in Ctrl+C / q summary
- Motion mode transition log (recent events)
- Phase 1 sign-check helper (wheel / IMU / EKF hints)
- Safe rendering on small terminal windows
- Exit with Ctrl+C or q

Run:
  source /opt/ros/jazzy/setup.bash
  source ~/Savo_Pi/install/setup.bash
  ros2 run savo_localization localization_dashboard.py

Notes:
- If a topic is not running, it is shown as STALE / no data.
- This node is read-only (diagnostic only).
- IMU publisher may be 50 Hz, but dashboard sampling/rendering rate is lower (normal).
"""

import math
import time
import curses
import signal
import statistics
from dataclasses import dataclass, field
from collections import deque, defaultdict
from typing import Optional, Deque, Dict, Any, Tuple, List, DefaultDict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


# ============================================================
# Utility helpers
# ============================================================

def now_monotonic() -> float:
    return time.monotonic()


def fmt_float(x: Optional[float], width: int = 8, prec: int = 3, nan_text: str = "n/a") -> str:
    if x is None:
        return f"{nan_text:>{width}}"
    if isinstance(x, float) and (math.isnan(x) or math.isinf(x)):
        return f"{nan_text:>{width}}"
    return f"{x:{width}.{prec}f}"


def rad2deg(x: float) -> float:
    return x * 180.0 / math.pi


def mean_or_none(seq) -> Optional[float]:
    vals = list(seq)
    if not vals:
        return None
    return sum(vals) / len(vals)


def safe_fmean(vals: List[float]) -> float:
    return statistics.fmean(vals) if vals else 0.0


def quat_to_euler_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Quaternion -> roll, pitch, yaw (radians)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# ============================================================
# Topic tracking
# ============================================================

@dataclass
class TopicStats:
    name: str
    stale_timeout_s: float = 0.50
    last_rx_wall_s: Optional[float] = None
    count: int = 0
    intervals: Deque[float] = field(default_factory=lambda: deque(maxlen=300))
    first_rx_wall_s: Optional[float] = None
    last_msg_stamp: Optional[Tuple[int, int]] = None  # (sec, nsec)

    def on_message(self, msg_stamp_sec: Optional[int] = None, msg_stamp_nsec: Optional[int] = None) -> None:
        t = now_monotonic()
        if self.last_rx_wall_s is not None:
            dt = t - self.last_rx_wall_s
            if dt > 0.0:
                self.intervals.append(dt)

        self.last_rx_wall_s = t
        if self.first_rx_wall_s is None:
            self.first_rx_wall_s = t
        self.count += 1

        if msg_stamp_sec is not None and msg_stamp_nsec is not None:
            self.last_msg_stamp = (msg_stamp_sec, msg_stamp_nsec)

    def age_s(self) -> Optional[float]:
        if self.last_rx_wall_s is None:
            return None
        return now_monotonic() - self.last_rx_wall_s

    def is_stale(self) -> bool:
        age = self.age_s()
        if age is None:
            return True
        return age > self.stale_timeout_s

    def hz(self) -> Optional[float]:
        if not self.intervals:
            return None
        mean_dt = sum(self.intervals) / len(self.intervals)
        if mean_dt <= 0.0:
            return None
        return 1.0 / mean_dt


# ============================================================
# Main dashboard node
# ============================================================

class LocalizationDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("localization_dashboard")

        # -------------------------
        # Parameters: topics
        # -------------------------
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("ekf_odom_topic", "/odometry/filtered")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe")
        self.declare_parameter("safety_stop_topic", "/safety/stop")

        # -------------------------
        # Parameters: UI
        # -------------------------
        self.declare_parameter("ui_hz", 10.0)
        self.declare_parameter("compact_mode", False)

        # -------------------------
        # Parameters: stale timeouts
        # -------------------------
        self.declare_parameter("imu_stale_timeout_s", 0.50)
        self.declare_parameter("wheel_odom_stale_timeout_s", 0.50)
        self.declare_parameter("ekf_odom_stale_timeout_s", 0.50)
        self.declare_parameter("cmd_vel_stale_timeout_s", 1.00)
        self.declare_parameter("safety_stop_stale_timeout_s", 1.00)

        # -------------------------
        # Parameters: QoS toggles
        # -------------------------
        self.declare_parameter("use_best_effort_for_imu", False)
        self.declare_parameter("use_best_effort_for_wheel_odom", False)
        self.declare_parameter("use_best_effort_for_ekf_odom", False)
        self.declare_parameter("use_best_effort_for_cmd_vel", False)
        self.declare_parameter("use_best_effort_for_safety_stop", True)

        # -------------------------
        # Parameters: IMU inference thresholds
        # -------------------------
        self.declare_parameter("imu_stationary_gyro_abs_th_rad_s", 0.03)
        self.declare_parameter("imu_stationary_acc_norm_err_th_m_s2", 0.35)
        self.declare_parameter("imu_rotating_gyro_abs_th_rad_s", 0.12)

        # -------------------------
        # Parameters: motion classification thresholds
        # -------------------------
        self.declare_parameter("motion_lin_deadband_m_s", 0.03)    # for vx/vy
        self.declare_parameter("motion_ang_deadband_rad_s", 0.08)  # for wz
        self.declare_parameter("motion_mix_allow_ratio", 1.8)
        # For localization testing, EKF/wheel-first is usually more meaningful than cmd_vel-first.
        self.declare_parameter("motion_source_preference", "ekf_then_wheel_then_cmd")

        # -------------------------
        # Parameters: Phase 1 helper
        # -------------------------
        self.declare_parameter("phase1_sign_checks_enabled", True)
        self.declare_parameter("phase1_sign_check_deadband_vx", 0.03)
        self.declare_parameter("phase1_sign_check_deadband_wz", 0.08)

        # -------------------------
        # Read params
        # -------------------------
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.wheel_odom_topic = str(self.get_parameter("wheel_odom_topic").value)
        self.ekf_odom_topic = str(self.get_parameter("ekf_odom_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)

        self.ui_hz = float(self.get_parameter("ui_hz").value)
        self.compact_mode = bool(self.get_parameter("compact_mode").value)

        self.imu_stationary_gyro_abs_th = float(self.get_parameter("imu_stationary_gyro_abs_th_rad_s").value)
        self.imu_stationary_acc_err_th = float(self.get_parameter("imu_stationary_acc_norm_err_th_m_s2").value)
        self.imu_rotating_gyro_abs_th = float(self.get_parameter("imu_rotating_gyro_abs_th_rad_s").value)

        self.motion_lin_deadband = float(self.get_parameter("motion_lin_deadband_m_s").value)
        self.motion_ang_deadband = float(self.get_parameter("motion_ang_deadband_rad_s").value)
        self.motion_mix_allow_ratio = float(self.get_parameter("motion_mix_allow_ratio").value)
        self.motion_source_preference = str(self.get_parameter("motion_source_preference").value)

        self.phase1_sign_checks_enabled = bool(self.get_parameter("phase1_sign_checks_enabled").value)
        self.phase1_vx_deadband = float(self.get_parameter("phase1_sign_check_deadband_vx").value)
        self.phase1_wz_deadband = float(self.get_parameter("phase1_sign_check_deadband_wz").value)

        # -------------------------
        # Topic stats
        # -------------------------
        self.stats: Dict[str, TopicStats] = {
            "imu": TopicStats(self.imu_topic, float(self.get_parameter("imu_stale_timeout_s").value)),
            "wheel_odom": TopicStats(self.wheel_odom_topic, float(self.get_parameter("wheel_odom_stale_timeout_s").value)),
            "ekf_odom": TopicStats(self.ekf_odom_topic, float(self.get_parameter("ekf_odom_stale_timeout_s").value)),
            "cmd_vel": TopicStats(self.cmd_vel_topic, float(self.get_parameter("cmd_vel_stale_timeout_s").value)),
            "safety_stop": TopicStats(self.safety_stop_topic, float(self.get_parameter("safety_stop_stale_timeout_s").value)),
        }

        # -------------------------
        # Latest data snapshots
        # -------------------------
        self.imu_data: Dict[str, Any] = {}
        self.wheel_odom_data: Dict[str, Any] = {}
        self.ekf_odom_data: Dict[str, Any] = {}
        self.cmd_vel_data: Dict[str, Any] = {}
        self.safety_stop_data: Dict[str, Any] = {}

        # -------------------------
        # Histories for exit summary
        # -------------------------
        self.imu_gyro_z_hist: Deque[float] = deque(maxlen=20000)
        self.imu_acc_norm_hist: Deque[float] = deque(maxlen=20000)
        self.imu_ax_hist: Deque[float] = deque(maxlen=20000)
        self.imu_ay_hist: Deque[float] = deque(maxlen=20000)
        self.imu_az_hist: Deque[float] = deque(maxlen=20000)

        self.wheel_vx_hist: Deque[float] = deque(maxlen=20000)
        self.wheel_vy_hist: Deque[float] = deque(maxlen=20000)
        self.wheel_wz_hist: Deque[float] = deque(maxlen=20000)

        self.ekf_vx_hist: Deque[float] = deque(maxlen=20000)
        self.ekf_vy_hist: Deque[float] = deque(maxlen=20000)
        self.ekf_wz_hist: Deque[float] = deque(maxlen=20000)

        # Short windows for IMU inferred state
        self.imu_recent_gyro_z: Deque[float] = deque(maxlen=40)      # ~0.8s at 50Hz
        self.imu_recent_acc_norm: Deque[float] = deque(maxlen=40)

        # -------------------------
        # Motion mode tracking
        # -------------------------
        self.current_motion_mode: str = "NO_DATA"
        self.current_motion_source: str = "none"
        self.current_motion_reason: str = "waiting"
        self.current_motion_since_s: float = now_monotonic()
        self.motion_transition_log: Deque[str] = deque(maxlen=20)

        # -------------------------
        # Phase 1 sign-check status
        # -------------------------
        self.phase1_sign_status: Dict[str, str] = {
            "wheel_vx": "n/a",
            "wheel_wz": "n/a",
            "imu_gz": "n/a",
            "ekf_vx": "n/a",
            "ekf_wz": "n/a",
        }
        self.phase1_sign_reason: Dict[str, str] = {
            "wheel_vx": "waiting",
            "wheel_wz": "waiting",
            "imu_gz": "waiting",
            "ekf_vx": "waiting",
            "ekf_wz": "waiting",
        }

        # Per-mode IMU stats
        self.imu_mode_stats: DefaultDict[str, Dict[str, List[float]]] = defaultdict(
            lambda: {
                "gz": [],
                "acc_norm": [],
                "ax": [],
                "ay": [],
                "az": [],
            }
        )

        # Build QoS profiles
        imu_qos = self._make_qos(bool(self.get_parameter("use_best_effort_for_imu").value))
        wheel_qos = self._make_qos(bool(self.get_parameter("use_best_effort_for_wheel_odom").value))
        ekf_qos = self._make_qos(bool(self.get_parameter("use_best_effort_for_ekf_odom").value))
        cmd_qos = self._make_qos(bool(self.get_parameter("use_best_effort_for_cmd_vel").value))
        stop_qos = self._make_qos(bool(self.get_parameter("use_best_effort_for_safety_stop").value))

        # Subscriptions
        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.cb_imu, imu_qos)
        self.sub_wheel = self.create_subscription(Odometry, self.wheel_odom_topic, self.cb_wheel_odom, wheel_qos)
        self.sub_ekf = self.create_subscription(Odometry, self.ekf_odom_topic, self.cb_ekf_odom, ekf_qos)
        self.sub_cmd = self.create_subscription(Twist, self.cmd_vel_topic, self.cb_cmd_vel, cmd_qos)
        self.sub_stop = self.create_subscription(Bool, self.safety_stop_topic, self.cb_safety_stop, stop_qos)

        self._start_wall = now_monotonic()

        self.get_logger().info(
            "[localization_dashboard] started | "
            f"imu={self.imu_topic}, wheel={self.wheel_odom_topic}, ekf={self.ekf_odom_topic}, "
            f"cmd={self.cmd_vel_topic}, stop={self.safety_stop_topic}, compact_mode={self.compact_mode}, "
            f"motion_source_preference={self.motion_source_preference}"
        )

    # --------------------------------------------------------
    # QoS helper
    # --------------------------------------------------------
    def _make_qos(self, use_best_effort: bool) -> QoSProfile:
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT if use_best_effort else ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            durability=DurabilityPolicy.VOLATILE,
        )

    # --------------------------------------------------------
    # Callbacks
    # --------------------------------------------------------
    def cb_imu(self, msg: Imu) -> None:
        self.stats["imu"].on_message(msg.header.stamp.sec, msg.header.stamp.nanosec)

        ox, oy, oz, ow = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        orientation_valid = not (len(msg.orientation_covariance) > 0 and msg.orientation_covariance[0] < 0.0)

        roll = pitch = yaw = None
        if orientation_valid:
            try:
                roll, pitch, yaw = quat_to_euler_rpy(ox, oy, oz, ow)
            except Exception:
                roll = pitch = yaw = None

        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        acc_norm = math.sqrt(ax * ax + ay * ay + az * az)

        self.imu_recent_gyro_z.append(gz)
        self.imu_recent_acc_norm.append(acc_norm)

        imu_state, imu_reason = self._infer_imu_motion_state()

        # Motion mode (odom/cmd preferred, IMU fallback)
        mode, source, reason = self._classify_motion_mode()
        self._update_motion_mode(mode, source, reason)

        self.imu_data = {
            "frame_id": msg.header.frame_id,
            "orientation_valid": orientation_valid,
            "roll_deg": rad2deg(roll) if roll is not None else None,
            "pitch_deg": rad2deg(pitch) if pitch is not None else None,
            "yaw_deg": rad2deg(yaw) if yaw is not None else None,
            "gx": gx, "gy": gy, "gz": gz,
            "ax": ax, "ay": ay, "az": az,
            "acc_norm": acc_norm,
            "imu_motion_state": imu_state,
            "imu_motion_reason": imu_reason,
            "motion_mode": self.current_motion_mode,
            "motion_mode_source": self.current_motion_source,
            "motion_mode_reason": self.current_motion_reason,
        }

        # Global histories
        self.imu_gyro_z_hist.append(gz)
        self.imu_acc_norm_hist.append(acc_norm)
        self.imu_ax_hist.append(ax)
        self.imu_ay_hist.append(ay)
        self.imu_az_hist.append(az)

        # Per-mode IMU histories
        mode_key = self.current_motion_mode
        self.imu_mode_stats[mode_key]["gz"].append(gz)
        self.imu_mode_stats[mode_key]["acc_norm"].append(acc_norm)
        self.imu_mode_stats[mode_key]["ax"].append(ax)
        self.imu_mode_stats[mode_key]["ay"].append(ay)
        self.imu_mode_stats[mode_key]["az"].append(az)

        self._update_phase1_sign_checks()

    def cb_wheel_odom(self, msg: Odometry) -> None:
        self.stats["wheel_odom"].on_message(msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.wheel_odom_data = self._extract_odom(msg)
        self.wheel_vx_hist.append(self.wheel_odom_data["vx"])
        self.wheel_vy_hist.append(self.wheel_odom_data["vy"])
        self.wheel_wz_hist.append(self.wheel_odom_data["wz"])

        # Keep motion mode responsive even if IMU callback rate changes.
        mode, source, reason = self._classify_motion_mode()
        self._update_motion_mode(mode, source, reason)
        self._update_phase1_sign_checks()

    def cb_ekf_odom(self, msg: Odometry) -> None:
        self.stats["ekf_odom"].on_message(msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.ekf_odom_data = self._extract_odom(msg)
        self.ekf_vx_hist.append(self.ekf_odom_data["vx"])
        self.ekf_vy_hist.append(self.ekf_odom_data["vy"])
        self.ekf_wz_hist.append(self.ekf_odom_data["wz"])

        mode, source, reason = self._classify_motion_mode()
        self._update_motion_mode(mode, source, reason)
        self._update_phase1_sign_checks()

    def cb_cmd_vel(self, msg: Twist) -> None:
        self.stats["cmd_vel"].on_message()
        self.cmd_vel_data = {
            "vx": msg.linear.x,
            "vy": msg.linear.y,
            "vz": msg.linear.z,
            "wx": msg.angular.x,
            "wy": msg.angular.y,
            "wz": msg.angular.z,
        }
        mode, source, reason = self._classify_motion_mode()
        self._update_motion_mode(mode, source, reason)

    def cb_safety_stop(self, msg: Bool) -> None:
        self.stats["safety_stop"].on_message()
        self.safety_stop_data = {"stop": bool(msg.data)}

    # --------------------------------------------------------
    # IMU inferred motion state (coarse)
    # --------------------------------------------------------
    def _infer_imu_motion_state(self) -> Tuple[str, str]:
        if not self.imu_recent_gyro_z or not self.imu_recent_acc_norm:
            return "NO_DATA", "waiting for samples"

        mean_abs_gz = mean_or_none(abs(v) for v in self.imu_recent_gyro_z)
        mean_acc_norm = mean_or_none(self.imu_recent_acc_norm)
        if mean_abs_gz is None or mean_acc_norm is None:
            return "NO_DATA", "waiting for samples"

        acc_err = abs(mean_acc_norm - 9.81)

        if mean_abs_gz >= self.imu_rotating_gyro_abs_th:
            return "ROTATING", f"|gz|~{mean_abs_gz:.3f} >= {self.imu_rotating_gyro_abs_th:.3f}"

        if (mean_abs_gz <= self.imu_stationary_gyro_abs_th) and (acc_err <= self.imu_stationary_acc_err_th):
            return (
                "STATIONARY",
                f"|gz|~{mean_abs_gz:.3f} <= {self.imu_stationary_gyro_abs_th:.3f}, "
                f"| |a|-g |~{acc_err:.3f} <= {self.imu_stationary_acc_err_th:.3f}",
            )

        return "MOVING", f"|gz|~{mean_abs_gz:.3f}, | |a|-g |~{acc_err:.3f}"

    # --------------------------------------------------------
    # Odom extraction
    # --------------------------------------------------------
    def _extract_odom(self, msg: Odometry) -> Dict[str, Any]:
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        q = msg.pose.pose.orientation

        try:
            roll, pitch, yaw = quat_to_euler_rpy(q.x, q.y, q.z, q.w)
            roll_deg = rad2deg(roll)
            pitch_deg = rad2deg(pitch)
            yaw_deg = rad2deg(yaw)
        except Exception:
            roll_deg = pitch_deg = yaw_deg = float("nan")

        tw = msg.twist.twist
        vx, vy, vz = tw.linear.x, tw.linear.y, tw.linear.z
        wx, wy, wz = tw.angular.x, tw.angular.y, tw.angular.z

        return {
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "px": px, "py": py, "pz": pz,
            "roll_deg": roll_deg,
            "pitch_deg": pitch_deg,
            "yaw_deg": yaw_deg,
            "vx": vx, "vy": vy, "vz": vz,
            "wx": wx, "wy": wy, "wz": wz,
        }

    # --------------------------------------------------------
    # Motion mode classification
    # --------------------------------------------------------
    def _classify_motion_mode(self) -> Tuple[str, str, str]:
        """
        Returns (mode, source, reason)
        mode in:
          NO_DATA, IDLE, FORWARD, REVERSE, STRAFE_LEFT, STRAFE_RIGHT, TURN_CW, TURN_CCW, MIXED
        """
        if self.motion_source_preference == "cmd_then_ekf_then_wheel":
            candidates = [
                ("cmd_vel", self.cmd_vel_data if self._topic_has_fresh_data("cmd_vel") else None),
                ("ekf_odom", self.ekf_odom_data if self._topic_has_fresh_data("ekf_odom") else None),
                ("wheel_odom", self.wheel_odom_data if self._topic_has_fresh_data("wheel_odom") else None),
            ]
        elif self.motion_source_preference == "wheel_then_ekf_then_cmd":
            candidates = [
                ("wheel_odom", self.wheel_odom_data if self._topic_has_fresh_data("wheel_odom") else None),
                ("ekf_odom", self.ekf_odom_data if self._topic_has_fresh_data("ekf_odom") else None),
                ("cmd_vel", self.cmd_vel_data if self._topic_has_fresh_data("cmd_vel") else None),
            ]
        else:
            candidates = [
                ("ekf_odom", self.ekf_odom_data if self._topic_has_fresh_data("ekf_odom") else None),
                ("wheel_odom", self.wheel_odom_data if self._topic_has_fresh_data("wheel_odom") else None),
                ("cmd_vel", self.cmd_vel_data if self._topic_has_fresh_data("cmd_vel") else None),
            ]

        for source, data in candidates:
            if not data:
                continue
            mode, reason = self._classify_from_twist_like(
                vx=float(data.get("vx", 0.0)),
                vy=float(data.get("vy", 0.0)),
                wz=float(data.get("wz", 0.0)),
            )
            return mode, source, reason

        # IMU fallback (coarse only)
        if self.imu_data:
            imu_state = self.imu_data.get("imu_motion_state", "NO_DATA")
            imu_reason = self.imu_data.get("imu_motion_reason", "n/a")
            if imu_state == "STATIONARY":
                return "IDLE", "imu_fallback", imu_reason
            if imu_state == "ROTATING":
                gz = self.imu_data.get("gz")
                if isinstance(gz, (int, float)):
                    if gz > self.motion_ang_deadband:
                        return "TURN_CCW", "imu_fallback", f"gz={gz:.3f} > +{self.motion_ang_deadband:.3f}"
                    if gz < -self.motion_ang_deadband:
                        return "TURN_CW", "imu_fallback", f"gz={gz:.3f} < -{self.motion_ang_deadband:.3f}"
                return "TURN_CCW", "imu_fallback", imu_reason
            if imu_state == "MOVING":
                return "MIXED", "imu_fallback", imu_reason

        return "NO_DATA", "none", "no fresh cmd_vel/odom and no imu fallback"

    def _classify_from_twist_like(self, vx: float, vy: float, wz: float) -> Tuple[str, str]:
        lv = self.motion_lin_deadband
        av = self.motion_ang_deadband
        ratio = max(1.0, self.motion_mix_allow_ratio)

        ax = abs(vx)
        ay = abs(vy)
        aw = abs(wz)

        lin_active_x = ax > lv
        lin_active_y = ay > lv
        ang_active = aw > av

        if not lin_active_x and not lin_active_y and not ang_active:
            return "IDLE", f"|vx|,|vy|<={lv:.2f}, |wz|<={av:.2f}"

        if ang_active and not lin_active_x and not lin_active_y:
            return ("TURN_CCW", f"wz={wz:.3f} (> {av:.2f})") if wz > 0.0 else ("TURN_CW", f"wz={wz:.3f} (< -{av:.2f})")

        if (lin_active_x or lin_active_y) and not ang_active:
            if ax > lv and ax >= ratio * max(ay, 1e-9):
                return ("FORWARD", f"vx={vx:.3f} dominates vy={vy:.3f}") if vx > 0.0 else ("REVERSE", f"vx={vx:.3f} dominates vy={vy:.3f}")

            if ay > lv and ay >= ratio * max(ax, 1e-9):
                return ("STRAFE_LEFT", f"vy={vy:.3f} dominates vx={vx:.3f}") if vy > 0.0 else ("STRAFE_RIGHT", f"vy={vy:.3f} dominates vx={vx:.3f}")

            return "MIXED", f"linear mixed vx={vx:.3f}, vy={vy:.3f}"

        return "MIXED", f"vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}"

    def _topic_has_fresh_data(self, key: str) -> bool:
        st = self.stats.get(key)
        if st is None:
            return False
        return (st.count > 0) and (not st.is_stale())

    def _update_motion_mode(self, mode: str, source: str, reason: str) -> None:
        now_s = now_monotonic()

        if mode != self.current_motion_mode or source != self.current_motion_source:
            elapsed = now_s - self.current_motion_since_s
            if self.current_motion_mode != "NO_DATA":
                t_rel = now_s - self._start_wall
                self.motion_transition_log.append(
                    f"{t_rel:7.1f}s  {self.current_motion_mode} -> {mode}  ({source})  after {elapsed:.1f}s"
                )
            self.current_motion_since_s = now_s

        self.current_motion_mode = mode
        self.current_motion_source = source
        self.current_motion_reason = reason

    # --------------------------------------------------------
    # Phase 1 sign checks (soft diagnostics)
    # --------------------------------------------------------
    def _set_sign_check(self, key: str, status: str, reason: str) -> None:
        self.phase1_sign_status[key] = status
        self.phase1_sign_reason[key] = reason

    def _sign_expected_from_mode(self, mode: str, signal: str) -> Optional[int]:
        """
        Returns expected sign:
          +1 positive, -1 negative, 0 near-zero/idle, None not applicable
        """
        # signal in {"vx", "wz", "gz"}
        if mode == "IDLE":
            return 0
        if signal == "vx":
            if mode == "FORWARD":
                return +1
            if mode == "REVERSE":
                return -1
            if mode in ("TURN_CCW", "TURN_CW", "STRAFE_LEFT", "STRAFE_RIGHT", "MIXED"):
                return None
        if signal in ("wz", "gz"):
            if mode == "TURN_CCW":
                return +1
            if mode == "TURN_CW":
                return -1
            if mode in ("FORWARD", "REVERSE", "STRAFE_LEFT", "STRAFE_RIGHT", "MIXED"):
                return None
        return None

    def _eval_sign(self, value: Optional[float], expected: Optional[int], deadband: float, label: str) -> Tuple[str, str]:
        if expected is None:
            return "n/a", f"{label}: not checked in this motion mode"
        if value is None:
            return "n/a", f"{label}: no data"

        if expected == 0:
            if abs(value) <= deadband:
                return "OK", f"{label}≈0 within deadband ({abs(value):.3f} <= {deadband:.3f})"
            return "WARN", f"{label} not near zero ({value:.3f})"

        if expected > 0:
            if value > deadband:
                return "OK", f"{label}>0 ({value:.3f})"
            if value < -deadband:
                return "FAIL", f"{label}<0 ({value:.3f}), expected positive"
            return "WARN", f"{label} near zero ({value:.3f}), expected positive"

        # expected < 0
        if value < -deadband:
            return "OK", f"{label}<0 ({value:.3f})"
        if value > deadband:
            return "FAIL", f"{label}>0 ({value:.3f}), expected negative"
        return "WARN", f"{label} near zero ({value:.3f}), expected negative"

    def _update_phase1_sign_checks(self) -> None:
        if not self.phase1_sign_checks_enabled:
            return

        mode = self.current_motion_mode

        wheel_vx = self.wheel_odom_data.get("vx") if self.wheel_odom_data else None
        wheel_wz = self.wheel_odom_data.get("wz") if self.wheel_odom_data else None
        ekf_vx = self.ekf_odom_data.get("vx") if self.ekf_odom_data else None
        ekf_wz = self.ekf_odom_data.get("wz") if self.ekf_odom_data else None
        imu_gz = self.imu_data.get("gz") if self.imu_data else None

        exp_vx = self._sign_expected_from_mode(mode, "vx")
        exp_wz = self._sign_expected_from_mode(mode, "wz")
        exp_gz = self._sign_expected_from_mode(mode, "gz")

        st, rs = self._eval_sign(wheel_vx, exp_vx, self.phase1_vx_deadband, "wheel.vx")
        self._set_sign_check("wheel_vx", st, rs)

        st, rs = self._eval_sign(wheel_wz, exp_wz, self.phase1_wz_deadband, "wheel.wz")
        self._set_sign_check("wheel_wz", st, rs)

        st, rs = self._eval_sign(imu_gz, exp_gz, self.phase1_wz_deadband, "imu.gz")
        self._set_sign_check("imu_gz", st, rs)

        st, rs = self._eval_sign(ekf_vx, exp_vx, self.phase1_vx_deadband, "ekf.vx")
        self._set_sign_check("ekf_vx", st, rs)

        st, rs = self._eval_sign(ekf_wz, exp_wz, self.phase1_wz_deadband, "ekf.wz")
        self._set_sign_check("ekf_wz", st, rs)

    # --------------------------------------------------------
    # Summary printing
    # --------------------------------------------------------
    def print_summary(self) -> None:
        print("\n" + "=" * 92)
        print("Robot Savo — Localization Dashboard Summary")
        print("=" * 92)
        up = now_monotonic() - self._start_wall
        print(f"Uptime: {up:.1f} s\n")

        for key in ["imu", "wheel_odom", "ekf_odom", "cmd_vel", "safety_stop"]:
            st = self.stats[key]
            hz = st.hz()
            age = st.age_s()
            print(f"{st.name}")
            print(f"  Messages: {st.count}")
            print(f"  Rate:     {hz:.2f} Hz" if hz is not None else "  Rate:     n/a")
            print(f"  Last age: {age:.3f} s" if age is not None else "  Last age: n/a")
            print(f"  Stale:    {'YES' if st.is_stale() else 'NO'}")
            print("")

        def _describe_hist(name: str, seq: Deque[float]) -> None:
            if not seq:
                print(f"{name}: no samples")
                return
            vals = list(seq)
            mean_v = statistics.fmean(vals)
            min_v = min(vals)
            max_v = max(vals)
            std_v = statistics.pstdev(vals) if len(vals) >= 2 else 0.0
            print(f"{name}: mean={mean_v:.4f}  min={min_v:.4f}  max={max_v:.4f}  std={std_v:.4f}  n={len(vals)}")

        print("Global signal summaries")
        _describe_hist("IMU gyro z (rad/s)", self.imu_gyro_z_hist)
        _describe_hist("IMU accel norm (m/s^2)", self.imu_acc_norm_hist)
        _describe_hist("wheel odom vx (m/s)", self.wheel_vx_hist)
        _describe_hist("wheel odom vy (m/s)", self.wheel_vy_hist)
        _describe_hist("wheel odom wz (rad/s)", self.wheel_wz_hist)
        _describe_hist("EKF odom vx (m/s)", self.ekf_vx_hist)
        _describe_hist("EKF odom vy (m/s)", self.ekf_vy_hist)
        _describe_hist("EKF odom wz (rad/s)", self.ekf_wz_hist)

        if self.imu_data:
            print("\nLatest IMU inferred state:")
            print(f"  state:  {self.imu_data.get('imu_motion_state', 'n/a')}")
            print(f"  reason: {self.imu_data.get('imu_motion_reason', 'n/a')}")

        print("\nLatest classified motion mode:")
        print(f"  mode:   {self.current_motion_mode}")
        print(f"  source: {self.current_motion_source}")
        print(f"  reason: {self.current_motion_reason}")

        print("\nLatest Phase 1 sign checks:")
        for k in ["wheel_vx", "wheel_wz", "imu_gz", "ekf_vx", "ekf_wz"]:
            print(f"  {k:<9}: {self.phase1_sign_status.get(k, 'n/a'):<5}  {self.phase1_sign_reason.get(k, '')}")

        print("\nPer-motion-mode IMU summaries (grouped by classified mode)")
        mode_order = [
            "IDLE",
            "FORWARD",
            "REVERSE",
            "STRAFE_LEFT",
            "STRAFE_RIGHT",
            "TURN_CCW",
            "TURN_CW",
            "MIXED",
            "NO_DATA",
        ]
        all_modes = set(self.imu_mode_stats.keys())
        ordered_modes = [m for m in mode_order if m in all_modes] + sorted(all_modes - set(mode_order))

        if not ordered_modes:
            print("  No IMU per-mode samples.")
        else:
            for mode in ordered_modes:
                data = self.imu_mode_stats[mode]
                gz = data["gz"]
                an = data["acc_norm"]
                ax = data["ax"]
                ay = data["ay"]
                az = data["az"]
                n = len(gz)
                if n == 0:
                    continue

                def _stats(vals: List[float]) -> Tuple[float, float, float, float]:
                    if not vals:
                        return (math.nan, math.nan, math.nan, math.nan)
                    mean_v = safe_fmean(vals)
                    min_v = min(vals)
                    max_v = max(vals)
                    std_v = statistics.pstdev(vals) if len(vals) >= 2 else 0.0
                    return mean_v, min_v, max_v, std_v

                gz_m, gz_min, gz_max, gz_std = _stats(gz)
                an_m, an_min, an_max, an_std = _stats(an)
                ax_m, _, _, _ = _stats(ax)
                ay_m, _, _, _ = _stats(ay)
                az_m, _, _, _ = _stats(az)

                print(f"  [{mode}] n={n}")
                print(f"    gyro_z   : mean={gz_m:.4f}  min={gz_min:.4f}  max={gz_max:.4f}  std={gz_std:.4f}")
                print(f"    acc_norm : mean={an_m:.4f}  min={an_min:.4f}  max={an_max:.4f}  std={an_std:.4f}")
                print(f"    acc_xyz  : mean_ax={ax_m:.4f}  mean_ay={ay_m:.4f}  mean_az={az_m:.4f}")

        if self.motion_transition_log:
            print("\nRecent motion transitions")
            for line in self.motion_transition_log:
                print(f"  {line}")

        print("=" * 92 + "\n")


# ============================================================
# Curses UI
# ============================================================

class CursesUI:
    def __init__(self, node: LocalizationDashboardNode) -> None:
        self.node = node
        self.stop_requested = False

    def _safe_addstr(self, stdscr, y: int, x: int, s: str, attr: int = 0) -> None:
        h, w = stdscr.getmaxyx()
        if y < 0 or y >= h or x < 0 or x >= w:
            return
        s = s[: max(0, w - x - 1)]
        try:
            stdscr.addstr(y, x, s, attr)
        except curses.error:
            pass

    def _topic_status_line(self, label: str, st: TopicStats) -> str:
        hz = st.hz()
        age = st.age_s()
        state = "STALE" if st.is_stale() else "OK"
        hz_str = f"{hz:6.2f}Hz" if hz is not None else "   n/a "
        age_str = f"{age:5.2f}s" if age is not None else "  n/a "
        return f"{label:<14} [{state:^5}]  rate={hz_str}  age={age_str}  count={st.count}"

    def _phase1_status_line(self, key: str, label: str) -> str:
        st = self.node.phase1_sign_status.get(key, "n/a")
        rs = self.node.phase1_sign_reason.get(key, "")
        return f"{label:<10} [{st:^5}]  {rs}"

    def run(self) -> None:
        curses.wrapper(self._main)

    def _main(self, stdscr) -> None:
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(50)

        while rclpy.ok() and not self.stop_requested:
            rclpy.spin_once(self.node, timeout_sec=0.02)

            key = stdscr.getch()
            if key in (ord("q"), ord("Q")):
                self.stop_requested = True
                break

            stdscr.erase()
            h, w = stdscr.getmaxyx()

            if h < 18 or w < 90:
                self._safe_addstr(stdscr, 0, 0, "Robot Savo — Localization Dashboard", curses.A_BOLD)
                self._safe_addstr(stdscr, 2, 0, f"Terminal too small ({w}x{h}). Please enlarge window.")
                self._safe_addstr(stdscr, 4, 0, "Minimum recommended: 90 cols x 18 rows")
                self._safe_addstr(stdscr, h - 2, 0, "Press q to quit")
                stdscr.refresh()
                time.sleep(0.05)
                continue

            y = 0
            self._safe_addstr(stdscr, y, 0, "Robot Savo — Localization Dashboard (Phase 1 bringup)  [q to quit]", curses.A_BOLD)
            y += 1

            up = now_monotonic() - self.node._start_wall
            self._safe_addstr(
                stdscr,
                y,
                0,
                f"Uptime: {up:7.1f}s   Node: {self.node.get_name()}   Compact: {self.node.compact_mode}   "
                f"MotionSrcPref: {self.node.motion_source_preference}",
            )
            y += 1

            mode_age = now_monotonic() - self.node.current_motion_since_s
            self._safe_addstr(
                stdscr,
                y,
                0,
                f"Motion mode: {self.node.current_motion_mode:<12} source={self.node.current_motion_source:<12} "
                f"for {mode_age:5.1f}s",
                curses.A_BOLD,
            )
            y += 1
            self._safe_addstr(stdscr, y, 0, f"Reason: {self.node.current_motion_reason}")
            y += 2

            # Topic health
            self._safe_addstr(stdscr, y, 0, "Topic Health", curses.A_UNDERLINE)
            y += 1
            self._safe_addstr(stdscr, y, 0, self._topic_status_line("IMU", self.node.stats["imu"])); y += 1
            self._safe_addstr(stdscr, y, 0, self._topic_status_line("Wheel Odom", self.node.stats["wheel_odom"])); y += 1
            self._safe_addstr(stdscr, y, 0, self._topic_status_line("EKF Odom", self.node.stats["ekf_odom"])); y += 1
            self._safe_addstr(stdscr, y, 0, self._topic_status_line("cmd_vel_safe", self.node.stats["cmd_vel"])); y += 1
            self._safe_addstr(stdscr, y, 0, self._topic_status_line("safety_stop", self.node.stats["safety_stop"])); y += 2

            # Phase 1 sign checks
            self._safe_addstr(stdscr, y, 0, "Phase 1 Sign Checks (soft hints)", curses.A_UNDERLINE)
            y += 1
            self._safe_addstr(stdscr, y, 0, self._phase1_status_line("wheel_vx", "wheel.vx")); y += 1
            self._safe_addstr(stdscr, y, 0, self._phase1_status_line("wheel_wz", "wheel.wz")); y += 1
            self._safe_addstr(stdscr, y, 0, self._phase1_status_line("imu_gz",   "imu.gz"));   y += 1
            self._safe_addstr(stdscr, y, 0, self._phase1_status_line("ekf_vx",   "ekf.vx"));   y += 1
            self._safe_addstr(stdscr, y, 0, self._phase1_status_line("ekf_wz",   "ekf.wz"));   y += 2

            if self.node.compact_mode:
                imu = self.node.imu_data
                wod = self.node.wheel_odom_data
                eod = self.node.ekf_odom_data

                self._safe_addstr(stdscr, y, 0, "Compact Summary", curses.A_UNDERLINE); y += 1

                if imu:
                    self._safe_addstr(
                        stdscr, y, 0,
                        f"IMU: coarse={imu.get('imu_motion_state','n/a'):<10} "
                        f"mode={imu.get('motion_mode','n/a'):<12} "
                        f"gz={fmt_float(imu.get('gz'),7,3)}  |a|={fmt_float(imu.get('acc_norm'),7,3)}"
                    )
                else:
                    self._safe_addstr(stdscr, y, 0, "IMU: no data")
                y += 1

                if wod:
                    self._safe_addstr(
                        stdscr, y, 0,
                        f"Wheel: x={fmt_float(wod.get('px'),7,3)} y={fmt_float(wod.get('py'),7,3)} "
                        f"yaw={fmt_float(wod.get('yaw_deg'),7,2)}deg  "
                        f"vx={fmt_float(wod.get('vx'),7,3)} vy={fmt_float(wod.get('vy'),7,3)} wz={fmt_float(wod.get('wz'),7,3)}"
                    )
                else:
                    self._safe_addstr(stdscr, y, 0, "Wheel: no data")
                y += 1

                if eod:
                    self._safe_addstr(
                        stdscr, y, 0,
                        f"EKF:   x={fmt_float(eod.get('px'),7,3)} y={fmt_float(eod.get('py'),7,3)} "
                        f"yaw={fmt_float(eod.get('yaw_deg'),7,2)}deg  "
                        f"vx={fmt_float(eod.get('vx'),7,3)} vy={fmt_float(eod.get('vy'),7,3)} wz={fmt_float(eod.get('wz'),7,3)}"
                    )
                else:
                    self._safe_addstr(stdscr, y, 0, "EKF: no data")
                y += 2

            else:
                # IMU block
                if y < h - 8:
                    self._safe_addstr(stdscr, y, 0, "IMU (/imu/data)", curses.A_UNDERLINE); y += 1
                    imu = self.node.imu_data
                    if imu:
                        self._safe_addstr(stdscr, y, 0, f"frame_id: {imu.get('frame_id', '')}"); y += 1
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"orientation: {'VALID' if imu.get('orientation_valid', False) else 'NOT PROVIDED'}   "
                            f"roll={fmt_float(imu.get('roll_deg'),7,2)} deg  "
                            f"pitch={fmt_float(imu.get('pitch_deg'),7,2)} deg  "
                            f"yaw={fmt_float(imu.get('yaw_deg'),7,2)} deg"
                        ); y += 1
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"gyro [rad/s]  x={fmt_float(imu.get('gx'))}  y={fmt_float(imu.get('gy'))}  z={fmt_float(imu.get('gz'))}"
                        ); y += 1
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"acc  [m/s^2]  x={fmt_float(imu.get('ax'))}  y={fmt_float(imu.get('ay'))}  z={fmt_float(imu.get('az'))}  "
                            f"|a|={fmt_float(imu.get('acc_norm'))}"
                        ); y += 1
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"IMU coarse state: {imu.get('imu_motion_state','n/a')}   ({imu.get('imu_motion_reason','')})"
                        ); y += 1
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"Classified mode : {imu.get('motion_mode','n/a')} via {imu.get('motion_mode_source','n/a')} "
                            f"({imu.get('motion_mode_reason','')})"
                        ); y += 1
                    else:
                        self._safe_addstr(stdscr, y, 0, "No IMU data received yet."); y += 1
                    y += 1

                # Wheel odom block
                if y < h - 8:
                    self._safe_addstr(stdscr, y, 0, "Wheel Odometry (/wheel/odom)", curses.A_UNDERLINE); y += 1
                    wod = self.node.wheel_odom_data
                    if wod:
                        self._safe_addstr(stdscr, y, 0, f"frame={wod.get('frame_id','')}  child={wod.get('child_frame_id','')}"); y += 1
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"pose : x={fmt_float(wod.get('px'))}  y={fmt_float(wod.get('py'))}  yaw={fmt_float(wod.get('yaw_deg'),7,2)} deg"
                        ); y += 1
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"twist: vx={fmt_float(wod.get('vx'))}  vy={fmt_float(wod.get('vy'))}  wz={fmt_float(wod.get('wz'))}"
                        ); y += 1
                    else:
                        self._safe_addstr(stdscr, y, 0, "No /wheel/odom data received yet."); y += 1
                    y += 1

                # EKF odom block
                if y < h - 8:
                    self._safe_addstr(stdscr, y, 0, "EKF Odometry (/odometry/filtered)", curses.A_UNDERLINE); y += 1
                    eod = self.node.ekf_odom_data
                    if eod:
                        self._safe_addstr(stdscr, y, 0, f"frame={eod.get('frame_id','')}  child={eod.get('child_frame_id','')}"); y += 1
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"pose : x={fmt_float(eod.get('px'))}  y={fmt_float(eod.get('py'))}  yaw={fmt_float(eod.get('yaw_deg'),7,2)} deg"
                        ); y += 1
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"twist: vx={fmt_float(eod.get('vx'))}  vy={fmt_float(eod.get('vy'))}  wz={fmt_float(eod.get('wz'))}"
                        ); y += 1
                    else:
                        self._safe_addstr(stdscr, y, 0, "No /odometry/filtered data received yet."); y += 1
                    y += 1

                # Motion + safety
                if y < h - 8:
                    self._safe_addstr(stdscr, y, 0, "Motion + Safety", curses.A_UNDERLINE); y += 1
                    cmd = self.node.cmd_vel_data
                    stp = self.node.safety_stop_data

                    if cmd:
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"/cmd_vel_safe: vx={fmt_float(cmd.get('vx'))}  vy={fmt_float(cmd.get('vy'))}  wz={fmt_float(cmd.get('wz'))}"
                        ); y += 1
                    else:
                        self._safe_addstr(stdscr, y, 0, "/cmd_vel_safe: no data"); y += 1

                    if stp:
                        self._safe_addstr(stdscr, y, 0, f"/safety/stop: {'TRUE (STOP)' if stp.get('stop') else 'false'}"); y += 1
                    else:
                        self._safe_addstr(stdscr, y, 0, "/safety/stop: no data"); y += 1
                    y += 1

                # Quick per-mode counts
                if y < h - 6:
                    self._safe_addstr(stdscr, y, 0, "Per-mode IMU sample counts (current session)", curses.A_UNDERLINE); y += 1
                    counts_line = []
                    for mode in ["IDLE", "FORWARD", "REVERSE", "STRAFE_LEFT", "STRAFE_RIGHT", "TURN_CCW", "TURN_CW", "MIXED"]:
                        n = len(self.node.imu_mode_stats.get(mode, {}).get("gz", [])) if mode in self.node.imu_mode_stats else 0
                        counts_line.append(f"{mode}:{n}")
                    self._safe_addstr(stdscr, y, 0, " | ".join(counts_line)); y += 1

            footer1 = "Phase 1 live script: 10s IDLE -> 5s FWD -> 5s REV -> 5s TURN_CCW -> 5s TURN_CW -> q"
            footer2 = "Verify separately (EKF mode): ros2 run tf2_ros tf2_echo odom base_link"
            self._safe_addstr(stdscr, h - 3, 0, footer1)
            self._safe_addstr(stdscr, h - 2, 0, footer2)

            stdscr.refresh()
            time.sleep(max(0.0, 1.0 / max(self.node.ui_hz, 1.0) - 0.01))


# ============================================================
# main
# ============================================================

def main() -> None:
    rclpy.init()
    node = LocalizationDashboardNode()
    ui = CursesUI(node)

    def _handle_sigint(signum, frame):
        ui.stop_requested = True

    signal.signal(signal.SIGINT, _handle_sigint)

    try:
        ui.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.print_summary()
        except Exception as e:
            print(f"[localization_dashboard] summary print failed: {e}")
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()