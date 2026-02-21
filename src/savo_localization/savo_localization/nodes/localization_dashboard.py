#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — Localization Dashboard (ROS2 Jazzy) [v1.1 Professional]
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
- Message rate estimate (Hz)
- Basic IMU / Odom summaries
- IMU inferred motion state (STATIONARY / MOVING / ROTATING)
- Safe rendering on small terminal windows
- Exit with Ctrl+C or q
- Prints summary statistics on exit (great for test sessions)

Run:
  source /opt/ros/jazzy/setup.bash
  source ~/Savo_Pi/install/setup.bash
  ros2 run savo_localization localization_dashboard

Notes:
- If a topic is not running, it is shown as STALE / no data.
- This node is read-only (diagnostic only).
- Default IMU QoS is RELIABLE (matches your current imu_node).
"""

import math
import time
import curses
import signal
import statistics
from dataclasses import dataclass, field
from collections import deque
from typing import Optional, Deque, Dict, Any, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


# -----------------------------
# Utility helpers
# -----------------------------

def now_monotonic() -> float:
    return time.monotonic()


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def fmt_float(x: Optional[float], width: int = 8, prec: int = 3, nan_text: str = "n/a") -> str:
    if x is None:
        return f"{nan_text:>{width}}"
    if isinstance(x, float) and (math.isnan(x) or math.isinf(x)):
        return f"{nan_text:>{width}}"
    return f"{x:{width}.{prec}f}"


def rad2deg(x: float) -> float:
    return x * 180.0 / math.pi


def quat_to_euler_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Quaternion -> roll, pitch, yaw (radians)."""
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def mean_or_none(seq: Deque[float]) -> Optional[float]:
    if not seq:
        return None
    return sum(seq) / len(seq)


# -----------------------------
# Topic tracking
# -----------------------------

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

    def total_runtime_s(self) -> Optional[float]:
        if self.first_rx_wall_s is None or self.last_rx_wall_s is None:
            return None
        return max(0.0, self.last_rx_wall_s - self.first_rx_wall_s)

    def summary(self) -> str:
        hz = self.hz()
        age = self.age_s()
        runtime = self.total_runtime_s()

        hz_part = f"hz~{hz:.2f}" if hz is not None else "hz~n/a"
        age_part = f"age={age:.3f}s" if age is not None else "age=n/a"
        runtime_part = f", runtime={runtime:.1f}s" if runtime is not None else ""

        return f"{self.name}: count={self.count}, {hz_part}, {age_part}{runtime_part}"


# -----------------------------
# Main dashboard node
# -----------------------------

class LocalizationDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("localization_dashboard")

        # ---- Parameters: topics ----
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("ekf_odom_topic", "/odometry/filtered")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe")
        self.declare_parameter("safety_stop_topic", "/safety/stop")

        # ---- Parameters: UI ----
        self.declare_parameter("ui_hz", 10.0)
        self.declare_parameter("compact_mode", False)

        # ---- Parameters: stale timeouts ----
        self.declare_parameter("imu_stale_timeout_s", 0.50)
        self.declare_parameter("wheel_odom_stale_timeout_s", 0.50)
        self.declare_parameter("ekf_odom_stale_timeout_s", 0.50)
        self.declare_parameter("cmd_vel_stale_timeout_s", 1.00)
        self.declare_parameter("safety_stop_stale_timeout_s", 1.00)

        # ---- Parameters: QoS toggles ----
        # Keep defaults aligned with typical publishers in your stack
        self.declare_parameter("use_best_effort_for_imu", False)          # your imu_node is reliable
        self.declare_parameter("use_best_effort_for_wheel_odom", False)   # C++ wheel odom usually reliable
        self.declare_parameter("use_best_effort_for_ekf_odom", False)     # robot_localization usually reliable
        self.declare_parameter("use_best_effort_for_cmd_vel", False)      # cmd_vel_safe often reliable
        self.declare_parameter("use_best_effort_for_safety_stop", True)   # Python publishers often BE

        # ---- Parameters: IMU inference thresholds ----
        self.declare_parameter("imu_stationary_gyro_abs_th_rad_s", 0.03)
        self.declare_parameter("imu_stationary_acc_norm_err_th_m_s2", 0.35)
        self.declare_parameter("imu_rotating_gyro_abs_th_rad_s", 0.12)

        # ---- Read params ----
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

        # Topic stats
        self.stats: Dict[str, TopicStats] = {
            "imu": TopicStats(self.imu_topic, float(self.get_parameter("imu_stale_timeout_s").value)),
            "wheel_odom": TopicStats(self.wheel_odom_topic, float(self.get_parameter("wheel_odom_stale_timeout_s").value)),
            "ekf_odom": TopicStats(self.ekf_odom_topic, float(self.get_parameter("ekf_odom_stale_timeout_s").value)),
            "cmd_vel": TopicStats(self.cmd_vel_topic, float(self.get_parameter("cmd_vel_stale_timeout_s").value)),
            "safety_stop": TopicStats(self.safety_stop_topic, float(self.get_parameter("safety_stop_stale_timeout_s").value)),
        }

        # Latest data snapshots
        self.imu_data: Dict[str, Any] = {}
        self.wheel_odom_data: Dict[str, Any] = {}
        self.ekf_odom_data: Dict[str, Any] = {}
        self.cmd_vel_data: Dict[str, Any] = {}
        self.safety_stop_data: Dict[str, Any] = {}

        # Histories for exit summary (and smoothing/inspection)
        self.imu_gyro_z_hist: Deque[float] = deque(maxlen=5000)
        self.imu_acc_norm_hist: Deque[float] = deque(maxlen=5000)
        self.wheel_vx_hist: Deque[float] = deque(maxlen=5000)
        self.wheel_wz_hist: Deque[float] = deque(maxlen=5000)
        self.ekf_vx_hist: Deque[float] = deque(maxlen=5000)
        self.ekf_wz_hist: Deque[float] = deque(maxlen=5000)

        # Short recent windows for inferred state
        self.imu_recent_gyro_z: Deque[float] = deque(maxlen=40)     # ~0.8s at 50Hz
        self.imu_recent_acc_norm: Deque[float] = deque(maxlen=40)

        # Build QoS profiles
        imu_qos = self._make_qos(use_best_effort=bool(self.get_parameter("use_best_effort_for_imu").value))
        wheel_qos = self._make_qos(use_best_effort=bool(self.get_parameter("use_best_effort_for_wheel_odom").value))
        ekf_qos = self._make_qos(use_best_effort=bool(self.get_parameter("use_best_effort_for_ekf_odom").value))
        cmd_qos = self._make_qos(use_best_effort=bool(self.get_parameter("use_best_effort_for_cmd_vel").value))
        stop_qos = self._make_qos(use_best_effort=bool(self.get_parameter("use_best_effort_for_safety_stop").value))

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
            f"cmd={self.cmd_vel_topic}, stop={self.safety_stop_topic}, compact_mode={self.compact_mode}"
        )

    def _make_qos(self, use_best_effort: bool) -> QoSProfile:
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT if use_best_effort else ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            durability=DurabilityPolicy.VOLATILE,
        )

    # ---- callbacks ----

    def cb_imu(self, msg: Imu) -> None:
        self.stats["imu"].on_message(msg.header.stamp.sec, msg.header.stamp.nanosec)

        ox, oy, oz, ow = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        orientation_valid = not (
            len(msg.orientation_covariance) > 0 and msg.orientation_covariance[0] < 0.0
        )

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

        motion_state, motion_reason = self._infer_imu_motion_state()

        self.imu_data = {
            "frame_id": msg.header.frame_id,
            "orientation_valid": orientation_valid,
            "roll_deg": rad2deg(roll) if roll is not None else None,
            "pitch_deg": rad2deg(pitch) if pitch is not None else None,
            "yaw_deg": rad2deg(yaw) if yaw is not None else None,
            "gx": gx, "gy": gy, "gz": gz,
            "ax": ax, "ay": ay, "az": az,
            "acc_norm": acc_norm,
            "motion_state": motion_state,
            "motion_reason": motion_reason,
        }

        self.imu_gyro_z_hist.append(gz)
        self.imu_acc_norm_hist.append(acc_norm)

    def _infer_imu_motion_state(self) -> Tuple[str, str]:
        """
        Simple and practical IMU motion inference for diagnostics.
        Uses gyro z and accel norm only (works even without orientation).
        """
        if not self.imu_recent_gyro_z or not self.imu_recent_acc_norm:
            return "NO_DATA", "waiting for samples"

        mean_abs_gz = mean_or_none(deque(abs(v) for v in self.imu_recent_gyro_z))
        mean_acc_norm = mean_or_none(self.imu_recent_acc_norm)

        if mean_abs_gz is None or mean_acc_norm is None:
            return "NO_DATA", "waiting for samples"

        acc_err = abs(mean_acc_norm - 9.81)

        if mean_abs_gz >= self.imu_rotating_gyro_abs_th:
            return "ROTATING", f"|gz|~{mean_abs_gz:.3f} >= {self.imu_rotating_gyro_abs_th:.3f}"

        if (mean_abs_gz <= self.imu_stationary_gyro_abs_th) and (acc_err <= self.imu_stationary_acc_err_th):
            return "STATIONARY", (
                f"|gz|~{mean_abs_gz:.3f} <= {self.imu_stationary_gyro_abs_th:.3f}, "
                f"| |a|-g |~{acc_err:.3f} <= {self.imu_stationary_acc_err_th:.3f}"
            )

        return "MOVING", (
            f"|gz|~{mean_abs_gz:.3f}, | |a|-g |~{acc_err:.3f}"
        )

    def _extract_odom(self, msg: Odometry) -> Dict[str, Any]:
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_euler_rpy(q.x, q.y, q.z, q.w)

        tw = msg.twist.twist
        vx, vy, vz = tw.linear.x, tw.linear.y, tw.linear.z
        wx, wy, wz = tw.angular.x, tw.angular.y, tw.angular.z

        return {
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "px": px, "py": py, "pz": pz,
            "roll_deg": rad2deg(roll),
            "pitch_deg": rad2deg(pitch),
            "yaw_deg": rad2deg(yaw),
            "vx": vx, "vy": vy, "vz": vz,
            "wx": wx, "wy": wy, "wz": wz,
        }

    def cb_wheel_odom(self, msg: Odometry) -> None:
        self.stats["wheel_odom"].on_message(msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.wheel_odom_data = self._extract_odom(msg)
        self.wheel_vx_hist.append(self.wheel_odom_data["vx"])
        self.wheel_wz_hist.append(self.wheel_odom_data["wz"])

    def cb_ekf_odom(self, msg: Odometry) -> None:
        self.stats["ekf_odom"].on_message(msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.ekf_odom_data = self._extract_odom(msg)
        self.ekf_vx_hist.append(self.ekf_odom_data["vx"])
        self.ekf_wz_hist.append(self.ekf_odom_data["wz"])

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

    def cb_safety_stop(self, msg: Bool) -> None:
        self.stats["safety_stop"].on_message()
        self.safety_stop_data = {"stop": bool(msg.data)}

    # ---- summary on exit ----

    def print_summary(self) -> None:
        print("\n" + "=" * 78)
        print("Robot Savo — Localization Dashboard Summary")
        print("=" * 78)
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

        print("Signal summaries")
        _describe_hist("IMU gyro z (rad/s)", self.imu_gyro_z_hist)
        _describe_hist("IMU accel norm (m/s^2)", self.imu_acc_norm_hist)
        _describe_hist("wheel odom vx (m/s)", self.wheel_vx_hist)
        _describe_hist("wheel odom wz (rad/s)", self.wheel_wz_hist)
        _describe_hist("EKF odom vx (m/s)", self.ekf_vx_hist)
        _describe_hist("EKF odom wz (rad/s)", self.ekf_wz_hist)

        if self.imu_data:
            print("\nLatest IMU inferred state:")
            print(f"  state:  {self.imu_data.get('motion_state', 'n/a')}")
            print(f"  reason: {self.imu_data.get('motion_reason', 'n/a')}")

        print("=" * 78 + "\n")


# -----------------------------
# Curses UI
# -----------------------------

class CursesUI:
    def __init__(self, node: LocalizationDashboardNode) -> None:
        self.node = node
        self.stop_requested = False

    def _safe_addstr(self, stdscr, y: int, x: int, s: str, attr: int = 0) -> None:
        h, w = stdscr.getmaxyx()
        if y < 0 or y >= h or x < 0 or x >= w:
            return
        s = s[:max(0, w - x - 1)]
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

            # Minimum terminal size guard
            if h < 16 or w < 70:
                self._safe_addstr(stdscr, 0, 0, "Robot Savo — Localization Dashboard", curses.A_BOLD)
                self._safe_addstr(stdscr, 2, 0, f"Terminal too small ({w}x{h}). Please enlarge window.")
                self._safe_addstr(stdscr, 4, 0, "Minimum recommended: 70 cols x 16 rows")
                self._safe_addstr(stdscr, h - 2, 0, "Press q to quit")
                stdscr.refresh()
                time.sleep(0.05)
                continue

            y = 0
            self._safe_addstr(stdscr, y, 0, "Robot Savo — Localization Dashboard (q to quit)", curses.A_BOLD)
            y += 1

            up = now_monotonic() - self.node._start_wall
            self._safe_addstr(
                stdscr, y, 0,
                f"Uptime: {up:7.1f}s   Node: {self.node.get_name()}   Compact: {self.node.compact_mode}"
            )
            y += 2

            # Topic health
            self._safe_addstr(stdscr, y, 0, "Topic Health", curses.A_UNDERLINE)
            y += 1
            self._safe_addstr(stdscr, y, 0, self._topic_status_line("IMU", self.node.stats["imu"])); y += 1
            self._safe_addstr(stdscr, y, 0, self._topic_status_line("Wheel Odom", self.node.stats["wheel_odom"])); y += 1
            self._safe_addstr(stdscr, y, 0, self._topic_status_line("EKF Odom", self.node.stats["ekf_odom"])); y += 1
            self._safe_addstr(stdscr, y, 0, self._topic_status_line("cmd_vel_safe", self.node.stats["cmd_vel"])); y += 1
            self._safe_addstr(stdscr, y, 0, self._topic_status_line("safety_stop", self.node.stats["safety_stop"])); y += 2

            # Compact mode only shows health + one-line summaries
            if self.node.compact_mode:
                imu = self.node.imu_data
                wod = self.node.wheel_odom_data
                eod = self.node.ekf_odom_data

                self._safe_addstr(stdscr, y, 0, "Compact Summary", curses.A_UNDERLINE); y += 1
                if imu:
                    self._safe_addstr(
                        stdscr, y, 0,
                        f"IMU: state={imu.get('motion_state','n/a')}  gz={fmt_float(imu.get('gz'),7,3)}  |a|={fmt_float(imu.get('acc_norm'),7,3)}"
                    )
                else:
                    self._safe_addstr(stdscr, y, 0, "IMU: no data")
                y += 1

                if wod:
                    self._safe_addstr(
                        stdscr, y, 0,
                        f"Wheel: x={fmt_float(wod.get('px'),7,3)} y={fmt_float(wod.get('py'),7,3)} yaw={fmt_float(wod.get('yaw_deg'),7,2)}deg "
                        f"vx={fmt_float(wod.get('vx'),7,3)} wz={fmt_float(wod.get('wz'),7,3)}"
                    )
                else:
                    self._safe_addstr(stdscr, y, 0, "Wheel: no data")
                y += 1

                if eod:
                    self._safe_addstr(
                        stdscr, y, 0,
                        f"EKF:   x={fmt_float(eod.get('px'),7,3)} y={fmt_float(eod.get('py'),7,3)} yaw={fmt_float(eod.get('yaw_deg'),7,2)}deg "
                        f"vx={fmt_float(eod.get('vx'),7,3)} wz={fmt_float(eod.get('wz'),7,3)}"
                    )
                else:
                    self._safe_addstr(stdscr, y, 0, "EKF: no data")
                y += 2

            else:
                # ---- IMU block ----
                if y < h - 4:
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
                            f"inferred state: {imu.get('motion_state','n/a')}   ({imu.get('motion_reason','')})"
                        ); y += 1
                    else:
                        self._safe_addstr(stdscr, y, 0, "No IMU data received yet."); y += 1
                    y += 1

                # ---- Wheel odom block ----
                if y < h - 4:
                    self._safe_addstr(stdscr, y, 0, "Wheel Odometry (/wheel/odom)", curses.A_UNDERLINE); y += 1
                    wod = self.node.wheel_odom_data
                    if wod:
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"frame={wod.get('frame_id','')}  child={wod.get('child_frame_id','')}"
                        ); y += 1
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"pose: x={fmt_float(wod.get('px'))}  y={fmt_float(wod.get('py'))}  "
                            f"yaw={fmt_float(wod.get('yaw_deg'),7,2)} deg"
                        ); y += 1
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"twist: vx={fmt_float(wod.get('vx'))}  vy={fmt_float(wod.get('vy'))}  wz={fmt_float(wod.get('wz'))}"
                        ); y += 1
                    else:
                        self._safe_addstr(stdscr, y, 0, "No /wheel/odom data received yet."); y += 1
                    y += 1

                # ---- EKF odom block ----
                if y < h - 4:
                    self._safe_addstr(stdscr, y, 0, "EKF Odometry (/odometry/filtered)", curses.A_UNDERLINE); y += 1
                    eod = self.node.ekf_odom_data
                    if eod:
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"frame={eod.get('frame_id','')}  child={eod.get('child_frame_id','')}"
                        ); y += 1
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"pose: x={fmt_float(eod.get('px'))}  y={fmt_float(eod.get('py'))}  "
                            f"yaw={fmt_float(eod.get('yaw_deg'),7,2)} deg"
                        ); y += 1
                        self._safe_addstr(
                            stdscr, y, 0,
                            f"twist: vx={fmt_float(eod.get('vx'))}  vy={fmt_float(eod.get('vy'))}  wz={fmt_float(eod.get('wz'))}"
                        ); y += 1
                    else:
                        self._safe_addstr(stdscr, y, 0, "No /odometry/filtered data received yet."); y += 1
                    y += 1

                # ---- Motion + safety block ----
                if y < h - 4:
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

            # Footer (always visible if possible)
            footer = "Tip: Start IMU + wheel odom + EKF, then move robot slowly forward/rotate to verify signs and rates."
            self._safe_addstr(stdscr, h - 2, 0, footer)

            stdscr.refresh()
            time.sleep(max(0.0, 1.0 / max(self.node.ui_hz, 1.0) - 0.01))


# -----------------------------
# main
# -----------------------------

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