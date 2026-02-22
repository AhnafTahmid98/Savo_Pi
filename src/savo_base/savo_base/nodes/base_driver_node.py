#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/nodes/base_driver_node.py
-------------------------------------------------
ROS 2 Jazzy base execution node for the real Robot Savo mecanum platform.

Role in stack
-------------
This is the *hardware execution* node inside `savo_base`.
It converts base motion commands (Twist) into wheel commands and sends them to
the physical motor board (Freenove PCA9685 mecanum board) through `savo_base.drivers`.

Why this node matters
---------------------
Without this layer, the robot does not move on real hardware.
Higher layers (teleop, nav, follow, LLM/voice pipeline) can decide *what to do*,
but `base_driver_node.py` is the layer that actually drives motors safely.

Inputs
------
- /cmd_vel_safe               (geometry_msgs/Twist)   [default]
- /safety/slowdown_factor     (std_msgs/Float32)      [optional]
- /safety/stop                (std_msgs/Bool)         [optional]

Outputs (optional, lightweight)
-------------------------------
- /savo_base/watchdog_state   (std_msgs/String JSON summary)
- /savo_base/base_state       (std_msgs/String JSON summary)

Safety
------
- Watchdog timeout on stale commands
- Optional safety stop topic integration
- Optional slowdown factor integration
- Clean stop on shutdown
- Dry-run backend support for no-hardware testing
- Optional centralized safety policy integration (`savo_base.safety`)
"""

from __future__ import annotations

import json
import math
import traceback
from dataclasses import dataclass
from typing import Optional, Any, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String


# =============================================================================
# Optional project imports (with graceful fallbacks)
# =============================================================================

# Models
try:
    from savo_base.models import (
        make_robot_savo_watchdog_state,
        make_robot_savo_base_state,          # optional / future use
        make_robot_savo_motor_board_status,  # optional / future use
        make_robot_savo_wheel_command,
        WheelNorm,
    )
    _HAS_MODELS = True
except Exception:
    _HAS_MODELS = False
    make_robot_savo_watchdog_state = None
    make_robot_savo_base_state = None
    make_robot_savo_motor_board_status = None
    make_robot_savo_wheel_command = None
    WheelNorm = None

# Drivers
try:
    from savo_base.drivers import make_motor_board
    _HAS_BOARD_FACTORY = True
except Exception:
    _HAS_BOARD_FACTORY = False
    make_motor_board = None

# Kinematics / scaling / conventions
try:
    from savo_base.kinematics.mecanum import mix_mecanum  # returns (fl, rl, fr, rr) normalized
    _HAS_KIN_MIX = True
except Exception:
    _HAS_KIN_MIX = False
    mix_mecanum = None

try:
    from savo_base.kinematics.scaling import normalized_to_signed_duty
    _HAS_SCALING_FN = True
except Exception:
    _HAS_SCALING_FN = False
    normalized_to_signed_duty = None

try:
    from savo_base.kinematics.conventions import ROBOT_SAVO_DEFAULT_CONVENTION
    _HAS_CONVENTIONS = True
except Exception:
    _HAS_CONVENTIONS = False
    ROBOT_SAVO_DEFAULT_CONVENTION = "robot_savo_default"

# Safety policies (optional)
try:
    from savo_base.safety import (
        CommandGuard,
        EstopLatch,
        TimeoutWatchdog,
        TimeoutWatchdogConfig,
        StaleCommandPolicy,
        StaleCommandPolicyConfig,
        WatchdogPolicy,
        WatchdogPolicyConfig,
        MotionPermission,  # optional / future use
    )
    _HAS_SAFETY_POLICIES = True
except Exception:
    _HAS_SAFETY_POLICIES = False
    CommandGuard = None
    EstopLatch = None
    TimeoutWatchdog = None
    TimeoutWatchdogConfig = None
    StaleCommandPolicy = None
    StaleCommandPolicyConfig = None
    WatchdogPolicy = None
    WatchdogPolicyConfig = None
    MotionPermission = None

# Timing helpers (preferred centralized monotonic clock)
try:
    from savo_base.utils.timing import now_mono_s
    _HAS_TIMING_UTILS = True
except Exception:
    _HAS_TIMING_UTILS = False

    def now_mono_s() -> float:
        import time
        return float(time.monotonic())

# Optional topic names (for clean defaults / consistency)
try:
    from savo_base.utils.topic_names import (
        CMD_VEL_SAFE as TOPIC_CMD_VEL_SAFE,
        SAFETY_STOP as TOPIC_SAFETY_STOP,
        SAFETY_SLOWDOWN_FACTOR as TOPIC_SAFETY_SLOWDOWN_FACTOR,
        SAVO_BASE_WATCHDOG_STATE as TOPIC_WATCHDOG_STATE,
        SAVO_BASE_BASE_STATE as TOPIC_BASE_STATE,
    )
    _HAS_TOPIC_NAMES = True
except Exception:
    _HAS_TOPIC_NAMES = False
    TOPIC_CMD_VEL_SAFE = "/cmd_vel_safe"
    TOPIC_SAFETY_STOP = "/safety/stop"
    TOPIC_SAFETY_SLOWDOWN_FACTOR = "/safety/slowdown_factor"
    TOPIC_WATCHDOG_STATE = "/savo_base/watchdog_state"
    TOPIC_BASE_STATE = "/savo_base/base_state"


# =============================================================================
# Local fallbacks (used if project helpers are not yet finalized)
# =============================================================================
def _fallback_mix_mecanum(
    vx: float,
    vy: float,
    wz: float,
    *,
    forward_sign: int = -1,
    strafe_sign: int = +1,
    rotate_sign: int = +1,
    turn_gain: float = 1.0,
) -> Tuple[float, float, float, float]:
    """
    Robot Savo mecanum mix (normalized).
    Wheel order: FL, RL, FR, RR

    Matches proven teleop math:
      fl = vx - vy - w
      rl = vx + vy - w
      fr = vx + vy + w
      rr = vx - vy + w
    """
    vx = float(vx) * int(forward_sign)
    vy = float(vy) * int(strafe_sign)
    w = float(wz) * int(rotate_sign) * float(turn_gain)

    fl = vx - vy - w
    rl = vx + vy - w
    fr = vx + vy + w
    rr = vx - vy + w

    m = max(1.0, abs(fl), abs(rl), abs(fr), abs(rr))
    return (fl / m, rl / m, fr / m, rr / m)


def _fallback_norm_to_duty(
    fl: float,
    rl: float,
    fr: float,
    rr: float,
    *,
    max_duty: int,
) -> Tuple[int, int, int, int]:
    md = int(max(0, min(4095, max_duty)))
    return (
        int(round(float(fl) * md)),
        int(round(float(rl) * md)),
        int(round(float(fr) * md)),
        int(round(float(rr) * md)),
    )


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


# =============================================================================
# Lightweight local state (if model imports are not available yet)
# =============================================================================
@dataclass
class _CmdSnapshot:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    t_mono: float = 0.0
    source: str = "twist"
    seq: int = 0


# =============================================================================
# Base Driver Node
# =============================================================================
class BaseDriverNode(Node):
    """
    Real motor execution node for Robot Savo base.

    Recommended upstream chain:
      /cmd_vel (planner/teleop/nav)
         -> /cmd_vel_safe (savo_perception cmd_vel_safety_gate)
         -> base_driver_node (this node)
         -> motor board (Freenove PCA9685)
    """

    def __init__(self) -> None:
        super().__init__("base_driver_node")

        # ---------------------------------------------------------------------
        # Parameters (ROS2 Jazzy style, hardware-safe defaults)
        # ---------------------------------------------------------------------
        self.declare_parameter("cmd_topic", TOPIC_CMD_VEL_SAFE)
        self.declare_parameter("safety_stop_topic", TOPIC_SAFETY_STOP)
        self.declare_parameter("slowdown_topic", TOPIC_SAFETY_SLOWDOWN_FACTOR)

        self.declare_parameter("use_safety_stop", True)
        self.declare_parameter("use_slowdown_factor", True)
        self.declare_parameter("slowdown_default", 1.0)
        self.declare_parameter("slowdown_min", 0.0)
        self.declare_parameter("slowdown_max", 1.0)

        self.declare_parameter("loop_hz", 30.0)
        self.declare_parameter("watchdog_timeout_s", 0.30)
        self.declare_parameter("publish_status_hz", 2.0)

        self.declare_parameter("vx_limit", 1.0)  # normalized command-space clamp
        self.declare_parameter("vy_limit", 1.0)
        self.declare_parameter("wz_limit", 1.0)

        self.declare_parameter("max_duty", 3000)
        self.declare_parameter("turn_gain", 1.0)

        # Conventions / signs (match proven teleop defaults)
        self.declare_parameter("forward_sign", -1)
        self.declare_parameter("strafe_sign", 1)
        self.declare_parameter("rotate_sign", 1)

        # Board backend
        self.declare_parameter("board_backend", "auto")  # auto | freenove | dryrun
        self.declare_parameter("board_name", "robot_savo_freenove_mecanum")
        self.declare_parameter("dryrun", False)

        # Freenove/PCA9685 hardware params
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("pca9685_addr", "0x40")
        self.declare_parameter("pwm_freq_hz", 50.0)
        self.declare_parameter("quench_ms", 18)

        # Per-wheel invert flags
        self.declare_parameter("invert_fl", False)
        self.declare_parameter("invert_rl", False)
        self.declare_parameter("invert_fr", False)
        self.declare_parameter("invert_rr", False)

        # Optional diagnostics topic names
        self.declare_parameter("watchdog_state_topic", TOPIC_WATCHDOG_STATE)
        self.declare_parameter("base_state_topic", TOPIC_BASE_STATE)

        # ---------------------------------------------------------------------
        # Read params
        # ---------------------------------------------------------------------
        self.cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value
        self.slowdown_topic = self.get_parameter("slowdown_topic").get_parameter_value().string_value

        self.use_safety_stop = bool(self.get_parameter("use_safety_stop").value)
        self.use_slowdown_factor = bool(self.get_parameter("use_slowdown_factor").value)

        self.slowdown_default = float(self.get_parameter("slowdown_default").value)
        self.slowdown_min = float(self.get_parameter("slowdown_min").value)
        self.slowdown_max = float(self.get_parameter("slowdown_max").value)

        self.loop_hz = max(5.0, float(self.get_parameter("loop_hz").value))
        self.watchdog_timeout_s = max(0.05, float(self.get_parameter("watchdog_timeout_s").value))
        self.publish_status_hz = max(0.2, float(self.get_parameter("publish_status_hz").value))

        self.vx_limit = max(0.0, float(self.get_parameter("vx_limit").value))
        self.vy_limit = max(0.0, float(self.get_parameter("vy_limit").value))
        self.wz_limit = max(0.0, float(self.get_parameter("wz_limit").value))

        self.max_duty = int(max(0, min(4095, int(self.get_parameter("max_duty").value))))
        self.turn_gain = float(self.get_parameter("turn_gain").value)

        self.forward_sign = int(self.get_parameter("forward_sign").value)
        self.strafe_sign = int(self.get_parameter("strafe_sign").value)
        self.rotate_sign = int(self.get_parameter("rotate_sign").value)

        self.board_backend = str(self.get_parameter("board_backend").value)
        self.board_name = str(self.get_parameter("board_name").value)
        self.dryrun = bool(self.get_parameter("dryrun").value)

        self.i2c_bus = int(self.get_parameter("i2c_bus").value)
        self.pca9685_addr = self._parse_int_param(str(self.get_parameter("pca9685_addr").value), default=0x40)
        self.pwm_freq_hz = float(self.get_parameter("pwm_freq_hz").value)
        self.quench_ms = int(self.get_parameter("quench_ms").value)

        self.invert_fl = bool(self.get_parameter("invert_fl").value)
        self.invert_rl = bool(self.get_parameter("invert_rl").value)
        self.invert_fr = bool(self.get_parameter("invert_fr").value)
        self.invert_rr = bool(self.get_parameter("invert_rr").value)

        self.watchdog_state_topic = str(self.get_parameter("watchdog_state_topic").value)
        self.base_state_topic = str(self.get_parameter("base_state_topic").value)

        # ---------------------------------------------------------------------
        # Internal runtime state
        # ---------------------------------------------------------------------
        self._last_cmd = _CmdSnapshot()
        self._slowdown_factor = _clamp(self.slowdown_default, self.slowdown_min, self.slowdown_max)
        self._safety_stop = False
        self._last_exec_mono = 0.0
        self._cmd_seq = 0
        self._loop_count = 0
        self._trip_count = 0
        self._zero_count = 0
        self._board_write_count = 0
        self._last_board_error = ""
        self._started_mono = now_mono_s()

        # ---------------------------------------------------------------------
        # Safety policy layer (optional integration)
        # ---------------------------------------------------------------------
        self._cmd_guard = None
        self._estop_latch = None
        self._timeout_wd = None
        self._stale_policy = None
        self._watchdog_policy = None
        self._manual_estop_input = False   # reserved for future physical e-stop source
        self._policy_last_decision = None  # cached for status/debug

        if _HAS_SAFETY_POLICIES:
            try:
                # Command guard (clamp/sanitize cmd_vel)
                self._cmd_guard = CommandGuard(
                    vx_limit=self.vx_limit,
                    vy_limit=self.vy_limit,
                    wz_limit=self.wz_limit,
                )

                # E-stop latch (latched semantics; /safety/stop can feed it)
                self._estop_latch = EstopLatch(auto_rearm=False)

                # Timeout watchdog
                self._timeout_wd = TimeoutWatchdog(
                    TimeoutWatchdogConfig(
                        timeout_s=self.watchdog_timeout_s,
                        warning_ratio=0.7,
                        enabled=True,
                        auto_rearm=True,
                        zero_on_trip=True,
                    )
                )

                # Stale policy (currently advisory / future expansion)
                self._stale_policy = StaleCommandPolicy(
                    StaleCommandPolicyConfig(
                        timeout_s=self.watchdog_timeout_s,
                        zero_on_stale=True,
                        hold_last_on_stale=False,
                    )
                )

                # Combined policy (preferred decision path)
                self._watchdog_policy = WatchdogPolicy(
                    WatchdogPolicyConfig(
                        timeout_s=self.watchdog_timeout_s,
                        zero_on_trip=True,
                        auto_rearm=True,
                    )
                )

                self.get_logger().info(
                    "savo_base safety policies enabled "
                    "(CommandGuard + EstopLatch + TimeoutWatchdog + WatchdogPolicy)."
                )
            except Exception as e:
                self.get_logger().warn(f"Safety policy init failed, using local logic only: {e}")
                self._cmd_guard = None
                self._estop_latch = None
                self._timeout_wd = None
                self._stale_policy = None
                self._watchdog_policy = None

        # ---------------------------------------------------------------------
        # Optional structured watchdog model
        # ---------------------------------------------------------------------
        self._watchdog = None
        if _HAS_MODELS and callable(make_robot_savo_watchdog_state):
            try:
                self._watchdog = make_robot_savo_watchdog_state(
                    timeout_s=self.watchdog_timeout_s,
                    loop_hz=self.loop_hz,
                    enabled=True,
                    auto_rearm=True,
                    zero_on_trip=True,
                    source_name="savo_base/base_driver_node",
                )
                self._watchdog.mark_initialized(active=True)
            except Exception as e:
                self.get_logger().warn(f"Watchdog model init failed, using local fallback: {e}")
                self._watchdog = None

        # ---------------------------------------------------------------------
        # QoS (sensor-style for safety topics, command topic compatible)
        # ---------------------------------------------------------------------
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------------------------------------------------------------
        # Subscriptions
        # ---------------------------------------------------------------------
        self.sub_cmd = self.create_subscription(Twist, self.cmd_topic, self._on_cmd_vel, qos_cmd)

        self.sub_safety_stop = None
        if self.use_safety_stop:
            self.sub_safety_stop = self.create_subscription(
                Bool, self.safety_stop_topic, self._on_safety_stop, qos_sensor
            )

        self.sub_slowdown = None
        if self.use_slowdown_factor:
            self.sub_slowdown = self.create_subscription(
                Float32, self.slowdown_topic, self._on_slowdown_factor, qos_sensor
            )

        # ---------------------------------------------------------------------
        # Publishers (JSON summary strings for easy dashboards/logging)
        # ---------------------------------------------------------------------
        self.pub_watchdog = self.create_publisher(String, self.watchdog_state_topic, 10)
        self.pub_base_state = self.create_publisher(String, self.base_state_topic, 10)

        # ---------------------------------------------------------------------
        # Motor board setup
        # ---------------------------------------------------------------------
        self.board = None
        self._setup_board()

        # ---------------------------------------------------------------------
        # Timers
        # ---------------------------------------------------------------------
        self.exec_timer = self.create_timer(1.0 / self.loop_hz, self._exec_loop)
        self.status_timer = self.create_timer(1.0 / self.publish_status_hz, self._publish_status)

        self.get_logger().info(
            "base_driver_node started | "
            f"cmd_topic={self.cmd_topic} loop_hz={self.loop_hz:.1f} "
            f"watchdog_timeout={self.watchdog_timeout_s:.2f}s max_duty={self.max_duty} "
            f"backend={self.board_backend} dryrun={self.dryrun} "
            f"timing_utils={_HAS_TIMING_UTILS} topic_names={_HAS_TOPIC_NAMES}"
        )

    # =========================================================================
    # Helper serialization
    # =========================================================================
    @staticmethod
    def _policy_decision_to_dict(decision: Any) -> Optional[Dict[str, Any]]:
        if decision is None:
            return None
        try:
            if hasattr(decision, "to_dict"):
                return decision.to_dict()
            return {
                "must_stop": bool(getattr(decision, "must_stop", False)),
                "reason": getattr(decision, "reason", None),
                "slowdown_factor": getattr(decision, "slowdown_factor", None),
            }
        except Exception:
            return None

    # =========================================================================
    # Parameter helpers
    # =========================================================================
    @staticmethod
    def _parse_int_param(text: str, default: int) -> int:
        try:
            return int(str(text), 0)
        except Exception:
            return int(default)

    # =========================================================================
    # Board setup / access
    # =========================================================================
    def _setup_board(self) -> None:
        """
        Create the motor board instance through board_factory (preferred),
        with graceful fallback attempts.
        """
        backend = "dryrun" if self.dryrun else self.board_backend

        if not _HAS_BOARD_FACTORY or make_motor_board is None:
            self.get_logger().warn(
                "savo_base.drivers.make_motor_board not available yet. "
                "Node will run but cannot drive hardware."
            )
            self.board = None
            return

        inv = (
            -1 if self.invert_fl else +1,
            -1 if self.invert_rl else +1,
            -1 if self.invert_fr else +1,
            -1 if self.invert_rr else +1,
        )

        # Try multiple signatures to stay compatible while factory evolves
        attempts = [
            dict(
                backend=backend,
                board_name=self.board_name,
                i2c_bus=self.i2c_bus,
                addr=self.pca9685_addr,
                pwm_freq=self.pwm_freq_hz,
                quench_ms=self.quench_ms,
                inv=inv,
            ),
            dict(
                backend=backend,
                i2c_bus=self.i2c_bus,
                addr=self.pca9685_addr,
                pwm_freq=self.pwm_freq_hz,
                quench_ms=self.quench_ms,
                inv=inv,
            ),
            dict(
                backend=backend,
                name=self.board_name,
                i2c_bus=self.i2c_bus,
                pca9685_addr=self.pca9685_addr,
                pwm_freq_hz=self.pwm_freq_hz,
                quench_ms=self.quench_ms,
                inv=inv,
            ),
        ]

        last_err = None
        for kwargs in attempts:
            try:
                self.board = make_motor_board(**kwargs)
                self.get_logger().info(
                    f"Motor board initialized successfully via factory ({backend}), "
                    f"I2C bus={self.i2c_bus}, addr=0x{self.pca9685_addr:02X}, pwm={self.pwm_freq_hz}Hz"
                )
                return
            except TypeError as e:
                last_err = e
                continue
            except Exception as e:
                last_err = e
                self.get_logger().error(f"Motor board init error: {e}")
                break

        self.board = None
        self.get_logger().error(
            "Failed to initialize motor board. "
            f"Factory signature mismatch or runtime error: {last_err}"
        )

    def _board_stop(self) -> None:
        if self.board is None:
            return
        try:
            if hasattr(self.board, "stop"):
                self.board.stop()
                self._zero_count += 1
            elif hasattr(self.board, "set_motor_model"):
                self.board.set_motor_model(0, 0, 0, 0)
                self._zero_count += 1
            else:
                self.get_logger().warn("Board object has no stop() / set_motor_model()")
        except Exception as e:
            self._last_board_error = str(e)
            self.get_logger().error(f"Failed to stop motor board: {e}")

    def _board_write(self, dfl: int, drl: int, dfr: int, drr: int) -> None:
        if self.board is None:
            return
        try:
            if hasattr(self.board, "set_motor_model"):
                self.board.set_motor_model(int(dfl), int(drl), int(dfr), int(drr))
                self._board_write_count += 1
            elif hasattr(self.board, "write_wheel_duty"):
                self.board.write_wheel_duty(int(dfl), int(drl), int(dfr), int(drr))
                self._board_write_count += 1
            else:
                raise RuntimeError("Motor board object missing expected write method")
        except Exception as e:
            self._last_board_error = str(e)
            self.get_logger().error(f"Motor board write failed: {e}")

    # =========================================================================
    # Subscribers
    # =========================================================================
    def _on_cmd_vel(self, msg: Twist) -> None:
        # Raw values first
        raw_vx = float(msg.linear.x)
        raw_vy = float(msg.linear.y)
        raw_wz = float(msg.angular.z)

        # Preferred path: centralized command guard (if available)
        if self._cmd_guard is not None:
            try:
                guarded = self._cmd_guard.guard(raw_vx, raw_vy, raw_wz)
                vx = float(guarded.vx)
                vy = float(guarded.vy)
                wz = float(guarded.wz)
            except Exception as e:
                self.get_logger().warn(f"CommandGuard.guard() failed, fallback clamp used: {e}")
                vx = _clamp(raw_vx, -self.vx_limit, self.vx_limit) if self.vx_limit > 0 else 0.0
                vy = _clamp(raw_vy, -self.vy_limit, self.vy_limit) if self.vy_limit > 0 else 0.0
                wz = _clamp(raw_wz, -self.wz_limit, self.wz_limit) if self.wz_limit > 0 else 0.0
        else:
            vx = _clamp(raw_vx, -self.vx_limit, self.vx_limit) if self.vx_limit > 0 else 0.0
            vy = _clamp(raw_vy, -self.vy_limit, self.vy_limit) if self.vy_limit > 0 else 0.0
            wz = _clamp(raw_wz, -self.wz_limit, self.wz_limit) if self.wz_limit > 0 else 0.0

        self._cmd_seq += 1
        t_now = now_mono_s()
        self._last_cmd = _CmdSnapshot(
            vx=vx,
            vy=vy,
            wz=wz,
            t_mono=t_now,
            source="cmd_vel_safe",
            seq=self._cmd_seq,
        )

        # Safety watchdog hooks (on command RX)
        if self._timeout_wd is not None:
            try:
                self._timeout_wd.record_command_rx(
                    now_monotonic_s=t_now,
                    sequence=self._cmd_seq,
                )
            except Exception as e:
                self.get_logger().warn(f"TimeoutWatchdog.record_command_rx failed: {e}")

        if self._watchdog_policy is not None:
            try:
                self._watchdog_policy.record_command_rx(
                    now_monotonic_s=t_now,
                    source="cmd_vel_safe",
                    sequence=self._cmd_seq,
                )
            except Exception as e:
                self.get_logger().warn(f"WatchdogPolicy.record_command_rx failed: {e}")

        # Structured watchdog model (if available)
        if self._watchdog is not None:
            try:
                self._watchdog.record_command_rx(source="cmd_vel_safe", note=f"seq={self._cmd_seq}")
            except Exception as e:
                self.get_logger().warn(f"watchdog.record_command_rx failed: {e}")

    def _on_safety_stop(self, msg: Bool) -> None:
        incoming_stop = bool(msg.data)
        self._safety_stop = incoming_stop  # keep existing behavior for compatibility
        t_now = now_mono_s()

        # Estop latch integration (latched behavior)
        if self._estop_latch is not None:
            try:
                self._estop_latch.update(input_stop=incoming_stop, now_monotonic_s=t_now)
            except Exception as e:
                self.get_logger().warn(f"EstopLatch.update failed: {e}")

        # Combined policy hook
        if self._watchdog_policy is not None:
            try:
                self._watchdog_policy.set_safety_stop(incoming_stop, now_monotonic_s=t_now)
            except Exception as e:
                self.get_logger().warn(f"WatchdogPolicy.set_safety_stop failed: {e}")

    def _on_slowdown_factor(self, msg: Float32) -> None:
        self._slowdown_factor = _clamp(float(msg.data), self.slowdown_min, self.slowdown_max)

    # =========================================================================
    # Core execution loop
    # =========================================================================
    def _exec_loop(self) -> None:
        now = now_mono_s()
        self._loop_count += 1
        self._last_exec_mono = now

        # -----------------------------------------------------------------
        # Safety policy ticking (once per control loop)
        # -----------------------------------------------------------------
        if self._timeout_wd is not None:
            try:
                self._timeout_wd.tick(now_monotonic_s=now)
            except Exception as e:
                self.get_logger().warn(f"TimeoutWatchdog.tick failed: {e}")

        if self._watchdog_policy is not None:
            try:
                self._policy_last_decision = self._watchdog_policy.step(
                    now_monotonic_s=now,
                    cmd_timestamp_s=self._last_cmd.t_mono if self._last_cmd.t_mono > 0.0 else None,
                    manual_estop=self._manual_estop_input,
                    safety_stop=self._safety_stop,
                )
            except Exception as e:
                self.get_logger().warn(f"WatchdogPolicy.step failed: {e}")
                self._policy_last_decision = None

        # Legacy / parallel telemetry watchdog model
        if self._watchdog is not None:
            try:
                self._watchdog.tick(now_monotonic_s=now)
            except Exception as e:
                self.get_logger().warn(f"watchdog.tick failed: {e}")

        # -----------------------------------------------------------------
        # Combined policy decision path (preferred when available)
        # -----------------------------------------------------------------
        if self._policy_last_decision is not None:
            try:
                if bool(getattr(self._policy_last_decision, "must_stop", False)):
                    reason = getattr(self._policy_last_decision, "reason", "watchdog_policy_stop")
                    self._apply_stop(reason=str(reason))
                    return

                policy_slowdown = getattr(self._policy_last_decision, "slowdown_factor", None)
                if policy_slowdown is not None:
                    self._slowdown_factor = _clamp(
                        float(policy_slowdown), self.slowdown_min, self.slowdown_max
                    )
            except Exception as e:
                self.get_logger().warn(f"Policy decision handling failed, fallback local logic used: {e}")

        # Local stale detection (always active as fallback)
        cmd_age = math.inf if self._last_cmd.t_mono <= 0.0 else max(0.0, now - self._last_cmd.t_mono)
        stale = (cmd_age >= self.watchdog_timeout_s)

        # Safety stop has top priority (compatibility path)
        if self._safety_stop:
            self._apply_stop(reason="safety_stop_topic")
            return

        # Latched e-stop (future physical e-stop ready)
        if self._estop_latch is not None:
            try:
                if bool(getattr(self._estop_latch, "latched", False)):
                    self._apply_stop(reason="estop_latch")
                    return
            except Exception as e:
                self.get_logger().warn(f"Estop latch state read failed: {e}")

        # Watchdog timeout -> stop
        if stale:
            # Advisory stale policy hook (best effort; API may evolve)
            if self._stale_policy is not None:
                try:
                    _ = self._stale_policy.evaluate(
                        now_monotonic_s=now,
                        cmd_timestamp_s=self._last_cmd.t_mono if self._last_cmd.t_mono > 0.0 else None,
                    )
                except TypeError:
                    try:
                        _ = self._stale_policy.step(
                            now_monotonic_s=now,
                            cmd_timestamp_s=self._last_cmd.t_mono if self._last_cmd.t_mono > 0.0 else None,
                        )
                    except Exception:
                        pass
                except Exception:
                    pass

            # NOTE: _apply_stop handles trip counter update; do not double-increment here.
            self._apply_stop(reason="watchdog_timeout")
            return

        # Fresh command -> execute
        vx = self._last_cmd.vx
        vy = self._last_cmd.vy
        wz = self._last_cmd.wz

        # Apply slowdown factor to translational + rotational commands
        if self.use_slowdown_factor:
            sf = self._slowdown_factor
            vx *= sf
            vy *= sf
            wz *= sf

        # Mecanum mixing
        try:
            if _HAS_KIN_MIX and callable(mix_mecanum):
                fl, rl, fr, rr = mix_mecanum(
                    vx, vy, wz,
                    forward_sign=self.forward_sign,
                    strafe_sign=self.strafe_sign,
                    rotate_sign=self.rotate_sign,
                    turn_gain=self.turn_gain,
                )
            else:
                fl, rl, fr, rr = _fallback_mix_mecanum(
                    vx, vy, wz,
                    forward_sign=self.forward_sign,
                    strafe_sign=self.strafe_sign,
                    rotate_sign=self.rotate_sign,
                    turn_gain=self.turn_gain,
                )
        except Exception as e:
            self.get_logger().error(f"Mecanum mixing failed: {e}")
            self._apply_stop(reason="mix_error")
            return

        # Duty scaling
        try:
            if _HAS_SCALING_FN and callable(normalized_to_signed_duty):
                dfl, drl, dfr, drr = normalized_to_signed_duty(
                    fl, rl, fr, rr, max_abs_duty=self.max_duty
                )
            else:
                dfl, drl, dfr, drr = _fallback_norm_to_duty(
                    fl, rl, fr, rr, max_duty=self.max_duty
                )
        except Exception as e:
            self.get_logger().error(f"Duty scaling failed: {e}")
            self._apply_stop(reason="scaling_error")
            return

        # Optional structured wheel command model (telemetry / future logging)
        if _HAS_MODELS and callable(make_robot_savo_wheel_command) and WheelNorm is not None:
            try:
                cmd_model = make_robot_savo_wheel_command(
                    source="base_driver_node",
                    mode="exec",
                    wheel_norm=WheelNorm(fl=fl, rl=rl, fr=fr, rr=rr),
                    timeout_s=self.watchdog_timeout_s,
                    sequence=self._last_cmd.seq,
                    applied_conventions=(
                        ROBOT_SAVO_DEFAULT_CONVENTION if _HAS_CONVENTIONS else "robot_savo_default"
                    ),
                    note="Twist -> mecanum -> duty",
                )
                cmd_model.vx_cmd = vx
                cmd_model.vy_cmd = vy
                cmd_model.wz_cmd = wz
                cmd_model.speed_scale = self._slowdown_factor if self.use_slowdown_factor else 1.0
                cmd_model.compute_wheel_duty(max_abs_duty=self.max_duty)
                if cmd_model.wheel_duty is not None:
                    dfl, drl, dfr, drr = cmd_model.wheel_duty.as_tuple()
            except Exception as e:
                # Telemetry/model failure must not block motion
                self.get_logger().warn(f"WheelCommand model path failed, using direct duty values: {e}")

        # Write to board
        self._board_write(dfl, drl, dfr, drr)

    def _apply_stop(self, *, reason: str) -> None:
        # Structured watchdog integration
        if self._watchdog is not None:
            try:
                if getattr(self._watchdog, "tripped", False) and getattr(self._watchdog, "zero_action_pending", False):
                    self._watchdog.mark_zero_action_done(note=f"Outputs zeroed ({reason})")
                    self._trip_count = int(getattr(self._watchdog.counters, "trip_count", self._trip_count))
                elif reason == "watchdog_timeout":
                    self._trip_count = max(
                        self._trip_count + 1,
                        int(getattr(self._watchdog.counters, "trip_count", 0)),
                    )
            except Exception:
                pass
        else:
            if reason == "watchdog_timeout":
                self._trip_count += 1

        self._board_stop()

    # =========================================================================
    # Status publishing (JSON string summaries)
    # =========================================================================
    def _publish_status(self) -> None:
        now = now_mono_s()
        uptime = max(0.0, now - self._started_mono)
        cmd_age = math.inf if self._last_cmd.t_mono <= 0.0 else max(0.0, now - self._last_cmd.t_mono)
        stale = (cmd_age >= self.watchdog_timeout_s)

        estop_latched = None
        if self._estop_latch is not None:
            try:
                estop_latched = bool(getattr(self._estop_latch, "latched", False))
            except Exception:
                estop_latched = None

        if self._safety_stop or bool(estop_latched):
            status_level = "SAFETY_STOP"
        elif stale:
            status_level = "STALE"
        else:
            status_level = "OK"

        # Watchdog status
        wd_payload: Dict[str, Any]
        if self._watchdog is not None:
            try:
                wd_payload = self._watchdog.to_dict()
            except Exception as e:
                wd_payload = {
                    "status_level": "ERROR",
                    "error": f"watchdog.to_dict failed: {e}",
                }
        else:
            policy_debug = self._policy_decision_to_dict(self._policy_last_decision)

            wd_payload = {
                "name": "base_command_watchdog",
                "status_level": "STALE" if stale else "OK",
                "flags": {
                    "initialized": True,
                    "active": True,
                    "tripped": stale,
                    "warning": (cmd_age >= 0.7 * self.watchdog_timeout_s) if math.isfinite(cmd_age) else True,
                    "stale": stale,
                    "zero_action_pending": False,
                },
                "timing": {
                    "command_age_s": cmd_age,
                    "timeout_s": self.watchdog_timeout_s,
                    "timeout_ratio": (cmd_age / self.watchdog_timeout_s) if math.isfinite(cmd_age) else float("inf"),
                },
                "counters": {
                    "tick_count": self._loop_count,
                    "trip_count": self._trip_count,
                    "zero_action_count": self._zero_count,
                },
                "diagnostics": {
                    "last_command_source": self._last_cmd.source,
                    "summary": f"[{'STALE' if stale else 'OK'}] age={cmd_age:.3f}s timeout={self.watchdog_timeout_s:.3f}s",
                    "policy_enabled": self._watchdog_policy is not None,
                    "policy_decision": policy_debug,
                },
            }

        msg_wd = String()
        msg_wd.data = json.dumps(wd_payload, ensure_ascii=False, default=str)
        self.pub_watchdog.publish(msg_wd)

        # Base state summary (lightweight but useful for dashboards)
        policy_decision_dict = self._policy_decision_to_dict(self._policy_last_decision)

        base_payload: Dict[str, Any] = {
            "node": "base_driver_node",
            "robot_name": "Robot Savo",
            "status_level": status_level,
            "uptime_s": uptime,
            "backend": {
                "board_backend": "dryrun" if self.dryrun else self.board_backend,
                "board_name": self.board_name,
                "connected": self.board is not None,
                "i2c_bus": self.i2c_bus,
                "pca9685_addr": f"0x{self.pca9685_addr:02X}",
                "pwm_freq_hz": self.pwm_freq_hz,
                "quench_ms": self.quench_ms,
            },
            "inputs": {
                "cmd_topic": self.cmd_topic,
                "safety_stop_topic": self.safety_stop_topic if self.use_safety_stop else None,
                "slowdown_topic": self.slowdown_topic if self.use_slowdown_factor else None,
            },
            "command": {
                "seq": self._last_cmd.seq,
                "vx": self._last_cmd.vx,
                "vy": self._last_cmd.vy,
                "wz": self._last_cmd.wz,
                "age_s": cmd_age,
                "stale": stale,
            },
            "safety": {
                "safety_stop": self._safety_stop,
                "estop_latched": estop_latched,
                "use_safety_stop": self.use_safety_stop,
                "use_slowdown_factor": self.use_slowdown_factor,
                "slowdown_factor": self._slowdown_factor,
                "watchdog_timeout_s": self.watchdog_timeout_s,
                "policy_enabled": self._watchdog_policy is not None,
                "policy_decision": policy_decision_dict,
            },
            "limits": {
                "vx_limit": self.vx_limit,
                "vy_limit": self.vy_limit,
                "wz_limit": self.wz_limit,
                "max_duty": self.max_duty,
                "turn_gain": self.turn_gain,
            },
            "conventions": {
                "forward_sign": self.forward_sign,
                "strafe_sign": self.strafe_sign,
                "rotate_sign": self.rotate_sign,
                "invert_fl": self.invert_fl,
                "invert_rl": self.invert_rl,
                "invert_fr": self.invert_fr,
                "invert_rr": self.invert_rr,
            },
            "counters": {
                "loop_count": self._loop_count,
                "board_write_count": self._board_write_count,
                "zero_count": self._zero_count,
                "trip_count": self._trip_count,
            },
            "diagnostics": {
                "last_board_error": self._last_board_error,
            },
        }

        msg_base = String()
        msg_base.data = json.dumps(base_payload, ensure_ascii=False, default=str)
        self.pub_base_state.publish(msg_base)

    # =========================================================================
    # Shutdown
    # =========================================================================
    def destroy_node(self) -> bool:
        try:
            self.get_logger().info("Shutting down base_driver_node: stopping motors...")
            self._board_stop()
            if self.board is not None and hasattr(self.board, "close"):
                try:
                    self.board.close()
                except Exception as e:
                    self.get_logger().warn(f"Motor board close() failed: {e}")
        finally:
            return super().destroy_node()


# =============================================================================
# Entry point
# =============================================================================
def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[BaseDriverNode] = None
    try:
        node = BaseDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[base_driver_node] Fatal error: {e}")
        traceback.print_exc()
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()