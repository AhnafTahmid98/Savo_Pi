#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — savo_control / auto_test_manager_node.py
=====================================================

Automatic test motion manager for Robot Savo.

Purpose
-------
This node runs safe, repeatable test motion profiles defined by parameters
normally loaded from:

    config/auto_test_modes.yaml

It publishes velocity commands to:

    /cmd_vel_auto

Correct command chain:

    auto_test_manager_node
        -> /cmd_vel_auto
        -> twist_mux_node
        -> /cmd_vel_mux
        -> cmd_vel_shaper_node
        -> /cmd_vel
        -> savo_perception/cmd_vel_safety_gate
        -> /cmd_vel_safe
        -> savo_base/base_driver_node
        -> motors

Architecture rules
------------------
- This node does NOT publish directly to /cmd_vel_safe.
- This node does NOT control hardware.
- This node does NOT touch PCA9685, GPIO, PWM, Freenove board, or motors.
- Final safety authority stays in savo_perception and savo_base.

Test control
------------
The node can run a default test automatically if auto_start:=true.

It can also listen for test name commands on:

    /savo_control/auto_test_cmd

Message type:

    std_msgs/String

Example:

    ros2 topic pub /savo_control/auto_test_cmd std_msgs/msg/String "{data: 'forward_slow'}" --once

Stop command:

    ros2 topic pub /savo_control/auto_test_cmd std_msgs/msg/String "{data: 'STOP'}" --once

Safety
------
First test with wheels lifted. Then test on open floor only with low values.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class AutoTestState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    FINISHED = "FINISHED"
    UNKNOWN_TEST = "UNKNOWN_TEST"
    SAFETY_STOP = "SAFETY_STOP"
    TIMEOUT = "TIMEOUT"
    DISABLED = "DISABLED"


@dataclass
class TestStage:
    name: str
    duration_s: float
    vx: float
    vy: float
    wz: float


@dataclass
class ActiveTest:
    name: str
    test_type: str
    start_time: float
    stages: List[TestStage]
    current_stage_index: int = 0
    pulses_done: int = 0
    pulse_active: bool = True
    last_pulse_switch_time: float = 0.0


class AutoTestManagerNode(Node):
    """Runs predefined safe auto-test motion profiles."""

    def __init__(self) -> None:
        super().__init__("auto_test_manager_node")

        # ------------------------------------------------------------------
        # Core parameters
        # ------------------------------------------------------------------
        self.declare_parameter("enabled", True)
        self.declare_parameter("auto_start", False)

        self.declare_parameter("output_topic", "/cmd_vel_auto")
        self.declare_parameter("mode_cmd_topic", "/savo_control/mode_cmd")
        self.declare_parameter("test_cmd_topic", "/savo_control/auto_test_cmd")
        self.declare_parameter("status_topic", "/savo_control/auto_test_status")
        self.declare_parameter("safety_stop_topic", "/safety/stop")

        self.declare_parameter("request_auto_mode_on_start", True)
        self.declare_parameter("required_mode", "AUTO")

        self.declare_parameter("send_stop_before_test", True)
        self.declare_parameter("send_stop_after_test", True)
        self.declare_parameter("stop_hold_s", 1.0)

        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("max_test_duration_s", 10.0)
        self.declare_parameter("require_known_test_name", True)
        self.declare_parameter("default_test_name", "forward_slow")

        self.declare_parameter("respect_safety_stop", True)
        self.declare_parameter("publish_status", True)
        self.declare_parameter("status_hz", 5.0)
        self.declare_parameter("log_throttle_s", 2.0)
        self.declare_parameter("shutdown_zero_count", 5)

        # Conservative fallback limits if YAML test values are too high.
        self.declare_parameter("limits.max_vx", 0.18)
        self.declare_parameter("limits.max_vy", 0.18)
        self.declare_parameter("limits.max_wz", 0.45)
        self.declare_parameter("limits.low_speed_scale", 0.50)

        # ------------------------------------------------------------------
        # Default test parameters
        #
        # These defaults allow the node to run even if YAML is not loaded.
        # auto_test_modes.yaml can override or extend these.
        # ------------------------------------------------------------------
        self._declare_default_tests()

        self._load_parameters()
        self._validate_parameters()

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self._cmd_pub = self.create_publisher(Twist, self._output_topic, 10)
        self._mode_pub = self.create_publisher(String, self._mode_cmd_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)

        self.create_subscription(String, self._test_cmd_topic, self._on_test_cmd, 10)

        self.create_subscription(Bool, self._safety_stop_topic, self._on_safety_stop, 10)

        self._timer = self.create_timer(1.0 / self._publish_hz, self._on_timer)

        # ------------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------------
        self._state = AutoTestState.IDLE if self._enabled else AutoTestState.DISABLED
        self._active_test: Optional[ActiveTest] = None
        self._safety_stop = False

        self._stop_until_time: Optional[float] = None
        self._pending_test_name: Optional[str] = None
        self._last_status_time = 0.0
        self._last_log_time = 0.0

        if self._auto_start and self._enabled:
            self._request_start_test(self._default_test_name)

        self.get_logger().info(
            "AutoTestManagerNode started | "
            f"output={self._output_topic} | cmd={self._test_cmd_topic} | "
            f"default_test={self._default_test_name} | auto_start={self._auto_start}"
        )

    # ----------------------------------------------------------------------
    # Parameter declarations
    # ----------------------------------------------------------------------
    def _declare_default_tests(self) -> None:
        """Declare default test parameters.

        YAML files may override these.
        """

        # Keep these names aligned with config/auto_test_modes.yaml.
        self.declare_parameter("tests.forward_slow.type", "constant_twist")
        self.declare_parameter("tests.forward_slow.duration_s", 2.0)
        self.declare_parameter("tests.forward_slow.vx", 0.10)
        self.declare_parameter("tests.forward_slow.vy", 0.0)
        self.declare_parameter("tests.forward_slow.wz", 0.0)

        self.declare_parameter("tests.backward_slow.type", "constant_twist")
        self.declare_parameter("tests.backward_slow.duration_s", 2.0)
        self.declare_parameter("tests.backward_slow.vx", -0.10)
        self.declare_parameter("tests.backward_slow.vy", 0.0)
        self.declare_parameter("tests.backward_slow.wz", 0.0)

        self.declare_parameter("tests.strafe_left_slow.type", "constant_twist")
        self.declare_parameter("tests.strafe_left_slow.duration_s", 1.5)
        self.declare_parameter("tests.strafe_left_slow.vx", 0.0)
        self.declare_parameter("tests.strafe_left_slow.vy", 0.10)
        self.declare_parameter("tests.strafe_left_slow.wz", 0.0)

        self.declare_parameter("tests.strafe_right_slow.type", "constant_twist")
        self.declare_parameter("tests.strafe_right_slow.duration_s", 1.5)
        self.declare_parameter("tests.strafe_right_slow.vx", 0.0)
        self.declare_parameter("tests.strafe_right_slow.vy", -0.10)
        self.declare_parameter("tests.strafe_right_slow.wz", 0.0)

        self.declare_parameter("tests.rotate_ccw_slow.type", "constant_twist")
        self.declare_parameter("tests.rotate_ccw_slow.duration_s", 1.5)
        self.declare_parameter("tests.rotate_ccw_slow.vx", 0.0)
        self.declare_parameter("tests.rotate_ccw_slow.vy", 0.0)
        self.declare_parameter("tests.rotate_ccw_slow.wz", 0.25)

        self.declare_parameter("tests.rotate_cw_slow.type", "constant_twist")
        self.declare_parameter("tests.rotate_cw_slow.duration_s", 1.5)
        self.declare_parameter("tests.rotate_cw_slow.vx", 0.0)
        self.declare_parameter("tests.rotate_cw_slow.vy", 0.0)
        self.declare_parameter("tests.rotate_cw_slow.wz", -0.25)

        self.declare_parameter("tests.forward_pulse.type", "pulse_twist")
        self.declare_parameter("tests.forward_pulse.pulses", 3)
        self.declare_parameter("tests.forward_pulse.active_s", 0.6)
        self.declare_parameter("tests.forward_pulse.rest_s", 0.8)
        self.declare_parameter("tests.forward_pulse.vx", 0.10)
        self.declare_parameter("tests.forward_pulse.vy", 0.0)
        self.declare_parameter("tests.forward_pulse.wz", 0.0)

        self.declare_parameter("tests.rotate_pulse.type", "pulse_twist")
        self.declare_parameter("tests.rotate_pulse.pulses", 3)
        self.declare_parameter("tests.rotate_pulse.active_s", 0.5)
        self.declare_parameter("tests.rotate_pulse.rest_s", 0.8)
        self.declare_parameter("tests.rotate_pulse.vx", 0.0)
        self.declare_parameter("tests.rotate_pulse.vy", 0.0)
        self.declare_parameter("tests.rotate_pulse.wz", 0.25)

    # ----------------------------------------------------------------------
    # Parameter loading
    # ----------------------------------------------------------------------
    def _load_parameters(self) -> None:
        self._enabled = bool(self.get_parameter("enabled").value)
        self._auto_start = bool(self.get_parameter("auto_start").value)

        self._output_topic = str(self.get_parameter("output_topic").value)
        self._mode_cmd_topic = str(self.get_parameter("mode_cmd_topic").value)
        self._test_cmd_topic = str(self.get_parameter("test_cmd_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)

        self._request_auto_mode_on_start = bool(
            self.get_parameter("request_auto_mode_on_start").value
        )
        self._required_mode = str(self.get_parameter("required_mode").value)

        self._send_stop_before_test = bool(self.get_parameter("send_stop_before_test").value)
        self._send_stop_after_test = bool(self.get_parameter("send_stop_after_test").value)
        self._stop_hold_s = float(self.get_parameter("stop_hold_s").value)

        self._publish_hz = float(self.get_parameter("publish_hz").value)
        self._max_test_duration_s = float(self.get_parameter("max_test_duration_s").value)
        self._require_known_test_name = bool(
            self.get_parameter("require_known_test_name").value
        )
        self._default_test_name = str(self.get_parameter("default_test_name").value)

        self._respect_safety_stop = bool(self.get_parameter("respect_safety_stop").value)
        self._publish_status = bool(self.get_parameter("publish_status").value)
        self._status_hz = float(self.get_parameter("status_hz").value)
        self._log_throttle_s = float(self.get_parameter("log_throttle_s").value)
        self._shutdown_zero_count = int(self.get_parameter("shutdown_zero_count").value)

        self._max_vx = abs(float(self.get_parameter("limits.max_vx").value))
        self._max_vy = abs(float(self.get_parameter("limits.max_vy").value))
        self._max_wz = abs(float(self.get_parameter("limits.max_wz").value))
        self._low_speed_scale = float(self.get_parameter("limits.low_speed_scale").value)

        self._known_tests = self._load_known_tests()

    def _validate_parameters(self) -> None:
        if self._publish_hz <= 0:
            raise ValueError("publish_hz must be > 0")

        if self._max_test_duration_s <= 0:
            raise ValueError("max_test_duration_s must be > 0")

        if self._stop_hold_s < 0:
            raise ValueError("stop_hold_s must be >= 0")

        if self._max_vx <= 0 or self._max_vy <= 0 or self._max_wz <= 0:
            raise ValueError("command limits must be > 0")

        if self._status_hz <= 0:
            raise ValueError("status_hz must be > 0")

    def _load_known_tests(self) -> Dict[str, Dict[str, Any]]:
        """Load tests from declared parameters.

        ROS 2 Python cannot easily accept arbitrary nested dictionaries from YAML
        as one Python dict parameter, so this node reads known test names from
        declared parameters. This is stable for our first production baseline.
        """

        test_names = [
            "forward_slow",
            "backward_slow",
            "strafe_left_slow",
            "strafe_right_slow",
            "rotate_ccw_slow",
            "rotate_cw_slow",
            "forward_pulse",
            "rotate_pulse",
            "mecanum_sanity",
        ]

        tests: Dict[str, Dict[str, Any]] = {}

        for name in test_names:
            param_type = f"tests.{name}.type"
            if not self.has_parameter(param_type):
                continue

            test_type = str(self.get_parameter(param_type).value)
            tests[name] = {"type": test_type}

            if test_type == "constant_twist":
                tests[name].update(
                    {
                        "duration_s": self._get_float_param(
                            f"tests.{name}.duration_s", 1.0
                        ),
                        "vx": self._get_float_param(f"tests.{name}.vx", 0.0),
                        "vy": self._get_float_param(f"tests.{name}.vy", 0.0),
                        "wz": self._get_float_param(f"tests.{name}.wz", 0.0),
                    }
                )

            elif test_type == "pulse_twist":
                tests[name].update(
                    {
                        "pulses": self._get_int_param(f"tests.{name}.pulses", 3),
                        "active_s": self._get_float_param(
                            f"tests.{name}.active_s", 0.5
                        ),
                        "rest_s": self._get_float_param(f"tests.{name}.rest_s", 0.5),
                        "vx": self._get_float_param(f"tests.{name}.vx", 0.0),
                        "vy": self._get_float_param(f"tests.{name}.vy", 0.0),
                        "wz": self._get_float_param(f"tests.{name}.wz", 0.0),
                    }
                )

        # Add mecanum_sanity as a fixed internal sequence if not defined by YAML.
        if "mecanum_sanity" not in tests:
            tests["mecanum_sanity"] = {
                "type": "sequence",
                "stages": [
                    {"name": "forward", "duration_s": 1.2, "vx": 0.10, "vy": 0.0, "wz": 0.0},
                    {"name": "stop_1", "duration_s": 0.8, "vx": 0.0, "vy": 0.0, "wz": 0.0},
                    {"name": "backward", "duration_s": 1.2, "vx": -0.10, "vy": 0.0, "wz": 0.0},
                    {"name": "stop_2", "duration_s": 0.8, "vx": 0.0, "vy": 0.0, "wz": 0.0},
                    {"name": "strafe_left", "duration_s": 1.0, "vx": 0.0, "vy": 0.10, "wz": 0.0},
                    {"name": "stop_3", "duration_s": 0.8, "vx": 0.0, "vy": 0.0, "wz": 0.0},
                    {"name": "strafe_right", "duration_s": 1.0, "vx": 0.0, "vy": -0.10, "wz": 0.0},
                    {"name": "stop_4", "duration_s": 0.8, "vx": 0.0, "vy": 0.0, "wz": 0.0},
                    {"name": "rotate_ccw", "duration_s": 1.0, "vx": 0.0, "vy": 0.0, "wz": 0.25},
                    {"name": "stop_5", "duration_s": 0.8, "vx": 0.0, "vy": 0.0, "wz": 0.0},
                    {"name": "rotate_cw", "duration_s": 1.0, "vx": 0.0, "vy": 0.0, "wz": -0.25},
                ],
            }

        return tests

    def _get_float_param(self, name: str, default: float) -> float:
        if not self.has_parameter(name):
            return default
        return float(self.get_parameter(name).value)

    def _get_int_param(self, name: str, default: int) -> int:
        if not self.has_parameter(name):
            return default
        return int(self.get_parameter(name).value)

    # ----------------------------------------------------------------------
    # Subscriptions
    # ----------------------------------------------------------------------
    def _on_test_cmd(self, msg: String) -> None:
        name = str(msg.data).strip()

        if not name:
            return

        upper = name.upper()
        if upper in ("STOP", "CANCEL", "ABORT"):
            self._stop_active_test(reason="operator_stop")
            return

        self._request_start_test(name)

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = bool(msg.data)

    # ----------------------------------------------------------------------
    # Test lifecycle
    # ----------------------------------------------------------------------
    def _request_start_test(self, name: str) -> None:
        if not self._enabled:
            self._state = AutoTestState.DISABLED
            self._publish_zero()
            self._publish_status()
            return

        if self._require_known_test_name and name not in self._known_tests:
            self._state = AutoTestState.UNKNOWN_TEST
            self._publish_zero()
            self._publish_status()
            self.get_logger().error(
                f"Unknown auto test '{name}'. Known tests: {sorted(self._known_tests.keys())}"
            )
            return

        if name not in self._known_tests:
            self.get_logger().error(f"Unknown test '{name}'")
            return

        if self._respect_safety_stop and self._safety_stop:
            self._state = AutoTestState.SAFETY_STOP
            self._publish_zero()
            self._publish_status()
            self.get_logger().warning("Refusing to start auto test while safety_stop=True")
            return

        self._pending_test_name = name

        if self._send_stop_before_test and self._stop_hold_s > 0.0:
            self._state = AutoTestState.STOPPING
            self._stop_until_time = time.monotonic() + self._stop_hold_s
            self._publish_zero()
        else:
            self._start_test_now(name)

    def _start_test_now(self, name: str) -> None:
        test_def = self._known_tests[name]
        test_type = str(test_def["type"])

        stages = self._build_stages(name, test_def)

        if not stages:
            self.get_logger().error(f"Test '{name}' has no stages")
            self._state = AutoTestState.UNKNOWN_TEST
            self._publish_zero()
            return

        if self._request_auto_mode_on_start:
            mode_msg = String()
            mode_msg.data = self._required_mode
            self._mode_pub.publish(mode_msg)

        now = time.monotonic()
        self._active_test = ActiveTest(
            name=name,
            test_type=test_type,
            start_time=now,
            stages=stages,
            current_stage_index=0,
            pulses_done=0,
            pulse_active=True,
            last_pulse_switch_time=now,
        )

        self._state = AutoTestState.RUNNING
        self.get_logger().info(f"Started auto test '{name}' type={test_type}")

    def _build_stages(self, name: str, test_def: Dict[str, Any]) -> List[TestStage]:
        test_type = str(test_def["type"])

        if test_type == "constant_twist":
            return [
                TestStage(
                    name=name,
                    duration_s=float(test_def.get("duration_s", 1.0)),
                    vx=self._limit_vx(float(test_def.get("vx", 0.0))),
                    vy=self._limit_vy(float(test_def.get("vy", 0.0))),
                    wz=self._limit_wz(float(test_def.get("wz", 0.0))),
                )
            ]

        if test_type == "pulse_twist":
            # Pulse is handled specially, but store active command as one stage.
            return [
                TestStage(
                    name=name,
                    duration_s=float(test_def.get("active_s", 0.5)),
                    vx=self._limit_vx(float(test_def.get("vx", 0.0))),
                    vy=self._limit_vy(float(test_def.get("vy", 0.0))),
                    wz=self._limit_wz(float(test_def.get("wz", 0.0))),
                )
            ]

        if test_type == "sequence":
            stages_raw = test_def.get("stages", [])
            stages: List[TestStage] = []
            for index, stage in enumerate(stages_raw):
                stages.append(
                    TestStage(
                        name=str(stage.get("name", f"stage_{index}")),
                        duration_s=float(stage.get("duration_s", 1.0)),
                        vx=self._limit_vx(float(stage.get("vx", 0.0))),
                        vy=self._limit_vy(float(stage.get("vy", 0.0))),
                        wz=self._limit_wz(float(stage.get("wz", 0.0))),
                    )
                )
            return stages

        self.get_logger().error(f"Unsupported test type '{test_type}' for test '{name}'")
        return []

    def _stop_active_test(self, reason: str = "stop") -> None:
        self._active_test = None
        self._pending_test_name = None
        self._state = AutoTestState.FINISHED
        self._publish_zero()
        self.get_logger().info(f"Auto test stopped: {reason}")

    # ----------------------------------------------------------------------
    # Timer
    # ----------------------------------------------------------------------
    def _on_timer(self) -> None:
        if not self._enabled:
            self._state = AutoTestState.DISABLED
            self._publish_zero()
            self._publish_status_if_needed()
            return

        if self._respect_safety_stop and self._safety_stop:
            if self._state == AutoTestState.RUNNING:
                self.get_logger().warning("Safety stop active. Stopping auto test.")
            self._state = AutoTestState.SAFETY_STOP
            self._active_test = None
            self._publish_zero()
            self._publish_status_if_needed()
            return

        if self._state == AutoTestState.STOPPING:
            self._publish_zero()
            if self._stop_until_time is not None and time.monotonic() >= self._stop_until_time:
                pending = self._pending_test_name
                self._pending_test_name = None
                self._stop_until_time = None
                if pending:
                    self._start_test_now(pending)
                else:
                    self._state = AutoTestState.IDLE
            self._publish_status_if_needed()
            return

        if self._active_test is None:
            self._publish_zero()
            self._publish_status_if_needed()
            return

        elapsed_total = time.monotonic() - self._active_test.start_time
        if elapsed_total > self._max_test_duration_s:
            self._state = AutoTestState.TIMEOUT
            self._active_test = None
            self._publish_zero()
            self._publish_status_if_needed()
            self._throttled_warn("Auto test timeout. Publishing zero.")
            return

        if self._active_test.test_type == "pulse_twist":
            self._run_pulse_test()
        else:
            self._run_stage_test()

        self._publish_status_if_needed()

    def _run_stage_test(self) -> None:
        assert self._active_test is not None

        now = time.monotonic()
        stage = self._active_test.stages[self._active_test.current_stage_index]

        # Compute stage start time by summing previous stage durations.
        stage_start_offset = sum(
            s.duration_s for s in self._active_test.stages[: self._active_test.current_stage_index]
        )
        stage_start_time = self._active_test.start_time + stage_start_offset

        if now - stage_start_time <= stage.duration_s:
            self._publish_cmd(stage.vx, stage.vy, stage.wz)
            return

        self._active_test.current_stage_index += 1

        if self._active_test.current_stage_index >= len(self._active_test.stages):
            self._finish_test()
            return

        next_stage = self._active_test.stages[self._active_test.current_stage_index]
        self.get_logger().info(
            f"Auto test '{self._active_test.name}' stage -> {next_stage.name}"
        )
        self._publish_cmd(next_stage.vx, next_stage.vy, next_stage.wz)

    def _run_pulse_test(self) -> None:
        assert self._active_test is not None

        test_def = self._known_tests[self._active_test.name]
        pulses = int(test_def.get("pulses", 3))
        active_s = float(test_def.get("active_s", 0.5))
        rest_s = float(test_def.get("rest_s", 0.5))
        stage = self._active_test.stages[0]

        now = time.monotonic()
        elapsed = now - self._active_test.last_pulse_switch_time

        if self._active_test.pulse_active:
            self._publish_cmd(stage.vx, stage.vy, stage.wz)
            if elapsed >= active_s:
                self._active_test.pulse_active = False
                self._active_test.last_pulse_switch_time = now
                self._publish_zero()
        else:
            self._publish_zero()
            if elapsed >= rest_s:
                self._active_test.pulses_done += 1
                if self._active_test.pulses_done >= pulses:
                    self._finish_test()
                    return
                self._active_test.pulse_active = True
                self._active_test.last_pulse_switch_time = now

    def _finish_test(self) -> None:
        if self._send_stop_after_test:
            self._publish_zero()

        name = self._active_test.name if self._active_test else "unknown"
        self._active_test = None
        self._state = AutoTestState.FINISHED
        self.get_logger().info(f"Auto test finished: {name}")

    # ----------------------------------------------------------------------
    # Publishing
    # ----------------------------------------------------------------------
    def _publish_cmd(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = self._safe_float(self._limit_vx(vx))
        msg.linear.y = self._safe_float(self._limit_vy(vy))
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self._safe_float(self._limit_wz(wz))
        self._cmd_pub.publish(msg)

    def _publish_zero(self) -> None:
        self._publish_cmd(0.0, 0.0, 0.0)

    def _publish_status_if_needed(self) -> None:
        if not self._publish_status:
            return

        now = time.monotonic()
        period = 1.0 / max(0.1, self._status_hz)
        if (now - self._last_status_time) < period:
            return

        self._last_status_time = now
        self._publish_status()

    def _publish_status(self) -> None:
        if self._active_test is not None:
            active = self._active_test.name
            stage = self._active_test.current_stage_index
        else:
            active = "none"
            stage = -1

        msg = String()
        msg.data = (
            f"state={self._state.value}; "
            f"active_test={active}; "
            f"stage={stage}; "
            f"safety_stop={self._safety_stop}; "
            f"output={self._output_topic}"
        )
        self._status_pub.publish(msg)

    # ----------------------------------------------------------------------
    # Limits and helpers
    # ----------------------------------------------------------------------
    def _limit_vx(self, value: float) -> float:
        return self._clamp(self._safe_float(value), -self._max_vx, self._max_vx)

    def _limit_vy(self, value: float) -> float:
        return self._clamp(self._safe_float(value), -self._max_vy, self._max_vy)

    def _limit_wz(self, value: float) -> float:
        return self._clamp(self._safe_float(value), -self._max_wz, self._max_wz)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _safe_float(value: float) -> float:
        if not math.isfinite(float(value)):
            return 0.0
        return float(value)

    def _throttled_warn(self, text: str) -> None:
        now = time.monotonic()
        if (now - self._last_log_time) >= self._log_throttle_s:
            self._last_log_time = now
            self.get_logger().warning(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoTestManagerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Stopping auto test.")

    finally:
        try:
            for _ in range(max(1, node._shutdown_zero_count)):
                node._publish_zero()
                time.sleep(0.05)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()