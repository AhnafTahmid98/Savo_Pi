#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/nodes/base_diag_runner_node.py
-----------------------------------------------------
Professional ROS 2 Jazzy diagnostic runner node for Robot Savo `savo_base`.

Purpose
-------
Runs whitelisted base diagnostics (real hardware / dry-run) as external commands
from a controlled registry and publishes structured JSON status/results.

Why this node exists
--------------------
During real robot testing, you often want to trigger diagnostics from ROS tools,
dashboards, or launch workflows without manually switching terminals every time.

This node provides:
- Whitelisted diagnostic execution (no arbitrary shell execution)
- One-job-at-a-time safety lock
- Timeout + terminate/kill handling
- Live state publishing (IDLE/RUNNING/SUCCEEDED/FAILED/TIMEOUT/CANCELLED)
- Trigger + cancel via ROS topics
- JSON outputs for dashboard integration

Primary topics
--------------
Subscriptions:
- /savo_base/diag/run_request      (std_msgs/String)  JSON or plain diagnostic key
- /savo_base/diag/cancel_request   (std_msgs/Bool)    true => cancel active job

Publications:
- /savo_base/diag/state            (std_msgs/String)  JSON state snapshot
- /savo_base/diag/event            (std_msgs/String)  JSON event messages
- /savo_base/diag/busy             (std_msgs/Bool)    runner busy flag

Request format (examples)
-------------------------
1) Plain text key:
   "motor_direction_test"

2) JSON:
   {
     "diag_key": "motor_direction_test",
     "args": ["--with-rotate"],
     "timeout_s": 25.0
   }

Notes
-----
- This node does NOT execute arbitrary shell strings.
- Only diagnostics defined in the internal registry can be run.
- Intended for real robot bringup and controlled hardware validation.
"""

from __future__ import annotations

import json
import shlex
import signal
import subprocess
import threading
import time
import traceback
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, String


# =============================================================================
# Helpers
# =============================================================================
def _now_mono() -> float:
    return float(time.monotonic())


def _utc_iso_like() -> str:
    # Lightweight timestamp for logs/UI (no timezone dependency)
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def _safe_json_loads(text: str) -> Optional[Dict[str, Any]]:
    try:
        val = json.loads(text)
        return val if isinstance(val, dict) else None
    except Exception:
        return None


def _truncate(text: str, limit: int) -> str:
    if limit <= 0:
        return ""
    if len(text) <= limit:
        return text
    return text[: max(0, limit - 3)] + "..."


# =============================================================================
# Data models
# =============================================================================
@dataclass
class DiagSpec:
    key: str
    description: str
    command: List[str]  # base command tokens (no shell)
    default_timeout_s: float = 20.0
    allow_extra_args: bool = True
    tags: List[str] = field(default_factory=list)


@dataclass
class ActiveJob:
    job_id: str
    diag_key: str
    command: List[str]
    timeout_s: float
    started_mono: float
    requested_args: List[str]
    process: Optional[subprocess.Popen] = None

    # Runtime result fields
    state: str = "RUNNING"  # RUNNING/SUCCEEDED/FAILED/TIMEOUT/CANCELLED/ERROR
    return_code: Optional[int] = None
    stdout_text: str = ""
    stderr_text: str = ""
    finished_mono: float = 0.0
    cancel_requested: bool = False
    timeout_hit: bool = False
    error_text: str = ""


# =============================================================================
# Node
# =============================================================================
class BaseDiagRunnerNode(Node):
    """
    Controlled diagnostics runner for Robot Savo base stack.
    """

    def __init__(self) -> None:
        super().__init__("base_diag_runner_node")

        # ---------------------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------------------
        self.declare_parameter("robot_name", "Robot Savo")

        # Topics
        self.declare_parameter("run_request_topic", "/savo_base/diag/run_request")
        self.declare_parameter("cancel_request_topic", "/savo_base/diag/cancel_request")
        self.declare_parameter("state_topic", "/savo_base/diag/state")
        self.declare_parameter("event_topic", "/savo_base/diag/event")
        self.declare_parameter("busy_topic", "/savo_base/diag/busy")

        # Timers / publication rates
        self.declare_parameter("state_publish_hz", 2.0)

        # Execution policy
        self.declare_parameter("default_timeout_s", 20.0)
        self.declare_parameter("max_timeout_s", 120.0)
        self.declare_parameter("terminate_grace_s", 2.0)
        self.declare_parameter("allow_parallel_runs", False)  # recommended false
        self.declare_parameter("shell_mode", False)  # MUST stay false for safety
        self.declare_parameter("capture_output", True)
        self.declare_parameter("max_output_chars", 4000)

        # Script discovery roots (for your real robot layout)
        self.declare_parameter("savo_pi_root", str(Path.home() / "Savo_Pi"))
        self.declare_parameter("tools_diag_root", str(Path.home() / "Savo_Pi" / "tools" / "diag"))

        # JSON formatting
        self.declare_parameter("pretty_json", False)

        # ---------------------------------------------------------------------
        # Read params
        # ---------------------------------------------------------------------
        self.robot_name = str(self.get_parameter("robot_name").value)

        self.run_request_topic = str(self.get_parameter("run_request_topic").value)
        self.cancel_request_topic = str(self.get_parameter("cancel_request_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.event_topic = str(self.get_parameter("event_topic").value)
        self.busy_topic = str(self.get_parameter("busy_topic").value)

        self.state_publish_hz = max(0.5, float(self.get_parameter("state_publish_hz").value))

        self.default_timeout_s = max(1.0, float(self.get_parameter("default_timeout_s").value))
        self.max_timeout_s = max(self.default_timeout_s, float(self.get_parameter("max_timeout_s").value))
        self.terminate_grace_s = max(0.2, float(self.get_parameter("terminate_grace_s").value))
        self.allow_parallel_runs = bool(self.get_parameter("allow_parallel_runs").value)
        self.shell_mode = bool(self.get_parameter("shell_mode").value)
        self.capture_output = bool(self.get_parameter("capture_output").value)
        self.max_output_chars = max(256, int(self.get_parameter("max_output_chars").value))

        self.savo_pi_root = str(self.get_parameter("savo_pi_root").value)
        self.tools_diag_root = str(self.get_parameter("tools_diag_root").value)
        self.pretty_json = bool(self.get_parameter("pretty_json").value)

        # Force shell mode off (safety)
        if self.shell_mode:
            self.get_logger().warn("shell_mode=true requested but not allowed. Forcing shell_mode=false.")
            self.shell_mode = False

        # ---------------------------------------------------------------------
        # Registry (whitelisted diagnostics)
        # ---------------------------------------------------------------------
        self._registry = self._build_registry()

        # ---------------------------------------------------------------------
        # Runtime state
        # ---------------------------------------------------------------------
        self._started_mono = _now_mono()
        self._lock = threading.Lock()
        self._active_job: Optional[ActiveJob] = None
        self._worker_thread: Optional[threading.Thread] = None

        self._request_count = 0
        self._reject_count = 0
        self._cancel_count = 0
        self._completed_count = 0
        self._event_count = 0
        self._state_pub_count = 0
        self._busy_pub_count = 0

        self._last_event: Dict[str, Any] = {
            "type": "startup",
            "message": "Diagnostic runner initialized",
            "time": _utc_iso_like(),
        }
        self._last_finished_summary: Optional[Dict[str, Any]] = None

        # ---------------------------------------------------------------------
        # QoS
        # ---------------------------------------------------------------------
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------------------
        self.sub_run = self.create_subscription(String, self.run_request_topic, self._on_run_request, qos_reliable)
        self.sub_cancel = self.create_subscription(Bool, self.cancel_request_topic, self._on_cancel_request, qos_reliable)

        self.pub_state = self.create_publisher(String, self.state_topic, 10)
        self.pub_event = self.create_publisher(String, self.event_topic, 10)
        self.pub_busy = self.create_publisher(Bool, self.busy_topic, 10)

        self.state_timer = self.create_timer(1.0 / self.state_publish_hz, self._publish_state)
        self.busy_timer = self.create_timer(0.5, self._publish_busy)

        self.get_logger().info(
            "base_diag_runner_node started | "
            f"registry={len(self._registry)} diagnostics, "
            f"run_request_topic={self.run_request_topic}, cancel_topic={self.cancel_request_topic}"
        )
        self._publish_event("info", "Diagnostic runner ready", {"registry_keys": sorted(self._registry.keys())})

    # =========================================================================
    # Registry
    # =========================================================================
    def _build_registry(self) -> Dict[str, DiagSpec]:
        """
        Whitelisted diagnostics registry. This is the professional, safe path:
        no arbitrary shell execution, only known commands.
        """
        py = "python3"
        root = Path(self.savo_pi_root)
        diag = Path(self.tools_diag_root)

        registry: Dict[str, DiagSpec] = {
            # Motion / motor diagnostics
            "motor_direction_test": DiagSpec(
                key="motor_direction_test",
                description="Verify wheel direction signs and rotation expectations (real hardware).",
                command=[py, str(diag / "motion" / "motor_direction_test.py")],
                default_timeout_s=35.0,
                allow_extra_args=True,
                tags=["motion", "motors", "encoders", "real-hw"],
            ),
            "drive_manual_direct": DiagSpec(
                key="drive_manual_direct",
                description="Manual non-ROS mecanum teleop (TTY required).",
                command=[py, str(diag / "motion" / "drive_manual_direct.py")],
                default_timeout_s=120.0,
                allow_extra_args=True,
                tags=["motion", "teleop", "real-hw"],
            ),
            "encoders_test": DiagSpec(
                key="encoders_test",
                description="Encoder diagnostics (lgpio) for wheel motion verification.",
                command=[py, str(diag / "motion" / "encoders_test.py")],
                default_timeout_s=30.0,
                allow_extra_args=True,
                tags=["motion", "encoders", "real-hw"],
            ),

            # Sensors
            "imu_test": DiagSpec(
                key="imu_test",
                description="IMU health/streaming diagnostics (BNO055).",
                command=[py, str(diag / "sensors" / "imu_test.py")],
                default_timeout_s=20.0,
                allow_extra_args=True,
                tags=["sensor", "imu", "real-hw"],
            ),
            "lidar_test": DiagSpec(
                key="lidar_test",
                description="RPLIDAR A1 diagnostic and near-field checks.",
                command=[py, str(diag / "sensors" / "lidar_test.py")],
                default_timeout_s=25.0,
                allow_extra_args=True,
                tags=["sensor", "lidar", "safety", "real-hw"],
            ),
            "range_vl53_test": DiagSpec(
                key="range_vl53_test",
                description="Dual VL53L1X ToF diagnostic (FL/FR).",
                command=[py, str(diag / "sensors" / "range_vl53_test.py")],
                default_timeout_s=20.0,
                allow_extra_args=True,
                tags=["sensor", "tof", "safety", "real-hw"],
            ),
            "camera_test": DiagSpec(
                key="camera_test",
                description="Camera diagnostic (libcamera/GStreamer-based).",
                command=[py, str(diag / "sensors" / "camera_test.py")],
                default_timeout_s=40.0,
                allow_extra_args=True,
                tags=["sensor", "camera", "real-hw"],
            ),
            "depth_camera_test": DiagSpec(
                key="depth_camera_test",
                description="RealSense D435 depth front obstacle diagnostic (ROS-based script).",
                command=[py, str(diag / "sensors" / "dept_camera_test.py")],
                default_timeout_s=30.0,
                allow_extra_args=True,
                tags=["sensor", "depth", "realsense", "safety", "real-hw"],
            ),

            # Power
            "battery_health": DiagSpec(
                key="battery_health",
                description="Battery health/UPS diagnostics.",
                command=[py, str(diag / "power" / "battery_health.py")],
                default_timeout_s=15.0,
                allow_extra_args=True,
                tags=["power", "ups", "battery", "real-hw"],
            ),

            # Convenience / self-check
            "echo_diag_runner": DiagSpec(
                key="echo_diag_runner",
                description="Minimal test command to validate runner pipeline.",
                command=["/bin/echo", "base_diag_runner ok"],
                default_timeout_s=5.0,
                allow_extra_args=True,
                tags=["selftest"],
            ),
            "pwd": DiagSpec(
                key="pwd",
                description="Show current working directory for diagnostics runner debugging.",
                command=["/bin/pwd"],
                default_timeout_s=5.0,
                allow_extra_args=False,
                tags=["selftest"],
            ),
        }

        # Optional note: commands are allowed even if files don't exist yet.
        # Validation occurs at execution time with a clean error event.
        _ = root  # kept for future expansion / readability
        return registry

    # =========================================================================
    # Subscribers
    # =========================================================================
    def _on_run_request(self, msg: String) -> None:
        self._request_count += 1
        raw = (msg.data or "").strip()

        diag_key, args, timeout_s, parse_note = self._parse_run_request(raw)
        if not diag_key:
            self._reject_count += 1
            self._publish_event("reject", "Missing diag_key in run request", {"raw": _truncate(raw, 500)})
            return

        spec = self._registry.get(diag_key)
        if spec is None:
            self._reject_count += 1
            self._publish_event(
                "reject",
                f"Unknown diag_key '{diag_key}'",
                {"diag_key": diag_key, "available_keys": sorted(self._registry.keys())},
            )
            return

        if not spec.allow_extra_args and args:
            self._reject_count += 1
            self._publish_event(
                "reject",
                f"Diagnostic '{diag_key}' does not allow extra args",
                {"diag_key": diag_key, "args": args},
            )
            return

        timeout_s = spec.default_timeout_s if timeout_s is None else float(timeout_s)
        timeout_s = max(1.0, min(timeout_s, self.max_timeout_s))

        full_cmd = list(spec.command) + list(args)

        with self._lock:
            if (self._active_job is not None) and (not self.allow_parallel_runs):
                self._reject_count += 1
                self._publish_event(
                    "reject",
                    "Runner is busy (parallel runs disabled)",
                    {
                        "requested_diag_key": diag_key,
                        "active_job_id": self._active_job.job_id,
                        "active_diag_key": self._active_job.diag_key,
                    },
                )
                return

            # This implementation intentionally supports one active job at a time.
            if self._active_job is not None:
                self._reject_count += 1
                self._publish_event(
                    "reject",
                    "Parallel runs are not implemented in this node version",
                    {"requested_diag_key": diag_key},
                )
                return

            job = ActiveJob(
                job_id=uuid.uuid4().hex[:12],
                diag_key=diag_key,
                command=full_cmd,
                timeout_s=timeout_s,
                started_mono=_now_mono(),
                requested_args=list(args),
            )
            self._active_job = job

            self._worker_thread = threading.Thread(
                target=self._run_job_worker,
                args=(job,),
                daemon=True,
                name=f"diag_runner_{job.job_id}",
            )
            self._worker_thread.start()

        details = {
            "job_id": job.job_id,
            "diag_key": diag_key,
            "timeout_s": timeout_s,
            "command": full_cmd,
        }
        if parse_note:
            details["parse_note"] = parse_note

        self.get_logger().info(
            f"Starting diagnostic job {job.job_id} | key={diag_key} timeout={timeout_s:.1f}s | cmd={full_cmd}"
        )
        self._publish_event("start", f"Started diagnostic '{diag_key}'", details)

    def _on_cancel_request(self, msg: Bool) -> None:
        if not bool(msg.data):
            return

        self._cancel_count += 1
        with self._lock:
            job = self._active_job
            if job is None:
                self._publish_event("info", "Cancel requested but no active job", {})
                return
            job.cancel_requested = True

            proc = job.process

        if proc is None:
            # Worker may still be starting process
            self._publish_event("info", "Cancel requested (job starting)", {"job_id": job.job_id})
            return

        self.get_logger().warn(f"Cancel requested for job {job.job_id} ({job.diag_key})")
        self._publish_event("cancel_request", "Cancel requested for active diagnostic", {"job_id": job.job_id})

        # Process termination handled by worker loop (coordinated)
        # We do not force kill here to avoid race conditions.

    # =========================================================================
    # Request parsing
    # =========================================================================
    def _parse_run_request(self, raw: str) -> Tuple[Optional[str], List[str], Optional[float], Optional[str]]:
        """
        Returns (diag_key, args, timeout_s, parse_note)
        """
        if not raw:
            return None, [], None, None

        # JSON request
        data = _safe_json_loads(raw)
        if data is not None:
            diag_key = str(data.get("diag_key", "")).strip() or None

            args_val = data.get("args", [])
            args: List[str] = []
            if isinstance(args_val, list):
                args = [str(a) for a in args_val]
            elif isinstance(args_val, str) and args_val.strip():
                # convenience (space-split)
                args = shlex.split(args_val)

            timeout_s = data.get("timeout_s", None)
            try:
                timeout_s = None if timeout_s is None else float(timeout_s)
            except Exception:
                timeout_s = None

            return diag_key, args, timeout_s, "parsed_json"

        # Plain text convenience:
        # - exact key: "imu_test"
        # - key + args: "imu_test --stats --health"
        parts = shlex.split(raw)
        if not parts:
            return None, [], None, None

        diag_key = parts[0]
        args = parts[1:]
        return diag_key, args, None, "parsed_plaintext"

    # =========================================================================
    # Worker
    # =========================================================================
    def _run_job_worker(self, job: ActiveJob) -> None:
        start = _now_mono()

        try:
            # Basic executable path validation (for python script commands)
            if not job.command:
                raise RuntimeError("Empty command in diagnostic spec")

            # If command begins with python3 <script.py>, validate script exists
            if len(job.command) >= 2 and job.command[0].endswith("python3") and job.command[1].endswith(".py"):
                script_path = Path(job.command[1])
                if not script_path.exists():
                    raise FileNotFoundError(f"Diagnostic script not found: {script_path}")

            popen_kwargs = dict(
                args=job.command,
                shell=False,
                cwd=self.savo_pi_root,
                text=True,
            )
            if self.capture_output:
                popen_kwargs["stdout"] = subprocess.PIPE
                popen_kwargs["stderr"] = subprocess.PIPE
            else:
                popen_kwargs["stdout"] = None
                popen_kwargs["stderr"] = None

            proc = subprocess.Popen(**popen_kwargs)  # nosec (whitelisted command, shell=False)
            with self._lock:
                if self._active_job and self._active_job.job_id == job.job_id:
                    self._active_job.process = proc

            # Poll loop for timeout/cancel support
            while True:
                rc = proc.poll()
                if rc is not None:
                    job.return_code = int(rc)
                    break

                elapsed = _now_mono() - start
                if job.cancel_requested:
                    self._terminate_process(proc, job, reason="cancel")
                    break

                if elapsed >= job.timeout_s:
                    job.timeout_hit = True
                    self._terminate_process(proc, job, reason="timeout")
                    break

                time.sleep(0.05)

            # Collect outputs after process exits
            if self.capture_output:
                try:
                    out, err = proc.communicate(timeout=1.0)
                except subprocess.TimeoutExpired:
                    out, err = "", "[diag_runner] communicate timeout after process exit"
                job.stdout_text = _truncate(out or "", self.max_output_chars)
                job.stderr_text = _truncate(err or "", self.max_output_chars)

            if job.timeout_hit:
                job.state = "TIMEOUT"
            elif job.cancel_requested:
                job.state = "CANCELLED"
            else:
                job.state = "SUCCEEDED" if (job.return_code == 0) else "FAILED"

        except Exception as e:
            job.state = "ERROR"
            job.error_text = f"{type(e).__name__}: {e}"
            job.stderr_text = _truncate(traceback.format_exc(), self.max_output_chars)
        finally:
            job.finished_mono = _now_mono()

            duration_s = max(0.0, job.finished_mono - job.started_mono)
            summary = {
                "job_id": job.job_id,
                "diag_key": job.diag_key,
                "state": job.state,
                "return_code": job.return_code,
                "timeout_hit": job.timeout_hit,
                "cancel_requested": job.cancel_requested,
                "duration_s": duration_s,
                "stdout_preview": _truncate(job.stdout_text, 500),
                "stderr_preview": _truncate(job.stderr_text, 500),
                "error_text": job.error_text or None,
            }

            with self._lock:
                if self._active_job and self._active_job.job_id == job.job_id:
                    self._active_job = None
                self._last_finished_summary = summary
                self._completed_count += 1

            level = "result"
            msg = f"Diagnostic '{job.diag_key}' finished with state={job.state}"
            self.get_logger().info(msg)
            self._publish_event(level, msg, summary)

    def _terminate_process(self, proc: subprocess.Popen, job: ActiveJob, reason: str) -> None:
        """
        Graceful terminate then kill if needed.
        """
        try:
            if proc.poll() is not None:
                return

            # POSIX-safe termination (Pi/Linux)
            proc.terminate()

            t0 = _now_mono()
            while (_now_mono() - t0) < self.terminate_grace_s:
                if proc.poll() is not None:
                    return
                time.sleep(0.05)

            if proc.poll() is None:
                proc.kill()
        except Exception as e:
            job.error_text = f"terminate_process_error: {type(e).__name__}: {e}"

    # =========================================================================
    # Publishers
    # =========================================================================
    def _publish_event(self, event_type: str, message: str, details: Optional[Dict[str, Any]] = None) -> None:
        payload = {
            "node": "base_diag_runner_node",
            "robot_name": self.robot_name,
            "event_type": event_type,
            "message": message,
            "time": _utc_iso_like(),
            "details": details or {},
        }

        self._last_event = payload
        self._event_count += 1

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), default=str)
        self.pub_event.publish(msg)

    def _publish_busy(self) -> None:
        with self._lock:
            busy = self._active_job is not None

        msg = Bool()
        msg.data = busy
        self.pub_busy.publish(msg)
        self._busy_pub_count += 1

    def _publish_state(self) -> None:
        now = _now_mono()
        uptime_s = max(0.0, now - self._started_mono)

        with self._lock:
            job = self._active_job
            active_job_payload = None

            if job is not None:
                active_job_payload = {
                    "job_id": job.job_id,
                    "diag_key": job.diag_key,
                    "state": job.state,
                    "timeout_s": job.timeout_s,
                    "started_age_s": max(0.0, now - job.started_mono),
                    "requested_args": list(job.requested_args),
                    "command": list(job.command),
                    "cancel_requested": job.cancel_requested,
                    "process_pid": job.process.pid if (job.process is not None and job.process.poll() is None) else None,
                }

            finished_summary = self._last_finished_summary

        registry_view = {
            k: {
                "description": v.description,
                "default_timeout_s": v.default_timeout_s,
                "allow_extra_args": v.allow_extra_args,
                "tags": v.tags,
                "command_preview": v.command,
            }
            for k, v in sorted(self._registry.items())
        }

        payload = {
            "node": "base_diag_runner_node",
            "robot_name": self.robot_name,
            "status_level": "BUSY" if active_job_payload else "IDLE",
            "uptime_s": uptime_s,
            "topics": {
                "run_request_topic": self.run_request_topic,
                "cancel_request_topic": self.cancel_request_topic,
                "state_topic": self.state_topic,
                "event_topic": self.event_topic,
                "busy_topic": self.busy_topic,
            },
            "execution_policy": {
                "allow_parallel_runs": self.allow_parallel_runs,
                "shell_mode": self.shell_mode,
                "capture_output": self.capture_output,
                "default_timeout_s": self.default_timeout_s,
                "max_timeout_s": self.max_timeout_s,
                "terminate_grace_s": self.terminate_grace_s,
                "cwd": self.savo_pi_root,
            },
            "counters": {
                "request_count": self._request_count,
                "reject_count": self._reject_count,
                "cancel_count": self._cancel_count,
                "completed_count": self._completed_count,
                "event_count": self._event_count,
                "state_pub_count": self._state_pub_count + 1,
                "busy_pub_count": self._busy_pub_count,
            },
            "active_job": active_job_payload,
            "last_finished_summary": finished_summary,
            "last_event": self._last_event,
            "registry": registry_view,
            "diagnostics": {
                "notes": [
                    "Only whitelisted diagnostics can be executed.",
                    "This node is intended for real robot diagnostics orchestration.",
                    "It does not directly drive motors or bypass safety nodes.",
                ]
            },
        }

        msg = String()
        if self.pretty_json:
            msg.data = json.dumps(payload, ensure_ascii=False, indent=2, default=str)
        else:
            msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), default=str)

        self.pub_state.publish(msg)
        self._state_pub_count += 1

    # =========================================================================
    # Shutdown
    # =========================================================================
    def destroy_node(self) -> bool:
        self.get_logger().info("Shutting down base_diag_runner_node...")

        # Best-effort cancel active process on shutdown
        with self._lock:
            job = self._active_job
            if job is not None:
                job.cancel_requested = True
                proc = job.process
            else:
                proc = None

        if proc is not None and proc.poll() is None:
            try:
                self._terminate_process(proc, job, reason="shutdown")
            except Exception:
                pass

        return super().destroy_node()


# =============================================================================
# Entry point
# =============================================================================
def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[BaseDiagRunnerNode] = None
    try:
        node = BaseDiagRunnerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[base_diag_runner_node] Fatal error: {e}")
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