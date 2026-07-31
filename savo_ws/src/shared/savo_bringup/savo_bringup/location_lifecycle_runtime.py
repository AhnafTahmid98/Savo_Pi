#!/usr/bin/env python3
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary
"""Run the production location launch through one complete typed lifecycle."""

from __future__ import annotations

import json
import os
import signal
import sqlite3
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32

from savo_msgs.action import NavigateToLocation, RegisterMappedLocation
from savo_msgs.msg import AprilTagObservation
from savo_msgs.srv import (
    AuthorizeLocationOperation,
    ResolveLocation,
    ReviewLocationCandidate,
)


RUNTIME_ROOT = (
    Path.home() / "Savo_Pi" / "runtime" / "location_lifecycle_phase2d"
)
ROS_DOMAIN_MIN = 140
ROS_DOMAIN_SPAN = 80


@dataclass
class ManagedProcess:
    """One child process and its persistent log."""

    name: str
    process: subprocess.Popen[str]
    log_handle: Any
    log_path: Path


class LifecycleNode(Node):
    """Drive the public typed APIs and synthetic AprilTag observations."""

    def __init__(self) -> None:
        super().__init__("phase2d_location_lifecycle_runtime")
        self.sequence = 0
        self.publish_observations = False
        self.last_navigation_goal: PoseStamped | None = None

        self.observation_publisher = self.create_publisher(
            AprilTagObservation,
            "/savo_head/apriltag/observations",
            10,
        )
        goal_qos = QoSProfile(depth=1)
        goal_qos.reliability = ReliabilityPolicy.RELIABLE
        goal_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            "/savo_nav/test/last_location_goal",
            self._on_navigation_goal,
            goal_qos,
        )
        self.observation_timer = self.create_timer(
            0.05,
            self._publish_observation,
        )
        self.safety_stop_publisher = self.create_publisher(
            Bool,
            "/safety/stop",
            10,
        )
        self.safety_slowdown_publisher = self.create_publisher(
            Float32,
            "/safety/slowdown_factor",
            10,
        )
        self.safety_timer = self.create_timer(
            0.10,
            self._publish_clear_safety,
        )

        self.registration_action = ActionClient(
            self,
            RegisterMappedLocation,
            "/savo_mapping/locations/register",
        )
        self.navigation_action = ActionClient(
            self,
            NavigateToLocation,
            "/savo_nav/locations/navigate",
        )
        self.review_client = self.create_client(
            ReviewLocationCandidate,
            "/savo_mapping/locations/review",
        )
        self.authorization_client = self.create_client(
            AuthorizeLocationOperation,
            "/savo_supervisor/authorize_location_operation",
        )
        self.resolve_client = self.create_client(
            ResolveLocation,
            "/savo_locations/resolve",
        )

    def _on_navigation_goal(self, message: PoseStamped) -> None:
        self.last_navigation_goal = message

    def _publish_clear_safety(self) -> None:
        """Publish fresh clear-safety evidence for this isolated fixture."""
        stop = Bool()
        stop.data = False
        slowdown = Float32()
        slowdown.data = 1.0
        self.safety_stop_publisher.publish(stop)
        self.safety_slowdown_publisher.publish(slowdown)

    def _publish_observation(self) -> None:
        if not self.publish_observations:
            return

        self.sequence += 1
        message = AprilTagObservation()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.detector_name = "phase2d_lifecycle_fixture"
        message.family = "tag36h11"
        message.tag_id = 27
        message.tag_size_m = 0.16
        message.observation_sequence = self.sequence
        message.image_width = 640
        message.image_height = 480
        message.detection_quality = 0.98
        message.decision_margin = 90.0
        message.hamming_distance = 0
        message.pose_valid = True
        message.pose.pose.position.x = 4.0
        message.pose.pose.position.y = 2.0
        message.pose.pose.position.z = 0.8
        message.pose.pose.orientation.w = 1.0
        message.pose_error = 0.01
        self.observation_publisher.publish(message)


def allocate_ros_domain_id() -> str:
    """Choose a per-run DDS domain to avoid stale-process collisions."""
    seed = time.time_ns() ^ os.getpid()
    return str(ROS_DOMAIN_MIN + (seed % ROS_DOMAIN_SPAN))


def wait_future(node: Node, future: Any, timeout_s: float, label: str) -> Any:
    """Spin until a ROS future completes or fail deterministically."""
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise RuntimeError(f"timeout waiting for {label}")
    return future.result()


def wait_condition(
    node: Node,
    condition: Callable[[], bool],
    timeout_s: float,
    label: str,
) -> None:
    """Spin until a runtime condition becomes true."""
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not condition() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not condition():
        raise RuntimeError(f"timeout waiting for {label}")


def start_process(
    run_dir: Path,
    environment: dict[str, str],
    name: str,
    command: list[str],
) -> ManagedProcess:
    """Start one isolated process group and retain its output."""
    log_path = run_dir / f"{name}.log"
    handle = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        stdout=handle,
        stderr=subprocess.STDOUT,
        text=True,
        env=environment,
        start_new_session=True,
    )
    return ManagedProcess(name, process, handle, log_path)


def stop_process(managed: ManagedProcess) -> None:
    """Stop one process group without leaving ROS children behind."""
    if managed.process.poll() is None:
        try:
            os.killpg(managed.process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            managed.process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(managed.process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                managed.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(managed.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                managed.process.wait(timeout=2.0)
    managed.log_handle.close()


def wait_dependencies(node: LifecycleNode) -> None:
    """Wait for every public lifecycle boundary."""
    if not node.registration_action.wait_for_server(timeout_sec=20.0):
        raise RuntimeError("registration action unavailable")
    if not node.navigation_action.wait_for_server(timeout_sec=20.0):
        raise RuntimeError("navigation action unavailable")
    if not node.review_client.wait_for_service(timeout_sec=20.0):
        raise RuntimeError("authorized review service unavailable")
    if not node.authorization_client.wait_for_service(timeout_sec=20.0):
        raise RuntimeError("supervisor authorization service unavailable")
    if not node.resolve_client.wait_for_service(timeout_sec=20.0):
        raise RuntimeError("location resolution service unavailable")


def wait_supervisor_ready(node: LifecycleNode) -> None:
    """Wait until the supervisor can authorize non-motion registration."""
    deadline = time.monotonic() + 10.0
    last_reason = "supervisor_authorization_not_evaluated"

    while rclpy.ok() and time.monotonic() < deadline:
        request = AuthorizeLocationOperation.Request()
        request.operation = (
            AuthorizeLocationOperation.Request.OP_REGISTER_LOCATION_CANDIDATE
        )
        request.request_id = "phase2d-readiness-probe"
        request.actor_id = "phase2d_operator"
        request.candidate_id = "candidate-phase2d-readiness-probe"
        request.map_id = "campus_main"
        request.map_revision = 7
        request.motion_required = False

        response = wait_future(
            node,
            node.authorization_client.call_async(request),
            2.0,
            "supervisor readiness authorization",
        )
        if response.authorized:
            return

        last_reason = response.reason
        if response.result_code != (
            AuthorizeLocationOperation.Response.RESULT_SUPERVISOR_NOT_READY
        ):
            raise RuntimeError(
                "supervisor readiness probe rejected: " + last_reason
            )

        rclpy.spin_once(node, timeout_sec=0.10)

    raise RuntimeError(
        "timeout waiting for supervisor readiness: " + last_reason
    )


def register_candidate(node: LifecycleNode) -> Any:
    """Confirm one tag and persist a pending candidate."""
    goal = RegisterMappedLocation.Goal()
    goal.request_id = "phase2d-register-001"
    goal.actor_id = "phase2d_operator"
    goal.candidate_id = "candidate-phase2d-a201-27"
    goal.expected_family = "tag36h11"
    goal.expected_tag_id = 27
    goal.map_id = "campus_main"
    goal.map_revision = 7
    goal.map_release_id = "campus-main-r7"
    goal.suggested_location_id = "A201"
    goal.suggested_display_name = "Room A201"
    goal.suggested_aliases = ["A 201", "classroom A201"]
    goal.suggested_semantic_type = "classroom"
    goal.building = "A"
    goal.floor = "2"
    goal.area = "teaching"
    goal.notes = "Phase 2D full lifecycle candidate"
    goal.source_session_id = "phase2d-map-session"
    goal.timeout.sec = 12

    node.publish_observations = True
    goal_handle = wait_future(
        node,
        node.registration_action.send_goal_async(goal),
        6.0,
        "registration goal response",
    )
    if not goal_handle.accepted:
        raise RuntimeError("registration goal rejected")

    wrapped = wait_future(
        node,
        goal_handle.get_result_async(),
        20.0,
        "registration result",
    )
    result = wrapped.result
    if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.registered:
        raise RuntimeError(f"registration failed: {result.reason}")
    if result.candidate.candidate_revision != 1:
        raise RuntimeError("candidate revision is not 1")
    return result.candidate


def approve_candidate(node: LifecycleNode, candidate: Any) -> Any:
    """Approve the candidate only through the mapping-owned gateway."""
    request = ReviewLocationCandidate.Request()
    request.request_id = "phase2d-review-001"
    request.actor_id = "phase2d_operator"
    request.decision = ReviewLocationCandidate.Request.DECISION_APPROVE
    request.candidate_id = candidate.candidate_id
    request.expected_candidate_revision = candidate.candidate_revision
    request.location_id = "A201"
    request.display_name = "Room A201"
    request.aliases = ["A 201", "classroom A201"]
    request.semantic_type = "classroom"
    request.approach_pose = candidate.approach_pose
    request.confirmation_pose_valid = True
    request.confirmation_pose = candidate.confirmation_pose
    request.arrival_confirmation_required = True
    request.building = candidate.building
    request.floor = candidate.floor
    request.area = candidate.area
    request.notes = candidate.notes

    response = wait_future(
        node,
        node.review_client.call_async(request),
        12.0,
        "authorized review response",
    )
    if not response.completed or not response.approved:
        raise RuntimeError(f"approval failed: {response.reason}")
    return response.location


def resolve_location(node: LifecycleNode) -> Any:
    """Resolve the approved location through a saved alias."""
    request = ResolveLocation.Request()
    request.query = "A 201"
    request.enforce_map_context = True
    request.map_id = "campus_main"
    request.map_revision = 7
    response = wait_future(
        node,
        node.resolve_client.call_async(request),
        10.0,
        "location resolution",
    )
    if not response.resolved:
        raise RuntimeError(f"resolution failed: {response.reason}")
    return response.location


def navigate_and_confirm(node: LifecycleNode) -> Any:
    """Navigate to the approach pose and confirm the saved tag at arrival."""
    node.last_navigation_goal = None
    goal = NavigateToLocation.Goal()
    goal.request_id = "phase2d-navigation-001"
    goal.actor_id = "phase2d_operator"
    goal.query = "classroom A201"
    goal.enforce_map_context = True
    goal.map_id = "campus_main"
    goal.map_revision = 7
    goal.require_arrival_confirmation = True
    goal.timeout.sec = 20

    goal_handle = wait_future(
        node,
        node.navigation_action.send_goal_async(goal),
        6.0,
        "navigation goal response",
    )
    if not goal_handle.accepted:
        raise RuntimeError("semantic navigation goal rejected")

    wrapped = wait_future(
        node,
        goal_handle.get_result_async(),
        25.0,
        "semantic navigation result",
    )
    result = wrapped.result
    if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.succeeded:
        raise RuntimeError(f"navigation failed: {result.reason}")
    if not result.navigation_succeeded or not result.arrival_confirmed:
        raise RuntimeError("navigation or arrival confirmation flag missing")
    if result.final_observation.tag_id != 27:
        raise RuntimeError("arrival confirmation returned the wrong tag")
    return result


def run() -> int:
    """Execute Phase 2D and return a process exit code."""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run_dir = RUNTIME_ROOT / f"PHASE2D_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=False)
    database = run_dir / "locations.db"
    report_path = run_dir / "lifecycle_report.json"

    domain_id = allocate_ros_domain_id()
    os.environ["ROS_DOMAIN_ID"] = domain_id
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    os.environ["PYTHONDONTWRITEBYTECODE"] = "1"
    environment = os.environ.copy()

    processes: list[ManagedProcess] = []
    node: LifecycleNode | None = None
    report: dict[str, Any] = {
        "phase": "2D",
        "run_directory": str(run_dir),
        "database": str(database),
        "ros_domain_id": int(domain_id),
        "checks": [],
    }

    def passed(check: str) -> None:
        report["checks"].append({"status": "PASS", "check": check})
        print(f"PASS: {check}")

    print("=" * 72)
    print("Robot Savo — Phase 2D Full Location Lifecycle Runtime")
    print("=" * 72)
    print(f"Run directory : {run_dir}")
    print(f"Database      : {database}")
    print(f"ROS domain    : {domain_id}")

    try:
        processes.append(
            start_process(
                run_dir,
                environment,
                "fake_nav2",
                [
                    "ros2",
                    "run",
                    "savo_nav",
                    "fake_location_nav2_server_node",
                    "--ros-args",
                    "-p",
                    "step_count:=8",
                    "-p",
                    "step_delay_ms:=40",
                ],
            )
        )
        processes.append(
            start_process(
                run_dir,
                environment,
                "location_lifecycle_launch",
                [
                    "ros2",
                    "launch",
                    "savo_bringup",
                    "location_integration.launch.py",
                    "start_head_observer:=false",
                    f"locations_database_path:={database}",
                    "locations_create_parent_directories:=true",
                    "supervisor_startup_grace_s:=0.0",
                    "supervisor_base_enabled:=false",
                    "supervisor_base_required:=false",
                    "supervisor_control_enabled:=false",
                    "supervisor_control_required:=false",
                    "supervisor_perception_enabled:=false",
                    "supervisor_perception_required:=false",
                    "supervisor_lidar_enabled:=false",
                    "supervisor_lidar_required:=false",
                    "supervisor_power_enabled:=false",
                    "supervisor_power_required:=false",
                    "supervisor_localization_enabled:=false",
                    "supervisor_localization_required:=false",
                    "head_minimum_observations:=3",
                    "head_maximum_observation_age_s:=2.0",
                    "head_wrong_tag_grace_s:=0.3",
                    "registration_dependency_wait_timeout_s:=3.0",
                    "review_dependency_wait_timeout_s:=3.0",
                    "review_operation_timeout_s:=6.0",
                    "navigation_action_name:=/navigate_to_pose",
                    "navigation_dependency_wait_timeout_s:=3.0",
                    "navigation_authorization_recheck_period_s:=0.2",
                    "arrival_confirmation_timeout_s:=4.0",
                ],
            )
        )

        rclpy.init(args=None)
        node = LifecycleNode()
        wait_dependencies(node)
        passed("production launch exposed all public lifecycle boundaries")
        wait_supervisor_ready(node)
        passed("supervisor authorized non-motion location registration")


        candidate = register_candidate(node)
        passed("AprilTag evidence produced one persistent pending candidate")
        if abs(candidate.tag_pose_map.pose.position.x - 4.0) > 1.0e-6:
            raise RuntimeError("candidate tag pose mismatch")
        if abs(candidate.approach_pose.pose.position.x - 3.2) > 1.0e-6:
            raise RuntimeError("derived approach pose mismatch")
        passed("tag evidence remained separate from the movement target")

        approved = approve_candidate(node, candidate)
        if approved.location_id != "A201" or approved.record_revision != 1:
            raise RuntimeError("approved location record mismatch")
        passed("supervisor-authorized review created location revision 1")

        resolved = resolve_location(node)
        if resolved.source_candidate_id != candidate.candidate_id:
            raise RuntimeError("resolved record lost candidate provenance")
        passed("approved location resolved by alias and active map context")

        navigation = navigate_and_confirm(node)
        wait_condition(
            node,
            lambda: node.last_navigation_goal is not None,
            5.0,
            "captured downstream navigation goal",
        )
        assert node.last_navigation_goal is not None
        goal_x = node.last_navigation_goal.pose.position.x
        if abs(goal_x - approved.approach_pose.pose.position.x) > 1.0e-6:
            raise RuntimeError(
                "Nav2 did not receive the approved approach pose"
            )
        if abs(goal_x - approved.tag_pose_map.pose.position.x) < 0.05:
            raise RuntimeError(
                "Nav2 received forbidden AprilTag evidence pose"
            )
        if navigation.location.location_id != "A201":
            raise RuntimeError("navigation returned the wrong location")
        passed("navigation forwarded only the approved approach pose")
        passed("arrival confirmation matched the saved AprilTag")

        connection = sqlite3.connect(database)
        try:
            integrity = connection.execute(
                "PRAGMA integrity_check"
            ).fetchone()[0]
            candidate_count = connection.execute(
                "SELECT COUNT(*) FROM location_candidates"
            ).fetchone()[0]
            location_count = connection.execute(
                "SELECT COUNT(*) FROM locations"
            ).fetchone()[0]
            event_count = connection.execute(
                "SELECT COUNT(*) FROM location_events"
            ).fetchone()[0]
        finally:
            connection.close()

        if integrity != "ok":
            raise RuntimeError("SQLite integrity check failed")
        if (candidate_count, location_count, event_count) != (1, 1, 2):
            raise RuntimeError(
                "unexpected persistent row counts: "
                f"candidates={candidate_count} locations={location_count} "
                f"events={event_count}"
            )
        passed(
            "SQLite contains exactly one candidate, one location "
            "and two events"
        )

        report["status"] = "PASS"
        report["database_counts"] = {
            "candidates": candidate_count,
            "locations": location_count,
            "events": event_count,
        }
        print("\nPHASE 2D LOCATION LIFECYCLE RUNTIME: PASS")
        print(f"Permanent artifacts: {run_dir}")
        return 0
    except Exception as exception:  # noqa: BLE001
        report["status"] = "FAIL"
        report["error"] = str(exception)
        print(f"\nPHASE 2D LOCATION LIFECYCLE RUNTIME: FAIL — {exception}")
        print(f"Permanent artifacts: {run_dir}")
        return 1
    finally:
        report_path.write_text(
            json.dumps(report, indent=2) + "\n",
            encoding="utf-8",
        )
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for managed in reversed(processes):
            stop_process(managed)


def main() -> None:
    """Console-script entry point."""
    sys.exit(run())


if __name__ == "__main__":
    main()
