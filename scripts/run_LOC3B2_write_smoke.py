#!/usr/bin/env python3

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time
from typing import Any, Callable, Optional, TypeVar

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String, UInt64

from savo_msgs.msg import LocationCandidate, LocationEvent
from savo_msgs.srv import (
    ApproveLocation,
    GetLocation,
    ListLocations,
    RegisterLocationCandidate,
    SetLocationEnabled,
)


T = TypeVar("T")

ROOT = Path.home() / "Savo_Pi"
WORKSPACE = ROOT / "savo_ws"
RUNTIME_ROOT = ROOT / "runtime" / "savo_locations_smoke"
STAMP = time.strftime("%Y%m%d_%H%M%S")
RUN_DIR = RUNTIME_ROOT / f"LOC3B2_{STAMP}"
DATABASE = RUN_DIR / "locations.db"
NODE_LOG = RUN_DIR / "savo_locations_node.log"
REPORT = RUN_DIR / "smoke_report.log"

ROS_DOMAIN_ID = "97"


def log(message: str = "") -> None:
    print(message, flush=True)
    with REPORT.open("a", encoding="utf-8") as stream:
        stream.write(message + "\n")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def wait_until(
    node: Node,
    predicate: Callable[[], bool],
    timeout_sec: float,
    description: str,
) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise RuntimeError(f"timeout waiting for {description}")


def call_service(
    node: Node,
    client: Any,
    request: Any,
    description: str,
    timeout_sec: float = 5.0,
) -> Any:
    require(
        client.wait_for_service(timeout_sec=timeout_sec),
        f"service unavailable: {description}",
    )
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if future.done():
            exception = future.exception()
            if exception is not None:
                raise RuntimeError(
                    f"{description} raised: {exception}"
                )
            return future.result()
    raise RuntimeError(f"service timeout: {description}")


def make_candidate() -> LocationCandidate:
    candidate = LocationCandidate()
    candidate.state = LocationCandidate.STATE_UNKNOWN
    candidate.candidate_revision = 0
    candidate.candidate_id = "candidate-loc3b2-smoke-27"

    candidate.map_id = "campus_main"
    candidate.map_revision = 7
    candidate.map_release_id = "campus_main_release_2026_07"

    candidate.tag_family = "tag36h11"
    candidate.tag_id = 27
    candidate.tag_pose_map.header.frame_id = "map"
    candidate.tag_pose_map.pose.position.x = 12.8
    candidate.tag_pose_map.pose.position.y = 8.1
    candidate.tag_pose_map.pose.orientation.w = 1.0

    candidate.detection_quality = 0.95
    candidate.accepted_observations = 8
    candidate.position_stddev_m = 0.02
    candidate.yaw_stddev_rad = 0.03

    candidate.approach_pose_valid = True
    candidate.approach_pose.header.frame_id = "map"
    candidate.approach_pose.pose.position.x = 12.0
    candidate.approach_pose.pose.position.y = 7.5
    candidate.approach_pose.pose.orientation.w = 1.0

    candidate.confirmation_pose_valid = True
    candidate.confirmation_pose.header.frame_id = "map"
    candidate.confirmation_pose.pose.position.x = 12.2
    candidate.confirmation_pose.pose.position.y = 7.7
    candidate.confirmation_pose.pose.orientation.w = 1.0

    candidate.suggested_location_id = "A201"
    candidate.suggested_display_name = "Room A201"
    candidate.suggested_aliases = ["East classroom"]
    candidate.suggested_semantic_type = "classroom"

    candidate.building = "Main"
    candidate.floor = "2"
    candidate.area = "East wing"
    candidate.notes = "LOC-3B2 isolated smoke fixture"

    candidate.source_session_id = "loc3b2-smoke-session"
    candidate.source_component = "savo_mapping"
    return candidate


class SmokeClient(Node):
    def __init__(self) -> None:
        super().__init__("savo_locations_loc3b2_smoke_client")

        transient_qos = QoSProfile(depth=10)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        event_qos = QoSProfile(depth=100)
        event_qos.reliability = ReliabilityPolicy.RELIABLE
        event_qos.durability = DurabilityPolicy.VOLATILE

        self.status: Optional[dict[str, Any]] = None
        self.snapshot: Optional[dict[str, Any]] = None
        self.heartbeat: Optional[int] = None
        self.events: list[LocationEvent] = []

        self.create_subscription(
            String,
            "/savo_locations/status",
            self._on_status,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/savo_locations/snapshot",
            self._on_snapshot,
            transient_qos,
        )
        self.create_subscription(
            UInt64,
            "/savo_locations/heartbeat",
            self._on_heartbeat,
            10,
        )
        self.create_subscription(
            LocationEvent,
            "/savo_locations/events",
            self.events.append,
            event_qos,
        )

        self.register_client = self.create_client(
            RegisterLocationCandidate,
            "/savo_locations/candidates/register",
        )
        self.approve_client = self.create_client(
            ApproveLocation,
            "/savo_locations/candidates/approve",
        )
        self.enabled_client = self.create_client(
            SetLocationEnabled,
            "/savo_locations/set_enabled",
        )
        self.get_client = self.create_client(
            GetLocation,
            "/savo_locations/get",
        )
        self.list_client = self.create_client(
            ListLocations,
            "/savo_locations/list",
        )

    def _on_status(self, message: String) -> None:
        try:
            self.status = json.loads(message.data)
        except json.JSONDecodeError:
            self.status = None

    def _on_snapshot(self, message: String) -> None:
        try:
            self.snapshot = json.loads(message.data)
        except json.JSONDecodeError:
            self.snapshot = None

    def _on_heartbeat(self, message: UInt64) -> None:
        self.heartbeat = int(message.data)


def start_registry() -> tuple[subprocess.Popen[str], Any]:
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID
    environment["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    environment.pop("ROS_LOCALHOST_ONLY", None)

    command = [
        "stdbuf",
        "-oL",
        "-eL",
        "ros2",
        "run",
        "savo_locations",
        "savo_locations_node",
        "--ros-args",
        "-p",
        f"database_path:={DATABASE}",
        "-p",
        "create_parent_directories:=true",
        "-p",
        "auto_migrate:=true",
        "-p",
        "enable_write_services:=true",
        "-p",
        "status_publish_hz:=10.0",
        "-p",
        "heartbeat_publish_hz:=10.0",
        "-p",
        "publish_snapshot:=true",
    ]

    stream = NODE_LOG.open("a", encoding="utf-8")
    process = subprocess.Popen(
        command,
        cwd=WORKSPACE,
        env=environment,
        stdout=stream,
        stderr=subprocess.STDOUT,
        text=True,
    )
    return process, stream


def stop_registry(
    process: Optional[subprocess.Popen[str]],
    stream: Optional[Any],
) -> None:
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
    if stream is not None:
        stream.close()


def inspect_database() -> None:
    require(DATABASE.is_file(), f"database missing: {DATABASE}")
    connection = sqlite3.connect(DATABASE)
    try:
        integrity = connection.execute(
            "PRAGMA integrity_check"
        ).fetchone()[0]
        version = connection.execute(
            "PRAGMA user_version"
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
        candidate = connection.execute(
            "SELECT state, candidate_revision "
            "FROM location_candidates "
            "WHERE candidate_id = ?",
            ("candidate-loc3b2-smoke-27",),
        ).fetchone()
        location = connection.execute(
            "SELECT state, enabled, record_revision "
            "FROM locations WHERE location_id = ?",
            ("A201",),
        ).fetchone()
        event_rows = connection.execute(
            "SELECT event_sequence, event_type, "
            "candidate_id, location_id, entity_revision "
            "FROM location_events ORDER BY event_sequence"
        ).fetchall()
    finally:
        connection.close()

    require(integrity == "ok", f"integrity check: {integrity}")
    require(version == 2, f"unexpected schema version: {version}")
    require(candidate_count == 1, "candidate count is not 1")
    require(location_count == 1, "location count is not 1")
    require(event_count == 3, "event count is not 3")
    require(candidate == (2, 2), f"candidate row mismatch: {candidate}")
    require(location == (1, 0, 2), f"location row mismatch: {location}")
    require(
        [row[0] for row in event_rows] == [1, 2, 3],
        f"event sequence mismatch: {event_rows}",
    )

    log("PASS: SQLite integrity, schema and committed rows")
    log(f"Database events: {event_rows}")


def main() -> int:
    RUN_DIR.mkdir(parents=True, exist_ok=False)
    REPORT.write_text("", encoding="utf-8")

    log("=" * 58)
    log("Robot Savo — LOC-3B2 Persistent Write Smoke Test")
    log("=" * 58)
    log(f"Run directory : {RUN_DIR}")
    log(f"Database      : {DATABASE}")
    log(f"Node log      : {NODE_LOG}")
    log(f"Report        : {REPORT}")
    log(f"ROS domain    : {ROS_DOMAIN_ID}")
    log()

    process: Optional[subprocess.Popen[str]] = None
    node_stream: Optional[Any] = None
    client: Optional[SmokeClient] = None

    try:
        os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        os.environ.pop("ROS_LOCALHOST_ONLY", None)

        rclpy.init()
        client = SmokeClient()
        process, node_stream = start_registry()

        wait_until(
            client,
            lambda: (
                client.status is not None
                and client.status.get("read_ready") is True
                and client.status.get("write_ready") is True
                and client.status.get("storage_healthy") is True
                and client.status.get("mode") == "read_write"
            ),
            10.0,
            "read/write-ready status",
        )
        wait_until(
            client,
            lambda: client.heartbeat is not None,
            5.0,
            "heartbeat",
        )
        require(process.poll() is None, "registry exited during startup")
        log("PASS: node started read/write ready")

        register_request = RegisterLocationCandidate.Request()
        register_request.candidate = make_candidate()
        register_request.actor_id = "mapping_operator"
        registered = call_service(
            client,
            client.register_client,
            register_request,
            "register candidate",
        )
        require(registered.registered, registered.reason)
        require(
            registered.result_code
            == RegisterLocationCandidate.Response.RESULT_REGISTERED,
            f"register code: {registered.result_code}",
        )
        require(
            registered.stored_candidate.candidate_revision == 1,
            "registered candidate revision is not 1",
        )
        log(f"PASS: registration committed — {registered.reason}")

        duplicate_request = RegisterLocationCandidate.Request()
        duplicate_request.candidate = make_candidate()
        duplicate_request.actor_id = "mapping_operator"
        duplicate = call_service(
            client,
            client.register_client,
            duplicate_request,
            "duplicate registration",
        )
        require(not duplicate.registered, "duplicate was accepted")
        require(
            duplicate.result_code
            == RegisterLocationCandidate.Response.RESULT_DUPLICATE_CANDIDATE_ID,
            f"duplicate code: {duplicate.result_code}",
        )
        log("PASS: duplicate candidate rejected")

        approve_request = ApproveLocation.Request()
        approve_request.candidate_id = "candidate-loc3b2-smoke-27"
        approve_request.expected_candidate_revision = 1
        approve_request.actor_id = "location_operator"
        approve_request.arrival_confirmation_required = True
        approved = call_service(
            client,
            client.approve_client,
            approve_request,
            "approve candidate",
        )
        require(approved.approved, approved.reason)
        require(
            approved.result_code
            == ApproveLocation.Response.RESULT_APPROVED,
            f"approve code: {approved.result_code}",
        )
        require(approved.location.location_id == "A201", "wrong location ID")
        require(approved.location.record_revision == 1, "wrong approval revision")
        require(approved.location.enabled, "approved location is disabled")
        log(f"PASS: approval committed — {approved.reason}")

        disable_request = SetLocationEnabled.Request()
        disable_request.location_id = "A201"
        disable_request.expected_record_revision = 1
        disable_request.enabled = False
        disable_request.actor_id = "location_operator"
        disable_request.reason = "LOC-3B2 smoke maintenance"
        disabled = call_service(
            client,
            client.enabled_client,
            disable_request,
            "disable location",
        )
        require(disabled.updated, disabled.reason)
        require(
            disabled.result_code
            == SetLocationEnabled.Response.RESULT_UPDATED,
            f"disable code: {disabled.result_code}",
        )
        require(not disabled.location.enabled, "location remained enabled")
        require(disabled.location.record_revision == 2, "wrong disable revision")
        log(f"PASS: disable committed — {disabled.reason}")

        wait_until(
            client,
            lambda: len(client.events) >= 3,
            5.0,
            "three committed events",
        )
        events = client.events[:3]
        require(
            [event.event_sequence for event in events] == [1, 2, 3],
            "published event sequence mismatch",
        )
        require(
            [event.event_type for event in events]
            == [
                LocationEvent.EVENT_CANDIDATE_REGISTERED,
                LocationEvent.EVENT_LOCATION_APPROVED,
                LocationEvent.EVENT_LOCATION_DISABLED,
            ],
            "published event types mismatch",
        )
        log("PASS: three post-commit typed events published in order")

        get_request = GetLocation.Request()
        get_request.location_id = "A201"
        get_request.include_disabled = True
        get_response = call_service(
            client,
            client.get_client,
            get_request,
            "get disabled location",
        )
        require(get_response.found, get_response.reason)
        require(not get_response.location.enabled, "get returned enabled location")
        require(get_response.location.record_revision == 2, "get revision mismatch")
        log("PASS: read service sees committed disabled location")

        list_request = ListLocations.Request()
        list_response = call_service(
            client,
            client.list_client,
            list_request,
            "list locations",
        )
        require(list_response.success, list_response.reason)
        require(len(list_response.locations) == 1, "list count is not 1")
        log("PASS: list service sees one committed location")

        wait_until(
            client,
            lambda: (
                client.status is not None
                and client.status.get("event_count") == 3
                and client.status.get("last_event_sequence") == 3
                and client.status.get("write_ready") is True
            ),
            5.0,
            "post-write status",
        )
        log("PASS: status reports event_count=3 and write_ready=true")

        stop_registry(process, node_stream)
        process = None
        node_stream = None
        inspect_database()

        # Restart against the same permanent database and verify bootstrap.
        client.events.clear()
        client.status = None
        process, node_stream = start_registry()
        wait_until(
            client,
            lambda: (
                client.status is not None
                and client.status.get("read_ready") is True
                and client.status.get("write_ready") is True
                and client.status.get("location_count") == 1
                and client.status.get("candidate_count") == 1
                and client.status.get("event_count") == 3
                and client.status.get("last_event_sequence") == 3
            ),
            10.0,
            "restart bootstrap status",
        )
        require(process.poll() is None, "registry exited after restart")

        restarted_get = call_service(
            client,
            client.get_client,
            get_request,
            "get after restart",
        )
        require(restarted_get.found, restarted_get.reason)
        require(not restarted_get.location.enabled, "restart lost disabled state")
        require(restarted_get.location.record_revision == 2, "restart lost revision")
        log("PASS: restart restored committed candidate, location and events")

        log()
        log("LOC-3B2 WRITE SMOKE TEST: PASS")
        log(f"Permanent artifacts: {RUN_DIR}")
        return 0

    except Exception as exception:
        log()
        log(f"LOC-3B2 WRITE SMOKE TEST: FAIL — {exception}")
        if NODE_LOG.exists():
            log("--- node log tail ---")
            for line in NODE_LOG.read_text(
                encoding="utf-8", errors="replace"
            ).splitlines()[-80:]:
                log(line)
        return 1

    finally:
        stop_registry(process, node_stream)
        if client is not None:
            client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
