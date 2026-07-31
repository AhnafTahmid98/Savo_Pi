# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Isolated runtime validation for location review gateway and CLI."""

import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from savo_msgs.msg import LocationCandidate
from savo_msgs.msg import LocationRecord
from savo_msgs.srv import ApproveLocation
from savo_msgs.srv import AuthorizeLocationOperation
from savo_msgs.srv import GetLocationCandidate
from savo_msgs.srv import ListLocationCandidates
from savo_msgs.srv import RejectLocationCandidate
from savo_msgs.srv import ReviewLocationCandidate


PROCESS_TIMEOUT_SEC = 12.0


def wait_until(predicate, timeout=6.0, interval=0.02):
    """Wait until a predicate succeeds."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def installed_executable(variable):
    """Return an executable path supplied by CMake."""
    path = Path(os.environ.get(variable, ''))
    assert path.is_file() and os.access(path, os.X_OK), path
    return str(path)


def make_candidate(*, state=None, revision=3):
    """Create a valid authoritative review candidate."""
    candidate = LocationCandidate()
    candidate.state = (
        LocationCandidate.STATE_PENDING_REVIEW
        if state is None
        else state
    )
    candidate.candidate_revision = revision
    candidate.candidate_id = 'candidate-campus-main-27'
    candidate.map_id = 'campus_main'
    candidate.map_revision = 7
    candidate.map_release_id = 'campus-main-r7'
    candidate.tag_family = 'tag36h11'
    candidate.tag_id = 27
    candidate.tag_pose_map.header.frame_id = 'map'
    candidate.tag_pose_map.pose.position.x = 4.0
    candidate.tag_pose_map.pose.position.y = 2.0
    candidate.tag_pose_map.pose.orientation.w = 1.0
    candidate.detection_quality = 0.97
    candidate.accepted_observations = 5
    candidate.position_stddev_m = 0.02
    candidate.yaw_stddev_rad = 0.03
    candidate.approach_pose_valid = True
    candidate.approach_pose.header.frame_id = 'map'
    candidate.approach_pose.pose.position.x = 3.2
    candidate.approach_pose.pose.position.y = 2.0
    candidate.approach_pose.pose.orientation.w = 1.0
    candidate.confirmation_pose_valid = True
    candidate.confirmation_pose = candidate.approach_pose
    candidate.suggested_location_id = 'A201'
    candidate.suggested_display_name = 'Room A201'
    candidate.suggested_aliases = ['A 201', 'classroom A201']
    candidate.suggested_semantic_type = 'classroom'
    candidate.building = 'A'
    candidate.floor = '2'
    candidate.area = 'teaching'
    candidate.notes = 'runtime fixture candidate'
    candidate.source_session_id = 'runtime-map-session'
    candidate.source_component = 'savo_mapping'
    return candidate


def make_location():
    """Create an approved location returned by the fake registry."""
    location = LocationRecord()
    location.state = LocationRecord.STATE_APPROVED
    location.enabled = True
    location.record_revision = 1
    location.location_id = 'A201'
    location.display_name = 'Room A201'
    location.aliases = ['A 201', 'classroom A201']
    location.semantic_type = 'classroom'
    location.map_id = 'campus_main'
    location.map_revision = 7
    location.map_release_id = 'campus-main-r7'
    location.approach_pose.header.frame_id = 'map'
    location.approach_pose.pose.position.x = 3.2
    location.approach_pose.pose.position.y = 2.0
    location.approach_pose.pose.orientation.w = 1.0
    location.confirmation_pose_valid = True
    location.confirmation_pose = location.approach_pose
    location.tag_family = 'tag36h11'
    location.tag_id = 27
    location.tag_pose_map_valid = True
    location.tag_pose_map.header.frame_id = 'map'
    location.tag_pose_map.pose.position.x = 4.0
    location.tag_pose_map.pose.position.y = 2.0
    location.tag_pose_map.pose.orientation.w = 1.0
    location.arrival_confirmation_required = True
    location.building = 'A'
    location.floor = '2'
    location.area = 'teaching'
    location.source_candidate_id = 'candidate-campus-main-27'
    return location


def review_request(*, approve=True, revision=3, suffix='runtime'):
    """Create a gateway request."""
    request = ReviewLocationCandidate.Request()
    request.request_id = f'review-{suffix}-{time.monotonic_ns()}'
    request.actor_id = 'runtime_operator'
    request.decision = (
        ReviewLocationCandidate.Request.DECISION_APPROVE
        if approve
        else ReviewLocationCandidate.Request.DECISION_REJECT
    )
    request.candidate_id = 'candidate-campus-main-27'
    request.expected_candidate_revision = revision
    request.arrival_confirmation_required = True
    if not approve:
        request.rejection_reason = 'duplicate doorway marker'
    return request


class RuntimeHarness:
    """Run the production gateway against controlled typed dependencies."""

    def __init__(
        self,
        *,
        operation_timeout=0.5,
        lookup_delay=0.0,
        authorization_delay=0.0,
        approval_delay=0.0,
        rejection_delay=0.0,
        authorized=True,
        approval_mode='success',
        rejection_mode='success',
    ):
        suffix = f'{os.getpid()}_{time.monotonic_ns()}'
        self.candidate_service = f'/test/candidate_get_{suffix}'
        self.candidate_list_service = f'/test/candidate_list_{suffix}'
        self.authorization_service = f'/test/authorize_{suffix}'
        self.approval_service = f'/test/approve_{suffix}'
        self.rejection_service = f'/test/reject_{suffix}'
        self.review_service = f'/test/review_{suffix}'
        self.status_topic = f'/test/review_status_{suffix}'
        self.result_topic = f'/test/review_results_{suffix}'
        self.heartbeat_topic = f'/test/review_heartbeat_{suffix}'

        self.lookup_delay = lookup_delay
        self.authorization_delay = authorization_delay
        self.approval_delay = approval_delay
        self.rejection_delay = rejection_delay
        self.authorized = authorized
        self.approval_mode = approval_mode
        self.rejection_mode = rejection_mode

        self.lookup_count = 0
        self.list_count = 0
        self.authorization_count = 0
        self.approval_count = 0
        self.rejection_count = 0
        self.lookup_started = threading.Event()
        self.last_authorization = None
        self.last_approval = None
        self.last_rejection = None

        self.candidate = make_candidate()
        self.node = rclpy.create_node(f'location_review_fixture_{suffix}')
        self.group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.executor.add_node(self.node)

        self.candidate_server = self.node.create_service(
            GetLocationCandidate,
            self.candidate_service,
            self.handle_candidate,
            callback_group=self.group,
        )
        self.candidate_list_server = self.node.create_service(
            ListLocationCandidates,
            self.candidate_list_service,
            self.handle_candidate_list,
            callback_group=self.group,
        )
        self.authorization_server = self.node.create_service(
            AuthorizeLocationOperation,
            self.authorization_service,
            self.handle_authorization,
            callback_group=self.group,
        )
        self.approval_server = self.node.create_service(
            ApproveLocation,
            self.approval_service,
            self.handle_approval,
            callback_group=self.group,
        )
        self.rejection_server = self.node.create_service(
            RejectLocationCandidate,
            self.rejection_service,
            self.handle_rejection,
            callback_group=self.group,
        )
        self.review_client = self.node.create_client(
            ReviewLocationCandidate,
            self.review_service,
            callback_group=self.group,
        )

        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.spin_thread.start()

        command = [
            installed_executable('LOCATION_REVIEW_GATEWAY_EXECUTABLE'),
            '--ros-args',
            '-p', f'service_name:={self.review_service}',
            '-p', f'candidate_lookup_service:={self.candidate_service}',
            '-p', f'authorization_service:={self.authorization_service}',
            '-p', f'approval_service:={self.approval_service}',
            '-p', f'rejection_service:={self.rejection_service}',
            '-p', f'status_topic:={self.status_topic}',
            '-p', f'result_topic:={self.result_topic}',
            '-p', f'heartbeat_topic:={self.heartbeat_topic}',
            '-p', 'dependency_wait_timeout_s:=0.25',
            '-p', f'operation_timeout_s:={operation_timeout}',
            '-p', 'status_publish_hz:=10.0',
            '-p', 'heartbeat_publish_hz:=10.0',
        ]
        environment = os.environ.copy()
        environment['PYTHONDONTWRITEBYTECODE'] = '1'
        environment['ROS_LOCALHOST_ONLY'] = '1'
        self.process = subprocess.Popen(
            command,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

        ready = wait_until(
            lambda: self.process.poll() is not None
            or self.review_client.service_is_ready()
        )
        if not ready or self.process.poll() is not None:
            diagnostics = self.diagnostics()
            self.close()
            raise AssertionError(diagnostics)

    def handle_candidate(self, request, response):
        self.lookup_count += 1
        self.lookup_started.set()
        if self.lookup_delay:
            time.sleep(self.lookup_delay)
        if request.candidate_id != self.candidate.candidate_id:
            response.found = False
            response.result_code = (
                GetLocationCandidate.Response.RESULT_NOT_FOUND
            )
            response.reason = 'candidate not found'
            return response
        response.found = True
        response.result_code = GetLocationCandidate.Response.RESULT_FOUND
        response.reason = 'candidate found'
        response.candidate = self.candidate
        return response

    def handle_candidate_list(self, request, response):
        self.list_count += 1
        matches_state = request.state_filter in (
            ListLocationCandidates.Request.STATE_FILTER_ALL,
            self.candidate.state,
        )
        matches_map = (
            not request.enforce_map_context
            or (
                request.map_id == self.candidate.map_id
                and request.map_revision == self.candidate.map_revision
            )
        )
        response.success = True
        response.result_code = ListLocationCandidates.Response.RESULT_OK
        response.reason = 'candidate list returned'
        if matches_state and matches_map:
            response.candidates = [self.candidate]
        return response

    def handle_authorization(self, request, response):
        self.authorization_count += 1
        self.last_authorization = request
        if self.authorization_delay:
            time.sleep(self.authorization_delay)
        response.authorized = self.authorized
        response.result_code = (
            AuthorizeLocationOperation.Response.RESULT_AUTHORIZED
            if self.authorized
            else AuthorizeLocationOperation.Response.RESULT_OPERATION_DISABLED
        )
        response.reason = (
            'authorized by runtime fixture'
            if self.authorized
            else 'review operation disabled by runtime fixture'
        )
        response.supervisor_lifecycle = 'RUNNING'
        response.supervisor_health = 'OK'
        response.supervisor_safety = 'UNKNOWN'
        return response

    def handle_approval(self, request, response):
        self.approval_count += 1
        self.last_approval = request
        if self.approval_delay:
            time.sleep(self.approval_delay)
        if self.approval_mode == 'success':
            response.approved = True
            response.result_code = ApproveLocation.Response.RESULT_APPROVED
            response.reason = 'candidate approved by runtime fixture'
            response.location = make_location()
        elif self.approval_mode == 'conflict':
            response.approved = False
            response.result_code = (
                ApproveLocation.Response.RESULT_LOCATION_ID_CONFLICT
            )
            response.reason = 'location ID already exists'
        elif self.approval_mode == 'storage':
            response.approved = False
            response.result_code = (
                ApproveLocation.Response.RESULT_STORAGE_UNAVAILABLE
            )
            response.reason = 'registry storage unavailable'
        else:
            raise AssertionError(self.approval_mode)
        return response

    def handle_rejection(self, request, response):
        self.rejection_count += 1
        self.last_rejection = request
        if self.rejection_delay:
            time.sleep(self.rejection_delay)
        if self.rejection_mode == 'success':
            response.rejected = True
            response.result_code = (
                RejectLocationCandidate.Response.RESULT_REJECTED
            )
            response.reason = 'candidate rejected by runtime fixture'
            response.candidate = make_candidate(
                state=LocationCandidate.STATE_REJECTED,
                revision=4,
            )
            response.candidate.review_reason = request.rejection_reason
        elif self.rejection_mode == 'storage':
            response.rejected = False
            response.result_code = (
                RejectLocationCandidate.Response.RESULT_STORAGE_UNAVAILABLE
            )
            response.reason = 'registry storage unavailable'
        else:
            raise AssertionError(self.rejection_mode)
        return response

    def call_review(self, request, timeout=5.0):
        assert self.review_client.wait_for_service(timeout_sec=2.0)
        future = self.review_client.call_async(request)
        assert wait_until(future.done, timeout=timeout), self.diagnostics()
        return future.result()

    def diagnostics(self):
        output = ''
        if self.process.poll() is not None and self.process.stdout:
            output = self.process.stdout.read()
        return (
            f'poll={self.process.poll()} lookups={self.lookup_count} '
            f'authorizations={self.authorization_count} '
            f'approvals={self.approval_count} '
            f'rejections={self.rejection_count} output={output}'
        )

    def close(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.communicate(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.communicate(timeout=2.0)

        self.executor.shutdown(timeout_sec=2.0)
        self.spin_thread.join(timeout=2.0)
        assert not self.spin_thread.is_alive()
        self.node.destroy_node()


@pytest.fixture(scope='module', autouse=True)
def isolated_ros_context():
    """Use an isolated ROS context for the runtime module."""
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') == '225'

    if not rclpy.ok():
        rclpy.init()

    yield

    if rclpy.ok():
        rclpy.shutdown()


def run_with_harness(test_body, **kwargs):
    """Create and always close a runtime harness."""
    harness = RuntimeHarness(**kwargs)
    try:
        test_body(harness)
    finally:
        harness.close()


def test_gateway_approves_after_authoritative_lookup_and_authorization():
    """Approval reaches exactly one raw mutation after authorization."""

    def body(harness):
        response = harness.call_review(review_request(approve=True))
        assert response.completed
        assert response.approved
        assert not response.rejected
        assert response.result_code == (
            ReviewLocationCandidate.Response.RESULT_APPROVED
        )
        assert response.location.location_id == 'A201'
        assert harness.lookup_count == 1
        assert harness.authorization_count == 1
        assert harness.approval_count == 1
        assert harness.rejection_count == 0
        assert harness.last_authorization.map_id == 'campus_main'
        assert harness.last_authorization.map_revision == 7
        assert harness.last_authorization.operation == (
            AuthorizeLocationOperation.Request.OP_APPROVE_LOCATION
        )

    run_with_harness(body)


def test_gateway_rejects_after_authorization():
    """Rejection reaches exactly one raw rejection mutation."""

    def body(harness):
        response = harness.call_review(review_request(approve=False))
        assert response.completed
        assert response.rejected
        assert not response.approved
        assert response.result_code == (
            ReviewLocationCandidate.Response.RESULT_REJECTED
        )
        assert response.candidate.state == LocationCandidate.STATE_REJECTED
        assert harness.authorization_count == 1
        assert harness.approval_count == 0
        assert harness.rejection_count == 1
        assert harness.last_authorization.operation == (
            AuthorizeLocationOperation.Request.OP_REJECT_LOCATION_CANDIDATE
        )
        assert harness.last_rejection.rejection_reason == (
            'duplicate doorway marker'
        )

    run_with_harness(body)


def test_stale_revision_is_rejected_before_authorization():
    """A stale operator view cannot reach the supervisor or registry."""

    def body(harness):
        response = harness.call_review(
            review_request(approve=True, revision=2)
        )
        assert not response.completed
        assert response.result_code == (
            ReviewLocationCandidate.Response.RESULT_STALE_REVISION
        )
        assert harness.lookup_count == 1
        assert harness.authorization_count == 0
        assert harness.approval_count == 0
        assert harness.rejection_count == 0

    run_with_harness(body)


def test_supervisor_denial_blocks_registry_mutation():
    """Supervisor denial remains permission-only and fail-closed."""

    def body(harness):
        response = harness.call_review(review_request(approve=True))
        assert not response.completed
        assert response.result_code == (
            ReviewLocationCandidate.Response.RESULT_SUPERVISOR_DENIED
        )
        assert harness.authorization_count == 1
        assert harness.approval_count == 0
        assert harness.rejection_count == 0

    run_with_harness(body, authorized=False)


def test_registry_conflict_is_reported_without_false_completion():
    """A registry conflict maps to deterministic gateway rejection."""

    def body(harness):
        response = harness.call_review(review_request(approve=True))
        assert not response.completed
        assert response.result_code == (
            ReviewLocationCandidate.Response.RESULT_REGISTRY_REJECTED
        )
        assert harness.approval_count == 1

    run_with_harness(body, approval_mode='conflict')


def test_registry_storage_failure_is_fail_closed():
    """A downstream storage failure cannot report review completion."""

    def body(harness):
        response = harness.call_review(review_request(approve=True))
        assert not response.completed
        assert response.result_code == (
            ReviewLocationCandidate.Response.RESULT_REGISTRY_UNAVAILABLE
        )
        assert harness.authorization_count == 1
        assert harness.approval_count == 1

    run_with_harness(body, approval_mode='storage')


def test_dependency_response_timeout_is_bounded():
    """A slow authoritative lookup returns a bounded timeout result."""

    def body(harness):
        started_at = time.monotonic()
        response = harness.call_review(
            review_request(approve=True),
            timeout=3.0,
        )
        duration = time.monotonic() - started_at
        assert not response.completed
        assert response.result_code == (
            ReviewLocationCandidate.Response.RESULT_TIMED_OUT
        )
        assert duration < 1.0
        assert harness.authorization_count == 0
        assert harness.approval_count == 0

    run_with_harness(
        body,
        operation_timeout=0.10,
        lookup_delay=0.45,
    )


def test_concurrent_review_returns_busy_without_second_lookup():
    """Only one review transaction can own the gateway at a time."""

    def body(harness):
        first = harness.review_client.call_async(
            review_request(approve=True, suffix='first')
        )
        assert harness.lookup_started.wait(timeout=2.0)
        second = harness.review_client.call_async(
            review_request(approve=True, suffix='second')
        )
        assert wait_until(second.done, timeout=2.0), harness.diagnostics()
        second_response = second.result()
        assert not second_response.completed
        assert second_response.result_code == (
            ReviewLocationCandidate.Response.RESULT_BUSY
        )
        assert wait_until(first.done, timeout=3.0), harness.diagnostics()
        assert first.result().result_code == (
            ReviewLocationCandidate.Response.RESULT_APPROVED
        )
        assert harness.lookup_count == 1
        assert harness.approval_count == 1

    run_with_harness(
        body,
        operation_timeout=1.0,
        lookup_delay=0.35,
    )


def run_cli(harness, *arguments):
    """Run the installed C++ CLI against the harness services."""
    command = [
        installed_executable('LOCATION_REVIEW_CLI_EXECUTABLE'),
        *arguments,
        '--json',
        '--timeout', '5.0',
        '--candidate-service', harness.candidate_service,
        '--candidate-list-service', harness.candidate_list_service,
        '--review-service', harness.review_service,
    ]
    result = subprocess.run(
        command,
        env=os.environ.copy(),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=PROCESS_TIMEOUT_SEC,
        check=False,
    )
    payload = None
    for line in reversed(result.stdout.splitlines()):
        if line.startswith('{'):
            payload = json.loads(line)
            break
    return result, payload


def test_cli_lists_inspects_and_approves_only_through_gateway():
    """The C++ CLI discovers candidates and approves via the gateway."""

    def body(harness):
        listed, list_payload = run_cli(
            harness,
            'list',
            '--state', 'pending',
            '--map-id', 'campus_main',
            '--map-revision', '7',
        )
        assert listed.returncode == 0, listed.stdout
        assert list_payload['count'] == 1
        assert list_payload['candidates'][0]['candidate_revision'] == 3

        shown, show_payload = run_cli(
            harness,
            'inspect',
            'candidate-campus-main-27',
        )
        assert shown.returncode == 0, shown.stdout
        assert show_payload['candidate']['candidate_id'] == (
            'candidate-campus-main-27'
        )

        approved, approve_payload = run_cli(
            harness,
            'approve',
            'candidate-campus-main-27',
            '--actor', 'runtime_cli_operator',
            '--request-id', 'runtime-cli-approve',
        )
        assert approved.returncode == 0, approved.stdout
        assert approve_payload['approved']
        assert approve_payload['location']['location_id'] == 'A201'
        assert harness.list_count == 1
        assert harness.approval_count == 1
        assert harness.authorization_count == 1

    run_with_harness(body)


def test_cli_reject_requires_reason_and_uses_gateway():
    """The C++ CLI validates and forwards a typed rejection."""

    def body(harness):
        missing_actor, actor_payload = run_cli(
            harness,
            'approve',
            'candidate-campus-main-27',
        )
        assert missing_actor.returncode == 2
        assert actor_payload is None
        assert '--actor is required' in missing_actor.stdout
        assert harness.approval_count == 0

        invalid, payload = run_cli(
            harness,
            'reject',
            'candidate-campus-main-27',
            '--actor', 'runtime_cli_operator',
        )
        assert invalid.returncode == 2
        assert payload is None
        assert '--reason is required for rejection' in invalid.stdout
        assert harness.rejection_count == 0

        rejected, reject_payload = run_cli(
            harness,
            'reject',
            'candidate-campus-main-27',
            '--actor', 'runtime_cli_operator',
            '--reason', 'duplicate tag evidence',
            '--revision', '3',
            '--request-id', 'runtime-cli-reject',
        )
        assert rejected.returncode == 0, rejected.stdout
        assert reject_payload['rejected']
        assert harness.rejection_count == 1
        assert harness.authorization_count == 1

    run_with_harness(body)
