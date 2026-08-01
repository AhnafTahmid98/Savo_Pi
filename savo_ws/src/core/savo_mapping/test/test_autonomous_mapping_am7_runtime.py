# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""End-to-end DDS validation for the AM-7 through AM-8 mission sequence."""

import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import threading
import time

from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from savo_msgs.action import RunAutonomousMapping
from savo_msgs.msg import AutonomousMappingStatus
from savo_msgs.msg import FrontierExplorationStatus
from savo_msgs.srv import CommitLocationRelease
from savo_msgs.srv import ListLocationCandidates
from savo_msgs.srv import ListLocations
from savo_msgs.srv import PrepareLocationRelease
from savo_msgs.srv import ReviewAutonomousMappingRelease
from savo_msgs.srv import RollbackLocationRelease
from savo_msgs.srv import VerifyLocationRelease
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def retained_qos():
    """Return reliable transient-local state QoS."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def command_qos():
    """Return reliable volatile command QoS."""
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def wait_until(predicate, timeout=15.0, interval=0.02):
    """Wait until a predicate succeeds."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


class Am7RuntimeHarness:
    """Provide fake hardware boundaries around the production orchestrator."""

    def __init__(
        self,
        return_behavior='succeed',
        scan_behavior='complete',
        head_behavior='complete',
        coverage_behavior='complete',
        tf_behavior='continuous',
        parameter_overrides=None,
    ):
        suffix = f'{os.getpid()}_{time.monotonic_ns()}'
        prefix = f'/test/am7_{suffix}'
        self.map_frame = f'map_{suffix}'
        self.base_frame = f'base_{suffix}'
        self.action_name = f'{prefix}/run'
        self.control_service = f'{prefix}/control'
        self.return_action_name = f'{prefix}/guarded_return'
        self.status_topic = f'{prefix}/status'
        self.mode_topic = f'{prefix}/mode'
        self.exploration_mode_topic = f'{prefix}/exploration_mode'
        self.workflow_phase_topic = f'{prefix}/workflow_phase'
        self.session_state_topic = f'{prefix}/session_state'
        self.readiness_topic = f'{prefix}/readiness'
        self.safety_stop_topic = f'{prefix}/safety_stop'
        self.runtime_authority_topic = f'{prefix}/runtime_authority'
        self.handoff_state_topic = f'{prefix}/handoff_state'
        self.frontier_status_topic = f'{prefix}/frontier_status'
        self.mode_command_topic = f'{prefix}/mode_cmd'
        self.start_session_topic = f'{prefix}/start_session_cmd'
        self.cancel_session_topic = f'{prefix}/cancel_session_cmd'
        self.handoff_cancel_service = f'{prefix}/handoff_cancel'
        self.map_save_service = f'{prefix}/map_save'
        self.scan_state_topic = f'{prefix}/scan360/state'
        self.scan_start_service = f'{prefix}/scan360/start'
        self.scan_cancel_service = f'{prefix}/scan360/cancel'
        self.head_state_topic = f'{prefix}/head/state'
        self.head_start_service = f'{prefix}/head/start'
        self.head_pause_service = f'{prefix}/head/pause'
        self.head_resume_service = f'{prefix}/head/resume'
        self.coverage_status_topic = f'{prefix}/coverage/status'
        self.coverage_operation_status_topic = f'{prefix}/operation/status'
        self.coverage_request_service = f'{prefix}/coverage/request'
        self.coverage_plan_reset_service = f'{prefix}/coverage/reset'
        self.coverage_approve_service = f'{prefix}/operation/approve'
        self.coverage_cancel_service = f'{prefix}/operation/cancel'
        self.coverage_reset_service = f'{prefix}/operation/reset'
        self.location_candidates_service = f'{prefix}/locations/candidates'
        self.location_list_service = f'{prefix}/locations/list'
        self.location_prepare_service = f'{prefix}/locations/prepare'
        self.location_verify_service = f'{prefix}/locations/verify'
        self.location_commit_service = f'{prefix}/locations/commit'
        self.location_rollback_service = f'{prefix}/locations/rollback'
        self.release_review_service = f'{prefix}/release/review'

        self.node = rclpy.create_node(f'am7_fixture_{suffix}')
        self.group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=10)
        self.executor.add_node(self.node)
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.mode_commands = []
        self.start_commands = []
        self.statuses = []
        self.scan_starts = 0
        self.scan_cancels = 0
        self.head_starts = 0
        self.head_pauses = 0
        self.scan_behavior = scan_behavior
        self.head_behavior = head_behavior
        self.coverage_requests = 0
        self.coverage_resets = 0
        self.coverage_approvals = 0
        self.coverage_cancels = 0
        self.coverage_behavior = coverage_behavior
        self.return_goals = []
        self.return_behavior = return_behavior
        self.return_cancel_requests = 0
        self.tf_behavior = tf_behavior
        self.return_goal_received_at = None
        self.closed = False
        self.delayed_timers = []
        self.map_saves = 0
        self.session_root = Path(tempfile.mkdtemp(prefix='savo_am7_runtime_'))
        self.session_directory = self._create_valid_session('campus_main')
        self.production_map_root = self.session_root / 'production_maps'
        self.release_journal_root = self.session_root / 'release_journals'
        self.location_snapshot = self.session_root / 'locations.json'
        self.location_snapshot.write_text(
            '{"locations":[],"schema_version":1}\n', encoding='utf-8'
        )
        self.location_snapshot_digest = hashlib.sha256(
            self.location_snapshot.read_bytes()
        ).hexdigest()
        self.location_transaction_token = f'transaction-{suffix}'
        self.location_release_id = ''

        self.mode_pub = self._publisher(String, self.mode_topic, retained_qos())
        self.exploration_mode_pub = self._publisher(
            String, self.exploration_mode_topic, retained_qos()
        )
        self.workflow_phase_pub = self._publisher(
            String, self.workflow_phase_topic, retained_qos()
        )
        self.session_state_pub = self._publisher(
            String, self.session_state_topic, retained_qos()
        )
        self.readiness_pub = self._publisher(
            String, self.readiness_topic, retained_qos()
        )
        self.safety_stop_pub = self._publisher(
            Bool, self.safety_stop_topic, command_qos()
        )
        self.runtime_authority_pub = self._publisher(
            Bool, self.runtime_authority_topic, retained_qos()
        )
        self.handoff_state_pub = self._publisher(
            String, self.handoff_state_topic, retained_qos()
        )
        self.frontier_status_pub = self._publisher(
            FrontierExplorationStatus,
            self.frontier_status_topic,
            retained_qos(),
        )
        self.scan_state_pub = self._publisher(
            String, self.scan_state_topic, retained_qos()
        )
        self.head_state_pub = self._publisher(
            String, self.head_state_topic, command_qos()
        )
        self.coverage_status_pub = self._publisher(
            String, self.coverage_status_topic, retained_qos()
        )
        self.coverage_operation_status_pub = self._publisher(
            String, self.coverage_operation_status_topic, retained_qos()
        )

        self.node.create_subscription(
            String,
            self.mode_command_topic,
            lambda msg: self.mode_commands.append(msg.data),
            command_qos(),
        )
        self.node.create_subscription(
            String,
            self.start_session_topic,
            lambda msg: self.start_commands.append(msg.data),
            command_qos(),
        )
        self.node.create_subscription(
            AutonomousMappingStatus,
            self.status_topic,
            lambda msg: self.statuses.append(msg),
            retained_qos(),
        )

        self._service(self.handoff_cancel_service, self._accept)
        self._service(self.scan_start_service, self._scan_start)
        self._service(self.scan_cancel_service, self._scan_cancel)
        self._service(self.head_start_service, self._head_start)
        self._service(self.head_pause_service, self._head_pause)
        self._service(self.head_resume_service, self._accept)
        self._service(self.coverage_plan_reset_service, self._coverage_reset)
        self._service(self.coverage_request_service, self._coverage_request)
        self._service(self.coverage_approve_service, self._coverage_approve)
        self._service(self.coverage_cancel_service, self._coverage_cancel)
        self._service(self.coverage_reset_service, self._accept)
        self._service(self.map_save_service, self._map_save)
        self.node.create_service(
            ListLocationCandidates,
            self.location_candidates_service,
            self._list_candidates,
            callback_group=self.group,
        )
        self.node.create_service(
            ListLocations,
            self.location_list_service,
            self._list_locations,
            callback_group=self.group,
        )
        self.node.create_service(
            PrepareLocationRelease,
            self.location_prepare_service,
            self._prepare_location_release,
            callback_group=self.group,
        )
        self.node.create_service(
            VerifyLocationRelease,
            self.location_verify_service,
            self._verify_location_release,
            callback_group=self.group,
        )
        self.node.create_service(
            CommitLocationRelease,
            self.location_commit_service,
            self._commit_location_release,
            callback_group=self.group,
        )
        self.node.create_service(
            RollbackLocationRelease,
            self.location_rollback_service,
            self._rollback_location_release,
            callback_group=self.group,
        )

        self.return_server = ActionServer(
            self.node,
            NavigateToPose,
            self.return_action_name,
            execute_callback=self._return_to_start,
            goal_callback=self._return_goal_response,
            cancel_callback=self._return_cancel_response,
            callback_group=self.group,
        )
        self.action_client = ActionClient(
            self.node,
            RunAutonomousMapping,
            self.action_name,
            callback_group=self.group,
        )
        self.review_client = self.node.create_client(
            ReviewAutonomousMappingRelease,
            self.release_review_service,
            callback_group=self.group,
        )
        self.tf_timer = self.node.create_timer(0.05, self.publish_transform)
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.spin_thread.start()
        self.process = self._start_orchestrator(parameter_overrides or {})
        assert wait_until(
            lambda: self.process.poll() is not None
            or self.action_client.server_is_ready()
        ), self.diagnostics()
        assert self.process.poll() is None, self.diagnostics()

    def _publisher(self, message_type, topic, qos):
        return self.node.create_publisher(message_type, topic, qos)

    def _service(self, name, callback):
        return self.node.create_service(
            Trigger, name, callback, callback_group=self.group
        )

    @staticmethod
    def string_message(value):
        message = String()
        message.data = value
        return message

    @staticmethod
    def bool_message(value):
        message = Bool()
        message.data = value
        return message

    def _create_valid_session(self, map_id):
        session = self.session_root / map_id
        session.mkdir()
        base = session / map_id
        quality_report = session / 'quality_report.yaml'
        pixels = bytes([0] * 25 + [254] * 600)
        (session / f'{map_id}.pgm').write_bytes(
            b'P5\n25 25\n255\n' + pixels
        )
        (session / f'{map_id}.posegraph').write_text('posegraph')
        (session / f'{map_id}.data').write_text('data')
        (session / f'{map_id}.yaml').write_text(
            f'image: {map_id}.pgm\nmode: trinary\nresolution: 0.05\n'
            'origin: [0.0, 0.0, 0.0]\nnegate: 0\n'
            'occupied_thresh: 0.65\nfree_thresh: 0.25\n'
        )
        quality_report.write_text(
            'schema_version: 1\n'
            f'map_id: "{map_id}"\n'
            'passed: true\n',
            encoding='utf-8',
        )
        (session / 'manifest.yaml').write_text(
            'schema_version: 1\n'
            f'map_id: "{map_id}"\n'
            f'frame_id: "{self.map_frame}"\n'
            f'session_directory: "{session}"\n'
            'occupancy_grid:\n'
            f'  yaml: "{base}.yaml"\n'
            f'  image: "{base}.pgm"\n'
            'pose_graph:\n'
            f'  posegraph: "{base}.posegraph"\n'
            f'  data: "{base}.data"\n'
            'map_quality:\n  structurally_valid: true\n'
            '  evaluated: true\n'
            '  passed: true\n'
            f'  report: "{quality_report}"\n'
            'navigation_handoff_ready: false\n'
        )
        return session

    def _start_orchestrator(self, parameter_overrides):
        executable = Path(os.environ['AUTONOMOUS_ORCHESTRATOR_EXECUTABLE'])
        assert executable.is_file() and os.access(executable, os.X_OK)
        parameters = {
            'action_name': self.action_name,
            'control_service': self.control_service,
            'status_topic': self.status_topic,
            'mode_topic': self.mode_topic,
            'exploration_mode_topic': self.exploration_mode_topic,
            'workflow_phase_topic': self.workflow_phase_topic,
            'session_state_topic': self.session_state_topic,
            'readiness_topic': self.readiness_topic,
            'safety_stop_topic': self.safety_stop_topic,
            'runtime_authority_topic': self.runtime_authority_topic,
            'handoff_state_topic': self.handoff_state_topic,
            'frontier_status_topic': self.frontier_status_topic,
            'mode_command_topic': self.mode_command_topic,
            'start_session_command_topic': self.start_session_topic,
            'cancel_session_command_topic': self.cancel_session_topic,
            'handoff_cancel_service': self.handoff_cancel_service,
            'save.map_session_service': self.map_save_service,
            'save.expected_frame': self.map_frame,
            'release.review_service': self.release_review_service,
            'release.location_candidates_service': (
                self.location_candidates_service
            ),
            'release.location_list_service': self.location_list_service,
            'release.location_prepare_service': self.location_prepare_service,
            'release.location_verify_service': self.location_verify_service,
            'release.location_commit_service': self.location_commit_service,
            'release.location_rollback_service': (
                self.location_rollback_service
            ),
            'release.production_map_root': str(self.production_map_root),
            'release.journal_root': str(self.release_journal_root),
            'release.require_locked_geometry': 'false',
            'release.allow_provisional_geometry': 'true',
            'release.location_verification_timeout_s': '3.0',
            'release.operator_approval_timeout_s': '5.0',
            'release.location_prepare_timeout_s': '3.0',
            'release.map_creation_verification_timeout_s': '3.0',
            'release.location_commit_timeout_s': '3.0',
            'release.map_promotion_timeout_s': '3.0',
            'release.rollback_recovery_timeout_s': '3.0',
            'sequence.scan360_state_topic': self.scan_state_topic,
            'sequence.scan360_start_service': self.scan_start_service,
            'sequence.scan360_cancel_service': self.scan_cancel_service,
            'sequence.head_scan_state_topic': self.head_state_topic,
            'sequence.head_scan_start_service': self.head_start_service,
            'sequence.head_scan_pause_service': self.head_pause_service,
            'sequence.head_scan_resume_service': self.head_resume_service,
            'sequence.start_pose_target_frame': self.map_frame,
            'sequence.start_pose_source_frame': self.base_frame,
            'coverage.enabled': 'true',
            'coverage.required': 'true',
            'coverage.request_plan_service': self.coverage_request_service,
            'coverage.reset_plan_service': self.coverage_plan_reset_service,
            'coverage.planner_status_topic': self.coverage_status_topic,
            'coverage.operation_status_topic': (
                self.coverage_operation_status_topic
            ),
            'coverage.operation_approve_service': self.coverage_approve_service,
            'coverage.operation_cancel_service': self.coverage_cancel_service,
            'coverage.operation_reset_service': self.coverage_reset_service,
            'return_to_start.enabled': 'true',
            'return_to_start.action_name': self.return_action_name,
            'final_sequence.require_final_scan360': 'true',
            'final_sequence.require_final_head_scan': 'true',
            'completion.minimum_exhaustion_observations': '1',
            'completion.minimum_stable_duration_s': '0.0',
            'evaluation_period_ms': '50',
            'command_retry_period_ms': '100',
            'default_mission_timeout_s': '0.0',
        }
        parameters.update(parameter_overrides)
        command = [str(executable), '--ros-args']
        for name, value in parameters.items():
            command.extend(['-p', f'{name}:={value}'])
        environment = os.environ.copy()
        environment['PYTHONDONTWRITEBYTECODE'] = '1'
        environment['ROS_LOCALHOST_ONLY'] = '1'
        return subprocess.Popen(
            command,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

    def _accept(self, _request, response):
        response.success = True
        response.message = 'accepted'
        return response

    def _delayed_publish(self, publisher, value, delay=0.08):
        if self.closed:
            return

        def publish_if_open():
            if not self.closed:
                publisher.publish(self.string_message(value))

        timer = threading.Timer(
            delay, publish_if_open
        )
        timer.daemon = True
        self.delayed_timers.append(timer)
        timer.start()

    def _scan_start(self, _request, response):
        self.scan_starts += 1
        response.success = True
        response.message = 'scan360_started'
        self._delayed_publish(self.scan_state_pub, 'rotating', delay=0.03)
        if self.scan_behavior == 'complete':
            self._delayed_publish(self.scan_state_pub, 'complete')
        return response

    def _scan_cancel(self, _request, response):
        self.scan_cancels += 1
        response.success = True
        response.message = 'scan360_cancel_requested'
        if self.scan_behavior == 'timeout_ack':
            self._delayed_publish(self.scan_state_pub, 'canceled')
        return response

    def _head_start(self, _request, response):
        self.head_starts += 1
        response.success = True
        response.message = 'head_scan_started'
        self._delayed_publish(
            self.head_state_pub, 'state=running;phase=scan', delay=0.03
        )
        if self.head_behavior == 'complete':
            self._delayed_publish(
                self.head_state_pub,
                'state=done;phase=complete;reason=fixture_complete',
            )
        return response

    def _head_pause(self, _request, response):
        self.head_pauses += 1
        response.success = True
        response.message = 'head_scan_pause_requested'
        if self.head_behavior == 'timeout_ack':
            self._delayed_publish(
                self.head_state_pub, 'state=paused;phase=paused'
            )
        return response

    def _coverage_request(self, _request, response):
        self.coverage_requests += 1
        response.success = True
        response.message = 'coverage_plan_requested'
        plan = {
            'state': 'plan_ready',
            'reason': 'fixture_plan_ready',
            'map_valid': True,
            'map_fresh': True,
            'map_sequence': 5,
            'plan_sequence': self.coverage_requests,
            'request_generation': self.coverage_requests,
            'reset_generation': self.coverage_resets,
            'waypoint_count': 3,
        }
        operation = {
            'state': 'ready_for_approval',
            'supervisor_authorized': True,
            'candidate_valid': True,
            'candidate_generation': self.coverage_requests,
            'approval_pending': False,
            'mission_id': '',
            'terminal_state': '',
            'result_reason': 'fixture_ready',
            'latest_feedback': '',
        }
        self._delayed_publish(self.coverage_status_pub, json.dumps(plan))
        self._delayed_publish(
            self.coverage_operation_status_pub,
            json.dumps(operation),
            delay=0.12,
        )
        return response

    def _coverage_reset(self, _request, response):
        self.coverage_resets += 1
        response.success = True
        response.message = 'coverage_plan_reset'
        return response

    def _coverage_approve(self, _request, response):
        self.coverage_approvals += 1
        generation = self.coverage_requests
        mission_id = f'coverage-{generation}-runtime'
        feedback = json.dumps(
            {
                'mission_id': mission_id,
                'current_waypoint': 2,
                'completed_waypoints': 1,
                'total_waypoints': 3,
                'completion_ratio': 0.33,
                'remaining_distance_m': 1.5,
            }
        )
        executing = {
            'state': 'executing',
            'supervisor_authorized': True,
            'candidate_valid': True,
            'candidate_generation': generation,
            'approval_pending': False,
            'mission_id': mission_id,
            'terminal_state': '',
            'result_reason': 'fixture_executing',
            'latest_feedback': feedback,
        }
        succeeded = dict(executing)
        succeeded.update(
            state='succeeded',
            terminal_state='succeeded',
            result_reason='fixture_coverage_complete',
        )
        self.coverage_operation_status_pub.publish(
            self.string_message(json.dumps(executing))
        )
        if self.coverage_behavior == 'complete':
            self._delayed_publish(
                self.coverage_operation_status_pub,
                json.dumps(succeeded),
                delay=0.25,
            )
        elif self.coverage_behavior == 'fresh_feedback':
            for sequence in range(1, 6):
                fresh = dict(executing)
                fresh.update(
                    feedback_received=True,
                    feedback_sequence=sequence,
                    feedback_age_s=0.0,
                )
                self._delayed_publish(
                    self.coverage_operation_status_pub,
                    json.dumps(fresh),
                    delay=0.07 * sequence,
                )
            self._delayed_publish(
                self.coverage_operation_status_pub,
                json.dumps(succeeded),
                delay=0.42,
            )
        response.success = True
        response.message = mission_id
        return response

    def publish_coverage_terminal(self, generation, terminal_state):
        """Publish a correlated fake public Coverage terminal status."""
        mission_id = f'coverage-{generation}-runtime'
        status = {
            'state': terminal_state,
            'supervisor_authorized': True,
            'candidate_valid': True,
            'candidate_generation': generation,
            'approval_pending': False,
            'mission_id': mission_id,
            'terminal_state': terminal_state,
            'result_reason': f'fixture_coverage_{terminal_state}',
            'latest_feedback': '',
        }
        self.coverage_operation_status_pub.publish(
            self.string_message(json.dumps(status))
        )

    def _coverage_cancel(self, _request, response):
        self.coverage_cancels += 1
        response.success = True
        response.message = 'coverage_cancel_requested'
        if self.coverage_behavior in ('pause_resume', 'stale_feedback'):
            self._delayed_publish(
                self.coverage_operation_status_pub,
                json.dumps(
                    {
                        'state': 'canceled',
                        'supervisor_authorized': True,
                        'candidate_valid': True,
                        'candidate_generation': self.coverage_requests,
                        'approval_pending': False,
                        'mission_id': (
                            f'coverage-{self.coverage_requests}-runtime'
                        ),
                        'terminal_state': 'canceled',
                        'result_reason': 'fixture_coverage_canceled',
                        'latest_feedback': '',
                    }
                ),
                delay=(
                    0.30 if self.coverage_behavior == 'stale_feedback' else 0.08
                ),
            )
        return response

    def _return_to_start(self, goal_handle):
        self.return_goals.append(goal_handle.request.pose)
        self.return_goal_received_at = time.monotonic()
        result = NavigateToPose.Result()
        if self.return_behavior in ('cancel_terminal', 'delay_accept'):
            assert wait_until(
                lambda: goal_handle.is_cancel_requested, timeout=3.0
            )
            goal_handle.canceled()
            return result
        if self.return_behavior in ('cancel_no_terminal', 'cancel_reject'):
            while rclpy.ok() and not self.closed:
                time.sleep(0.05)
            return result
        if self.tf_behavior in ('delayed_after_return', 'unavailable_after_return'):
            time.sleep(0.6)
        elif self.tf_behavior == 'outside_after_return':
            time.sleep(0.15)
        goal_handle.succeed()
        return result

    def _return_goal_response(self, _goal):
        if self.return_behavior == 'reject':
            return GoalResponse.REJECT
        if self.return_behavior in ('delay_accept', 'cancel_reject'):
            time.sleep(0.5)
        return GoalResponse.ACCEPT

    def _return_cancel_response(self, _goal_handle):
        self.return_cancel_requests += 1
        if self.return_behavior == 'cancel_reject':
            return CancelResponse.REJECT
        return CancelResponse.ACCEPT

    def _map_save(self, _request, response):
        self.map_saves += 1
        response.success = True
        response.message = f'map_session_saved:{self.session_directory}'
        return response

    @staticmethod
    def _list_candidates(_request, response):
        response.success = True
        response.result_code = ListLocationCandidates.Response.RESULT_OK
        response.reason = 'fixture_candidates_verified'
        response.candidates = []
        return response

    @staticmethod
    def _list_locations(_request, response):
        response.success = True
        response.result_code = ListLocations.Response.RESULT_OK
        response.reason = 'fixture_locations_verified'
        response.locations = []
        return response

    def _prepare_location_release(self, request, response):
        self.location_release_id = request.release_id
        response.success = True
        response.result_code = PrepareLocationRelease.Response.RESULT_SUCCEEDED
        response.reason = 'fixture_location_release_prepared'
        response.release_id = request.release_id
        response.transaction_token = self.location_transaction_token
        response.snapshot_path = str(self.location_snapshot)
        response.snapshot_sha256 = self.location_snapshot_digest
        response.location_count = 0
        response.candidate_count = 0
        response.previous_active_location_release_id = ''
        return response

    def _verify_location_release(self, request, response):
        response.success = (
            request.release_id == self.location_release_id
            and request.transaction_token == self.location_transaction_token
            and request.expected_snapshot_sha256
            == self.location_snapshot_digest
        )
        response.result_code = VerifyLocationRelease.Response.RESULT_SUCCEEDED
        response.reason = 'fixture_location_release_verified'
        response.release_id = request.release_id
        response.snapshot_path = str(self.location_snapshot)
        response.snapshot_sha256 = self.location_snapshot_digest
        response.location_count = 0
        response.candidate_count = 0
        response.state = 'prepared_verified'
        return response

    def _commit_location_release(self, request, response):
        response.success = request.release_id == self.location_release_id
        response.result_code = CommitLocationRelease.Response.RESULT_SUCCEEDED
        response.reason = 'fixture_location_release_committed'
        response.release_id = request.release_id
        response.active_location_release_id = request.release_id
        response.state = 'active'
        return response

    def _rollback_location_release(self, request, response):
        response.success = True
        response.result_code = RollbackLocationRelease.Response.RESULT_SUCCEEDED
        response.reason = 'fixture_location_release_rolled_back'
        response.release_id = request.release_id
        response.active_location_release_id = ''
        response.state = 'rolled_back'
        return response

    def approve_release(self):
        """Approve the correlated AM-8 review after both verifications pass."""
        assert self.review_client.wait_for_service(timeout_sec=3.0)
        awaiting = next(
            status
            for status in reversed(self.statuses)
            if status.state == AutonomousMappingStatus.STATE_AWAITING_APPROVAL
        )
        request = ReviewAutonomousMappingRelease.Request()
        request.contract_version = request.CONTRACT_VERSION
        request.request_id = 'review-am8-runtime'
        request.mission_id = 'mission_am7_runtime'
        request.map_id = 'campus_main'
        request.map_revision = 1
        request.expected_review_generation = awaiting.review_generation
        request.actor_id = 'operator_am8_runtime'
        request.decision = request.DECISION_APPROVE
        request.requested_release_id = 'campus-main-r1-runtime'
        request.review_reason = 'runtime_joint_release_approved'
        future = self.review_client.call_async(request)
        assert wait_until(future.done), self.diagnostics()
        response = future.result()
        if (
            response.result_code
            == ReviewAutonomousMappingRelease.Response.RESULT_STALE_GENERATION
        ):
            request.request_id = 'review-am8-runtime-retry'
            request.expected_review_generation = (
                response.current_review_generation
            )
            future = self.review_client.call_async(request)
            assert wait_until(future.done), self.diagnostics()
            response = future.result()
        assert response.accepted and response.approved, response.reason
        return response.release_id

    def publish_transform(self):
        if self.return_goal_received_at is not None:
            if self.tf_behavior == 'unavailable_after_return':
                return
            if (
                self.tf_behavior == 'delayed_after_return'
                and time.monotonic() - self.return_goal_received_at < 0.8
            ):
                return
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = (
            2.25
            if self.tf_behavior == 'outside_after_return'
            and self.return_goal_received_at is not None
            else 1.25
        )
        transform.transform.translation.y = -0.50
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def publish_initial_state(self):
        for _ in range(8):
            self.mode_pub.publish(self.string_message('monitor_only'))
            self.exploration_mode_pub.publish(self.string_message('idle'))
            self.workflow_phase_pub.publish(self.string_message('idle'))
            self.session_state_pub.publish(self.string_message('idle'))
            self.readiness_pub.publish(self.string_message('ready'))
            self.safety_stop_pub.publish(self.bool_message(False))
            self.runtime_authority_pub.publish(self.bool_message(False))
            self.handoff_state_pub.publish(self.string_message('idle'))
            time.sleep(0.05)

    def publish_workflow(self, mode, exploration, phase, authorized):
        self.mode_pub.publish(self.string_message(mode))
        self.exploration_mode_pub.publish(self.string_message(exploration))
        self.workflow_phase_pub.publish(self.string_message(phase))
        self.runtime_authority_pub.publish(self.bool_message(authorized))
        self.handoff_state_pub.publish(self.string_message('idle'))

    def publish_exhaustion(self):
        status = FrontierExplorationStatus()
        status.contract_version = FrontierExplorationStatus.CONTRACT_VERSION
        status.stamp = self.node.get_clock().now().to_msg()
        status.enabled = True
        status.map_received = True
        status.map_generation = 5
        status.planned_map_generation = 5
        status.plan_sequence = 1
        status.planning_status = 'no_frontiers'
        status.handoff_state = 'idle'
        status.goal_pending = False
        status.detected_frontiers = 0
        status.reachable_frontiers = 0
        self.frontier_status_pub.publish(status)

    def send_goal(self):
        assert self.action_client.wait_for_server(timeout_sec=5.0)
        goal = RunAutonomousMapping.Goal()
        goal.contract_version = RunAutonomousMapping.Goal.CONTRACT_VERSION
        goal.mission_id = 'mission_am7_runtime'
        goal.actor_id = 'operator_am7'
        goal.map_id = 'campus_main'
        goal.map_revision = 1
        goal.strategy = RunAutonomousMapping.Goal.STRATEGY_FRONTIER
        goal.auto_save = True
        goal.require_quality_approval = True
        future = self.action_client.send_goal_async(goal)
        assert wait_until(future.done), self.diagnostics()
        handle = future.result()
        assert handle.accepted, self.diagnostics()
        return handle

    def latest_state(self):
        return self.statuses[-1].state if self.statuses else None

    def drive_to_return(self):
        """Drive the production mission through Coverage to return dispatch."""
        self.publish_initial_state()
        result_future = self.send_goal().get_result_async()
        assert wait_until(
            lambda: 'mission_am7_runtime' in self.start_commands
        ), self.diagnostics()
        self.session_state_pub.publish(self.string_message('active'))

        assert wait_until(
            lambda: 'autonomous:scan360' in self.mode_commands
        ), self.diagnostics()
        self.publish_workflow('autonomous', 'scan360', 'scan360', False)
        assert wait_until(lambda: self.scan_starts == 1), self.diagnostics()
        assert wait_until(
            lambda: 'monitor_only' in self.mode_commands
        ), self.diagnostics()
        self.publish_workflow('monitor_only', 'idle', 'idle', False)
        assert wait_until(lambda: self.head_starts == 1), self.diagnostics()

        assert wait_until(
            lambda: 'autonomous:frontier' in self.mode_commands
        ), self.diagnostics()
        self.publish_workflow('autonomous', 'frontier', 'exploring', True)
        assert wait_until(
            lambda: self.latest_state()
            == AutonomousMappingStatus.STATE_EXPLORING
        ), self.diagnostics()
        self.publish_exhaustion()

        previous_monitors = self.mode_commands.count('monitor_only')
        assert wait_until(
            lambda: self.mode_commands.count('monitor_only')
            > previous_monitors
        ), self.diagnostics()
        self.publish_workflow('monitor_only', 'idle', 'idle', False)
        assert wait_until(
            lambda: self.coverage_approvals == 1
            and any(
                status.state == AutonomousMappingStatus.STATE_COVERAGE
                for status in self.statuses
            )
        ), self.diagnostics()
        return result_future

    def start_initial_scan(self):
        """Start a mission and reach the first Scan360 operation."""
        self.publish_initial_state()
        result_future = self.send_goal().get_result_async()
        assert wait_until(
            lambda: 'mission_am7_runtime' in self.start_commands
        ), self.diagnostics()
        self.session_state_pub.publish(self.string_message('active'))
        assert wait_until(
            lambda: 'autonomous:scan360' in self.mode_commands
        ), self.diagnostics()
        self.publish_workflow('autonomous', 'scan360', 'scan360', False)
        assert wait_until(lambda: self.scan_starts == 1), self.diagnostics()
        return result_future

    def diagnostics(self):
        output = ''
        if hasattr(self, 'process') and self.process.poll() is not None:
            if self.process.stdout:
                output = self.process.stdout.read()
        return (
            f'process={getattr(self, "process", None)} '
            f'modes={self.mode_commands[-12:]} scans={self.scan_starts} '
            f'heads={self.head_starts} plans={self.coverage_requests} '
            f'approvals={self.coverage_approvals} returns={len(self.return_goals)} '
            f'saves={self.map_saves} '
            f'states={[status.state_text for status in self.statuses[-15:]]} '
            f'reasons={[status.reason for status in self.statuses[-8:]]} '
            f'coverage_reasons='
            f'{[status.coverage_reason for status in self.statuses[-8:]]} '
            f'output={output}'
        )

    def close(self):
        self.closed = True
        for timer in self.delayed_timers:
            timer.cancel()
        for timer in self.delayed_timers:
            timer.join(timeout=0.5)
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5.0)
        self.return_server.destroy()
        self.executor.shutdown(timeout_sec=2.0)
        self.action_client.destroy()
        self.node.destroy_node()
        self.spin_thread.join(timeout=2.0)
        for path in self.session_root.rglob('*'):
            path.chmod(0o700 if path.is_dir() else 0o600)
        shutil.rmtree(self.session_root)


@pytest.fixture(autouse=True, scope='module')
def ros_context():
    """Initialize one isolated ROS context."""
    rclpy.init()
    yield
    rclpy.shutdown()


def test_full_am7_am8_runtime_sequence():
    """Drive the production orchestrator through AM-7 and AM-8."""
    harness = Am7RuntimeHarness()
    try:
        result_future = harness.drive_to_return()
        assert wait_until(lambda: len(harness.return_goals) == 1), (
            harness.diagnostics()
        )
        return_goal = harness.return_goals[0]
        assert return_goal.header.frame_id == harness.map_frame
        assert return_goal.pose.position.x == pytest.approx(1.25)
        assert return_goal.pose.position.y == pytest.approx(-0.50)

        initial_scan_modes = harness.mode_commands.count('autonomous:scan360')
        assert wait_until(
            lambda: harness.mode_commands.count('autonomous:scan360')
            > initial_scan_modes
        ), harness.diagnostics()
        harness.publish_workflow('autonomous', 'scan360', 'scan360', False)
        assert wait_until(lambda: harness.scan_starts == 2), harness.diagnostics()

        initial_monitors = harness.mode_commands.count('monitor_only')
        assert wait_until(
            lambda: harness.mode_commands.count('monitor_only')
            > initial_monitors
        ), harness.diagnostics()
        harness.publish_workflow('monitor_only', 'idle', 'idle', False)
        assert wait_until(lambda: harness.head_starts == 2), harness.diagnostics()
        assert wait_until(
            lambda: any(
                status.state
                == AutonomousMappingStatus.STATE_AWAITING_APPROVAL
                for status in harness.statuses
            )
        ), harness.diagnostics()
        release_id = harness.approve_release()
        assert wait_until(result_future.done, timeout=20.0), harness.diagnostics()
        wrapped = result_future.result()
        assert wrapped.result.success
        assert (
            wrapped.result.result_code
            == RunAutonomousMapping.Result.RESULT_SUCCEEDED
        )
        assert wrapped.result.map_saved
        assert wrapped.result.map_release_id == release_id
        assert harness.map_saves == 1
        assert harness.statuses[-1].state == AutonomousMappingStatus.STATE_COMPLETED
        assert harness.statuses[-1].final_scan360_succeeded
        assert harness.statuses[-1].final_head_scan_succeeded
        assert harness.statuses[-1].return_to_start_succeeded
        assert harness.statuses[-1].return_to_start_distance_m <= 0.35
        assert harness.statuses[-1].location_verification_passed
        assert harness.statuses[-1].approval_recorded
        assert harness.statuses[-1].release_succeeded
        assert harness.statuses[-1].joint_active_release_verified
    finally:
        harness.close()


def test_rejected_return_fails_without_becoming_active():
    """A rejected guarded return is terminal and was never accepted-active."""
    harness = Am7RuntimeHarness(
        return_behavior='reject',
        parameter_overrides={'return_to_start.maximum_attempts': '1'},
    )
    try:
        harness.drive_to_return()
        assert wait_until(
            lambda: any(
                status.return_to_start_state == 'rejected'
                for status in harness.statuses
            )
        ), harness.diagnostics()
        rejected = next(
            status
            for status in reversed(harness.statuses)
            if status.return_to_start_state == 'rejected'
        )
        assert not rejected.return_to_start_active
        assert rejected.return_to_start_complete
        assert not rejected.return_to_start_succeeded
        assert rejected.return_to_start_reason == 'guarded_return_goal_rejected'
    finally:
        harness.close()


def test_late_return_acceptance_is_canceled_and_reaches_terminal():
    """A response after the goal-response deadline is canceled immediately."""
    harness = Am7RuntimeHarness(
        return_behavior='delay_accept',
        parameter_overrides={
            'return_to_start.goal_response_timeout_s': '0.15',
            'return_to_start.cancel_timeout_s': '1.5',
            'return_to_start.maximum_attempts': '1',
        },
    )
    try:
        harness.drive_to_return()
        assert wait_until(
            lambda: harness.return_cancel_requests == 1, timeout=4.0
        ), harness.diagnostics()
        assert wait_until(
            lambda: any(
                status.return_to_start_complete
                and status.return_to_start_reason
                == 'return_goal_response_timeout'
                for status in harness.statuses
            ),
            timeout=4.0,
        ), harness.diagnostics()
        assert not harness.statuses[-1].return_to_start_active
    finally:
        harness.close()


def test_return_cancellation_rejection_latches_non_quiesced_fault():
    """Cancellation rejection never advances to saving or a later scan."""
    harness = Am7RuntimeHarness(
        return_behavior='cancel_reject',
        parameter_overrides={
            'return_to_start.goal_response_timeout_s': '0.15',
            'return_to_start.cancel_timeout_s': '1.0',
            'return_to_start.maximum_attempts': '1',
        },
    )
    try:
        harness.drive_to_return()
        assert wait_until(
            lambda: any(
                'return_non_quiesced_fault' in status.reason
                for status in harness.statuses
            ),
            timeout=4.0,
        ), harness.diagnostics()
        assert harness.return_cancel_requests == 1
        assert harness.scan_starts == 1
        assert harness.map_saves == 0
    finally:
        harness.close()


def test_return_cancellation_timeout_survives_primary_timeout():
    """The independent cancel timer fires after the primary execution timeout."""
    harness = Am7RuntimeHarness(
        return_behavior='cancel_no_terminal',
        parameter_overrides={
            'return_to_start.execution_timeout_s': '0.20',
            'return_to_start.feedback_stale_timeout_s': '5.0',
            'return_to_start.cancel_timeout_s': '0.25',
            'return_to_start.maximum_attempts': '1',
        },
    )
    try:
        harness.drive_to_return()
        assert wait_until(
            lambda: any(
                status.return_to_start_reason.startswith(
                    'return_execution_timeout;return_non_quiesced_fault:'
                )
                for status in harness.statuses
            ),
            timeout=4.0,
        ), harness.diagnostics()
        assert harness.return_cancel_requests == 1
        assert harness.scan_starts == 1
        assert harness.map_saves == 0
    finally:
        harness.close()


def test_unchanged_but_fresh_coverage_feedback_remains_healthy():
    """Actual feedback callbacks refresh age even without numeric progress."""
    harness = Am7RuntimeHarness(
        coverage_behavior='fresh_feedback',
        parameter_overrides={'coverage.feedback_stale_timeout_s': '0.16'},
    )
    try:
        harness.drive_to_return()
        assert wait_until(lambda: len(harness.return_goals) == 1), (
            harness.diagnostics()
        )
        assert not any(
            'coverage_feedback_stale_timeout' in status.reason
            for status in harness.statuses
        )
        assert harness.coverage_cancels == 0
    finally:
        harness.close()


def test_no_coverage_feedback_times_out_and_still_cancels():
    """No feedback triggers the primary stale timeout and bounded cancellation."""
    harness = Am7RuntimeHarness(
        coverage_behavior='stale_feedback',
        parameter_overrides={
            'coverage.feedback_stale_timeout_s': '0.18',
            'coverage.cancel_timeout_s': '1.0',
        },
    )
    try:
        harness.drive_to_return()
        assert wait_until(lambda: harness.coverage_cancels >= 1), (
            harness.diagnostics()
        )
        assert wait_until(
            lambda: any(
                status.reason == 'coverage_feedback_stale_timeout'
                for status in harness.statuses
            )
        ), harness.diagnostics()
        assert len(harness.return_goals) == 0
        assert harness.map_saves == 0
    finally:
        harness.close()


def test_coverage_cancel_timeout_survives_feedback_timeout():
    """Coverage quiescence timeout remains armed after its primary timeout."""
    harness = Am7RuntimeHarness(
        coverage_behavior='stale_feedback_no_terminal',
        parameter_overrides={
            'coverage.feedback_stale_timeout_s': '0.18',
            'coverage.cancel_timeout_s': '0.22',
        },
    )
    try:
        harness.drive_to_return()
        assert wait_until(
            lambda: any(
                status.coverage_reason.startswith(
                    'coverage_feedback_stale_timeout;'
                    'coverage_non_quiesced_fault:'
                )
                for status in harness.statuses
            )
        ), harness.diagnostics()
        assert harness.coverage_cancels >= 1
        assert len(harness.return_goals) == 0
        assert harness.map_saves == 0
    finally:
        harness.close()


def test_delayed_fresh_tf_verifies_without_resending_return_goal():
    """Proximity polling accepts delayed TF without a second navigation goal."""
    harness = Am7RuntimeHarness(
        tf_behavior='delayed_after_return',
        parameter_overrides={
            'return_to_start.proximity_timeout_s': '0.8',
            'return_to_start.proximity_poll_period_s': '0.05',
            'sequence.start_pose_stale_timeout_s': '0.10',
        },
    )
    try:
        harness.drive_to_return()
        assert wait_until(
            lambda: any(
                status.return_to_start_succeeded for status in harness.statuses
            ),
            timeout=4.0,
        ), harness.diagnostics()
        assert len(harness.return_goals) == 1
    finally:
        harness.close()


def test_return_tf_outside_tolerance_fails_after_bounded_polling():
    """Fresh poses outside tolerance fail with the specific proximity reason."""
    harness = Am7RuntimeHarness(
        tf_behavior='outside_after_return',
        parameter_overrides={
            'return_to_start.proximity_timeout_s': '0.25',
            'return_to_start.proximity_poll_period_s': '0.05',
            'return_to_start.maximum_attempts': '1',
            'sequence.start_pose_stale_timeout_s': '0.10',
        },
    )
    try:
        harness.drive_to_return()
        assert wait_until(
            lambda: any(
                status.return_to_start_reason
                == 'return_to_start_outside_tolerance'
                for status in harness.statuses
            )
        ), harness.diagnostics()
        assert len(harness.return_goals) == 1
        assert harness.scan_starts == 1
    finally:
        harness.close()


def test_return_tf_unavailable_fails_after_bounded_polling():
    """Missing fresh TF fails as unverifiable without resending navigation."""
    harness = Am7RuntimeHarness(
        tf_behavior='unavailable_after_return',
        parameter_overrides={
            'return_to_start.proximity_timeout_s': '0.25',
            'return_to_start.proximity_poll_period_s': '0.05',
            'return_to_start.maximum_attempts': '1',
            'sequence.start_pose_stale_timeout_s': '0.10',
        },
    )
    try:
        harness.drive_to_return()
        assert wait_until(
            lambda: any(
                status.return_to_start_reason
                == 'return_to_start_proximity_unverifiable'
                for status in harness.statuses
            )
        ), harness.diagnostics()
        assert len(harness.return_goals) == 1
        assert harness.scan_starts == 1
    finally:
        harness.close()


def test_scan_timeout_cancels_and_waits_for_terminal_state():
    """Scan timeout remains active until canceled state acknowledges quiescence."""
    harness = Am7RuntimeHarness(
        scan_behavior='timeout_ack',
        parameter_overrides={
            'sequence.scan360_operation_timeout_s': '0.15',
            'sequence.scan360_cancel_timeout_s': '0.8',
        },
    )
    try:
        harness.start_initial_scan()
        assert wait_until(lambda: harness.scan_cancels == 1), harness.diagnostics()
        assert any(
            status.scan360_active and not status.scan360_complete
            for status in harness.statuses
        )
        assert wait_until(
            lambda: any(
                status.scan360_complete
                and not status.scan360_succeeded
                and status.scan360_reason == 'scan360_operation_timeout'
                for status in harness.statuses
            )
        ), harness.diagnostics()
        assert harness.head_starts == 0
        assert harness.map_saves == 0
    finally:
        harness.close()


def test_scan_missing_terminal_state_latches_non_quiesced_fault():
    """A cancel service reply alone cannot claim Scan360 is stopped."""
    harness = Am7RuntimeHarness(
        scan_behavior='timeout_missing',
        parameter_overrides={
            'sequence.scan360_operation_timeout_s': '0.15',
            'sequence.scan360_cancel_timeout_s': '0.20',
        },
    )
    try:
        harness.start_initial_scan()
        assert wait_until(
            lambda: any(
                'scan360_non_quiesced_fault' in status.reason
                for status in harness.statuses
            )
        ), harness.diagnostics()
        assert harness.scan_cancels >= 1
        assert harness.statuses[-1].scan360_active
        assert harness.head_starts == 0
        assert harness.map_saves == 0
    finally:
        harness.close()


def test_stale_retained_scan_completion_does_not_finish_new_generation():
    """A retained completion received before start acknowledgement is ignored."""
    harness = Am7RuntimeHarness(
        scan_behavior='timeout_missing',
        parameter_overrides={
            'sequence.scan360_operation_timeout_s': '0.15',
            'sequence.scan360_cancel_timeout_s': '0.20',
        },
    )
    try:
        harness.scan_state_pub.publish(harness.string_message('complete'))
        time.sleep(0.1)
        harness.start_initial_scan()
        assert wait_until(lambda: harness.scan_cancels == 1), harness.diagnostics()
        assert not any(
            status.initial_scan360_succeeded for status in harness.statuses
        )
    finally:
        harness.close()


def test_head_timeout_pauses_and_waits_for_paused_state():
    """Head timeout uses pause and requires a state acknowledgement."""
    harness = Am7RuntimeHarness(
        head_behavior='timeout_ack',
        parameter_overrides={
            'sequence.head_scan_operation_timeout_s': '0.15',
            'sequence.head_scan_quiescence_timeout_s': '0.8',
        },
    )
    try:
        harness.start_initial_scan()
        assert wait_until(
            lambda: 'monitor_only' in harness.mode_commands
        ), harness.diagnostics()
        harness.publish_workflow('monitor_only', 'idle', 'idle', False)
        assert wait_until(lambda: harness.head_starts == 1), harness.diagnostics()
        assert wait_until(lambda: harness.head_pauses == 1), harness.diagnostics()
        assert wait_until(
            lambda: any(
                status.head_scan_complete
                and not status.head_scan_succeeded
                and status.head_scan_reason == 'head_scan_operation_timeout'
                for status in harness.statuses
            )
        ), harness.diagnostics()
        assert harness.coverage_approvals == 0
        assert harness.map_saves == 0
    finally:
        harness.close()


def test_head_missing_paused_state_latches_non_quiesced_fault():
    """A pause service reply alone cannot claim head motion is stopped."""
    harness = Am7RuntimeHarness(
        head_behavior='timeout_missing',
        parameter_overrides={
            'sequence.head_scan_operation_timeout_s': '0.15',
            'sequence.head_scan_quiescence_timeout_s': '0.20',
        },
    )
    try:
        harness.start_initial_scan()
        assert wait_until(
            lambda: 'monitor_only' in harness.mode_commands
        ), harness.diagnostics()
        harness.publish_workflow('monitor_only', 'idle', 'idle', False)
        assert wait_until(lambda: harness.head_starts == 1), harness.diagnostics()
        assert wait_until(
            lambda: any(
                'head_scan_non_quiesced_fault' in status.reason
                for status in harness.statuses
            )
        ), harness.diagnostics()
        assert harness.head_pauses >= 1
        assert harness.statuses[-1].head_scan_active
        assert harness.map_saves == 0
    finally:
        harness.close()
