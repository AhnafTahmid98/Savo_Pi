# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Hardware-independent runtime test for the AM-6 typed happy path."""

import os
from pathlib import Path
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from savo_msgs.action import RegisterMappedLocation
from savo_msgs.msg import AprilTagObservation
from savo_msgs.msg import AutonomousMappingStatus
from savo_msgs.msg import SemanticInterruptionStatus
from savo_msgs.srv import ControlAutonomousMapping
from savo_msgs.srv import SubmitSemanticLocation


def wait_until(predicate, timeout=8.0, interval=0.02):
    """Wait until a predicate succeeds."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


class SemanticRuntimeHarness:
    """Provide typed AM control and registration dependencies."""

    def __init__(self):
        self.node = rclpy.create_node('semantic_interruption_runtime_fixture')
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        observation_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.mission_publisher = self.node.create_publisher(
            AutonomousMappingStatus,
            '/savo_mapping/autonomous/status',
            state_qos,
        )
        self.observation_publisher = self.node.create_publisher(
            AprilTagObservation,
            '/savo_head/apriltag/observations',
            observation_qos,
        )
        self.statuses = []
        self.status_subscription = self.node.create_subscription(
            SemanticInterruptionStatus,
            '/savo_mapping/semantic_interruption/status',
            self.statuses.append,
            state_qos,
        )
        self.pause_requests = 0
        self.resume_requests = 0
        self.registration_goals = []
        self.control_service = self.node.create_service(
            ControlAutonomousMapping,
            '/savo_mapping/autonomous/control',
            self._control,
        )
        self.registration_server = ActionServer(
            self.node,
            RegisterMappedLocation,
            '/savo_mapping/locations/register',
            execute_callback=self._register,
        )
        self.submit_client = self.node.create_client(
            SubmitSemanticLocation,
            '/savo_mapping/semantic_interruption/submit',
        )

        executable = Path(os.environ['SEMANTIC_INTERRUPTION_EXECUTABLE'])
        assert executable.is_file() and os.access(executable, os.X_OK)
        self.process = subprocess.Popen(
            [
                str(executable),
                '--ros-args',
                '-p',
                'semantic_input_timeout_s:=5.0',
                '-p',
                'registration_timeout_s:=5.0',
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.spin_thread.start()

    def mission_status(self, state):
        """Build a current active mission status."""
        message = AutonomousMappingStatus()
        message.contract_version = AutonomousMappingStatus.CONTRACT_VERSION
        message.stamp = self.node.get_clock().now().to_msg()
        message.mission_id = 'mission_am6_runtime'
        message.actor_id = 'runtime_fixture'
        message.map_id = 'runtime_map'
        message.map_revision = 3
        message.strategy = AutonomousMappingStatus.STRATEGY_FRONTIER
        message.state = state
        message.state_text = 'runtime_fixture'
        message.active = True
        return message

    def publish_observation(self, sequence=1):
        """Publish fresh valid typed detector evidence."""
        message = AprilTagObservation()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = 'pi_camera_optical_frame'
        message.detector_name = 'runtime_fixture'
        message.family = 'tag36h11'
        message.tag_id = 42
        message.tag_size_m = 0.16
        message.observation_sequence = sequence
        message.detection_quality = 0.95
        message.pose_valid = True
        message.pose.pose.position.z = 1.2
        message.pose.pose.orientation.w = 1.0
        self.observation_publisher.publish(message)

    def submit(self):
        """Submit matching operator semantic information."""
        request = SubmitSemanticLocation.Request()
        request.contract_version = SubmitSemanticLocation.Request.CONTRACT_VERSION
        request.mission_id = 'mission_am6_runtime'
        request.actor_id = 'runtime_operator'
        request.tag_family = 'tag36h11'
        request.tag_id = 42
        request.suggested_location_id = 'runtime_lab'
        request.suggested_display_name = 'Runtime Lab'
        request.suggested_aliases = ['test lab']
        request.suggested_semantic_type = 'laboratory'
        request.building = 'test'
        request.floor = '1'
        request.area = 'runtime'
        request.notes = 'AM-6 runtime fixture'
        future = self.submit_client.call_async(request)
        assert wait_until(future.done)
        return future.result()

    def latest_state(self):
        """Return the latest received coordinator state."""
        return self.statuses[-1].state if self.statuses else None

    def latest_status(self):
        """Return the latest received coordinator status."""
        return self.statuses[-1] if self.statuses else None

    def _control(self, request, response):
        """Accept typed pause/resume and publish the resulting mission state."""
        response.accepted = True
        response.result_code = ControlAutonomousMapping.Response.RESULT_ACCEPTED
        response.reason = 'runtime_control_accepted'
        if request.command == ControlAutonomousMapping.Request.COMMAND_PAUSE:
            self.pause_requests += 1
            response.status = self.mission_status(
                AutonomousMappingStatus.STATE_PAUSED
            )
            self.mission_publisher.publish(response.status)
        elif request.command == ControlAutonomousMapping.Request.COMMAND_RESUME:
            self.resume_requests += 1
            response.status = self.mission_status(
                AutonomousMappingStatus.STATE_EXPLORING
            )
            self.mission_publisher.publish(response.status)
        return response

    def _register(self, goal_handle):
        """Return a successfully persisted pending candidate."""
        goal = goal_handle.request
        self.registration_goals.append(goal)
        result = RegisterMappedLocation.Result()
        result.registered = True
        result.result_code = RegisterMappedLocation.Result.RESULT_REGISTERED
        result.reason = 'runtime_candidate_registered'
        result.candidate.candidate_id = goal.candidate_id
        result.candidate.map_id = goal.map_id
        result.candidate.map_revision = goal.map_revision
        result.candidate.tag_family = goal.expected_family
        result.candidate.tag_id = goal.expected_tag_id
        goal_handle.succeed()
        return result

    def close(self):
        """Stop the production process and fixture executor."""
        self.process.terminate()
        try:
            self.process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=3.0)
        self.registration_server.destroy()
        self.executor.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        self.spin_thread.join(timeout=2.0)


@pytest.fixture(autouse=True, scope='module')
def ros_context():
    """Initialize one isolated ROS context."""
    rclpy.init()
    yield
    rclpy.shutdown()


def test_detect_pause_register_and_resume_runtime_flow():
    """Exercise the complete AM-6 typed interruption lifecycle."""
    harness = SemanticRuntimeHarness()
    try:
        assert harness.submit_client.wait_for_service(timeout_sec=5.0)
        assert wait_until(
            lambda: harness.mission_publisher.get_subscription_count() >= 1
        )
        for _ in range(5):
            harness.mission_publisher.publish(
                harness.mission_status(
                    AutonomousMappingStatus.STATE_EXPLORING
                )
            )
            time.sleep(0.05)

        harness.publish_observation(sequence=1)
        assert wait_until(lambda: harness.pause_requests == 1)
        harness.publish_observation(sequence=2)
        assert wait_until(
            lambda: harness.latest_state()
            == SemanticInterruptionStatus.STATE_WAITING_FOR_SEMANTICS
        )
        assert wait_until(
            lambda: harness.latest_status() is not None
            and harness.latest_status().duplicate_observations_suppressed >= 1
        )

        response = harness.submit()
        assert response.accepted
        assert response.result_code == SubmitSemanticLocation.Response.RESULT_ACCEPTED
        assert response.candidate_id == 'am6_mission_am6_runtime_tag36h11_42'

        assert wait_until(lambda: len(harness.registration_goals) == 1)
        goal = harness.registration_goals[0]
        assert goal.expected_family == 'tag36h11'
        assert goal.expected_tag_id == 42
        assert goal.map_id == 'runtime_map'
        assert harness.resume_requests == 1 or wait_until(
            lambda: harness.resume_requests == 1
        )
        assert wait_until(
            lambda: harness.latest_state()
            == SemanticInterruptionStatus.STATE_COMPLETED
        )
        assert harness.latest_status().registration_complete
        assert harness.latest_status().resume_complete
        assert not harness.latest_status().active
    finally:
        harness.close()
