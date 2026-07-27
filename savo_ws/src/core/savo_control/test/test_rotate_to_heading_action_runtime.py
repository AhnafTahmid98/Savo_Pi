"""Fixture-only runtime tests for RotateToHeading."""

from pathlib import Path
import math
import os
import signal
import subprocess
import tempfile
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from savo_msgs.action import RotateToHeading
from std_msgs.msg import Bool


ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[2]

DOMAIN_ID = str(100 + (os.getpid() % 100))


def find_rotate_executable() -> Path:
    """Find the installed production rotate node."""

    candidates = []

    for variable in (
        "AMENT_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
    ):
        for raw_prefix in os.environ.get(
            variable,
            "",
        ).split(os.pathsep):
            if not raw_prefix:
                continue

            candidates.append(
                Path(raw_prefix) /
                "lib/savo_control/"
                "rotate_to_heading_node"
            )

    candidates.append(
        WORKSPACE /
        "install/savo_control/lib/"
        "savo_control/"
        "rotate_to_heading_node"
    )

    for candidate in candidates:
        if (
            candidate.is_file() and
            os.access(candidate, os.X_OK)
        ):
            return candidate.resolve()

    checked = "\n".join(
        f"- {candidate}"
        for candidate in candidates
    )

    raise RuntimeError(
        "rotate_to_heading_node was not found. "
        f"Checked:\n{checked}"
    )


class RuntimeHarness:
    """Run the real action server against synthetic ROS data."""

    def __init__(self) -> None:
        self._process = None
        self._log_stream = None
        self._log_directory = None
        self._executor = None
        self._executor_thread = None
        self._node = None

        self._lock = threading.Lock()
        self._current_yaw_rad = 0.0
        self._commands = []
        self._feedback = []

        self.prefix = (
            f"/savo_control_action_runtime_"
            f"{os.getpid()}"
        )

        self.odom_topic = (
            f"{self.prefix}/odom"
        )
        self.command_topic = (
            f"{self.prefix}/cmd_vel"
        )
        self.safety_topic = (
            f"{self.prefix}/safety_stop"
        )
        self.action_name = (
            f"{self.prefix}/rotate_to_heading"
        )

        try:
            self._start()
        except Exception:
            self.close()
            raise

    def _start(self) -> None:
        os.environ["ROS_DOMAIN_ID"] = DOMAIN_ID
        os.environ["ROS_LOCALHOST_ONLY"] = "1"

        executable = find_rotate_executable()

        self._log_directory = (
            tempfile.TemporaryDirectory(
                prefix="savo_rotate_runtime_"
            )
        )

        log_path = (
            Path(self._log_directory.name) /
            "rotate_to_heading_node.log"
        )

        self._log_stream = log_path.open(
            "w",
            encoding="utf-8",
        )

        environment = os.environ.copy()
        environment["ROS_DOMAIN_ID"] = DOMAIN_ID
        environment["ROS_LOCALHOST_ONLY"] = "1"
        environment["RCUTILS_COLORIZED_OUTPUT"] = "0"

        command = [
            str(executable),
            "--ros-args",
            "-p",
            f"odom_topic:={self.odom_topic}",
            "-p",
            (
                "target_topic:="
                f"{self.prefix}/legacy_target"
            ),
            "-p",
            (
                "enable_topic:="
                f"{self.prefix}/legacy_enable"
            ),
            "-p",
            (
                "cancel_topic:="
                f"{self.prefix}/legacy_cancel"
            ),
            "-p",
            (
                "safety_stop_topic:="
                f"{self.safety_topic}"
            ),
            "-p",
            (
                "output_topic:="
                f"{self.command_topic}"
            ),
            "-p",
            (
                "state_topic:="
                f"{self.prefix}/state"
            ),
            "-p",
            (
                "status_topic:="
                f"{self.prefix}/status"
            ),
            "-p",
            (
                "action_name:="
                f"{self.action_name}"
            ),
            "-p",
            "publish_hz:=50.0",
            "-p",
            "input_timeout_s:=0.50",
            "-p",
            "enabled:=true",
            "-p",
            "start_on_target:=false",
            "-p",
            "publish_zero_when_inactive:=true",
            "-p",
            "target_tolerance_rad:=0.035",
            "-p",
            "max_duration_s:=1.0",
            "-p",
            "max_wz_rad_s:=0.35",
            "-p",
            "min_wz_when_active:=0.08",
        ]

        self._process = subprocess.Popen(
            command,
            env=environment,
            stdout=self._log_stream,
            stderr=subprocess.STDOUT,
            text=True,
        )

        rclpy.init(args=None)

        self._node = rclpy.create_node(
            f"savo_rotate_runtime_test_"
            f"{os.getpid()}"
        )

        self._odom_pub = (
            self._node.create_publisher(
                Odometry,
                self.odom_topic,
                10,
            )
        )

        self._safety_pub = (
            self._node.create_publisher(
                Bool,
                self.safety_topic,
                10,
            )
        )

        self._cmd_sub = (
            self._node.create_subscription(
                Twist,
                self.command_topic,
                self._on_command,
                10,
            )
        )

        self._client = ActionClient(
            self._node,
            RotateToHeading,
            self.action_name,
        )

        self._fixture_timer = (
            self._node.create_timer(
                0.02,
                self._publish_fixture_inputs,
            )
        )

        self._executor = MultiThreadedExecutor(
            num_threads=2,
        )

        self._executor.add_node(
            self._node
        )

        self._executor_thread = threading.Thread(
            target=self._executor.spin,
            daemon=True,
        )

        self._executor_thread.start()

        self.wait_for(
            self._client.server_is_ready,
            timeout_sec=10.0,
            description="action server discovery",
        )

        self.wait_for(
            lambda: bool(
                self._node.get_publishers_info_by_topic(
                    self.command_topic
                )
            ),
            timeout_sec=5.0,
            description="isolated command publisher discovery",
        )

        time.sleep(0.20)

    def _publish_fixture_inputs(self) -> None:
        with self._lock:
            yaw_rad = self._current_yaw_rad

        odom = Odometry()
        odom.header.stamp = (
            self._node.get_clock()
            .now()
            .to_msg()
        )
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.orientation.z = math.sin(
            yaw_rad * 0.5
        )
        odom.pose.pose.orientation.w = math.cos(
            yaw_rad * 0.5
        )

        self._odom_pub.publish(odom)

        safety = Bool()
        safety.data = False
        self._safety_pub.publish(safety)

    def _on_command(self, message: Twist) -> None:
        with self._lock:
            self._commands.append(
                (
                    time.monotonic(),
                    float(message.angular.z),
                )
            )

    def _on_feedback(self, message) -> None:
        with self._lock:
            self._feedback.append(
                message.feedback
            )

    def process_log(self) -> str:
        if self._log_stream is not None:
            self._log_stream.flush()

        if self._log_directory is None:
            return "[runtime log unavailable]"

        path = (
            Path(self._log_directory.name) /
            "rotate_to_heading_node.log"
        )

        if not path.is_file():
            return "[runtime log unavailable]"

        return path.read_text(
            encoding="utf-8",
            errors="replace",
        )

    def wait_for(
        self,
        predicate,
        timeout_sec: float,
        description: str,
    ) -> None:
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            if (
                self._process is not None and
                self._process.poll() is not None
            ):
                raise AssertionError(
                    "rotate_to_heading_node exited "
                    f"while waiting for {description}.\n"
                    f"{self.process_log()}"
                )

            if predicate():
                return

            time.sleep(0.01)

        raise AssertionError(
            f"Timed out waiting for {description}.\n"
            f"{self.process_log()}"
        )

    def wait_future(
        self,
        future,
        timeout_sec: float,
        description: str,
    ):
        self.wait_for(
            future.done,
            timeout_sec=timeout_sec,
            description=description,
        )

        result = future.result()

        if result is None:
            raise AssertionError(
                f"{description} returned no result.\n"
                f"{self.process_log()}"
            )

        return result

    def set_yaw(self, yaw_rad: float) -> None:
        with self._lock:
            self._current_yaw_rad = yaw_rad

    def reset_observations(
        self,
        yaw_rad: float = 0.0,
    ) -> None:
        self.set_yaw(yaw_rad)

        time.sleep(0.15)

        with self._lock:
            self._commands.clear()
            self._feedback.clear()

    def command_values(self):
        with self._lock:
            return [
                value
                for _, value in self._commands
            ]

    def feedback_values(self):
        with self._lock:
            return list(self._feedback)

    def has_motion_command(self) -> bool:
        return any(
            abs(value) > 0.01
            for value in self.command_values()
        )

    def has_positive_command(self) -> bool:
        return any(
            value > 0.01
            for value in self.command_values()
        )

    def has_zero_after_motion(self) -> bool:
        commands = self.command_values()

        motion_index = None

        for index, value in enumerate(commands):
            if abs(value) > 0.01:
                motion_index = index
                break

        if motion_index is None:
            return False

        return any(
            abs(value) <= 1.0e-6
            for value in commands[
                motion_index + 1:
            ]
        )

    def send_goal(
        self,
        target_yaw_rad: float,
        max_duration_sec: float,
    ):
        goal = RotateToHeading.Goal()
        goal.target_yaw_rad = target_yaw_rad
        goal.max_duration_sec = max_duration_sec

        future = self._client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback,
        )

        return self.wait_future(
            future,
            timeout_sec=5.0,
            description="goal response",
        )

    def wait_result(
        self,
        goal_handle,
        timeout_sec: float = 5.0,
    ):
        return self.wait_future(
            goal_handle.get_result_async(),
            timeout_sec=timeout_sec,
            description="action result",
        )

    def cancel_goal(self, goal_handle):
        response = self.wait_future(
            goal_handle.cancel_goal_async(),
            timeout_sec=5.0,
            description="cancel acknowledgement",
        )

        assert response.goals_canceling

        return response

    def close(self) -> None:
        if (
            self._process is not None and
            self._process.poll() is None
        ):
            self._process.send_signal(
                signal.SIGINT
            )

            try:
                self._process.wait(
                    timeout=3.0
                )
            except subprocess.TimeoutExpired:
                self._process.terminate()

                try:
                    self._process.wait(
                        timeout=2.0
                    )
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    self._process.wait(
                        timeout=2.0
                    )

        if self._executor is not None:
            self._executor.shutdown(
                timeout_sec=2.0
            )

        if self._executor_thread is not None:
            self._executor_thread.join(
                timeout=2.0
            )

        if self._node is not None:
            self._node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        if self._log_stream is not None:
            self._log_stream.close()

        if self._log_directory is not None:
            self._log_directory.cleanup()


@pytest.fixture(scope="module")
def runtime_harness():
    """Provide one isolated production-node fixture."""

    harness = RuntimeHarness()

    try:
        yield harness
    finally:
        harness.close()


def test_runtime_uses_only_isolated_command_topic(
    runtime_harness,
) -> None:
    """The fixture must not publish to robot command topics."""

    node = runtime_harness._node

    assert node.get_publishers_info_by_topic(
        runtime_harness.command_topic
    )

    assert not node.get_publishers_info_by_topic(
        "/cmd_vel_auto"
    )

    assert not node.get_publishers_info_by_topic(
        "/cmd_vel"
    )


def test_nonfinite_goal_is_rejected(
    runtime_harness,
) -> None:
    """A NaN heading must be rejected before execution."""

    runtime_harness.reset_observations()

    goal_handle = runtime_harness.send_goal(
        float("nan"),
        1.0,
    )

    assert not goal_handle.accepted

    time.sleep(0.20)

    assert not runtime_harness.has_motion_command()


def test_goal_succeeds_with_synthetic_odom(
    runtime_harness,
) -> None:
    """Synthetic odometry can complete a valid goal."""

    runtime_harness.reset_observations(
        yaw_rad=0.0,
    )

    goal_handle = runtime_harness.send_goal(
        target_yaw_rad=0.60,
        max_duration_sec=2.0,
    )

    assert goal_handle.accepted

    runtime_harness.wait_for(
        runtime_harness.has_positive_command,
        timeout_sec=2.0,
        description="positive angular command",
    )

    runtime_harness.wait_for(
        lambda: bool(
            runtime_harness.feedback_values()
        ),
        timeout_sec=2.0,
        description="action feedback",
    )

    runtime_harness.set_yaw(0.60)

    wrapped_result = runtime_harness.wait_result(
        goal_handle,
        timeout_sec=3.0,
    )

    assert (
        wrapped_result.status ==
        GoalStatus.STATUS_SUCCEEDED
    )

    result = wrapped_result.result

    assert result.success
    assert result.reason == "goal_reached"
    assert math.isfinite(result.final_yaw_rad)
    assert math.isfinite(result.final_error_rad)
    assert abs(result.final_error_rad) <= 0.04

    feedback = (
        runtime_harness.feedback_values()[0]
    )

    assert math.isfinite(
        feedback.current_yaw_rad
    )
    assert math.isfinite(
        feedback.target_yaw_rad
    )
    assert math.isfinite(
        feedback.error_rad
    )
    assert math.isfinite(
        feedback.commanded_wz_rad_s
    )
    assert math.isfinite(
        feedback.elapsed_sec
    )
    assert feedback.state == "tracking"
    assert not feedback.safety_stop_active

    runtime_harness.wait_for(
        runtime_harness.has_zero_after_motion,
        timeout_sec=2.0,
        description="terminal zero command",
    )


def test_second_goal_is_rejected_and_first_cancels(
    runtime_harness,
) -> None:
    """Only one action goal may own rotation."""

    runtime_harness.reset_observations(
        yaw_rad=0.0,
    )

    first_goal = runtime_harness.send_goal(
        target_yaw_rad=0.90,
        max_duration_sec=2.0,
    )

    assert first_goal.accepted

    runtime_harness.wait_for(
        runtime_harness.has_motion_command,
        timeout_sec=2.0,
        description="first goal motion command",
    )

    second_goal = runtime_harness.send_goal(
        target_yaw_rad=-0.90,
        max_duration_sec=2.0,
    )

    assert not second_goal.accepted

    runtime_harness.cancel_goal(
        first_goal
    )

    wrapped_result = runtime_harness.wait_result(
        first_goal,
        timeout_sec=3.0,
    )

    assert (
        wrapped_result.status ==
        GoalStatus.STATUS_CANCELED
    )

    assert not wrapped_result.result.success
    assert (
        wrapped_result.result.reason ==
        "canceled"
    )

    runtime_harness.wait_for(
        runtime_harness.has_zero_after_motion,
        timeout_sec=2.0,
        description="cancel terminal zero command",
    )


def test_goal_timeout_aborts_and_stops(
    runtime_harness,
) -> None:
    """A goal-specific timeout must abort safely."""

    runtime_harness.reset_observations(
        yaw_rad=0.0,
    )

    goal_handle = runtime_harness.send_goal(
        target_yaw_rad=0.80,
        max_duration_sec=0.35,
    )

    assert goal_handle.accepted

    runtime_harness.wait_for(
        runtime_harness.has_motion_command,
        timeout_sec=2.0,
        description="timeout test motion command",
    )

    wrapped_result = runtime_harness.wait_result(
        goal_handle,
        timeout_sec=3.0,
    )

    assert (
        wrapped_result.status ==
        GoalStatus.STATUS_ABORTED
    )

    assert not wrapped_result.result.success
    assert (
        wrapped_result.result.reason ==
        "timeout"
    )

    runtime_harness.wait_for(
        runtime_harness.has_zero_after_motion,
        timeout_sec=2.0,
        description="timeout terminal zero command",
    )
