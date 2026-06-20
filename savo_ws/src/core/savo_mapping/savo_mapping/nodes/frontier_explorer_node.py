#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Frontier explorer node for Robot Savo mapping."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from savo_mapping.constants import (
    DEFAULT_MAP_STALE_TIMEOUT_S,
    DEFAULT_ODOM_STALE_TIMEOUT_S,
    DEFAULT_PUBLISH_RATE_HZ,
    TOPIC_MAP,
    TOPIC_ODOM,
)
from savo_mapping.exploration.frontier_explorer_core import (
    DECISION_GOAL_SELECTED,
    ExplorationCycleInput,
    ExplorationMapInput,
    ExplorationSafetyState,
    FrontierExplorerCore,
)
from savo_mapping.exploration.goal_selector import RobotPose2D
from savo_mapping.models.exploration_status import ExplorationGoal
from savo_mapping.ros.adapters import json_msg
from savo_mapping.ros.qos_profiles import get_topic_qos_profile, status_qos
from savo_mapping.utils.timing import age_s, now_s


@dataclass
class MapRuntime:
    latest_msg: Optional[OccupancyGrid] = None
    msg_count: int = 0
    last_wall_s: Optional[float] = None

    @property
    def sample_age_s(self) -> Optional[float]:
        return age_s(self.last_wall_s)

    def update(self, msg: OccupancyGrid) -> None:
        self.latest_msg = msg
        self.msg_count += 1
        self.last_wall_s = now_s()


@dataclass
class OdomRuntime:
    latest_msg: Optional[Odometry] = None
    msg_count: int = 0
    last_wall_s: Optional[float] = None

    @property
    def sample_age_s(self) -> Optional[float]:
        return age_s(self.last_wall_s)

    def update(self, msg: Odometry) -> None:
        self.latest_msg = msg
        self.msg_count += 1
        self.last_wall_s = now_s()


@dataclass(frozen=True)
class FrontierExplorerNodeStatus:
    enabled: bool
    ok: bool
    decision: str
    message: str
    map_ready: bool
    odom_ready: bool
    safety_stop: bool
    selected_goal: Optional[ExplorationGoal] = None
    map_age_s: Optional[float] = None
    odom_age_s: Optional[float] = None
    map_msg_count: int = 0
    odom_msg_count: int = 0
    timestamp_s: float = 0.0
    extra: Optional[dict[str, Any]] = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "ok": self.ok,
            "decision": self.decision,
            "message": self.message,
            "map_ready": self.map_ready,
            "odom_ready": self.odom_ready,
            "safety_stop": self.safety_stop,
            "selected_goal": (
                self.selected_goal.to_dict()
                if self.selected_goal is not None
                else None
            ),
            "map_age_s": self.map_age_s,
            "odom_age_s": self.odom_age_s,
            "map_msg_count": self.map_msg_count,
            "odom_msg_count": self.odom_msg_count,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra or {}),
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True)


class FrontierExplorerNode(Node):
    def __init__(self) -> None:
        super().__init__("frontier_explorer_node")

        self._declare_parameters()
        self._load_parameters()

        self._map = MapRuntime()
        self._odom = OdomRuntime()
        self._safety_stop = False
        self._last_status_text = ""
        self._last_goal_key: Optional[tuple[float, float, float]] = None

        self._core = FrontierExplorerCore()

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            status_qos(),
        )
        self.goal_pub = self.create_publisher(
            String,
            self.goal_topic,
            status_qos(),
        )
        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            self.goal_pose_topic,
            status_qos(),
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._on_map,
            get_topic_qos_profile(self.map_topic),
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            get_topic_qos_profile(self.odom_topic),
        )
        self.safety_stop_sub = self.create_subscription(
            Bool,
            self.safety_stop_topic,
            self._on_safety_stop,
            status_qos(),
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._on_timer,
        )

        self.get_logger().info(
            "Frontier explorer started: "
            f"enabled={self.enabled} map={self.map_topic} odom={self.odom_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("enabled", False)

        self.declare_parameter("map_topic", TOPIC_MAP)
        self.declare_parameter("odom_topic", TOPIC_ODOM)
        self.declare_parameter("safety_stop_topic", "/safety/stop")

        self.declare_parameter(
            "status_topic",
            "/savo_mapping/frontier_explorer/status",
        )
        self.declare_parameter(
            "goal_topic",
            "/savo_mapping/frontier_explorer/goal",
        )
        self.declare_parameter(
            "goal_pose_topic",
            "/savo_mapping/frontier_explorer/goal_pose",
        )

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter("map_stale_timeout_s", DEFAULT_MAP_STALE_TIMEOUT_S)
        self.declare_parameter("odom_stale_timeout_s", DEFAULT_ODOM_STALE_TIMEOUT_S)

        self.declare_parameter("require_odom", True)
        self.declare_parameter("publish_repeated_goal", False)
        self.declare_parameter("verbose_status_log", False)

    def _load_parameters(self) -> None:
        self.enabled = bool(self.get_parameter("enabled").value)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)

        self.status_topic = str(self.get_parameter("status_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)

        self.publish_rate_hz = self._positive_float_parameter(
            "publish_rate_hz",
            DEFAULT_PUBLISH_RATE_HZ,
        )
        self.map_stale_timeout_s = self._positive_float_parameter(
            "map_stale_timeout_s",
            DEFAULT_MAP_STALE_TIMEOUT_S,
        )
        self.odom_stale_timeout_s = self._positive_float_parameter(
            "odom_stale_timeout_s",
            DEFAULT_ODOM_STALE_TIMEOUT_S,
        )

        self.require_odom = bool(self.get_parameter("require_odom").value)
        self.publish_repeated_goal = bool(
            self.get_parameter("publish_repeated_goal").value
        )
        self.verbose_status_log = bool(
            self.get_parameter("verbose_status_log").value
        )

    def _positive_float_parameter(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)

        if value <= 0.0:
            self.get_logger().warning(
                f"Parameter {name} must be positive. Using {default}."
            )
            return float(default)

        return value

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map.update(msg)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom.update(msg)

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = bool(msg.data)

    def _on_timer(self) -> None:
        status = self._run_cycle()

        self.status_pub.publish(json_msg(status.to_dict()))

        if status.selected_goal is not None:
            should_publish_goal = (
                self.publish_repeated_goal
                or self._goal_key(status.selected_goal) != self._last_goal_key
            )

            if should_publish_goal:
                self.goal_pub.publish(json_msg(status.selected_goal.to_dict()))
                self.goal_pose_pub.publish(_goal_to_pose_stamped(status.selected_goal))
                self._last_goal_key = self._goal_key(status.selected_goal)

        status_text = (
            f"Frontier explorer: enabled={status.enabled} ok={status.ok} "
            f"decision={status.decision} map_ready={status.map_ready} "
            f"odom_ready={status.odom_ready} safety_stop={status.safety_stop}"
        )

        if self.verbose_status_log or status_text != self._last_status_text:
            self.get_logger().info(status_text)

        self._last_status_text = status_text

    def _run_cycle(self) -> FrontierExplorerNodeStatus:
        stamp = now_s()

        if not self.enabled:
            return FrontierExplorerNodeStatus(
                enabled=False,
                ok=True,
                decision="disabled",
                message="Frontier explorer disabled.",
                map_ready=self._map_ready(),
                odom_ready=self._odom_ready(),
                safety_stop=self._safety_stop,
                map_age_s=self._map.sample_age_s,
                odom_age_s=self._odom.sample_age_s,
                map_msg_count=self._map.msg_count,
                odom_msg_count=self._odom.msg_count,
                timestamp_s=stamp,
            )

        if not self._map_ready():
            return FrontierExplorerNodeStatus(
                enabled=True,
                ok=False,
                decision="waiting",
                message="Waiting for fresh occupancy grid map.",
                map_ready=False,
                odom_ready=self._odom_ready(),
                safety_stop=self._safety_stop,
                map_age_s=self._map.sample_age_s,
                odom_age_s=self._odom.sample_age_s,
                map_msg_count=self._map.msg_count,
                odom_msg_count=self._odom.msg_count,
                timestamp_s=stamp,
            )

        if self.require_odom and not self._odom_ready():
            return FrontierExplorerNodeStatus(
                enabled=True,
                ok=False,
                decision="waiting",
                message="Waiting for fresh odometry.",
                map_ready=True,
                odom_ready=False,
                safety_stop=self._safety_stop,
                map_age_s=self._map.sample_age_s,
                odom_age_s=self._odom.sample_age_s,
                map_msg_count=self._map.msg_count,
                odom_msg_count=self._odom.msg_count,
                timestamp_s=stamp,
            )

        cycle_result = self._core.run_cycle(
            ExplorationCycleInput(
                map_input=self._map_input(),
                robot_pose=self._robot_pose(),
                safety=ExplorationSafetyState(safety_stop=self._safety_stop),
            )
        )

        return FrontierExplorerNodeStatus(
            enabled=True,
            ok=cycle_result.ok,
            decision=cycle_result.decision,
            message=cycle_result.message,
            map_ready=True,
            odom_ready=self._odom_ready(),
            safety_stop=self._safety_stop,
            selected_goal=cycle_result.selected_goal,
            map_age_s=self._map.sample_age_s,
            odom_age_s=self._odom.sample_age_s,
            map_msg_count=self._map.msg_count,
            odom_msg_count=self._odom.msg_count,
            timestamp_s=stamp,
            extra={
                "core_state": cycle_result.state.to_dict(),
                "goal_selected": cycle_result.decision == DECISION_GOAL_SELECTED,
            },
        )

    def _map_ready(self) -> bool:
        if self._map.latest_msg is None:
            return False

        age = self._map.sample_age_s

        return age is not None and age <= self.map_stale_timeout_s

    def _odom_ready(self) -> bool:
        if self._odom.latest_msg is None:
            return False

        age = self._odom.sample_age_s

        return age is not None and age <= self.odom_stale_timeout_s

    def _map_input(self) -> ExplorationMapInput:
        msg = self._map.latest_msg

        if msg is None:
            raise RuntimeError("Cannot build exploration map input without map.")

        return ExplorationMapInput(
            width=int(msg.info.width),
            height=int(msg.info.height),
            resolution_m=float(msg.info.resolution),
            values=list(msg.data),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            origin_yaw=_yaw_from_quaternion(
                msg.info.origin.orientation.x,
                msg.info.origin.orientation.y,
                msg.info.origin.orientation.z,
                msg.info.origin.orientation.w,
            ),
            frame_id=str(msg.header.frame_id or "map"),
        )

    def _robot_pose(self) -> RobotPose2D:
        msg = self._odom.latest_msg

        if msg is None:
            return RobotPose2D()

        pose = msg.pose.pose

        return RobotPose2D(
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=_yaw_from_quaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            frame_id=str(msg.header.frame_id or "map"),
        )

    def _goal_key(self, goal: ExplorationGoal) -> tuple[float, float, float]:
        return (round(goal.x, 3), round(goal.y, 3), round(goal.yaw, 3))


def _goal_to_pose_stamped(goal: ExplorationGoal) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = goal.frame_id
    msg.header.stamp.sec = int(time.time())
    msg.pose.position.x = float(goal.x)
    msg.pose.position.y = float(goal.y)
    msg.pose.position.z = 0.0

    qx, qy, qz, qw = _quaternion_from_yaw(goal.yaw)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw

    return msg


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))

    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = float(yaw) * 0.5

    return 0.0, 0.0, math.sin(half), math.cos(half)


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)

    node = FrontierExplorerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Frontier explorer stopped.")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())