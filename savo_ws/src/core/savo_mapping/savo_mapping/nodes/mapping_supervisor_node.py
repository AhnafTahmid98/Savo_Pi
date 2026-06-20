#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Fallback mapping supervisor for Robot Savo."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

from savo_mapping.constants import (
    DEFAULT_MAP_STALE_TIMEOUT_S,
    DEFAULT_MIN_SCAN_RATE_HZ,
    DEFAULT_ODOM_STALE_TIMEOUT_S,
    DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
    DEFAULT_PUBLISH_RATE_HZ,
    DEFAULT_SCAN_STALE_TIMEOUT_S,
    DEFAULT_TF_TIMEOUT_S,
    FRAME_BASE_LINK,
    FRAME_LASER,
    FRAME_MAP,
    FRAME_ODOM,
    TOPIC_MAP,
    TOPIC_ODOM,
    TOPIC_REALSENSE_POINTS,
    TOPIC_SCAN,
)
from savo_mapping.diagnostics import (
    evaluate_default_tf_ready,
    evaluate_map_msg,
    evaluate_odom_msg,
    evaluate_pointcloud_msg,
    evaluate_scan_msg,
    map_topic_result_to_diagnostic,
    odom_result_to_diagnostic,
    pointcloud_result_to_diagnostic,
    scan_result_to_diagnostic,
    tf_result_to_diagnostic,
)
from savo_mapping.models.mapping_mode import MappingMode, require_valid_mapping_mode
from savo_mapping.models.mapping_status import make_mapping_status
from savo_mapping.models.readiness_state import build_readiness_state, make_check
from savo_mapping.ros.adapters import json_msg
from savo_mapping.ros.qos_profiles import get_topic_qos_profile, status_qos, tf_static_qos
from savo_mapping.utils.diagnostics import DiagnosticItem, build_diagnostic_report
from savo_mapping.utils.timing import RateTracker, age_s, now_s


@dataclass
class TopicRuntime:
    name: str
    latest_msg: Optional[Any] = None
    msg_count: int = 0
    last_wall_s: Optional[float] = None

    def __post_init__(self) -> None:
        self.rate_tracker = RateTracker()

    @property
    def rate_hz(self) -> float:
        return self.rate_tracker.rate_hz

    @property
    def sample_age_s(self) -> Optional[float]:
        return age_s(self.last_wall_s)

    def update(self, msg: Any) -> None:
        stamp = now_s()

        self.latest_msg = msg
        self.msg_count += 1
        self.last_wall_s = stamp
        self.rate_tracker.tick(stamp)


@dataclass
class TfEdgeRuntime:
    parent_frame: str
    child_frame: str
    last_wall_s: Optional[float] = None
    available: bool = False
    static: bool = False

    @property
    def key(self) -> str:
        return f"{self.parent_frame}->{self.child_frame}"

    @property
    def sample_age_s(self) -> Optional[float]:
        if self.static and self.available:
            return 0.0

        return age_s(self.last_wall_s)

    def update(self, *, static: bool = False) -> None:
        self.available = True
        self.static = self.static or bool(static)
        self.last_wall_s = now_s()


class MappingSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("mapping_supervisor_node")

        self._declare_parameters()
        self._load_parameters()

        self._scan = TopicRuntime("scan")
        self._odom = TopicRuntime("odom")
        self._map = TopicRuntime("map")
        self._pointcloud = TopicRuntime("pointcloud")

        self._tf_edges = {
            f"{self.map_frame}->{self.odom_frame}": TfEdgeRuntime(
                self.map_frame,
                self.odom_frame,
            ),
            f"{self.odom_frame}->{self.base_frame}": TfEdgeRuntime(
                self.odom_frame,
                self.base_frame,
            ),
            f"{self.base_frame}->{self.laser_frame}": TfEdgeRuntime(
                self.base_frame,
                self.laser_frame,
            ),
        }

        self._last_log_state: Optional[tuple[bool, bool, str]] = None

        self._create_publishers()
        self._create_subscriptions()
        self._create_timers()

        self.get_logger().info(
            "Mapping supervisor started: "
            f"mode={self.mode} scan={self.scan_topic} odom={self.odom_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("mode", MappingMode.MANUAL_MAPPING.value)
        self.declare_parameter("map_name", "")
        self.declare_parameter("session_id", "")

        self.declare_parameter("scan_topic", TOPIC_SCAN)
        self.declare_parameter("odom_topic", TOPIC_ODOM)
        self.declare_parameter("map_topic", TOPIC_MAP)
        self.declare_parameter("pointcloud_topic", TOPIC_REALSENSE_POINTS)

        self.declare_parameter("map_frame", FRAME_MAP)
        self.declare_parameter("odom_frame", FRAME_ODOM)
        self.declare_parameter("base_frame", FRAME_BASE_LINK)
        self.declare_parameter("laser_frame", FRAME_LASER)

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)

        self.declare_parameter("require_scan", True)
        self.declare_parameter("require_odom", True)
        self.declare_parameter("require_tf", True)
        self.declare_parameter("require_map", False)
        self.declare_parameter("require_pointcloud", False)

        self.declare_parameter("min_scan_rate_hz", DEFAULT_MIN_SCAN_RATE_HZ)
        self.declare_parameter("min_odom_rate_hz", 5.0)
        self.declare_parameter("min_pointcloud_rate_hz", 3.0)
        self.declare_parameter("min_pointcloud_points", 100)

        self.declare_parameter("scan_stale_timeout_s", DEFAULT_SCAN_STALE_TIMEOUT_S)
        self.declare_parameter("odom_stale_timeout_s", DEFAULT_ODOM_STALE_TIMEOUT_S)
        self.declare_parameter("tf_timeout_s", DEFAULT_TF_TIMEOUT_S)
        self.declare_parameter("map_stale_timeout_s", DEFAULT_MAP_STALE_TIMEOUT_S)
        self.declare_parameter(
            "pointcloud_stale_timeout_s",
            DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
        )

        self.declare_parameter("publish_map_quality", True)
        self.declare_parameter("verbose_status_log", False)

    def _load_parameters(self) -> None:
        self.mode = require_valid_mapping_mode(self.get_parameter("mode").value)

        self.map_name = str(self.get_parameter("map_name").value or "")
        self.session_id = str(self.get_parameter("session_id").value or "") or None

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.laser_frame = str(self.get_parameter("laser_frame").value)

        self.publish_rate_hz = self._positive_float_parameter(
            "publish_rate_hz",
            DEFAULT_PUBLISH_RATE_HZ,
        )

        self.require_scan = bool(self.get_parameter("require_scan").value)
        self.require_odom = bool(self.get_parameter("require_odom").value)
        self.require_tf = bool(self.get_parameter("require_tf").value)
        self.require_map = bool(self.get_parameter("require_map").value)
        self.require_pointcloud = bool(
            self.get_parameter("require_pointcloud").value
        )

        self.min_scan_rate_hz = self._non_negative_float_parameter(
            "min_scan_rate_hz",
            DEFAULT_MIN_SCAN_RATE_HZ,
        )
        self.min_odom_rate_hz = self._non_negative_float_parameter(
            "min_odom_rate_hz",
            5.0,
        )
        self.min_pointcloud_rate_hz = self._non_negative_float_parameter(
            "min_pointcloud_rate_hz",
            3.0,
        )
        self.min_pointcloud_points = max(
            0,
            int(self.get_parameter("min_pointcloud_points").value),
        )

        self.scan_stale_timeout_s = self._positive_float_parameter(
            "scan_stale_timeout_s",
            DEFAULT_SCAN_STALE_TIMEOUT_S,
        )
        self.odom_stale_timeout_s = self._positive_float_parameter(
            "odom_stale_timeout_s",
            DEFAULT_ODOM_STALE_TIMEOUT_S,
        )
        self.tf_timeout_s = self._positive_float_parameter(
            "tf_timeout_s",
            DEFAULT_TF_TIMEOUT_S,
        )
        self.map_stale_timeout_s = self._positive_float_parameter(
            "map_stale_timeout_s",
            DEFAULT_MAP_STALE_TIMEOUT_S,
        )
        self.pointcloud_stale_timeout_s = self._positive_float_parameter(
            "pointcloud_stale_timeout_s",
            DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
        )

        self.publish_map_quality = bool(
            self.get_parameter("publish_map_quality").value
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

    def _non_negative_float_parameter(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)

        if value < 0.0:
            self.get_logger().warning(
                f"Parameter {name} must be non-negative. Using {default}."
            )
            return float(default)

        return value

    def _create_publishers(self) -> None:
        self.ready_pub = self.create_publisher(
            Bool,
            "/savo_mapping/ready",
            status_qos(),
        )
        self.status_pub = self.create_publisher(
            String,
            "/savo_mapping/status",
            status_qos(),
        )
        self.mode_pub = self.create_publisher(
            String,
            "/savo_mapping/mode",
            status_qos(),
        )
        self.map_quality_pub = self.create_publisher(
            String,
            "/savo_mapping/map_quality",
            status_qos(),
        )

    def _create_subscriptions(self) -> None:
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._on_scan,
            get_topic_qos_profile(self.scan_topic),
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            get_topic_qos_profile(self.odom_topic),
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._on_map,
            get_topic_qos_profile(self.map_topic),
        )
        self.tf_sub = self.create_subscription(
            TFMessage,
            "/tf",
            self._on_tf,
            get_topic_qos_profile("/tf"),
        )
        self.tf_static_sub = self.create_subscription(
            TFMessage,
            "/tf_static",
            self._on_tf_static,
            tf_static_qos(),
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._on_pointcloud,
            get_topic_qos_profile(self.pointcloud_topic),
        )

    def _create_timers(self) -> None:
        self.status_timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._on_status_timer,
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan.update(msg)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom.update(msg)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map.update(msg)

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        self._pointcloud.update(msg)

    def _on_tf(self, msg: TFMessage) -> None:
        self._update_tf_edges(msg, static=False)

    def _on_tf_static(self, msg: TFMessage) -> None:
        self._update_tf_edges(msg, static=True)

    def _update_tf_edges(self, msg: TFMessage, *, static: bool) -> None:
        for transform in msg.transforms:
            parent = str(transform.header.frame_id).strip()
            child = str(transform.child_frame_id).strip()
            edge = self._tf_edges.get(f"{parent}->{child}")

            if edge is not None:
                edge.update(static=static)

    def _on_status_timer(self) -> None:
        status, report = self._build_status()

        self.ready_pub.publish(Bool(data=bool(status.ready)))
        self.mode_pub.publish(String(data=str(status.mode)))
        self.status_pub.publish(json_msg(status.to_dict()))

        if self.publish_map_quality and self._map.latest_msg is not None:
            map_result = self._evaluate_map()
            self.map_quality_pub.publish(json_msg(map_result.quality.to_dict()))

        log_state = (
            bool(status.ready),
            bool(status.degraded),
            str(status.mode),
        )

        if self.verbose_status_log or log_state != self._last_log_state:
            self.get_logger().info(
                "Mapping status: "
                f"ready={status.ready} degraded={status.degraded} "
                f"mode={status.mode} report_ok={report.ok}"
            )

        self._last_log_state = log_state

    def _evaluate_scan(self):
        if self._scan.latest_msg is None:
            return evaluate_scan_msg(
                _empty_scan_msg(self.laser_frame),
                msg_count=0,
                rate_hz=0.0,
                age_s=None,
                topic=self.scan_topic,
                min_rate_hz=self.min_scan_rate_hz,
                stale_timeout_s=self.scan_stale_timeout_s,
            )

        return evaluate_scan_msg(
            self._scan.latest_msg,
            msg_count=self._scan.msg_count,
            rate_hz=self._scan.rate_hz,
            age_s=self._scan.sample_age_s,
            topic=self.scan_topic,
            min_rate_hz=self.min_scan_rate_hz,
            stale_timeout_s=self.scan_stale_timeout_s,
        )

    def _evaluate_odom(self):
        if self._odom.latest_msg is None:
            return evaluate_odom_msg(
                _empty_odom_msg(self.odom_frame, self.base_frame),
                msg_count=0,
                rate_hz=0.0,
                age_s=None,
                topic=self.odom_topic,
                min_rate_hz=self.min_odom_rate_hz,
                stale_timeout_s=self.odom_stale_timeout_s,
            )

        return evaluate_odom_msg(
            self._odom.latest_msg,
            msg_count=self._odom.msg_count,
            rate_hz=self._odom.rate_hz,
            age_s=self._odom.sample_age_s,
            topic=self.odom_topic,
            min_rate_hz=self.min_odom_rate_hz,
            stale_timeout_s=self.odom_stale_timeout_s,
        )

    def _evaluate_tf(self):
        ages = {
            key: edge.sample_age_s
            for key, edge in self._tf_edges.items()
        }
        available = {
            key: edge.available
            for key, edge in self._tf_edges.items()
        }

        return evaluate_default_tf_ready(
            edge_ages_s=ages,
            available_edges=available,
            frame_count=sum(1 for edge in self._tf_edges.values() if edge.available),
            timeout_s=self.tf_timeout_s,
        )

    def _evaluate_map(self):
        if self._map.latest_msg is None:
            return evaluate_map_msg(
                _empty_map_msg(self.map_frame),
                msg_count=0,
                rate_hz=0.0,
                age_s=None,
                topic=self.map_topic,
                stale_timeout_s=self.map_stale_timeout_s,
                required=self.require_map,
            )

        return evaluate_map_msg(
            self._map.latest_msg,
            msg_count=self._map.msg_count,
            rate_hz=self._map.rate_hz,
            age_s=self._map.sample_age_s,
            topic=self.map_topic,
            stale_timeout_s=self.map_stale_timeout_s,
            required=self.require_map,
        )

    def _evaluate_pointcloud(self):
        if self._pointcloud.latest_msg is None:
            return evaluate_pointcloud_msg(
                _empty_pointcloud_msg("camera_depth_optical_frame"),
                enabled=self.require_pointcloud,
                msg_count=0,
                rate_hz=0.0,
                age_s=None,
                topic=self.pointcloud_topic,
                min_rate_hz=self.min_pointcloud_rate_hz,
                min_points=self.min_pointcloud_points,
                stale_timeout_s=self.pointcloud_stale_timeout_s,
            )

        return evaluate_pointcloud_msg(
            self._pointcloud.latest_msg,
            enabled=self.require_pointcloud,
            msg_count=self._pointcloud.msg_count,
            rate_hz=self._pointcloud.rate_hz,
            age_s=self._pointcloud.sample_age_s,
            topic=self.pointcloud_topic,
            min_rate_hz=self.min_pointcloud_rate_hz,
            min_points=self.min_pointcloud_points,
            stale_timeout_s=self.pointcloud_stale_timeout_s,
        )

    def _build_status(self):
        scan_result = self._evaluate_scan()
        odom_result = self._evaluate_odom()
        tf_result = self._evaluate_tf()
        map_result = self._evaluate_map()
        pointcloud_result = self._evaluate_pointcloud()

        items: list[DiagnosticItem] = [
            scan_result_to_diagnostic(scan_result, required=self.require_scan),
            odom_result_to_diagnostic(odom_result, required=self.require_odom),
            tf_result_to_diagnostic(tf_result, required=self.require_tf),
            map_topic_result_to_diagnostic(map_result, required=self.require_map),
            pointcloud_result_to_diagnostic(
                pointcloud_result,
                required=self.require_pointcloud,
            ),
        ]

        report = build_diagnostic_report("savo_mapping_supervisor", items)

        readiness = build_readiness_state(
            tuple(
                make_check(
                    name=item.name,
                    ok=item.ok,
                    required=item.required,
                    enabled=item.enabled,
                    message=item.message,
                    age_s=_extract_age_s(item.values),
                )
                for item in items
            )
        )

        if readiness.ready:
            message = "Mapping supervisor ready."
        else:
            failed = ", ".join(readiness.failed_required_checks)
            message = f"Mapping supervisor waiting: {failed}."

        status = make_mapping_status(
            mode=self.mode,
            readiness=readiness,
            active=self.mode != MappingMode.IDLE.value,
            message=message,
            map_name=self.map_name or None,
            session_id=self.session_id,
            extra={
                "report": report.to_dict(),
                "topics": {
                    "scan": self.scan_topic,
                    "odom": self.odom_topic,
                    "map": self.map_topic,
                    "pointcloud": self.pointcloud_topic,
                },
                "requirements": {
                    "scan": self.require_scan,
                    "odom": self.require_odom,
                    "tf": self.require_tf,
                    "map": self.require_map,
                    "pointcloud": self.require_pointcloud,
                },
            },
        )

        return status, report


def _empty_scan_msg(frame_id: str) -> LaserScan:
    msg = LaserScan()
    msg.header.frame_id = str(frame_id)
    msg.range_min = 0.15
    msg.range_max = 12.0
    return msg


def _empty_odom_msg(frame_id: str, child_frame_id: str) -> Odometry:
    msg = Odometry()
    msg.header.frame_id = str(frame_id)
    msg.child_frame_id = str(child_frame_id)
    return msg


def _empty_map_msg(frame_id: str) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.frame_id = str(frame_id)
    return msg


def _empty_pointcloud_msg(frame_id: str) -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = str(frame_id)
    return msg


def _extract_age_s(values: dict[str, Any]) -> Optional[float]:
    raw = values.get("age_s")

    if raw is None:
        return None

    try:
        return float(raw)
    except (TypeError, ValueError):
        return None


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)

    node = MappingSupervisorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Mapping supervisor stopped.")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
