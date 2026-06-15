# Copyright 2026 Ahnaf Tahmid
import json

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String

from savo_realsense.constants import DEFAULT_STATUS_HZ
from savo_realsense.models.camera_status import CameraStatus
from savo_realsense.ros import DEFAULT_TOPICS, load_stream_monitor_params
from savo_realsense.ros.qos_profiles import (
    camera_info_qos,
    diagnostics_qos,
    sensor_data_qos,
    status_qos,
)
from savo_realsense.utils.camera_checks import build_stream_status
from savo_realsense.utils.diagnostics import make_camera_diagnostic, make_diagnostic_array
from savo_realsense.utils.param_loader import get_bool_param, get_float_param
from savo_realsense.utils.timing import RateTracker


class CameraHealthNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_health_node")

        self._params = load_stream_monitor_params(self)
        self._status_hz = get_float_param(self, "status_hz", DEFAULT_STATUS_HZ, min_value=0.1)
        self._require_pointcloud = get_bool_param(self, "require_pointcloud", False)

        self._color_tracker = RateTracker()
        self._color_info_tracker = RateTracker()
        self._depth_tracker = RateTracker()
        self._depth_info_tracker = RateTracker()
        self._pointcloud_tracker = RateTracker()

        self.create_subscription(
            Image,
            DEFAULT_TOPICS.color_image,
            self._on_color_image,
            sensor_data_qos(),
        )
        self.create_subscription(
            CameraInfo,
            DEFAULT_TOPICS.color_info,
            self._on_color_info,
            camera_info_qos(),
        )
        self.create_subscription(
            Image,
            DEFAULT_TOPICS.depth_image,
            self._on_depth_image,
            sensor_data_qos(),
        )
        self.create_subscription(
            CameraInfo,
            DEFAULT_TOPICS.depth_info,
            self._on_depth_info,
            camera_info_qos(),
        )
        self.create_subscription(
            PointCloud2,
            DEFAULT_TOPICS.pointcloud,
            self._on_pointcloud,
            sensor_data_qos(),
        )

        self._status_pub = self.create_publisher(
            String,
            DEFAULT_TOPICS.status,
            status_qos(),
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            DEFAULT_TOPICS.diagnostics,
            diagnostics_qos(),
        )

        self.create_timer(1.0 / self._status_hz, self._publish_status)

    def _on_color_image(self, _: Image) -> None:
        self._color_tracker.tick()

    def _on_color_info(self, _: CameraInfo) -> None:
        self._color_info_tracker.tick()

    def _on_depth_image(self, _: Image) -> None:
        self._depth_tracker.tick()

    def _on_depth_info(self, _: CameraInfo) -> None:
        self._depth_info_tracker.tick()

    def _on_pointcloud(self, _: PointCloud2) -> None:
        self._pointcloud_tracker.tick()

    def _publish_status(self) -> None:
        status = self._build_camera_status()

        self._status_pub.publish(String(data=self._status_json(status)))
        self._diagnostics_pub.publish(
            make_diagnostic_array(
                [make_camera_diagnostic(status)],
                self.get_clock(),
            )
        )

    def _build_camera_status(self) -> CameraStatus:
        color = build_stream_status(
            topic=DEFAULT_TOPICS.color_image,
            seen=self._color_tracker.seen,
            rate_hz=self._color_tracker.rate_hz,
            expected_hz=self._params.expected_color_hz,
            last_age_s=self._color_tracker.last_age_s,
            stale_timeout_s=self._params.stale_timeout_s,
        )
        color_info = build_stream_status(
            topic=DEFAULT_TOPICS.color_info,
            seen=self._color_info_tracker.seen,
            rate_hz=self._color_info_tracker.rate_hz,
            expected_hz=self._params.expected_camera_info_hz,
            last_age_s=self._color_info_tracker.last_age_s,
            stale_timeout_s=self._params.stale_timeout_s,
        )
        depth = build_stream_status(
            topic=DEFAULT_TOPICS.depth_image,
            seen=self._depth_tracker.seen,
            rate_hz=self._depth_tracker.rate_hz,
            expected_hz=self._params.expected_depth_hz,
            last_age_s=self._depth_tracker.last_age_s,
            stale_timeout_s=self._params.stale_timeout_s,
        )
        depth_info = build_stream_status(
            topic=DEFAULT_TOPICS.depth_info,
            seen=self._depth_info_tracker.seen,
            rate_hz=self._depth_info_tracker.rate_hz,
            expected_hz=self._params.expected_camera_info_hz,
            last_age_s=self._depth_info_tracker.last_age_s,
            stale_timeout_s=self._params.stale_timeout_s,
        )
        pointcloud = build_stream_status(
            topic=DEFAULT_TOPICS.pointcloud,
            seen=self._pointcloud_tracker.seen,
            rate_hz=self._pointcloud_tracker.rate_hz,
            expected_hz=self._params.expected_pointcloud_hz,
            last_age_s=self._pointcloud_tracker.last_age_s,
            stale_timeout_s=self._params.stale_timeout_s,
        )

        return CameraStatus(
            color=color,
            color_info=color_info,
            depth=depth,
            depth_info=depth_info,
            pointcloud=pointcloud,
            require_pointcloud=self._require_pointcloud,
        )

    def _status_json(self, status: CameraStatus) -> str:
        payload = {
            "ok": status.ok,
            "message": status.message,
            "color_ok": status.color_ok,
            "color_info_ok": status.color_info_ok,
            "depth_ok": status.depth_ok,
            "depth_info_ok": status.depth_info_ok,
            "pointcloud_ok": status.pointcloud_ok,
            "require_pointcloud": status.require_pointcloud,
        }
        return json.dumps(payload, separators=(",", ":"))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = CameraHealthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
