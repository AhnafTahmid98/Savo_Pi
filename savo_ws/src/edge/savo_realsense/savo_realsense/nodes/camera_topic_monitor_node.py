# Copyright 2026 Ahnaf Tahmid
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from savo_realsense.constants import DEFAULT_STATUS_HZ
from savo_realsense.ros import DEFAULT_TOPICS, load_stream_monitor_params
from savo_realsense.ros.qos_profiles import (
    camera_info_qos,
    diagnostics_qos,
    sensor_data_qos,
)
from savo_realsense.utils.camera_checks import build_stream_status
from savo_realsense.utils.diagnostics import make_diagnostic_array, make_stream_diagnostic
from savo_realsense.utils.param_loader import get_bool_param, get_float_param
from savo_realsense.utils.timing import RateTracker


class CameraTopicMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_topic_monitor_node")

        self._params = load_stream_monitor_params(self)
        self._publish_hz = get_float_param(self, "publish_hz", DEFAULT_STATUS_HZ, min_value=0.1)
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

        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            DEFAULT_TOPICS.diagnostics,
            diagnostics_qos(),
        )

        self.create_timer(1.0 / self._publish_hz, self._publish_diagnostics)

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

    def _publish_diagnostics(self) -> None:
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

        diagnostics = [
            make_stream_diagnostic("RealSense color image", color),
            make_stream_diagnostic("RealSense color camera info", color_info),
            make_stream_diagnostic("RealSense depth image", depth),
            make_stream_diagnostic("RealSense depth camera info", depth_info),
        ]

        if self._require_pointcloud or pointcloud.seen:
            diagnostics.append(make_stream_diagnostic("RealSense pointcloud", pointcloud))

        self._diagnostics_pub.publish(
            make_diagnostic_array(diagnostics, self.get_clock())
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = CameraTopicMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
