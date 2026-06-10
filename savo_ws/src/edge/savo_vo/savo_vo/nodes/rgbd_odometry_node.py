"""ROS 2 node that publishes RGB-D visual odometry."""

from __future__ import annotations

import message_filters
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from sensor_msgs.msg import CameraInfo, Image

from savo_vo.adapters.camera_info_adapter import camera_info_to_intrinsics
from savo_vo.adapters.image_adapter import (
    color_msg_to_gray_array,
    depth_msg_to_array,
    depth_to_meters,
)
from savo_vo.adapters.odometry_adapter import sample_to_odometry_msg
from savo_vo.contracts.frame_names import BASE_LINK_FRAME, ODOM_FRAME
from savo_vo.contracts.parameter_names import (
    BASE_FRAME_PARAM,
    COLOR_CAMERA_INFO_TOPIC_PARAM,
    COLOR_IMAGE_TOPIC_PARAM,
    DEPTH_CAMERA_INFO_TOPIC_PARAM,
    DEPTH_IMAGE_TOPIC_PARAM,
    HEALTH_TOPIC_PARAM,
    MAX_DEPTH_DELAY_S_PARAM,
    MAX_FEATURES_PARAM,
    MAX_IMAGE_DELAY_S_PARAM,
    MAX_ROTATION_JUMP_RAD_PARAM,
    MAX_TRANSLATION_JUMP_M_PARAM,
    MIN_FEATURES_PARAM,
    MIN_TRACKING_QUALITY_PARAM,
    ODOM_FRAME_PARAM,
    ODOM_TOPIC_PARAM,
    PUBLISH_DIAGNOSTICS_PARAM,
    PUBLISH_TF_PARAM,
    STATUS_TOPIC_PARAM,
    TRACKING_QUALITY_TOPIC_PARAM,
)
from savo_vo.contracts.topic_names import (
    COLOR_CAMERA_INFO_TOPIC,
    COLOR_IMAGE_TOPIC,
    DEPTH_CAMERA_INFO_TOPIC,
    DEPTH_IMAGE_TOPIC,
    VO_HEALTH_TOPIC,
    VO_ODOM_TOPIC,
    VO_STATUS_TOPIC,
    VO_TRACKING_QUALITY_TOPIC,
)
from savo_vo.core.odom_checks import is_sample_publishable, reject_reason
from savo_vo.estimators.rgbd_vo_estimator import RGBDVOEstimator
from savo_vo.ros.qos import camera_qos, odometry_qos, status_qos
from savo_vo.utils.ros_time import stamp_to_seconds


class RGBDOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("rgbd_odometry_node")

        self._declare_parameters()
        self._load_parameters()

        self._estimator = RGBDVOEstimator(
            min_features=self._min_features,
            max_features=self._max_features,
            min_tracking_quality=self._min_tracking_quality,
            max_translation_jump_m=self._max_translation_jump_m,
        )

        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, odometry_qos())
        self._status_pub = self.create_publisher(String, self._status_topic, status_qos())
        self._health_pub = self.create_publisher(String, self._health_topic, status_qos())
        self._tracking_quality_pub = self.create_publisher(
            Float32,
            self._tracking_quality_topic,
            status_qos(),
        )

        color_sub = message_filters.Subscriber(
            self,
            Image,
            self._color_image_topic,
            qos_profile=camera_qos(),
        )
        depth_sub = message_filters.Subscriber(
            self,
            Image,
            self._depth_image_topic,
            qos_profile=camera_qos(),
        )
        info_sub = message_filters.Subscriber(
            self,
            CameraInfo,
            self._color_camera_info_topic,
            qos_profile=camera_qos(),
        )

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, info_sub],
            queue_size=10,
            slop=max(self._max_image_delay_s, self._max_depth_delay_s),
        )
        self._sync.registerCallback(self._on_rgbd_frame)

        self.get_logger().info(
            "RGB-D VO node started: "
            f"color={self._color_image_topic}, "
            f"depth={self._depth_image_topic}, "
            f"odom={self._odom_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter(COLOR_IMAGE_TOPIC_PARAM, COLOR_IMAGE_TOPIC)
        self.declare_parameter(COLOR_CAMERA_INFO_TOPIC_PARAM, COLOR_CAMERA_INFO_TOPIC)
        self.declare_parameter(DEPTH_IMAGE_TOPIC_PARAM, DEPTH_IMAGE_TOPIC)
        self.declare_parameter(DEPTH_CAMERA_INFO_TOPIC_PARAM, DEPTH_CAMERA_INFO_TOPIC)

        self.declare_parameter(ODOM_TOPIC_PARAM, VO_ODOM_TOPIC)
        self.declare_parameter(STATUS_TOPIC_PARAM, VO_STATUS_TOPIC)
        self.declare_parameter(HEALTH_TOPIC_PARAM, VO_HEALTH_TOPIC)
        self.declare_parameter(TRACKING_QUALITY_TOPIC_PARAM, VO_TRACKING_QUALITY_TOPIC)

        self.declare_parameter(ODOM_FRAME_PARAM, ODOM_FRAME)
        self.declare_parameter(BASE_FRAME_PARAM, BASE_LINK_FRAME)

        self.declare_parameter(MAX_IMAGE_DELAY_S_PARAM, 0.15)
        self.declare_parameter(MAX_DEPTH_DELAY_S_PARAM, 0.15)

        self.declare_parameter(MIN_FEATURES_PARAM, 80)
        self.declare_parameter(MAX_FEATURES_PARAM, 800)
        self.declare_parameter(MIN_TRACKING_QUALITY_PARAM, 0.35)
        self.declare_parameter(MAX_TRANSLATION_JUMP_M_PARAM, 0.30)
        self.declare_parameter(MAX_ROTATION_JUMP_RAD_PARAM, 0.35)

        self.declare_parameter(PUBLISH_TF_PARAM, False)
        self.declare_parameter(PUBLISH_DIAGNOSTICS_PARAM, True)

    def _load_parameters(self) -> None:
        self._color_image_topic = self.get_parameter(
            COLOR_IMAGE_TOPIC_PARAM
        ).get_parameter_value().string_value
        self._color_camera_info_topic = self.get_parameter(
            COLOR_CAMERA_INFO_TOPIC_PARAM
        ).get_parameter_value().string_value
        self._depth_image_topic = self.get_parameter(
            DEPTH_IMAGE_TOPIC_PARAM
        ).get_parameter_value().string_value
        self._depth_camera_info_topic = self.get_parameter(
            DEPTH_CAMERA_INFO_TOPIC_PARAM
        ).get_parameter_value().string_value

        self._odom_topic = self.get_parameter(
            ODOM_TOPIC_PARAM
        ).get_parameter_value().string_value
        self._status_topic = self.get_parameter(
            STATUS_TOPIC_PARAM
        ).get_parameter_value().string_value
        self._health_topic = self.get_parameter(
            HEALTH_TOPIC_PARAM
        ).get_parameter_value().string_value
        self._tracking_quality_topic = self.get_parameter(
            TRACKING_QUALITY_TOPIC_PARAM
        ).get_parameter_value().string_value

        self._odom_frame = self.get_parameter(
            ODOM_FRAME_PARAM
        ).get_parameter_value().string_value
        self._base_frame = self.get_parameter(
            BASE_FRAME_PARAM
        ).get_parameter_value().string_value

        self._max_image_delay_s = self.get_parameter(
            MAX_IMAGE_DELAY_S_PARAM
        ).get_parameter_value().double_value
        self._max_depth_delay_s = self.get_parameter(
            MAX_DEPTH_DELAY_S_PARAM
        ).get_parameter_value().double_value

        self._min_features = self.get_parameter(
            MIN_FEATURES_PARAM
        ).get_parameter_value().integer_value
        self._max_features = self.get_parameter(
            MAX_FEATURES_PARAM
        ).get_parameter_value().integer_value
        self._min_tracking_quality = self.get_parameter(
            MIN_TRACKING_QUALITY_PARAM
        ).get_parameter_value().double_value
        self._max_translation_jump_m = self.get_parameter(
            MAX_TRANSLATION_JUMP_M_PARAM
        ).get_parameter_value().double_value
        self._max_rotation_jump_rad = self.get_parameter(
            MAX_ROTATION_JUMP_RAD_PARAM
        ).get_parameter_value().double_value

        self._publish_tf = self.get_parameter(
            PUBLISH_TF_PARAM
        ).get_parameter_value().bool_value
        self._publish_diagnostics = self.get_parameter(
            PUBLISH_DIAGNOSTICS_PARAM
        ).get_parameter_value().bool_value

    def _on_rgbd_frame(
        self,
        color_msg: Image,
        depth_msg: Image,
        camera_info_msg: CameraInfo,
    ) -> None:
        try:
            timestamp_s = stamp_to_seconds(color_msg.header.stamp)
            gray = color_msg_to_gray_array(color_msg)
            depth_raw = depth_msg_to_array(depth_msg)
            depth_m = depth_to_meters(depth_raw)
            intrinsics = camera_info_to_intrinsics(camera_info_msg)

            if not intrinsics.is_valid:
                self._publish_status("camera intrinsics are invalid")
                return

            sample = self._estimator.process_frame(
                timestamp_s=timestamp_s,
                gray_image=gray,
                depth_m=depth_m,
                intrinsics=intrinsics,
            )

            if sample is None:
                self._publish_status("waiting for visual odometry reference frame")
                return

            self._publish_tracking_quality(sample.tracking.tracking_quality)

            if not is_sample_publishable(sample):
                reason = reject_reason(sample)
                self._publish_status(reason)
                return

            odom_msg = sample_to_odometry_msg(
                sample=sample,
                odom_frame=self._odom_frame,
                base_frame=self._base_frame,
            )
            self._odom_pub.publish(odom_msg)

            self._publish_status(sample.status.message)
            self._publish_health(sample.status.state.value)

        except Exception as exc:
            self.get_logger().error(f"RGB-D VO frame processing failed: {exc}")
            self._publish_status(f"RGB-D VO error: {exc}")

    def _publish_status(self, message: str) -> None:
        msg = String()
        msg.data = message
        self._status_pub.publish(msg)

    def _publish_health(self, message: str) -> None:
        msg = String()
        msg.data = message
        self._health_pub.publish(msg)

    def _publish_tracking_quality(self, quality: float) -> None:
        msg = Float32()
        msg.data = float(quality)
        self._tracking_quality_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RGBDOdometryNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
