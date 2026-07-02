# -*- coding: utf-8 -*-

"""Python fallback AprilTag semantic confirmation node."""

from __future__ import annotations

import json
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import String

from savo_head.constants import (
    APRILTAG_FAMILY_DEFAULT,
    APRILTAG_MAX_DETECTION_DISTANCE_M_DEFAULT,
    APRILTAG_MIN_STABLE_FRAMES_DEFAULT,
    SEMANTIC_CONFIRMATION_SOURCE,
    SEMANTIC_CONFIRMATION_TYPE_KNOWN_LOCATION,
    STATUS_DRYRUN,
    STATUS_ERROR,
    STATUS_OK,
)
from savo_head.contracts.frame_names import BASE_LINK, MAP, PI_CAMERA_OPTICAL_FRAME
from savo_head.contracts.topic_names import (
    APRILTAG_DETECTIONS,
    DASHBOARD_TEXT,
    SEMANTIC_CONFIRMATIONS,
    STATUS,
)
from savo_head.models.semantic_confirmation import (
    AprilTagObservation,
    RobotPoseSnapshot,
    SemanticConfirmationPolicy,
    TagRegistration,
    make_confirmation,
    make_rejection,
)


class AprilTagConfirmNode(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_confirm_node_py")

        self._declare_parameters()

        self._registrations = self._load_registrations()
        self._policy = self._load_policy()
        self._latest_robot_pose: Optional[RobotPoseSnapshot] = None
        self._last_error = ""
        self._confirm_count = 0
        self._reject_count = 0

        self._detection_sub = self.create_subscription(
            String,
            self.get_parameter("apriltag_detections_topic").value,
            self._on_detection,
            10,
        )
        self._robot_pose_sub = self.create_subscription(
            String,
            self.get_parameter("robot_pose_snapshot_topic").value,
            self._on_robot_pose,
            10,
        )

        self._semantic_pub = self.create_publisher(
            String,
            self.get_parameter("semantic_confirmations_topic").value,
            10,
        )
        self._status_pub = self.create_publisher(
            DiagnosticArray,
            self.get_parameter("status_topic").value,
            10,
        )
        self._dashboard_pub = self.create_publisher(
            String,
            self.get_parameter("dashboard_text_topic").value,
            10,
        )

        status_period = 1.0 / max(0.1, float(self.get_parameter("status_hz").value))
        self._status_timer = self.create_timer(status_period, self._publish_status)

        self.get_logger().info(
            f"apriltag confirmation fallback node started with "
            f"{len(self._registrations)} registered tags"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("apriltag_enabled", True)
        self.declare_parameter("apriltag_family", APRILTAG_FAMILY_DEFAULT)

        self.declare_parameter("apriltag_detections_topic", APRILTAG_DETECTIONS)
        self.declare_parameter("semantic_confirmations_topic", SEMANTIC_CONFIRMATIONS)
        self.declare_parameter("robot_pose_snapshot_topic", "/savo_head/robot_pose_snapshot")

        self.declare_parameter("status_topic", STATUS)
        self.declare_parameter("dashboard_text_topic", DASHBOARD_TEXT)
        self.declare_parameter("status_hz", 2.0)

        self.declare_parameter("camera_optical_frame", PI_CAMERA_OPTICAL_FRAME)
        self.declare_parameter("robot_base_frame", BASE_LINK)
        self.declare_parameter("map_frame", MAP)

        self.declare_parameter("confirmation_source", SEMANTIC_CONFIRMATION_SOURCE)
        self.declare_parameter("default_confirmation_type", SEMANTIC_CONFIRMATION_TYPE_KNOWN_LOCATION)

        self.declare_parameter("allow_unknown_tags", False)
        self.declare_parameter("registered_tag_ids_csv", "")
        self.declare_parameter("tag_param_prefix", "tag_")

        self.declare_parameter("min_stable_frames", APRILTAG_MIN_STABLE_FRAMES_DEFAULT)
        self.declare_parameter("min_detection_confidence", 0.70)
        self.declare_parameter("max_detection_distance_m", APRILTAG_MAX_DETECTION_DISTANCE_M_DEFAULT)
        self.declare_parameter("max_detection_age_s", 0.50)

        self.declare_parameter("require_tf_available", True)
        self.declare_parameter("tf_timeout_s", 0.25)

        self.declare_parameter("require_robot_stationary", True)
        self.declare_parameter("max_robot_linear_speed_mps", 0.03)
        self.declare_parameter("max_robot_angular_speed_radps", 0.05)

        self.declare_parameter("require_localization_ok", True)
        self.declare_parameter("max_pose_covariance_xy", 0.25)
        self.declare_parameter("max_yaw_covariance", 0.20)

        self.declare_parameter("require_lidar_map_pose", True)
        self.declare_parameter("require_semantic_label", True)
        self.declare_parameter("save_as_summon_point_by_default", False)

        self.declare_parameter("publish_rejections", True)
        self.declare_parameter("require_robot_pose", True)

        for tag_id in self._registered_tag_ids_from_param():
            prefix = f"{self.get_parameter('tag_param_prefix').value}{tag_id}_"
            self.declare_parameter(f"{prefix}enabled", True)
            self.declare_parameter(f"{prefix}label", "")
            self.declare_parameter(f"{prefix}type", SEMANTIC_CONFIRMATION_TYPE_KNOWN_LOCATION)
            self.declare_parameter(f"{prefix}save_as_summon_point", False)
            self.declare_parameter(f"{prefix}aliases_csv", "")

    def _registered_tag_ids_from_param(self) -> tuple[int, ...]:
        raw = str(self.get_parameter("registered_tag_ids_csv").value).strip()
        if not raw:
            return ()

        out: list[int] = []
        for item in raw.split(","):
            text = item.strip()
            if not text:
                continue
            try:
                tag_id = int(text)
            except ValueError:
                self.get_logger().warn(f"ignoring invalid tag id in registered_tag_ids_csv: {text!r}")
                continue
            if tag_id >= 0 and tag_id not in out:
                out.append(tag_id)

        return tuple(out)

    def _load_registrations(self) -> dict[int, TagRegistration]:
        registrations: dict[int, TagRegistration] = {}

        for tag_id in self._registered_tag_ids_from_param():
            prefix = f"{self.get_parameter('tag_param_prefix').value}{tag_id}_"

            aliases = tuple(
                item.strip()
                for item in str(self.get_parameter(f"{prefix}aliases_csv").value).split(",")
                if item.strip()
            )

            registration = TagRegistration(
                tag_id=tag_id,
                label=str(self.get_parameter(f"{prefix}label").value).strip(),
                confirmation_type=str(self.get_parameter(f"{prefix}type").value).strip(),
                enabled=bool(self.get_parameter(f"{prefix}enabled").value),
                save_as_summon_point=bool(self.get_parameter(f"{prefix}save_as_summon_point").value),
                aliases=aliases,
            ).normalized()

            if registration.is_valid():
                registrations[tag_id] = registration
            else:
                self.get_logger().warn(
                    f"ignoring invalid tag registration {tag_id}: "
                    f"{registration.validation_errors()}"
                )

        return registrations

    def _load_policy(self) -> SemanticConfirmationPolicy:
        return SemanticConfirmationPolicy(
            min_stable_frames=int(self.get_parameter("min_stable_frames").value),
            min_detection_confidence=float(self.get_parameter("min_detection_confidence").value),
            max_detection_distance_m=float(self.get_parameter("max_detection_distance_m").value),
            max_detection_age_s=float(self.get_parameter("max_detection_age_s").value),
            require_tf_available=bool(self.get_parameter("require_tf_available").value),
            require_robot_stationary=bool(self.get_parameter("require_robot_stationary").value),
            max_robot_linear_speed_mps=float(self.get_parameter("max_robot_linear_speed_mps").value),
            max_robot_angular_speed_radps=float(self.get_parameter("max_robot_angular_speed_radps").value),
            require_localization_ok=bool(self.get_parameter("require_localization_ok").value),
            max_pose_covariance_xy=float(self.get_parameter("max_pose_covariance_xy").value),
            max_yaw_covariance=float(self.get_parameter("max_yaw_covariance").value),
            require_lidar_map_pose=bool(self.get_parameter("require_lidar_map_pose").value),
            require_semantic_label=bool(self.get_parameter("require_semantic_label").value),
        )

    def _on_robot_pose(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self._latest_robot_pose = RobotPoseSnapshot(
                x_m=float(payload.get("x_m", 0.0)),
                y_m=float(payload.get("y_m", 0.0)),
                yaw_rad=float(payload.get("yaw_rad", 0.0)),
                frame_id=str(payload.get("frame_id", self.get_parameter("map_frame").value)),
                base_frame=str(payload.get("base_frame", self.get_parameter("robot_base_frame").value)),
                stamp_s=float(payload.get("stamp_s", self._now_s())),
                linear_speed_mps=float(payload.get("linear_speed_mps", 0.0)),
                angular_speed_radps=float(payload.get("angular_speed_radps", 0.0)),
                pose_covariance_xy=float(payload.get("pose_covariance_xy", 0.0)),
                yaw_covariance=float(payload.get("yaw_covariance", 0.0)),
                localization_ok=bool(payload.get("localization_ok", True)),
                lidar_map_pose_ok=bool(payload.get("lidar_map_pose_ok", True)),
                tf_ok=bool(payload.get("tf_ok", True)),
            )
            self._last_error = ""
        except Exception as exc:
            self._last_error = f"robot_pose_parse_failed: {exc}"
            self.get_logger().warn(self._last_error)

    def _on_detection(self, msg: String) -> None:
        if not bool(self.get_parameter("apriltag_enabled").value):
            return

        now_s = self._now_s()

        try:
            observation = self._parse_observation(msg.data, now_s)
            robot_pose = self._latest_robot_pose

            reasons: list[str] = []
            registration = self._registration_for(observation.tag_id)

            if robot_pose is None:
                if bool(self.get_parameter("require_robot_pose").value):
                    reasons.append("robot_pose_missing")
                else:
                    robot_pose = RobotPoseSnapshot(stamp_s=now_s)

            if robot_pose is not None:
                reasons.extend(
                    self._policy.rejection_reasons(
                        observation=observation,
                        robot_pose=robot_pose,
                        registration=registration,
                        now_s=now_s,
                    )
                )

            if reasons:
                self._reject_count += 1
                confirmation = make_rejection(
                    observation,
                    stamp_s=now_s,
                    reasons=reasons,
                )

                if bool(self.get_parameter("publish_rejections").value):
                    self._publish_confirmation(confirmation.to_dict())

                self._last_error = ",".join(reasons)
                return

            assert robot_pose is not None
            assert registration is not None

            confirmation = make_confirmation(
                observation,
                robot_pose,
                registration,
                stamp_s=now_s,
                reason="apriltag_confirm_node_py",
            )

            self._confirm_count += 1
            self._last_error = ""
            self._publish_confirmation(confirmation.to_dict())

        except Exception as exc:
            self._reject_count += 1
            self._last_error = str(exc)
            self.get_logger().error(f"AprilTag confirmation failed: {exc}")

    def _parse_observation(self, text: str, now_s: float) -> AprilTagObservation:
        payload = json.loads(text)

        xyz = payload.get("pose_camera_xyz_m", [0.0, 0.0, 0.0])
        rpy = payload.get("pose_camera_rpy_rad", [0.0, 0.0, 0.0])

        return AprilTagObservation(
            tag_id=int(payload["tag_id"]),
            family=str(payload.get("family", self.get_parameter("apriltag_family").value)),
            confidence=float(payload.get("confidence", 0.0)),
            distance_m=float(payload.get("distance_m", 0.0)),
            stable_frames=int(payload.get("stable_frames", 0)),
            stamp_s=float(payload.get("stamp_s", now_s)),
            frame_id=str(payload.get("frame_id", self.get_parameter("camera_optical_frame").value)),
            pose_camera_xyz_m=(float(xyz[0]), float(xyz[1]), float(xyz[2])),
            pose_camera_rpy_rad=(float(rpy[0]), float(rpy[1]), float(rpy[2])),
        ).normalized()

    def _registration_for(self, tag_id: int) -> Optional[TagRegistration]:
        registration = self._registrations.get(int(tag_id))
        if registration is not None:
            return registration

        if not bool(self.get_parameter("allow_unknown_tags").value):
            return None

        return TagRegistration(
            tag_id=int(tag_id),
            label=f"tag_{int(tag_id)}",
            confirmation_type=str(self.get_parameter("default_confirmation_type").value),
            enabled=True,
            save_as_summon_point=bool(self.get_parameter("save_as_summon_point_by_default").value),
        ).normalized()

    def _publish_confirmation(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._semantic_pub.publish(msg)

    def _publish_status(self) -> None:
        status = DiagnosticStatus()
        status.name = "savo_head.apriltag_confirm_py"
        status.hardware_id = "savo_head"

        if self._last_error:
            status.level = DiagnosticStatus.WARN
            status.message = STATUS_ERROR
        elif not self._registrations and not bool(self.get_parameter("allow_unknown_tags").value):
            status.level = DiagnosticStatus.WARN
            status.message = STATUS_DRYRUN
        else:
            status.level = DiagnosticStatus.OK
            status.message = STATUS_OK

        status.values = [
            KeyValue(key="registered_tags", value=str(sorted(self._registrations))),
            KeyValue(key="confirm_count", value=str(self._confirm_count)),
            KeyValue(key="reject_count", value=str(self._reject_count)),
            KeyValue(key="has_robot_pose", value=str(self._latest_robot_pose is not None)),
            KeyValue(key="last_error", value=self._last_error),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._status_pub.publish(array)

        dashboard = String()
        dashboard.data = (
            f"savo_head apriltag={status.message} "
            f"registered={len(self._registrations)} "
            f"confirmed={self._confirm_count} rejected={self._reject_count}"
        )
        self._dashboard_pub.publish(dashboard)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    node = AprilTagConfirmNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
