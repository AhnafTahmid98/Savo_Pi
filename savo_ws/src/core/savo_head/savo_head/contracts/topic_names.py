# -*- coding: utf-8 -*-

"""ROS topic names for Robot Savo active head."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Tuple


NAMESPACE: Final[str] = "/savo_head"

PAN_TILT_CMD: Final[str] = "/savo_head/pan_tilt_cmd"
PAN_TILT_STATE: Final[str] = "/savo_head/pan_tilt_state"

SCAN_CMD: Final[str] = "/savo_head/scan_cmd"
SCAN_STATE: Final[str] = "/savo_head/scan_state"

STATUS: Final[str] = "/savo_head/status"
DASHBOARD_TEXT: Final[str] = "/savo_head/dashboard_text"
DIAGNOSTICS: Final[str] = "/diagnostics"

CAMERA_STREAM_CMD: Final[str] = "/savo_head/camera_stream_cmd"
CAMERA_STREAM_STATE: Final[str] = "/savo_head/camera_stream_state"
CAMERA_STATUS: Final[str] = "/savo_head/camera_status"

IMAGE_RAW: Final[str] = "/savo_head/image_raw"
CAMERA_INFO: Final[str] = "/savo_head/camera_info"

APRILTAG_DETECTIONS: Final[str] = "/savo_head/apriltag_detections"
SEMANTIC_CONFIRMATIONS: Final[str] = "/savo_head/semantic_confirmations"

EMERGENCY_CENTER: Final[str] = "/savo_head/emergency_center"

MAPPING_SEMANTIC_SAVE_REQUEST: Final[str] = "/savo_mapping/semantic_save_request"
MAPPING_SEMANTIC_SAVE_RESULT: Final[str] = "/savo_mapping/semantic_save_result"

LOCATION_SAVE_REQUEST: Final[str] = "/savo_location/save_request"
LOCATION_SAVE_RESULT: Final[str] = "/savo_location/save_result"

NAV_LOCATION_CONFIRMATION: Final[str] = "/savo_nav/location_confirmation"


@dataclass(frozen=True)
class HeadTopicNames:
    namespace: str = NAMESPACE

    pan_tilt_cmd: str = PAN_TILT_CMD
    pan_tilt_state: str = PAN_TILT_STATE

    scan_cmd: str = SCAN_CMD
    scan_state: str = SCAN_STATE

    status: str = STATUS
    dashboard_text: str = DASHBOARD_TEXT
    diagnostics: str = DIAGNOSTICS

    camera_stream_cmd: str = CAMERA_STREAM_CMD
    camera_stream_state: str = CAMERA_STREAM_STATE
    camera_status: str = CAMERA_STATUS

    image_raw: str = IMAGE_RAW
    camera_info: str = CAMERA_INFO

    apriltag_detections: str = APRILTAG_DETECTIONS
    semantic_confirmations: str = SEMANTIC_CONFIRMATIONS

    emergency_center: str = EMERGENCY_CENTER

    mapping_semantic_save_request: str = MAPPING_SEMANTIC_SAVE_REQUEST
    mapping_semantic_save_result: str = MAPPING_SEMANTIC_SAVE_RESULT

    location_save_request: str = LOCATION_SAVE_REQUEST
    location_save_result: str = LOCATION_SAVE_RESULT

    nav_location_confirmation: str = NAV_LOCATION_CONFIRMATION

    def all_head_topics(self) -> Tuple[str, ...]:
        return (
            self.pan_tilt_cmd,
            self.pan_tilt_state,
            self.scan_cmd,
            self.scan_state,
            self.status,
            self.dashboard_text,
            self.camera_stream_cmd,
            self.camera_stream_state,
            self.camera_status,
            self.image_raw,
            self.camera_info,
            self.apriltag_detections,
            self.semantic_confirmations,
            self.emergency_center,
        )

    def integration_topics(self) -> Tuple[str, ...]:
        return (
            self.mapping_semantic_save_request,
            self.mapping_semantic_save_result,
            self.location_save_request,
            self.location_save_result,
            self.nav_location_confirmation,
        )

    def all_topics(self) -> Tuple[str, ...]:
        return self.all_head_topics() + self.integration_topics() + (self.diagnostics,)


TOPICS: Final[HeadTopicNames] = HeadTopicNames()


def get_topic_names() -> HeadTopicNames:
    return TOPICS


def is_savo_head_topic(topic: str) -> bool:
    return str(topic).startswith(f"{NAMESPACE}/")


__all__ = [
    "NAMESPACE",
    "PAN_TILT_CMD",
    "PAN_TILT_STATE",
    "SCAN_CMD",
    "SCAN_STATE",
    "STATUS",
    "DASHBOARD_TEXT",
    "DIAGNOSTICS",
    "CAMERA_STREAM_CMD",
    "CAMERA_STREAM_STATE",
    "CAMERA_STATUS",
    "IMAGE_RAW",
    "CAMERA_INFO",
    "APRILTAG_DETECTIONS",
    "SEMANTIC_CONFIRMATIONS",
    "EMERGENCY_CENTER",
    "MAPPING_SEMANTIC_SAVE_REQUEST",
    "MAPPING_SEMANTIC_SAVE_RESULT",
    "LOCATION_SAVE_REQUEST",
    "LOCATION_SAVE_RESULT",
    "NAV_LOCATION_CONFIRMATION",
    "HeadTopicNames",
    "TOPICS",
    "get_topic_names",
    "is_savo_head_topic",
]

# Robot pose evidence used by AprilTag semantic confirmation.
ROBOT_POSE_SNAPSHOT_TOPIC = "/savo_head/robot_pose_snapshot"

