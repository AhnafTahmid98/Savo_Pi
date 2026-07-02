# -*- coding: utf-8 -*-

"""TF frame and joint names for Robot Savo active head."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Tuple


BASE_LINK: Final[str] = "base_link"

PANTILT_PAN_LINK: Final[str] = "pantilt_pan_link"
PANTILT_TILT_LINK: Final[str] = "pantilt_tilt_link"

PI_CAMERA_LINK: Final[str] = "pi_camera_link"
PI_CAMERA_OPTICAL_FRAME: Final[str] = "pi_camera_optical_frame"

HEAD_PAN_JOINT: Final[str] = "head_pan_joint"
HEAD_TILT_JOINT: Final[str] = "head_tilt_joint"

MAP: Final[str] = "map"
ODOM: Final[str] = "odom"


@dataclass(frozen=True)
class HeadFrameNames:
    base_link: str = BASE_LINK

    pantilt_pan_link: str = PANTILT_PAN_LINK
    pantilt_tilt_link: str = PANTILT_TILT_LINK

    pi_camera_link: str = PI_CAMERA_LINK
    pi_camera_optical_frame: str = PI_CAMERA_OPTICAL_FRAME

    head_pan_joint: str = HEAD_PAN_JOINT
    head_tilt_joint: str = HEAD_TILT_JOINT

    map_frame: str = MAP
    odom_frame: str = ODOM

    def tf_chain(self) -> Tuple[str, ...]:
        return (
            self.base_link,
            self.pantilt_pan_link,
            self.pantilt_tilt_link,
            self.pi_camera_link,
            self.pi_camera_optical_frame,
        )

    def moving_head_frames(self) -> Tuple[str, ...]:
        return (
            self.pantilt_pan_link,
            self.pantilt_tilt_link,
            self.pi_camera_link,
            self.pi_camera_optical_frame,
        )

    def joint_names(self) -> Tuple[str, str]:
        return (
            self.head_pan_joint,
            self.head_tilt_joint,
        )

    def localization_frames(self) -> Tuple[str, str, str]:
        return (
            self.map_frame,
            self.odom_frame,
            self.base_link,
        )


FRAMES: Final[HeadFrameNames] = HeadFrameNames()


def get_frame_names() -> HeadFrameNames:
    return FRAMES


def is_head_frame(frame: str) -> bool:
    return str(frame) in FRAMES.moving_head_frames()


def is_robot_frame(frame: str) -> bool:
    return str(frame) in FRAMES.tf_chain() or str(frame) in {
        FRAMES.map_frame,
        FRAMES.odom_frame,
    }


__all__ = [
    "BASE_LINK",
    "PANTILT_PAN_LINK",
    "PANTILT_TILT_LINK",
    "PI_CAMERA_LINK",
    "PI_CAMERA_OPTICAL_FRAME",
    "HEAD_PAN_JOINT",
    "HEAD_TILT_JOINT",
    "MAP",
    "ODOM",
    "HeadFrameNames",
    "FRAMES",
    "get_frame_names",
    "is_head_frame",
    "is_robot_frame",
]
