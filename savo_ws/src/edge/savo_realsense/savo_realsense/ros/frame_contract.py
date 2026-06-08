from dataclasses import dataclass

from savo_realsense.constants import (
    DEFAULT_BASE_FRAME,
    DEFAULT_CAMERA_LINK_FRAME,
    DEFAULT_COLOR_OPTICAL_FRAME,
    DEFAULT_DEPTH_OPTICAL_FRAME,
)


@dataclass(frozen=True)
class RealSenseFrames:
    base_frame: str = DEFAULT_BASE_FRAME
    camera_link: str = DEFAULT_CAMERA_LINK_FRAME
    color_optical_frame: str = DEFAULT_COLOR_OPTICAL_FRAME
    depth_optical_frame: str = DEFAULT_DEPTH_OPTICAL_FRAME


DEFAULT_FRAMES = RealSenseFrames()


CAMERA_OPTICAL_FRAMES = (
    DEFAULT_FRAMES.color_optical_frame,
    DEFAULT_FRAMES.depth_optical_frame,
)


REQUIRED_TF_FRAMES = (
    DEFAULT_FRAMES.base_frame,
    DEFAULT_FRAMES.camera_link,
    DEFAULT_FRAMES.color_optical_frame,
    DEFAULT_FRAMES.depth_optical_frame,
)


def all_realsense_frames(frames: RealSenseFrames = DEFAULT_FRAMES) -> tuple[str, ...]:
    return (
        frames.base_frame,
        frames.camera_link,
        frames.color_optical_frame,
        frames.depth_optical_frame,
    )


def camera_optical_frames(frames: RealSenseFrames = DEFAULT_FRAMES) -> tuple[str, ...]:
    return (
        frames.color_optical_frame,
        frames.depth_optical_frame,
    )