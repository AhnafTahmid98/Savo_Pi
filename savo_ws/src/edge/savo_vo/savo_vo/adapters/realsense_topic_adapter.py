"""Topic bundle helpers for RealSense RGB-D input."""

from dataclasses import dataclass

from savo_vo.contracts.topic_names import (
    COLOR_CAMERA_INFO_TOPIC,
    COLOR_IMAGE_TOPIC,
    DEPTH_CAMERA_INFO_TOPIC,
    DEPTH_IMAGE_TOPIC,
)


@dataclass(frozen=True)
class RealSenseTopicSet:
    color_image_topic: str = COLOR_IMAGE_TOPIC
    color_camera_info_topic: str = COLOR_CAMERA_INFO_TOPIC
    depth_image_topic: str = DEPTH_IMAGE_TOPIC
    depth_camera_info_topic: str = DEPTH_CAMERA_INFO_TOPIC

    def as_tuple(self) -> tuple[str, str, str, str]:
        return (
            self.color_image_topic,
            self.color_camera_info_topic,
            self.depth_image_topic,
            self.depth_camera_info_topic,
        )

    def as_dict(self) -> dict[str, str]:
        return {
            "color_image_topic": self.color_image_topic,
            "color_camera_info_topic": self.color_camera_info_topic,
            "depth_image_topic": self.depth_image_topic,
            "depth_camera_info_topic": self.depth_camera_info_topic,
        }


def build_realsense_topic_set(
    color_image_topic: str = COLOR_IMAGE_TOPIC,
    color_camera_info_topic: str = COLOR_CAMERA_INFO_TOPIC,
    depth_image_topic: str = DEPTH_IMAGE_TOPIC,
    depth_camera_info_topic: str = DEPTH_CAMERA_INFO_TOPIC,
) -> RealSenseTopicSet:
    return RealSenseTopicSet(
        color_image_topic=color_image_topic,
        color_camera_info_topic=color_camera_info_topic,
        depth_image_topic=depth_image_topic,
        depth_camera_info_topic=depth_camera_info_topic,
    )