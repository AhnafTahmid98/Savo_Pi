"""Convert ROS image messages into arrays for VO processing."""

import numpy as np
from cv_bridge import CvBridge


_bridge = CvBridge()


def image_msg_to_array(msg: object, desired_encoding: str = "passthrough") -> np.ndarray:
    return _bridge.imgmsg_to_cv2(msg, desired_encoding=desired_encoding)


def color_msg_to_bgr_array(msg: object) -> np.ndarray:
    return image_msg_to_array(msg, desired_encoding="bgr8")


def color_msg_to_gray_array(msg: object) -> np.ndarray:
    return image_msg_to_array(msg, desired_encoding="mono8")


def depth_msg_to_array(msg: object) -> np.ndarray:
    depth = image_msg_to_array(msg, desired_encoding="passthrough")
    return np.asarray(depth)


def depth_to_meters(depth: np.ndarray, depth_scale: float = 0.001) -> np.ndarray:
    if depth_scale <= 0.0:
        raise ValueError("depth_scale must be positive")

    if depth.dtype == np.uint16:
        return depth.astype(np.float32) * depth_scale

    return depth.astype(np.float32)