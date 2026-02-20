#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — savo_localization/utils/frames.py
----------------------------------------------
Professional frame-id utilities for localization stack.

Why this file exists:
- Frame-id mismatches are a top cause of "Nav2 doesn't move" and broken TF trees.
- Keeping frame names centralized avoids silent inconsistencies across nodes and configs.

Conventions (locked for Robot Savo localization):
- map frame:        "map"           (global frame from AMCL/SLAM)
- odom frame:       "odom"          (local continuous frame)
- base frame:       "base_link"     (robot body frame used by Nav2 + EKF)
- base footprint:   "base_footprint" (optional planar base; EKF may publish odom->base_link)
- imu frame:        "imu_link"      (IMU sensor frame)
- wheel frame:      "wheel_odom"    (optional child frame if ever needed; not required)

Notes:
- Use base_link for localization and Nav2 unless you explicitly decide to use base_footprint.
- For planar EKF, base_footprint is sometimes used; we keep both constants available.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional


# ---------------------------
# Canonical frame IDs
# ---------------------------

MAP: str = "map"
ODOM: str = "odom"
BASE_LINK: str = "base_link"
BASE_FOOTPRINT: str = "base_footprint"

IMU_LINK: str = "imu_link"

# Optional / advanced:
WHEEL_ODOM_CHILD: str = "wheel_odom"


@dataclass(frozen=True)
class FrameIds:
    """
    Container for frame IDs used by a node.

    Typical usage:
        frames = FrameIds()
        msg.header.frame_id = frames.odom
        msg.child_frame_id = frames.base

    You can override if needed:
        frames = FrameIds(odom="odom", base="base_footprint")
    """
    map: str = MAP
    odom: str = ODOM
    base: str = BASE_LINK
    base_footprint: str = BASE_FOOTPRINT
    imu: str = IMU_LINK


# ---------------------------
# Validation helpers
# ---------------------------

# ROS frame id rules (practical subset):
# - non-empty
# - no spaces
# - avoid leading '/'
# - allow [A-Za-z0-9_]
_FRAME_RE = re.compile(r"^[A-Za-z0-9_]+$")


def normalize_frame_id(frame_id: str) -> str:
    """
    Normalize a frame id:
    - strip whitespace
    - remove a leading '/' (TF2 typically uses frame ids without leading slash)

    Returns normalized string (may become empty if input was invalid).
    """
    if frame_id is None:
        return ""
    fid = str(frame_id).strip()
    if fid.startswith("/"):
        fid = fid[1:]
    return fid


def is_valid_frame_id(frame_id: str) -> bool:
    """
    Validate a frame id for TF usage.

    Returns False if:
    - empty
    - contains spaces
    - contains characters outside [A-Za-z0-9_]
    """
    fid = normalize_frame_id(frame_id)
    if not fid:
        return False
    if " " in fid or "\t" in fid or "\n" in fid:
        return False
    return _FRAME_RE.match(fid) is not None


def require_valid_frame_id(frame_id: str, name: str = "frame_id") -> str:
    """
    Validate and return normalized frame_id, raising ValueError if invalid.
    """
    fid = normalize_frame_id(frame_id)
    if not is_valid_frame_id(fid):
        raise ValueError(f"Invalid {name}: '{frame_id}' (normalized: '{fid}')")
    return fid


def validate_frames(frames: FrameIds) -> FrameIds:
    """
    Validate all fields in a FrameIds object and return a normalized copy.
    """
    return FrameIds(
        map=require_valid_frame_id(frames.map, "map frame"),
        odom=require_valid_frame_id(frames.odom, "odom frame"),
        base=require_valid_frame_id(frames.base, "base frame"),
        base_footprint=require_valid_frame_id(frames.base_footprint, "base_footprint frame"),
        imu=require_valid_frame_id(frames.imu, "imu frame"),
    )


# ---------------------------
# Convenience setters
# ---------------------------

def set_header_frame(msg: object, frame_id: str) -> None:
    """
    Set msg.header.frame_id safely (normalized + validated).
    """
    fid = require_valid_frame_id(frame_id, "header.frame_id")
    header = getattr(msg, "header", None)
    if header is None:
        raise AttributeError("Message has no 'header' attribute")
    setattr(header, "frame_id", fid)


def set_child_frame(odom_msg: object, child_frame_id: str) -> None:
    """
    Set nav_msgs/Odometry.child_frame_id safely (normalized + validated).
    """
    fid = require_valid_frame_id(child_frame_id, "child_frame_id")
    if not hasattr(odom_msg, "child_frame_id"):
        raise AttributeError("Object has no 'child_frame_id' attribute")
    setattr(odom_msg, "child_frame_id", fid)