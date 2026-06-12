#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Frame-id helpers for localization nodes."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional


MAP: str = "map"
ODOM: str = "odom"
BASE_LINK: str = "base_link"
BASE_FOOTPRINT: str = "base_footprint"

IMU_LINK: str = "imu_link"

WHEEL_ODOM_CHILD: str = "wheel_odom"


@dataclass(frozen=True)
class FrameIds:
    """Frame IDs used by a localization node."""
    map: str = MAP
    odom: str = ODOM
    base: str = BASE_LINK
    base_footprint: str = BASE_FOOTPRINT
    imu: str = IMU_LINK


# Practical ROS frame-id subset: no slash prefix, whitespace, or punctuation.
_FRAME_RE = re.compile(r"^[A-Za-z0-9_]+$")


def normalize_frame_id(frame_id: str) -> str:
    """Normalize whitespace and a leading slash from a frame id."""
    if frame_id is None:
        return ""
    fid = str(frame_id).strip()
    if fid.startswith("/"):
        fid = fid[1:]
    return fid


def is_valid_frame_id(frame_id: str) -> bool:
    """Return True when a frame id fits the project TF naming rules."""
    fid = normalize_frame_id(frame_id)
    if not fid:
        return False
    if " " in fid or "\t" in fid or "\n" in fid:
        return False
    return _FRAME_RE.match(fid) is not None


def require_valid_frame_id(frame_id: str, name: str = "frame_id") -> str:
    """Validate and return a normalized frame id."""
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
