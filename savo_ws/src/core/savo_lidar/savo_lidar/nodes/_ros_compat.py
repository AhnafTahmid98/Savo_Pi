#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Import-time ROS compatibility helpers for node modules."""

from __future__ import annotations


try:
    import rclpy
    from rclpy.node import Node
except ModuleNotFoundError:
    class _MissingRclpy:
        def init(self, *args, **kwargs):
            raise ModuleNotFoundError("rclpy is required to run LiDAR nodes")

        def spin(self, *args, **kwargs):
            raise ModuleNotFoundError("rclpy is required to run LiDAR nodes")

        def shutdown(self, *args, **kwargs):
            raise ModuleNotFoundError("rclpy is required to run LiDAR nodes")

    class Node:
        def __init__(self, *args, **kwargs):
            raise ModuleNotFoundError("rclpy is required to instantiate LiDAR nodes")

    rclpy = _MissingRclpy()


try:
    from sensor_msgs.msg import LaserScan
except ModuleNotFoundError:
    class LaserScan:
        pass


try:
    from std_msgs.msg import String
except ModuleNotFoundError:
    class String:
        pass


__all__ = [
    "LaserScan",
    "Node",
    "String",
    "rclpy",
]
