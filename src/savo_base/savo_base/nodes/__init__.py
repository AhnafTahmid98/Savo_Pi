#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/nodes/__init__.py
----------------------------------------
Package marker for ROS2 Python nodes in `savo_base`.

This file intentionally stays lightweight.

Why keep it:
- Makes `savo_base.nodes` an importable Python package
- Supports clean imports in tests/tools if needed
- Keeps node modules organized under a professional package namespace
"""

# NOTE:
# We intentionally do NOT import node classes here by default, to avoid
# side effects during package import (e.g., ROS initialization assumptions,
# heavy imports, or hardware-related imports in some node modules).

__all__ = []