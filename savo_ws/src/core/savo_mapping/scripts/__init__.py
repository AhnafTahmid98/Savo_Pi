#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Command-line helper scripts for Robot Savo mapping.

These scripts are mainly used for:
- local PC validation
- real robot operator checks
- map file/metadata helpers
- diagnostics before ROS launch testing

Most scripts can run directly from the source tree during development.
After colcon install, selected scripts are exposed through setup.py entry points.
"""

from __future__ import annotations


__all__: list[str] = []
