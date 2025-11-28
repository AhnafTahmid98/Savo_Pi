"""
Robot Savo — UI package

This package contains the screen user interface for Robot Savo.
It is responsible for driving the 7" DFRobot DSI display on the Pi and
rendering three main modes:

- INTERACT mode:
    Friendly face with eyes and a mouth. The mouth animation is driven by
    /savo_speech/mouth_level and subtitles come from /savo_speech/tts_text.

- NAVIGATE mode:
    Live forward camera view with an overlay such as "Guiding to: <location>".
    This mode is typically active while the robot is escorting a user.

- MAP mode:
    Mapping / exploration status view (e.g. "Mapping in progress, please keep
    distance."), optionally with a smaller face overlay.

Key nodes (entry points) in this package:
- display_manager_node.py
- ui_mode_router_node.py
- ui_debug_node.py

Helper modules (imported by the nodes, not run directly):
- face_view.py
- nav_cam_view.py

This __init__ keeps imports minimal on purpose to avoid pulling in heavy
dependencies (like pygame or rclpy) just by importing the package.
"""

from __future__ import annotations

from importlib.metadata import PackageNotFoundError, version

__all__ = ["__version__"]


def _detect_version() -> str:
    """
    Detect the installed package version.

    Returns a string version (e.g. '0.1.0'). If the package metadata is not
    available (for example when running directly from a source tree without
    installation), a sensible fallback is returned.
    """
    try:
        # The distribution name must match the one defined in setup.cfg
        return version("savo_ui")
    except PackageNotFoundError:
        # Fallback for devel runs where the package is not installed as a wheel
        return "0.0.0.dev0"


__version__: str = _detect_version()
