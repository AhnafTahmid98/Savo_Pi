from setuptools import setup
from glob import glob
import os

# ============================================================================
# Robot Savo — savo_ui setup.py
# ROS 2 package setup file for the savo_ui package.
# ament_python-style ROS 2 package:
#   - Package name  : savo_ui
#   - Python module : savo_ui
#
# Nodes (console_scripts):
#   - display_manager_node  → savo_ui.display_manager_node:main
#   - ui_mode_router_node   → savo_ui.ui_mode_router_node:main
#   - ui_debug_node         → savo_ui.ui_debug_node:main
#
# After `colcon build` + `source install/setup.bash` you can run:
#   ros2 run savo_ui display_manager_node
#   ros2 run savo_ui ui_mode_router_node
#   ros2 run savo_ui ui_debug_node
#
# IMPORTANT:
#   Make sure you have this file present:
#     resource/savo_ui
#   (even empty is fine). It registers the package in the ament index.
# ============================================================================

package_name = "savo_ui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],  # Python package directory: savo_ui/
    # ------------------------------------------------------------------------
    # Data files installed into the share/ tree
    # ------------------------------------------------------------------------
    data_files=[
        # Register package with ament so `ros2 pkg` can find it
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        # Install package.xml
        (
            os.path.join("share", package_name),
            ["package.xml"],
        ),
        # Install config files (ui_params.yaml, etc.)
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        # Install launch files (savo_ui_bringup.launch.py, etc.)
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),
    ],
    # ------------------------------------------------------------------------
    # Core Python packaging metadata
    # ------------------------------------------------------------------------
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        'Robot Savo UI stack: drives the 7" DFRobot DSI display with '
        "INTERACT (face), NAVIGATE (camera + overlay), and MAP modes."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    # ------------------------------------------------------------------------
    # Entry points: ROS 2 nodes exposed as console_scripts
    # ------------------------------------------------------------------------
    entry_points={
        "console_scripts": [
            # Full-screen display manager (Pygame)
            "display_manager_node = savo_ui.display_manager_node:main",
            # Maps LLM intents + mapping state → UI mode + status text
            "ui_mode_router_node = savo_ui.ui_mode_router_node:main",
            # Simple debug driver (cycles modes, optional fake mouth_level)
            "ui_debug_node = savo_ui.ui_debug_node:main",
        ],
    },
)
