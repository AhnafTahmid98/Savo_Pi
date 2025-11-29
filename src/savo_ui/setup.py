from setuptools import setup, find_packages
from glob import glob
import os

# ============================================================================
# Robot Savo — savo_ui setup.py
#
# ament_python-style ROS 2 package:
#   - Package name  : savo_ui
#   - Python module : savo_ui
#
# Nodes (console_scripts):
#   - display_manager_node  → savo_ui.display_manager_node:main
#   - ui_mode_router_node   → savo_ui.ui_mode_router_node:main   (planned / optional)
#   - ui_debug_node         → savo_ui.ui_debug_node:main         (planned / optional)
#   - fake_cam_node         → savo_ui.fake_cam_node:main         (test camera source)
#
# After `colcon build` + `source install/setup.bash` you can run for example:
#   ros2 run savo_ui display_manager_node
#   ros2 run savo_ui fake_cam_node
#
# IMPORTANT:
#   Make sure this file exists:
#     resource/savo_ui
#   (even empty is fine). It registers the package in the ament index.
# ============================================================================

package_name = "savo_ui"

setup(
    name=package_name,
    version="0.1.0",
    # Discover Python packages (should find the `savo_ui` directory)
    packages=find_packages(),
    # ----------------------------------------------------------------------
    # Data files installed into the share/ tree
    # ----------------------------------------------------------------------
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
        # Install config files (ui_params.yaml, etc.) if present
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        # Install launch files (savo_ui_bringup.launch.py, etc.) if present
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),
    ],
    # ----------------------------------------------------------------------
    # Core Python packaging metadata
    # ----------------------------------------------------------------------
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
    # ----------------------------------------------------------------------
    # Entry points: ROS 2 nodes exposed as console_scripts
    # ----------------------------------------------------------------------
    entry_points={
        "console_scripts": [
            # Full-screen display manager (Pygame, face + camera views)
            "display_manager_node = savo_ui.display_manager_node:main",

            # UI mode router (optional, future):
            # maps intents / Nav2 state → /savo_ui/mode + /savo_ui/status_text
            "ui_mode_router_node = savo_ui.ui_mode_router_node:main",

            # Simple debug driver (optional, future):
            # can cycle modes, set fake mouth_level, etc.
            "ui_debug_node = savo_ui.ui_debug_node:main",

            # Fake camera publisher for NAVIGATE testing (gradient video on
            # /camera/image_rect so the UI can be tested without a real camera).
            "fake_cam_node = savo_ui.fake_cam_node:main",
        ],
    },
)
