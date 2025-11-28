from setuptools import setup
from glob import glob
import os

# ============================================================================
# Robot Savo — savo_ui setup.py
#
# This is an ament_python-style ROS 2 package:
#   - Package name:  savo_ui
#   - Python module: savo_ui
#   - Nodes (console_scripts):
#       * display_manager_node
#       * ui_mode_router_node
#       * ui_debug_node
#
# `ros2 run savo_ui <name>` will use the console_scripts defined below.
# ============================================================================

package_name = "savo_ui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
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
        # Install launch files (savo_ui_bringup.launch.py)
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
        "Robot Savo UI stack: drives the 7\" DFRobot DSI display with "
        "INTERACT (face), NAVIGATE (camera + overlay), and MAP modes."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    # ------------------------------------------------------------------------
    # Entry points: ROS 2 nodes exposed as console_scripts
    #
    # After `colcon build` + `source install/setup.bash` you can run:
    #   ros2 run savo_ui display_manager_node
    #   ros2 run savo_ui ui_mode_router_node
    #   ros2 run savo_ui ui_debug_node
    # ------------------------------------------------------------------------
    entry_points={
        "console_scripts": [
            "display_manager_node = savo_ui.display_manager_node:main",
            "ui_mode_router_node = savo_ui.ui_mode_router_node:main",
            "ui_debug_node = savo_ui.ui_debug_node:main",
        ],
    },
)
