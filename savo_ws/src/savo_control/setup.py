# =============================================================================
# Robot SAVO — savo_control / setup.py (ROS 2 Jazzy, hybrid C++ + Python)
# =============================================================================
# Purpose
# -------
# Installs Python modules/packages for the `savo_control` ROS 2 package:
#   - savo_control.controllers
#   - savo_control.interfaces
#   - savo_control.nodes
#   - savo_control.utils
#
# Hybrid package note (important)
# -------------------------------
# In this project, Python ROS executable nodes are installed explicitly by
# CMakeLists.txt using:
#
#   install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME})
#
# This is intentional and follows the proven Robot SAVO hybrid-package pattern
# (same reliability approach used in `savo_localization`) to avoid cases where
# `ros2 run` cannot find Python executables in install/lib/<pkg>.
#
# Therefore:
#   - setup.py is used for Python package/module installation and metadata
#   - CMake handles:
#       * C++ executables
#       * Python ROS executable scripts
#       * launch/config/docs installation
#
# Consistency requirements
# ------------------------
# Keep these names exactly aligned:
#   package.xml   -> <name>savo_control</name>
#   CMakeLists    -> project(savo_control)
#   CMakeLists    -> set(PYTHON_PACKAGE_NAME "savo_control")
#   setup.py      -> package_name = "savo_control"
#   folder        -> savo_control/
# =============================================================================

from setuptools import find_packages, setup

package_name = "savo_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(
        include=[
            "savo_control",
            "savo_control.*",
        ]
    ),
    data_files=[
        # Required by ament index
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # Package manifest
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
    ],
    python_requires=">=3.10",
    zip_safe=True,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        "Robot Savo control layer (hybrid C++ + Python) for command multiplexing, "
        "velocity shaping, control mode arbitration, PID-based motion control, "
        "autonomous control tests, and local recovery primitives "
        "(stuck detection / backup escape) before full Nav2 integration."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    # NOTE:
    # Python ROS nodes are installed explicitly in CMakeLists.txt with install(PROGRAMS ...),
    # so we intentionally keep console_scripts empty here to avoid duplicate/conflicting
    # executable installation paths in this hybrid package pattern.
    entry_points={
        "console_scripts": [],
    },
)