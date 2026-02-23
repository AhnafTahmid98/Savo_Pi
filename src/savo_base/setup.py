#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — setup.py (ROS 2 Jazzy, hybrid package)
---------------------------------------------------
Purpose:
- Package the Python modules under `savo_base/`
- Expose ROS 2 Python node entry points (from `savo_base/nodes/`)
- Work together with:
    * CMakeLists.txt  -> installs scripts/, launch/, config/, resources
    * setup.cfg       -> places console_scripts into lib/savo_base

Layout note:
- CLI tools are kept in `scripts/` and installed by CMakeLists.txt
  via install(PROGRAMS ...).
- ROS nodes live in `savo_base/nodes/` and are exposed here via console_scripts.

Package role:
- `savo_base` is Robot Savo's low-level drivetrain hardware execution package
  (motor board driver, mecanum kinematics, watchdog/safety, base state/diagnostics).
"""

from setuptools import find_packages, setup

package_name = "savo_base"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=("test", "test.*")),
    include_package_data=True,
    data_files=[
        # ament index resource (required for ROS 2 package discovery)
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # package manifest
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=False,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        "Robot SAVO low-level drivetrain hardware execution package "
        "(PCA9685/Freenove motor board, mecanum kinematics, watchdog safety, "
        "base state/diagnostics) for ROS 2 Jazzy."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # ROS 2 nodes (savo_base/nodes/)
            "base_driver_node = savo_base.nodes.base_driver_node:main",
            "base_watchdog_node = savo_base.nodes.base_watchdog_node:main",
            "base_state_publisher_node = savo_base.nodes.base_state_publisher_node:main",
            "base_diag_runner_node = savo_base.nodes.base_diag_runner_node:main",
            "base_heartbeat_node = savo_base.nodes.base_heartbeat_node:main",
        ],
    },
)