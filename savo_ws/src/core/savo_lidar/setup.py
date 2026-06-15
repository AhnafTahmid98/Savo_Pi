#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Python ROS node executables are installed by CMakeLists.txt install(PROGRAMS),
# not entry_points, because this package uses a hybrid ament_cmake layout.

from setuptools import find_packages, setup

package_name = "savo_lidar"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=("test", "test.*")),
    include_package_data=True,
    data_files=[
        # ament index resource required for ROS 2 package discovery
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
        "Robot SAVO LiDAR hardware ownership package for RPLIDAR A1 bringup, "
        "LaserScan publishing, scan filtering, health monitoring, watchdogs, "
        "diagnostics, and mapping/navigation-ready scan output for ROS 2 Jazzy."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Optional in this hybrid package.
            # CMake installs .py node executables directly for ros2 run.
            "lidar_driver_node = savo_lidar.nodes.lidar_py_driver_node:main",
            "lidar_health_node = savo_lidar.nodes.lidar_health_node:main",
            "lidar_filter_node = savo_lidar.nodes.lidar_filter_node:main",
            "lidar_watchdog_node = savo_lidar.nodes.lidar_watchdog_node:main",
            "lidar_state_publisher_node = savo_lidar.nodes.lidar_state_publisher_node:main",
        ],
    },
)
