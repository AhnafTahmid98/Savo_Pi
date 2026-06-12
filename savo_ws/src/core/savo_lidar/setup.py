#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from setuptools import find_packages, setup

package_name = "savo_lidar"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=("test", "test.*")),
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=False,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        "Robot Savo LiDAR hardware ownership package for RPLIDAR A1 bringup, "
        "scan publishing, filtering, health monitoring, and diagnostics."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lidar_driver_node = savo_lidar.nodes.lidar_driver_node:main",
            "lidar_health_node = savo_lidar.nodes.lidar_health_node:main",
            "lidar_filter_node = savo_lidar.nodes.lidar_filter_node:main",
            "lidar_watchdog_node = savo_lidar.nodes.lidar_watchdog_node:main",
            "lidar_state_publisher_node = savo_lidar.nodes.lidar_state_publisher_node:main",
        ],
    },
)