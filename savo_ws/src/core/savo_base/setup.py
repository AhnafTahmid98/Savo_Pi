#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Python ROS node executables are installed by CMakeLists.txt install(PROGRAMS), not entry_points,
# because entry_points has been unreliable for ros2 run in this hybrid ament_cmake layout.

from setuptools import find_packages, setup

package_name = "savo_base"

setup(
    name=package_name,
    version="0.1.0",
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
        "console_scripts": [],
    },
)
