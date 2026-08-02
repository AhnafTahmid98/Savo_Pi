#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from setuptools import find_packages, setup

package_name = "savo_localization"

setup(
    name=package_name,
    version="0.1.0",
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
        "Robot Savo localization ownership package for C++ IMU publishing, "
        "four-wheel encoder odometry, EKF bringup, localization health "
        "monitoring, dashboards, and diagnostic tools."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
