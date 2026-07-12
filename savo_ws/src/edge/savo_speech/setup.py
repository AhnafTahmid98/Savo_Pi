#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from setuptools import find_packages, setup


package_name = "savo_speech"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(
        include=[
            "savo_speech",
            "savo_speech.*",
        ]
    ),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (
            f"share/{package_name}",
            ["package.xml"],
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    python_requires=">=3.10",
    zip_safe=False,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        "Robot Savo edge-side speech package with C++ production audio runtime "
        "and Python AI speech services, diagnostics, and SavoMind integration."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
