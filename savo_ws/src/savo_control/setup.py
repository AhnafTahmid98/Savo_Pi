# Python ROS executables are installed by CMakeLists.txt install(PROGRAMS), not via
# console_scripts, because entry_points has been unreliable for ros2 run in this repo.
# Keep package_name aligned with package.xml, CMakeLists project(), and the folder name.

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