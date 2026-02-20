# setup.py
# Robot SAVO — savo_localization (ROS 2 Jazzy)
#
# Hybrid package:
# - C++: wheel_odom_node (built/installed by CMake to lib/savo_localization)
# - Python: imu_node + wheel_odom_fallback_node + sensors_api + utils installed by setuptools
#
# Notes:
# - RViz2 runs on PC/Mac; no rviz assets required here.
# - Keep encoders_api.py for fallback/diagnostics; production wheel odom is C++.
# - This setup.py is kept robust: it installs launch/config folders if present,
#   and registers console_scripts so `ros2 run` can see Python nodes.

from setuptools import find_packages, setup
from glob import glob
import os

package_name = "savo_localization"


def _maybe_glob(pattern: str):
    """Return glob(pattern) if any files exist; otherwise return empty list."""
    return glob(pattern) or []


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament index resource marker + package manifest
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        ("share/" + package_name, ["package.xml"]),

        # launch + config
        (os.path.join("share", package_name, "launch"), _maybe_glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), _maybe_glob("config/*.yaml") + _maybe_glob("config/*.yml")),

        # optional: rviz folder (safe if empty)
        (os.path.join("share", package_name, "rviz"), _maybe_glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        "Robot SAVO localization: C++ wheel odom (rear encoders) + Python IMU publisher + EKF config/bringup."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Python nodes (must exist under savo_localization/nodes/)
            "imu_node = savo_localization.nodes.imu_node:main",
            "wheel_odom_fallback_node = savo_localization.nodes.wheel_odom_fallback_node:main",
        ],
    },
)