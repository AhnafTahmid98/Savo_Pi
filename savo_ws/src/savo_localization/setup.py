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
# - In this hybrid package, CMake installs launch/config/resources, while setup.py
#   installs Python modules and console_scripts.

from setuptools import find_packages, setup

package_name = "savo_localization"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        "Robot SAVO localization package: IMU publisher, wheel odometry fallback, "
        "localization dashboard, and hybrid C++/Python localization tools for ROS 2 Jazzy."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # IMU publisher (BNO055)
            "imu_node = savo_localization.nodes.imu_node:main",

            # Python fallback wheel odom (diagnostics / backup)
            "wheel_odom_fallback_node = savo_localization.nodes.wheel_odom_fallback_node:main",

            # Terminal localization dashboard (IMU / encoders / odom summary)
            "localization_dashboard = savo_localization.nodes.localization_dashboard:main",
        ],
    },
)