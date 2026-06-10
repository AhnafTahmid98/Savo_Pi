from glob import glob
from setuptools import find_packages, setup

package_name = "savo_vo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        "Robot Savo visual odometry package for RGB-D motion estimation, "
        "VO health monitoring, and odometry output from edge-side camera data."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rgbd_odometry_node = savo_vo.nodes.rgbd_odometry_node:main",
            "vo_health_node = savo_vo.nodes.vo_health_node:main",
            "vo_diagnostics_node = savo_vo.nodes.vo_diagnostics_node:main",
            "vo_republisher_node = savo_vo.nodes.vo_republisher_node:main",
        ],
    },
)