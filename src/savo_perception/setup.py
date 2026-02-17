from setuptools import find_packages, setup
from glob import glob
import os

package_name = "savo_perception"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=("test",)),
    data_files=[
        # ament index + package manifest
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),

        # install launch + config
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description="Robot Savo perception and safety package (hybrid Python + C++).",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Core perception + safety nodes
            "vl53_node = savo_perception.nodes.vl53_node:main",
            "ultrasonic_node = savo_perception.nodes.ultrasonic_node:main",
            "depth_front_min_node = savo_perception.nodes.depth_front_min_node:main",
            "safety_stop_node = savo_perception.nodes.safety_stop_node:main",
            "sensor_dashboard = savo_perception.nodes.sensor_dashboard_node:main",

        ],
    },
)
