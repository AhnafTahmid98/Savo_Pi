from setuptools import find_packages, setup

package_name = "savo_realsense"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", [
            "config/realsense_d435.yaml",
            "config/realsense_minimal.yaml",
            "config/realsense_vo_profile.yaml",
            "config/realsense_nav_profile.yaml",
            "config/camera_frames.yaml",
            "config/qos.yaml",
        ]),
        (f"share/{package_name}/launch", [
            "launch/realsense_bringup.launch.py",
            "launch/realsense_minimal.launch.py",
            "launch/realsense_vo.launch.py",
            "launch/realsense_diagnostics.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description="RealSense camera bringup, stream monitoring, and diagnostics for Robot Savo edge perception.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_health_node = savo_realsense.nodes.camera_health_node:main",
            "camera_topic_monitor_node = savo_realsense.nodes.camera_topic_monitor_node:main",
            "depth_front_min_node = savo_realsense.nodes.depth_front_min_node:main",
            "realsense_smoke_test_cli = savo_realsense.tools.realsense_smoke_test_cli:main",
            "dump_effective_realsense_params = savo_realsense.tools.dump_effective_realsense_params:main",
        ],
    },
)