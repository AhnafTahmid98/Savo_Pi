from setuptools import find_packages, setup

package_name = "savo_power"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        "Robot Savo shared power monitoring package for core UPS HAT, "
        "edge UPS HAT, and Freenove/base battery monitoring. "
        "C++ nodes are production default; Python nodes are fallback and diagnostics."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Python fallback ROS nodes.
            "core_ups_node_py = savo_power.nodes.ups_hat_node_py:main_core",
            "edge_ups_node_py = savo_power.nodes.ups_hat_node_py:main_edge",
            "base_battery_node_py = savo_power.nodes.kit_battery_node_py:main",
            "power_aggregator_node_py = savo_power.nodes.power_aggregator_node_py:main",
            "power_health_node_py = savo_power.nodes.power_health_node_py:main",
            "power_dashboard_node_py = savo_power.nodes.power_dashboard_node_py:main",

            # Hardware diagnostics / CLI checks.
            "power_i2c_check = savo_power.diagnostics.i2c_power_check:main",
            "ups_check = savo_power.diagnostics.ups_check:main",
            "kit_battery_check = savo_power.diagnostics.kit_battery_check:main",
        ],
    },
)
