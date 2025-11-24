from setuptools import setup

package_name = "savo_intent"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        # So ROS 2 can discover this package
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Install package.xml so tools like ros2 pkg can read metadata
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description="Robot Savo — ROS2 bridge between the robot and the LLM server.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # ros2 run savo_intent intent_client_node
            "intent_client_node = savo_intent.intent_client_node:main",
        ],
    },
)
