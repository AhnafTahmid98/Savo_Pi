from setuptools import setup


package_name = "savo_bringup"


setup(
    name=package_name,
    version="0.3.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/savo_bringup"],
        ),
        (
            f"share/{package_name}",
            ["package.xml", "README.md"],
        ),
        (
            f"share/{package_name}/launch",
            ["launch/location_integration.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO_NAME",
    maintainer_email="todo@example.com",
    description=(
        "Production launch and lifecycle integration for Robot Savo."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            (
                "run_location_lifecycle_runtime = "
                "savo_bringup.location_lifecycle_runtime:main"
            ),
        ],
    },
)
