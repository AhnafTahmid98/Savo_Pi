from setuptools import setup


package_name = "savo_bringup"


setup(
    name=package_name,
    version="0.5.0",
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
            [
                "launch/autonomous_mapping.launch.py",
                "launch/location_integration.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        "Production launch and lifecycle integration for Robot Savo."
    ),
    license="Proprietary",
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
