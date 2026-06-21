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
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
    ],
    python_requires=">=3.10",
    zip_safe=False,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        "Robot Savo control layer with C++ runtime nodes and Python fallback "
        "diagnostics."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
