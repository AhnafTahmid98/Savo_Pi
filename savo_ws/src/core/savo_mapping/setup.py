from setuptools import find_packages, setup

package_name = "savo_mapping"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        "Robot Savo mapping package with manual mapping, autonomous mapping, "
        "SLAM toolbox integration, map tools, diagnostics, and future semantic "
        "landmark support."
    ),
    license="Proprietary",
    tests_require=["pytest"],
)