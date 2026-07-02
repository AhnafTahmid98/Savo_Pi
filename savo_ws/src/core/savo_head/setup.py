from setuptools import find_packages
from setuptools import setup

package_name = "savo_head"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(
        include=(package_name, f"{package_name}.*"),
        exclude=("test", "test.*", "tests", "tests.*"),
    ),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        "Robot Savo active head package for Pi Camera 2 NoIR pan-tilt control, "
        "GStreamer/libcamerasrc camera diagnostics, dynamic head TF, AprilTag "
        "semantic confirmation, and human-facing scan behavior."
    ),
    license="Proprietary",
    tests_require=["pytest"],
)
