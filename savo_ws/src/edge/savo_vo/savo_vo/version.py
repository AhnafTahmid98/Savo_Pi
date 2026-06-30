# Copyright 2026 Ahnaf Tahmid

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Tuple


VERSION_MAJOR: Final[int] = 0
VERSION_MINOR: Final[int] = 1
VERSION_PATCH: Final[int] = 0

PACKAGE_NAME: Final[str] = "savo_vo"
ROBOT_NAME: Final[str] = "Robot Savo"
ORG_NAME: Final[str] = "Robot SAVO Project"
SUPPORTED_ROS_DISTRO: Final[str] = "jazzy"

__version__: Final[str] = f"{VERSION_MAJOR}.{VERSION_MINOR}.{VERSION_PATCH}"
VERSION: Final[str] = __version__
VERSION_TUPLE: Final[Tuple[int, int, int]] = (
    VERSION_MAJOR,
    VERSION_MINOR,
    VERSION_PATCH,
)


@dataclass(frozen=True)
class PackageVersionInfo:
    package_name: str
    robot_name: str
    version: str
    ros_distro: str
    org_name: str

    def to_dict(self) -> dict:
        return {
            "package_name": self.package_name,
            "robot_name": self.robot_name,
            "version": self.version,
            "ros_distro": self.ros_distro,
            "org_name": self.org_name,
        }

    def short(self) -> str:
        return f"{self.package_name} {self.version}"

    def banner(self) -> str:
        return f"{self.robot_name} | {self.package_name} {self.version} | ROS 2 {self.ros_distro}"


def get_version() -> str:
    return VERSION


def get_version_tuple() -> Tuple[int, int, int]:
    return VERSION_TUPLE


def get_package_version_info() -> PackageVersionInfo:
    return PackageVersionInfo(
        package_name=PACKAGE_NAME,
        robot_name=ROBOT_NAME,
        version=VERSION,
        ros_distro=SUPPORTED_ROS_DISTRO,
        org_name=ORG_NAME,
    )


def main() -> None:
    print(get_package_version_info().banner())


if __name__ == "__main__":
    main()


__all__ = [
    "PACKAGE_NAME",
    "ROBOT_NAME",
    "ORG_NAME",
    "SUPPORTED_ROS_DISTRO",
    "VERSION",
    "VERSION_TUPLE",
    "__version__",
    "PackageVersionInfo",
    "get_version",
    "get_version_tuple",
    "get_package_version_info",
]
