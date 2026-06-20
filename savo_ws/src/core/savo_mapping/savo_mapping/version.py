#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Package version and metadata. No ROS imports — safe to import anywhere."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Tuple


# =============================================================================
# Semantic Version
# =============================================================================
VERSION_MAJOR: Final[int] = 0
VERSION_MINOR: Final[int] = 0
VERSION_PATCH: Final[int] = 0

PRERELEASE: Final[str] = ""
BUILD_METADATA: Final[str] = ""


# =============================================================================
# Package Identity Metadata
# =============================================================================
PACKAGE_NAME: Final[str] = "savo_mapping"
ROBOT_NAME: Final[str] = "Robot Savo"
ORG_NAME: Final[str] = "Robot SAVO Project"
SUPPORTED_ROS_DISTRO: Final[str] = "jazzy"


# =============================================================================
# Derived
# =============================================================================
def _build_version_string() -> str:
    base = f"{VERSION_MAJOR}.{VERSION_MINOR}.{VERSION_PATCH}"

    if PRERELEASE.strip():
        base += f"-{PRERELEASE.strip()}"

    if BUILD_METADATA.strip():
        base += f"+{BUILD_METADATA.strip()}"

    return base


__version__: Final[str] = "0.0.1"
VERSION: Final[str] = __version__
VERSION_TUPLE: Final[Tuple[int, int, int]] = (VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH)


# =============================================================================
# Structured metadata
# =============================================================================
@dataclass(frozen=True)
class PackageVersionInfo:
    package_name: str
    robot_name: str
    version: str
    major: int
    minor: int
    patch: int
    prerelease: str
    build_metadata: str
    ros_distro: str
    org_name: str

    def to_dict(self) -> dict:
        return {
            "package_name": self.package_name,
            "robot_name": self.robot_name,
            "version": self.version,
            "major": self.major,
            "minor": self.minor,
            "patch": self.patch,
            "prerelease": self.prerelease or None,
            "build_metadata": self.build_metadata or None,
            "ros_distro": self.ros_distro,
            "org_name": self.org_name,
        }

    def short(self) -> str:
        return f"{self.package_name} {self.version}"

    def banner(self) -> str:
        return (
            f"{self.robot_name} | {self.package_name} {self.version} "
            f"(ROS 2 {self.ros_distro})"
        )


def get_version() -> str:
    """Return package version string (SemVer-style)."""
    return VERSION


def get_version_tuple() -> Tuple[int, int, int]:
    """Return version tuple: (major, minor, patch)."""
    return VERSION_TUPLE


def get_package_version_info() -> PackageVersionInfo:
    """Return structured package version metadata."""
    return PackageVersionInfo(
        package_name=PACKAGE_NAME,
        robot_name=ROBOT_NAME,
        version=VERSION,
        major=VERSION_MAJOR,
        minor=VERSION_MINOR,
        patch=VERSION_PATCH,
        prerelease=PRERELEASE,
        build_metadata=BUILD_METADATA,
        ros_distro=SUPPORTED_ROS_DISTRO,
        org_name=ORG_NAME,
    )


def get_package_info() -> dict:
    """Return package metadata as a plain dictionary."""
    return get_package_version_info().to_dict()


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    """Simple CLI output for quick checks: `python -m savo_mapping.version`."""
    info = get_package_version_info()
    print(info.banner())


if __name__ == "__main__":
    main()


__all__ = [
    "__version__",
    "VERSION",
    "VERSION_TUPLE",
    "VERSION_MAJOR",
    "VERSION_MINOR",
    "VERSION_PATCH",
    "PRERELEASE",
    "BUILD_METADATA",
    "PACKAGE_NAME",
    "ROBOT_NAME",
    "ORG_NAME",
    "SUPPORTED_ROS_DISTRO",
    "PackageVersionInfo",
    "get_version",
    "get_version_tuple",
    "get_package_version_info",
    "get_package_info",
    "main",
]
