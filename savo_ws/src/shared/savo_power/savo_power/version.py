"""Version metadata for the Robot Savo power package."""

from __future__ import annotations

from dataclasses import dataclass


PACKAGE_NAME = "savo_power"

VERSION_MAJOR = 0
VERSION_MINOR = 1
VERSION_PATCH = 0

__version__ = f"{VERSION_MAJOR}.{VERSION_MINOR}.{VERSION_PATCH}"


@dataclass(frozen=True)
class VersionInfo:
    """Structured semantic version information."""

    major: int
    minor: int
    patch: int

    @property
    def text(self) -> str:
        return f"{self.major}.{self.minor}.{self.patch}"

    @property
    def tuple(self) -> tuple[int, int, int]:
        return (self.major, self.minor, self.patch)


VERSION_INFO = VersionInfo(
    major=VERSION_MAJOR,
    minor=VERSION_MINOR,
    patch=VERSION_PATCH,
)


def package_version() -> str:
    """Return the package version as a string."""

    return __version__


def package_name() -> str:
    """Return the package name."""

    return PACKAGE_NAME


__all__ = [
    "PACKAGE_NAME",
    "VERSION_MAJOR",
    "VERSION_MINOR",
    "VERSION_PATCH",
    "VERSION_INFO",
    "VersionInfo",
    "__version__",
    "package_name",
    "package_version",
]
