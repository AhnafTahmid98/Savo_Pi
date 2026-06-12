"""Robot Savo LiDAR hardware ownership package."""

try:
    from .version import __version__
except ImportError:
    __version__ = "0.0.1"

__all__ = ["__version__"]