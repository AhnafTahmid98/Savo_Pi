"""Speech nodes for STT, TTS, and mouth animation."""

from importlib.metadata import version, PackageNotFoundError

__all__ = [
    "__version__",
]

try:
    __version__ = version("savo_speech")
except PackageNotFoundError:
    __version__ = "0.0.0"
