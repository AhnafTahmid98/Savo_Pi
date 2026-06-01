"""
savo_speech
===========

Robot Savo speech stack:

- STT (speech-to-text) using faster-whisper
- TTS (text-to-speech) using Piper
- Mouth animation signals for the on-robot UI

This package is designed to be used as a ROS 2 Python package
(ament_python) inside the Robot Savo workspace.

The actual ROS 2 nodes live in:

- stt_node.py      → /savo_intent/user_text publisher
- tts_node.py      → /savo_intent/reply_text subscriber, audio output
- mouth_anim.py    → UI-facing mouth animation publisher
"""

from importlib.metadata import version, PackageNotFoundError

__all__ = [
    "__version__",
]

try:
    __version__ = version("savo_speech")
except PackageNotFoundError:
    # Fallback when running from source without installation
    __version__ = "0.0.0"
