from setuptools import setup, find_packages
from glob import glob
import os

package_name = "savo_speech"

resource_files = [os.path.join("resource", package_name)]

package_xml = ["package.xml"]

launch_files = glob(os.path.join("launch", "*.launch.py"))

config_files = glob(os.path.join("config", "*.yaml"))

data_files = [
    (
        "share/ament_index/resource_index/packages",
        resource_files,
    ),
    (
        f"share/{package_name}",
        package_xml,
    ),
]

if launch_files:
    data_files.append(
        (
            f"share/{package_name}/launch",
            launch_files,
        )
    )

if config_files:
    data_files.append(
        (
            f"share/{package_name}/config",
            config_files,
        )
    )

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test*", "tests*"]),
    data_files=data_files,
    # Keep install_requires minimal; heavy deps (sounddevice, faster-whisper,
    # piper-tts, etc.) are installed at system/user level.
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        "Robot Savo speech stack: faster-whisper STT, Piper TTS, and mouth "
        "animation integration for the on-robot UI."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stt_node = savo_speech.stt_node:main",
            "remote_stt_client_node = savo_speech.remote_stt_client_node:main",
            "remote_speech_client_node = savo_speech.remote_speech_client_node:main",
            "tts_node = savo_speech.tts_node:main",
            "speech_bridge_node = savo_speech.speech_bridge_node:main",
            "mouth_anim_node = savo_speech.mouth_anim:main",
        ],
    },
)
