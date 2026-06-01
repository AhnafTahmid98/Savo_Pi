from setuptools import setup, find_packages
from glob import glob
import os

package_name = "savo_speech"

#
# Collect extra data files (ament index, launch files, config files)
#

# ament index resource file
resource_files = [os.path.join("resource", package_name)]

# package manifest
package_xml = ["package.xml"]

# all launch files under launch/
launch_files = glob(os.path.join("launch", "*.launch.py"))

# all YAML config files under config/
config_files = glob(os.path.join("config", "*.yaml"))

data_files = [
    # Required by ament so the package is discoverable
    (
        "share/ament_index/resource_index/packages",
        resource_files,
    ),
    # Install package.xml
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
            # Speech-to-text node (Faster-Whisper)
            "stt_node = savo_speech.stt_node:main",

            #STT remote
            "remote_stt_client_node = savo_speech.remote_stt_client_node:main",

            # Remote speech client (Pi mic -> /speech -> IntentResult + TTS text)
            "remote_speech_client_node = savo_speech.remote_speech_client_node:main",


            # Text-to-speech node (Piper)
            "tts_node = savo_speech.tts_node:main",

            # Bridge STT ↔ LLM ↔ TTS
            "speech_bridge_node = savo_speech.speech_bridge_node:main",

            # Mouth / face animation node for the DSI UI
            # (module file is mouth_anim.py, executable name is mouth_anim_node)
            "mouth_anim_node = savo_speech.mouth_anim:main",
        ],
    },
)
