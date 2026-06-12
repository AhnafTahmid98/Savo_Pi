"""Launch the Pi-side remote STT client."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    stt_server_url_arg = DeclareLaunchArgument(
        name="stt_server_url",
        default_value="http://robot-llm.local:9000",
        description=(
            "Base URL of the remote STT server (without /transcribe). "
            "Example: http://192.168.1.120:9000"
        ),
    )

    sample_rate_arg = DeclareLaunchArgument(
        name="sample_rate",
        default_value="16000",
        description="Audio sample rate for capture (Hz). Must match STT server assumption.",
    )

    block_duration_s_arg = DeclareLaunchArgument(
        name="block_duration_s",
        default_value="2.0",
        description=(
            "Length of each audio block in seconds. "
            "2.0 s is a good balance for utterance VAD."
        ),
    )

    energy_threshold_arg = DeclareLaunchArgument(
        name="energy_threshold",
        default_value="0.0003",
        description=(
            "Simple VAD energy threshold. "
            "Increase if you get too many false triggers; "
            "decrease if speech is not detected."
        ),
    )

    input_device_index_arg = DeclareLaunchArgument(
        name="input_device_index",
        default_value="0",
        description=(
            "sounddevice input device index for ReSpeaker. "
            "Use a small Python script to list devices if needed."
        ),
    )

    max_utterance_duration_s_arg = DeclareLaunchArgument(
        name="max_utterance_duration_s",
        default_value="15.0",
        description=(
            "Safety limit for a single utterance duration in seconds. "
            "If exceeded, the node forces a finalize and starts a new utterance."
        ),
    )

    min_transcript_chars_arg = DeclareLaunchArgument(
        name="min_transcript_chars",
        default_value="3",
        description=(
            "Minimum length of transcript to accept. "
            "Shorter outputs (noise, half syllables) are ignored."
        ),
    )

    tts_gate_enable_arg = DeclareLaunchArgument(
        name="tts_gate_enable",
        default_value="true",
        description=(
            "Whether to enable TTS gate so the robot does not transcribe its own voice. "
            "true/false."
        ),
    )

    tts_speaking_topic_arg = DeclareLaunchArgument(
        name="tts_speaking_topic",
        default_value="/savo_speech/tts_speaking",
        description=(
            "Topic (std_msgs/Bool) on which TTS node publishes speaking state. "
            "Must match tts_piper.yaml."
        ),
    )

    tts_gate_cooldown_s_arg = DeclareLaunchArgument(
        name="tts_gate_cooldown_s",
        default_value="0.8",
        description=(
            "Extra time (seconds) to ignore audio after TTS stops to avoid echo/ringing."
        ),
    )

    remote_stt_node = Node(
        package="savo_speech",
        executable="remote_stt_client_node",
        name="remote_stt_client_node",
        output="screen",
        parameters=[
            {
                "stt_server_url": LaunchConfiguration("stt_server_url"),
                "endpoint_path": "/transcribe",

                "sample_rate": LaunchConfiguration("sample_rate"),
                "block_duration_s": LaunchConfiguration("block_duration_s"),
                "energy_threshold": LaunchConfiguration("energy_threshold"),
                "input_device_index": LaunchConfiguration("input_device_index"),
                "max_utterance_duration_s": LaunchConfiguration(
                    "max_utterance_duration_s"
                ),
                "min_transcript_chars": LaunchConfiguration("min_transcript_chars"),

                "tts_gate_enable": LaunchConfiguration("tts_gate_enable"),
                "tts_speaking_topic": LaunchConfiguration("tts_speaking_topic"),
                "tts_gate_cooldown_s": LaunchConfiguration("tts_gate_cooldown_s"),
            }
        ],
    )

    return LaunchDescription(
        [
            stt_server_url_arg,
            sample_rate_arg,
            block_duration_s_arg,
            energy_threshold_arg,
            input_device_index_arg,
            max_utterance_duration_s_arg,
            min_transcript_chars_arg,
            tts_gate_enable_arg,
            tts_speaking_topic_arg,
            tts_gate_cooldown_s_arg,
            remote_stt_node,
        ]
    )
