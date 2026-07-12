# -*- coding: utf-8 -*-

"""Canonical public ROS topic and service contract for savo_speech."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Final

from savo_speech import constants as c


DIRECTION_PUBLISH: Final[str] = "pub"
DIRECTION_SUBSCRIBE: Final[str] = "sub"
DIRECTION_PUBLISH_SUBSCRIBE: Final[str] = "pubsub"


@dataclass(frozen=True)
class TopicSpec:
    name: str
    msg_type: str
    direction: str
    description: str
    required: bool = True
    qos_key: str = "status"
    authoritative_owner: str = c.PACKAGE_NAME

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class ServiceSpec:
    name: str
    srv_type: str
    description: str
    required: bool = True
    authoritative_owner: str = c.PACKAGE_NAME

    def to_dict(self) -> dict:
        return asdict(self)


TOPIC_CONTRACT: Final[dict[str, TopicSpec]] = {
    "state": TopicSpec(
        name=c.TOPIC_STATE,
        msg_type="std_msgs/msg/String",
        direction=DIRECTION_PUBLISH,
        description="Authoritative speech runtime state token.",
        qos_key="state",
    ),
    "status": TopicSpec(
        name=c.TOPIC_STATUS,
        msg_type="std_msgs/msg/String",
        direction=DIRECTION_PUBLISH,
        description="Consolidated speech runtime status encoded as JSON.",
        qos_key="status",
    ),
    "health": TopicSpec(
        name=c.TOPIC_HEALTH,
        msg_type="diagnostic_msgs/msg/DiagnosticArray",
        direction=DIRECTION_PUBLISH,
        description="Microphone, playback, provider, and pipeline health.",
        qos_key="health",
    ),
    "heartbeat": TopicSpec(
        name=c.TOPIC_HEARTBEAT,
        msg_type="std_msgs/msg/String",
        direction=DIRECTION_PUBLISH,
        description="Periodic supervision heartbeat encoded as JSON.",
        qos_key="heartbeat",
    ),
    "wake_state": TopicSpec(
        name=c.TOPIC_WAKE_STATE,
        msg_type="std_msgs/msg/String",
        direction=DIRECTION_PUBLISH,
        description="Authoritative wake subsystem state token.",
        qos_key="state",
    ),
    "wake_word_detected": TopicSpec(
        name=c.TOPIC_WAKE_WORD_DETECTED,
        msg_type="std_msgs/msg/String",
        direction=DIRECTION_PUBLISH,
        description="Wake detection event encoded as JSON.",
        required=False,
        qos_key="event",
    ),
    "listening": TopicSpec(
        name=c.TOPIC_LISTENING,
        msg_type="std_msgs/msg/Bool",
        direction=DIRECTION_PUBLISH,
        description="True while collecting an utterance.",
        qos_key="state",
    ),
    "input_muted": TopicSpec(
        name=c.TOPIC_INPUT_MUTED,
        msg_type="std_msgs/msg/Bool",
        direction=DIRECTION_PUBLISH,
        description="True while microphone input processing is muted.",
        qos_key="state",
    ),
    "output_muted": TopicSpec(
        name=c.TOPIC_OUTPUT_MUTED,
        msg_type="std_msgs/msg/Bool",
        direction=DIRECTION_PUBLISH,
        description="True while speaker playback is muted.",
        qos_key="state",
    ),
    "transcript": TopicSpec(
        name=c.TOPIC_TRANSCRIPT,
        msg_type="std_msgs/msg/String",
        direction=DIRECTION_PUBLISH,
        description="Final accepted transcript for the active session.",
        required=False,
        qos_key="event",
    ),
    "tts_gate": TopicSpec(
        name=c.TOPIC_TTS_GATE,
        msg_type="std_msgs/msg/Bool",
        direction=DIRECTION_PUBLISH,
        description="True while microphone input must reject playback echo.",
        qos_key="state",
    ),
    "tts_started": TopicSpec(
        name=c.TOPIC_TTS_STARTED,
        msg_type="std_msgs/msg/String",
        direction=DIRECTION_PUBLISH,
        description="Synthesized speech playback-started event.",
        required=False,
        qos_key="event",
    ),
    "tts_finished": TopicSpec(
        name=c.TOPIC_TTS_FINISHED,
        msg_type="std_msgs/msg/String",
        direction=DIRECTION_PUBLISH,
        description="Physical speech playback-finished event.",
        required=False,
        qos_key="event",
    ),
    "tts_cancelled": TopicSpec(
        name=c.TOPIC_TTS_CANCELLED,
        msg_type="std_msgs/msg/String",
        direction=DIRECTION_PUBLISH,
        description="Speech playback-cancelled event.",
        required=False,
        qos_key="event",
    ),
    "face_state": TopicSpec(
        name=c.TOPIC_FACE_STATE,
        msg_type="std_msgs/msg/String",
        direction=DIRECTION_PUBLISH,
        description="Speech-derived UI hint; savo_ui remains the renderer.",
        required=False,
        qos_key="state",
    ),
    "last_error": TopicSpec(
        name=c.TOPIC_LAST_ERROR,
        msg_type="std_msgs/msg/String",
        direction=DIRECTION_PUBLISH,
        description="Most recent speech subsystem error.",
        qos_key="state",
    ),
}


SERVICE_CONTRACT: Final[dict[str, ServiceSpec]] = {
    "wake": ServiceSpec(
        name=c.ROS_SERVICE_WAKE,
        srv_type="std_srvs/srv/Trigger",
        description="Force the speech subsystem into awake idle.",
    ),
    "sleep": ServiceSpec(
        name=c.ROS_SERVICE_SLEEP,
        srv_type="std_srvs/srv/Trigger",
        description="Return the speech subsystem to sleeping.",
    ),
    "start_listening": ServiceSpec(
        name=c.ROS_SERVICE_START_LISTENING,
        srv_type="std_srvs/srv/Trigger",
        description="Request one explicit listening turn.",
    ),
    "stop_listening": ServiceSpec(
        name=c.ROS_SERVICE_STOP_LISTENING,
        srv_type="std_srvs/srv/Trigger",
        description="Stop the active listening turn safely.",
    ),
    "cancel": ServiceSpec(
        name=c.ROS_SERVICE_CANCEL,
        srv_type="std_srvs/srv/Trigger",
        description="Cancel the active speech turn or playback.",
    ),
    "mute_input": ServiceSpec(
        name=c.ROS_SERVICE_MUTE_INPUT,
        srv_type="std_srvs/srv/SetBool",
        description="Enable or disable microphone processing.",
    ),
    "mute_output": ServiceSpec(
        name=c.ROS_SERVICE_MUTE_OUTPUT,
        srv_type="std_srvs/srv/SetBool",
        description="Enable or disable speaker output.",
    ),
    "reload_audio_device": ServiceSpec(
        name=c.ROS_SERVICE_RELOAD_AUDIO_DEVICE,
        srv_type="std_srvs/srv/Trigger",
        description="Rediscover and reopen the reSpeaker device.",
    ),
}


def get_topic_spec(key: str) -> TopicSpec:
    return TOPIC_CONTRACT[key]


def get_service_spec(key: str) -> ServiceSpec:
    return SERVICE_CONTRACT[key]


def get_topic_names() -> tuple[str, ...]:
    return tuple(spec.name for spec in TOPIC_CONTRACT.values())


def get_service_names() -> tuple[str, ...]:
    return tuple(spec.name for spec in SERVICE_CONTRACT.values())


def is_public_topic(name: str) -> bool:
    return str(name) in get_topic_names()


def is_public_service(name: str) -> bool:
    return str(name) in get_service_names()


def is_forbidden_motion_topic(name: str) -> bool:
    return str(name) in c.FORBIDDEN_MOTION_TOPICS


def validate_contract() -> list[str]:
    errors: list[str] = []

    topic_names = get_topic_names()
    service_names = get_service_names()

    if len(topic_names) != len(set(topic_names)):
        errors.append("duplicate topic name")

    if len(service_names) != len(set(service_names)):
        errors.append("duplicate service name")

    for key, spec in TOPIC_CONTRACT.items():
        if not spec.name.startswith("/savo_speech/"):
            errors.append(
                f"topic {key} is outside /savo_speech: {spec.name}"
            )

        if spec.direction not in {
            DIRECTION_PUBLISH,
            DIRECTION_SUBSCRIBE,
            DIRECTION_PUBLISH_SUBSCRIBE,
        }:
            errors.append(
                f"topic {key} has invalid direction: {spec.direction}"
            )

        if is_forbidden_motion_topic(spec.name):
            errors.append(
                f"topic {key} illegally owns motion topic: {spec.name}"
            )

    for key, spec in SERVICE_CONTRACT.items():
        if not spec.name.startswith("/savo_speech/"):
            errors.append(
                f"service {key} is outside /savo_speech: {spec.name}"
            )

    return errors


def assert_contract_valid() -> None:
    errors = validate_contract()

    if errors:
        raise ValueError(
            "Invalid savo_speech ROS contract: " + "; ".join(errors)
        )
