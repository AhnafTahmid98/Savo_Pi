from savo_speech import constants as c
from savo_speech.ros.topic_contract import (
    SERVICE_CONTRACT,
    TOPIC_CONTRACT,
    assert_contract_valid,
    get_service_names,
    get_topic_names,
    is_forbidden_motion_topic,
    is_public_service,
    is_public_topic,
    validate_contract,
)


def test_contract_is_valid_and_unique():
    assert validate_contract() == []
    assert_contract_valid()

    assert len(get_topic_names()) == len(set(get_topic_names()))
    assert len(get_service_names()) == len(set(get_service_names()))


def test_constants_and_registry_match():
    assert get_topic_names() == c.PUBLIC_TOPICS
    assert get_service_names() == c.PUBLIC_SERVICES


def test_topics_remain_inside_package_namespace():
    assert all(
        spec.name.startswith("/savo_speech/")
        for spec in TOPIC_CONTRACT.values()
    )

    assert all(
        spec.authoritative_owner == "savo_speech"
        for spec in TOPIC_CONTRACT.values()
    )


def test_services_use_standard_interfaces():
    assert {spec.srv_type for spec in SERVICE_CONTRACT.values()} == {
        "std_srvs/srv/SetBool",
        "std_srvs/srv/Trigger",
    }


def test_tts_finished_is_explicit():
    spec = TOPIC_CONTRACT["tts_finished"]

    assert spec.name == "/savo_speech/tts/finished"
    assert spec.msg_type == "std_msgs/msg/String"
    assert spec.qos_key == "event"


def test_motion_topics_are_forbidden():
    for topic in c.FORBIDDEN_MOTION_TOPICS:
        assert is_forbidden_motion_topic(topic)
        assert not is_public_topic(topic)


def test_deferred_contracts_are_not_locked():
    assert is_public_topic("/savo_speech/state")
    assert is_public_service("/savo_speech/cancel")

    assert not is_public_topic("/savo_speech/raw_audio")
    assert not is_public_service("/savo_speech/set_volume")
