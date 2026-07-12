from savo_speech.ros.qos_profiles import (
    QOS_DEFAULT,
    QOS_EVENT,
    QOS_HEALTH,
    QOS_HEARTBEAT,
    QOS_STATE,
    QOS_STATUS,
    get_qos_spec,
    qos_name_for_topic,
)


def test_state_qos_is_latched():
    spec = get_qos_spec(QOS_STATE)

    assert spec.depth == 1
    assert spec.reliability == "reliable"
    assert spec.durability == "transient_local"


def test_event_status_and_health_are_reliable():
    for name in (QOS_EVENT, QOS_STATUS, QOS_HEALTH):
        spec = get_qos_spec(name)

        assert spec.reliability == "reliable"
        assert spec.durability == "volatile"


def test_heartbeat_avoids_backpressure():
    spec = get_qos_spec(QOS_HEARTBEAT)

    assert spec.depth == 5
    assert spec.reliability == "best_effort"
    assert spec.durability == "volatile"


def test_topic_qos_mapping():
    assert qos_name_for_topic("/savo_speech/state") == QOS_STATE
    assert qos_name_for_topic("/savo_speech/input_muted") == QOS_STATE
    assert qos_name_for_topic("/savo_speech/output_muted") == QOS_STATE
    assert qos_name_for_topic("/savo_speech/tts/finished") == QOS_EVENT
    assert qos_name_for_topic("/savo_speech/status") == QOS_STATUS
    assert qos_name_for_topic("/savo_speech/health") == QOS_HEALTH
    assert qos_name_for_topic("/savo_speech/heartbeat") == QOS_HEARTBEAT
    assert qos_name_for_topic("/unknown") == QOS_DEFAULT
