from pathlib import Path

import yaml

from savo_speech import constants as c
from savo_speech.ros.topic_contract import (
    SERVICE_CONTRACT,
    TOPIC_CONTRACT,
)


ROOT = Path(__file__).resolve().parents[1]


def load_contract() -> dict:
    path = ROOT / "config" / "topics.yaml"

    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    assert isinstance(data, dict)
    return data


def test_yaml_identity():
    data = load_contract()

    assert data["schema_version"] == 1
    assert data["package"] == "savo_speech"
    assert data["namespace"] == "/savo_speech"
    assert data["owner"] == "savo_speech"


def test_yaml_topics_match_python_contract():
    yaml_topics = load_contract()["topics"]

    assert set(yaml_topics) == set(TOPIC_CONTRACT)

    for key, spec in TOPIC_CONTRACT.items():
        item = yaml_topics[key]

        assert item["name"] == spec.name
        assert item["type"] == spec.msg_type
        assert item["direction"] == spec.direction
        assert item["required"] is spec.required
        assert item["qos"] == spec.qos_key


def test_yaml_services_match_python_contract():
    yaml_services = load_contract()["services"]

    assert set(yaml_services) == set(SERVICE_CONTRACT)

    for key, spec in SERVICE_CONTRACT.items():
        item = yaml_services[key]

        assert item["name"] == spec.name
        assert item["type"] == spec.srv_type
        assert item["required"] is spec.required


def test_forbidden_topics_match_constants():
    data = load_contract()

    assert (
        tuple(data["forbidden_motion_topics"])
        == c.FORBIDDEN_MOTION_TOPICS
    )
