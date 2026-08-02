from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (PACKAGE / relative).read_text(encoding='utf-8')


def test_ui_uses_typed_mapping_and_location_contracts() -> None:
    source = read('src/app/ui_node.cpp')
    header = read('include/savo_ui/app/ui_node.hpp')
    assert 'savo_msgs::msg::AutonomousMappingStatus' in source
    assert 'savo_msgs::msg::LocationEvent' in source
    assert 'mapping_status_subscription_' in header
    assert 'location_event_subscription_' in header


def test_ui_subscribes_to_live_speech_and_bridge_state() -> None:
    config = read('config/ui.yaml')
    for topic in (
        '/savo_speech/state',
        '/savo_speech/transcript',
        '/savo_speech/response',
        '/savo_speech/playback/state',
        '/savo_speech/playback/finished',
        '/savo_bridge/readiness',
    ):
        assert topic in config


def test_ui_remains_read_only() -> None:
    source = read('src/app/ui_node.cpp')
    header = read('include/savo_ui/app/ui_node.hpp')
    corpus = source + header
    assert 'create_publisher' not in corpus
    assert '/cmd_vel' not in corpus
    assert 'NavigateToLocation::Goal' not in corpus
    assert 'ReviewAutonomousMappingRelease' not in corpus


def test_ui_camera_is_explicitly_optional_until_validated() -> None:
    readme = read('README.md')
    assert 'Camera preview remains optional' in readme
    assert 'disabled' in readme
