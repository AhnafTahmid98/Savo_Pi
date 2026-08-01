from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding='utf-8')


def test_coordinator_uses_existing_typed_owners() -> None:
    source = read('src/nodes/semantic_interruption_coordinator_node.cpp')

    for token in (
        'savo_msgs/msg/april_tag_observation.hpp',
        'savo_msgs/msg/autonomous_mapping_status.hpp',
        'savo_msgs/msg/location_event.hpp',
        'savo_msgs/msg/semantic_interruption_status.hpp',
        'savo_msgs/srv/control_autonomous_mapping.hpp',
        'savo_msgs/srv/submit_semantic_location.hpp',
        'savo_msgs/action/register_mapped_location.hpp',
        'COMMAND_PAUSE',
        'COMMAND_RESUME',
        'STATE_FILTER_ALL',
        'rclcpp::QoS(1).reliable().transient_local()',
    ):
        assert token in source


def test_coordinator_does_not_bypass_owners() -> None:
    source = read('src/nodes/semantic_interruption_coordinator_node.cpp')

    forbidden = (
        'ConfirmAprilTag',
        'confirm_april_tag.hpp',
        'NavigateToPose',
        'nav2_msgs',
        'cmd_vel',
        'sqlite',
        'database_path',
        'RegisterLocationCandidate',
        'register_location_candidate.hpp',
        'AuthorizeOperation',
        'authorize_operation.hpp',
        'AuthorizeLocationOperation',
        'authorize_location_operation.hpp',
    )
    for token in forbidden:
        assert token not in source


def test_configuration_uses_current_repository_endpoints() -> None:
    config = read('config/semantic_interruption.yaml')

    for token in (
        'expected_family: "tag36h11"',
        'failure_policy: "remain_paused"',
        'observation_topic: "/savo_head/apriltag/observations"',
        'mission_status_topic: "/savo_mapping/autonomous/status"',
        'mission_control_service: "/savo_mapping/autonomous/control"',
        'registration_action: "/savo_mapping/locations/register"',
        'location_events_topic: "/savo_locations/events"',
        'status_topic: "/savo_mapping/semantic_interruption/status"',
        (
            'semantic_submit_service: '
            '"/savo_mapping/semantic_interruption/submit"'
        ),
    ):
        assert token in config


def test_launch_and_build_install_am6_additively() -> None:
    cmake = read('CMakeLists.txt')
    launch = read('launch/autonomous_mapping.launch.xml')

    for token in (
        'src/semantic/semantic_interruption.cpp',
        'include/savo_mapping/semantic_interruption.hpp',
        'src/nodes/semantic_interruption_coordinator_node.cpp',
        'semantic_interruption_coordinator_node',
        'config/semantic_interruption.yaml',
        'test_semantic_interruption',
        'test_semantic_interruption_contract',
    ):
        assert token in cmake

    assert 'name="semantic_interruption_enabled"' in launch
    assert 'if="$(var semantic_interruption_enabled)"' in launch
    assert 'exec="semantic_interruption_coordinator_node"' in launch
    assert 'semantic_interruption.yaml' in launch


def test_core_encodes_eligible_states_and_duplicate_policy() -> None:
    header = read('include/savo_mapping/semantic_interruption.hpp')
    source = read('src/semantic/semantic_interruption.cpp')

    assert 'kMissionStateExploring{3U}' in header
    assert 'kMissionStateCoverage{19U}' in header
    assert 'kMissionStatePaused{5U}' in header
    assert 'registered_tags_' in header
    assert 'cooldown_until_ns_' in header
    assert 'observation_predates_current_mission' in source
    assert 'mission_state_not_semantic_observable' in source
    assert 'tag_registered_during_process_lifetime' in source
    assert 'tag_already_present_in_location_registry' in source
