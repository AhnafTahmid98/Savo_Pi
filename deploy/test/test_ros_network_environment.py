from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def _read(relative: str) -> str:
    return ROOT.joinpath(relative).read_text(encoding="utf-8")


def test_common_ros_network_defaults_use_fastdds_subnet_discovery():
    common = _read("deploy/common/env_common.sh")
    assert 'ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"' in common
    assert 'ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"' in common
    assert 'RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"' in common
    assert "export ROS_LOCALHOST_ONLY=" not in common


def test_systemd_example_matches_network_contract():
    example = _read("deploy/systemd/robot-savo.env.example")
    assert "ROS_DOMAIN_ID=0" in example
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" in example
    assert "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in example
    assert "ROS_LOCALHOST_ONLY=" not in example
    assert "rmw_cyclonedds_cpp" not in example


def test_observer_uses_shared_network_environment():
    expected = 'source "${REPO_ROOT}/deploy/common/env_common.sh"'
    for relative in (
        "deploy/observer/run_observer.sh",
        "deploy/observer/check_connection.sh",
    ):
        assert expected in _read(relative)
