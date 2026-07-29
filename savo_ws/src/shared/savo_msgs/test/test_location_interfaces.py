from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_location_interfaces_registered() -> None:
    cmake = read("CMakeLists.txt")

    for path in (
        "msg/LocationRecord.msg",
        "msg/LocationCandidate.msg",
        "msg/LocationEvent.msg",
        "srv/ResolveLocation.srv",
        "srv/GetLocation.srv",
        "srv/ListLocations.srv",
        "srv/RegisterLocationCandidate.srv",
        "srv/ApproveLocation.srv",
        "srv/SetLocationEnabled.srv",
    ):
        assert f'"{path}"' in cmake


def test_map_revision_is_canonical() -> None:
    for path in (
        "msg/LocationRecord.msg",
        "msg/LocationCandidate.msg",
        "srv/ResolveLocation.srv",
        "srv/ListLocations.srv",
        "action/ConfirmAprilTag.action",
    ):
        text = read(path)

        assert "map_revision" in text
        assert "map_version" not in text


def test_navigation_resolution_guards() -> None:
    text = read("srv/ResolveLocation.srv")

    assert "bool enforce_map_context" in text
    assert "RESULT_AMBIGUOUS=3" in text
    assert "RESULT_MAP_MISMATCH=6" in text
    assert "string[] ambiguous_location_ids" in text


def test_revision_guards() -> None:
    assert (
        "uint64 expected_candidate_revision"
        in read("srv/ApproveLocation.srv")
    )

    assert (
        "uint64 expected_record_revision"
        in read("srv/SetLocationEnabled.srv")
    )
