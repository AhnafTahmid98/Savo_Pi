import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]

if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


def flatten_public_values(value, seen=None):
    if seen is None:
        seen = set()

    value_id = id(value)
    if value_id in seen:
        return

    seen.add(value_id)

    if isinstance(value, str):
        yield value
        return

    if isinstance(value, dict):
        for key, item in value.items():
            yield str(key)
            yield from flatten_public_values(item, seen)
        return

    if isinstance(value, (list, tuple, set, frozenset)):
        for item in value:
            yield from flatten_public_values(item, seen)
        return

    if hasattr(value, "__dict__"):
        for name, item in vars(value).items():
            if not name.startswith("_"):
                yield name
                yield from flatten_public_values(item, seen)


def test_topic_contract_values():
    from savo_head.contracts import topic_names

    values = set(flatten_public_values(topic_names))

    required_topics = {
        "/savo_head/pan_tilt_cmd",
        "/savo_head/pan_tilt_state",
        "/savo_head/scan_cmd",
        "/savo_head/scan_state",
        "/savo_head/status",
        "/savo_head/dashboard_text",
        "/savo_head/camera_status",
        "/savo_head/apriltag_detections",
        "/savo_head/semantic_confirmations",
        "/savo_head/robot_pose_snapshot",
        "/diagnostics",
    }

    missing = sorted(required_topics - values)
    assert not missing, f"Missing topic names: {missing}"


def test_topic_contract_has_head_namespace():
    from savo_head.contracts import topic_names

    values = set(flatten_public_values(topic_names))

    head_topics = sorted(
        value for value in values
        if isinstance(value, str) and value.startswith("/savo_head/")
    )

    assert head_topics, "No /savo_head topics found"

    for topic in head_topics:
        assert topic.startswith("/savo_head/"), f"Unexpected head topic namespace: {topic}"
        assert " " not in topic, f"Topic contains spaces: {topic}"


def test_topic_contract_no_old_camera_assumptions():
    from savo_head.contracts import topic_names

    values = "\n".join(sorted(set(flatten_public_values(topic_names))))

    banned = [
        "rpicam",
        "libcamera-hello",
        "libcamera-vid",
    ]

    found = [item for item in banned if item in values]
    assert not found, f"Topic contract contains old camera assumptions: {found}"


if __name__ == "__main__":
    test_topic_contract_values()
    test_topic_contract_has_head_namespace()
    test_topic_contract_no_old_camera_assumptions()
    print("PASS: savo_head Python topic contract is complete.")
