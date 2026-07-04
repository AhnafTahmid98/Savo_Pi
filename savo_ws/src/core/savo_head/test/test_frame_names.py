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


def test_frame_contract_values():
    from savo_head.contracts import frame_names

    values = set(flatten_public_values(frame_names))

    required_frames = {
        "base_link",
        "pantilt_pan_link",
        "pantilt_tilt_link",
        "pi_camera_link",
        "pi_camera_optical_frame",
        "head_pan_joint",
        "head_tilt_joint",
    }

    missing = sorted(required_frames - values)
    assert not missing, f"Missing frame/joint names: {missing}"


def test_frame_names_are_ros_safe():
    from savo_head.contracts import frame_names

    values = set(flatten_public_values(frame_names))

    checked = [
        value for value in values
        if isinstance(value, str)
        and (
            value.endswith("_link")
            or value.endswith("_frame")
            or value.endswith("_joint")
            or value == "base_link"
        )
    ]

    assert checked, "No ROS frame/joint names found"

    for name in checked:
        assert not name.startswith("/"), f"Frame must not start with slash: {name}"
        assert " " not in name, f"Frame contains spaces: {name}"


def test_frame_contract_has_camera_chain():
    from savo_head.contracts import frame_names

    values = set(flatten_public_values(frame_names))

    chain = [
        "base_link",
        "pantilt_pan_link",
        "pantilt_tilt_link",
        "pi_camera_link",
        "pi_camera_optical_frame",
    ]

    for frame in chain:
        assert frame in values, f"Missing camera TF chain frame: {frame}"


if __name__ == "__main__":
    test_frame_contract_values()
    test_frame_names_are_ros_safe()
    test_frame_contract_has_camera_chain()
    print("PASS: savo_head Python frame contract is complete.")
