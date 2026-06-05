import re
import subprocess
from collections import Counter
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_FILE = PACKAGE_ROOT / "urdf" / "robot_savo.urdf.xacro"


def _expanded_urdf():
    result = subprocess.run(
        [
            "xacro",
            str(XACRO_FILE),
        ],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    return result.stdout


def test_no_duplicate_link_names():
    urdf = _expanded_urdf()
    link_names = re.findall(r'<link\s+name="([^"]+)"', urdf)

    duplicates = [
        name for name, count in Counter(link_names).items()
        if count > 1
    ]

    assert not duplicates, f"Duplicate link names found: {duplicates}"


def test_no_duplicate_joint_names():
    urdf = _expanded_urdf()
    joint_names = re.findall(r'<joint\s+name="([^"]+)"', urdf)

    duplicates = [
        name for name, count in Counter(joint_names).items()
        if count > 1
    ]

    assert not duplicates, f"Duplicate joint names found: {duplicates}"