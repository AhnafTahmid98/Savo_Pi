import subprocess
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_FILE = PACKAGE_ROOT / "urdf" / "robot_savo.urdf.xacro"


def test_main_xacro_expands_to_urdf():
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
    assert "<robot" in result.stdout
    assert "name=\"robot_savo\"" in result.stdout
    assert "base_link" in result.stdout