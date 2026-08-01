"""End-to-end runtime test for the production location lifecycle launch."""

import os
import subprocess
from pathlib import Path


def test_production_launch_completes_full_location_lifecycle(
    tmp_path: Path,
) -> None:
    """The production launch completes registration through arrival."""
    environment = os.environ.copy()
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    ros_log_dir = tmp_path / "ros_logs"
    ros_log_dir.mkdir()
    environment["ROS_LOG_DIR"] = str(ros_log_dir)
    completed = subprocess.run(
        [
            "ros2",
            "run",
            "savo_bringup",
            "run_location_lifecycle_runtime",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=120.0,
        check=False,
        env=environment,
    )

    assert completed.returncode == 0, completed.stdout
    assert "PHASE 2D LOCATION LIFECYCLE RUNTIME: PASS" in completed.stdout
    assert "production launch exposed all public lifecycle boundaries" in (
        completed.stdout
    )
    assert "navigation forwarded only the approved approach pose" in (
        completed.stdout
    )
    assert "arrival confirmation matched the saved AprilTag" in (
        completed.stdout
    )
