import subprocess
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


def test_required_frames_exist():
    urdf = _expanded_urdf()

    required_frames = [
        "base_footprint",
        "base_link",
        "chassis_link",
        "wheel_fl_link",
        "wheel_fr_link",
        "wheel_rl_link",
        "wheel_rr_link",
        "laser_frame",
        "imu_link",
        "camera_link",
        "camera_color_frame",
        "camera_color_optical_frame",
        "camera_depth_frame",
        "camera_depth_optical_frame",
        "tof_left_link",
        "tof_right_link",
        "ultrasonic_front_link",
        "display_link",
        "respeaker_link",
        "savo_core_board_link",
        "savo_core_hat_link",
        "savo_core_frame",
        "savo_edge_board_link",
        "savo_edge_hat_link",
        "savo_edge_frame",
        "depth_obstacle_frame",
        "local_costmap_frame",
        "footprint_debug_frame",
    ]

    for frame in required_frames:
        assert frame in urdf