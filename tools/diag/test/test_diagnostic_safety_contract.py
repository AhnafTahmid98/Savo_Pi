from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
DIAG = ROOT / "tools" / "diag"


def text(relative: str) -> str:
    return (DIAG / relative).read_text(encoding="utf-8")


def test_motion_diagnostics_do_not_access_motor_or_servo_hardware_directly() -> None:
    prohibited = (
        "import smbus",
        "from smbus",
        "PCA9685(",
        "Adafruit_PCA9685",
        "/dev/gpiochip",
        "RPi.GPIO",
    )
    for relative in (
        "motion/automode.py",
        "motion/drive_automode.py",
        "motion/drive_manual_direct.py",
        "motion/motor_direction_test.py",
        "motion/pantilt_camera_view.py",
        "motion/pantilt_servo.py",
        "ui/head_pan_tilt_test.py",
    ):
        source = text(relative)
        for token in prohibited:
            assert token not in source, f"{relative} contains direct hardware token {token}"


def test_motor_motion_uses_only_approved_manual_control_boundary() -> None:
    source = text("motion/motor_direction_test.py")
    assert "--allow-motion" in source
    assert "--wheels-raised" in source
    assert "/cmd_vel_manual" in source
    assert "/cmd_vel_safe" not in source
    assert "PCA9685" not in source
    assert "for _ in range(5)" in source


def test_head_motion_uses_only_head_controller_boundary() -> None:
    source = text("ui/head_pan_tilt_test.py")
    assert "parser(__doc__, motion=True)" in source
    assert "args.allow_motion" in source
    assert "--head-clear" in source
    assert "/savo_head/pan_tilt_cmd" in source
    assert "/savo_head/pan_tilt_state" in source
    assert "PCA9685" not in source


def test_run_all_never_starts_moving_diagnostics() -> None:
    source = text("infra/run_all.sh")
    assert "Moving diagnostics are never started" in source
    assert "motor_direction_test.py" not in source
    assert "head_pan_tilt_test.py" not in source
    assert "saw_failure" in source
    assert "saw_blocked" in source


def test_legacy_direct_motion_tools_are_inert() -> None:
    for relative in (
        "motion/automode.py",
        "motion/drive_automode.py",
        "motion/drive_manual_direct.py",
        "motion/pantilt_camera_view.py",
        "motion/pantilt_servo.py",
    ):
        source = text(relative)
        assert "BLOCKED" in source
        assert "bypass_disabled" in source


def test_direct_diagnostic_entrypoints_bootstrap_repository_imports() -> None:
    for path in DIAG.rglob("*.py"):
        if path.parts[-2] == "test":
            continue
        source = path.read_text(encoding="utf-8")
        if "from tools.diag." not in source and "import tools.diag." not in source:
            continue
        assert "_REPO_ROOT = Path(__file__).resolve().parents[3]" in source, path
        assert "sys.path.insert(0, str(_REPO_ROOT))" in source, path
