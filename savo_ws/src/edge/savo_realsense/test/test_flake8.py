# Copyright 2026 Ahnaf Tahmid

"""Check Python style with flake8."""

from pathlib import Path

from ament_flake8.main import main_with_errors


def test_flake8() -> None:
    """Run flake8 on source files only."""
    config = Path(__file__).resolve().parents[1] / "setup.cfg"
    rc, errors = main_with_errors(
        argv=[
            "--config",
            str(config),
            "savo_realsense",
            "test",
            "launch",
            "setup.py",
        ]
    )
    assert rc == 0, "Found code style errors:\n" + "\n".join(errors)
