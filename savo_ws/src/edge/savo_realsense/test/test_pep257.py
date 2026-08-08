# Copyright 2026 Ahnaf Tahmid

"""Check Python docstring style."""

from pathlib import Path

from ament_pep257.main import main


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_pep257() -> None:
    """Run pep257 on Python source files independent of working directory."""
    rc = main(
        argv=[
            str(PACKAGE_ROOT / "savo_realsense"),
            str(PACKAGE_ROOT / "test"),
            str(PACKAGE_ROOT / "launch"),
            "--ignore=D100,D101,D102,D103,D104,D105,D106,D107,D203,D212,D213",
        ]
    )
    assert rc == 0
