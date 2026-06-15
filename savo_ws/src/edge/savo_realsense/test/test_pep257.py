# Copyright 2026 Ahnaf Tahmid

"""Check Python docstring style."""

from ament_pep257.main import main


def test_pep257() -> None:
    """Run pep257 on source files only."""
    rc = main(
        argv=[
            "savo_realsense",
            "test",
            "launch",
            "setup.py",
            "--ignore=D100,D101,D102,D103,D104,D105,D106,D107,D203,D212,D213",
        ]
    )
    assert rc == 0
