# Copyright 2026 Ahnaf Tahmid
from pathlib import Path

from ament_flake8.main import main_with_errors


def test_flake8() -> None:
    """Run flake8 on package source files."""
    package_root = Path(__file__).resolve().parents[1]
    config = package_root / "setup.cfg"

    check_paths = [
        package_root / "savo_realsense",
        package_root / "test",
        package_root / "launch",
        package_root / "setup.py",
    ]

    existing_paths = [str(path) for path in check_paths if path.exists()]

    rc, errors = main_with_errors(
        argv=[
            "--config",
            str(config),
            *existing_paths,
        ]
    )

    assert rc == 0, "Found code style errors:\n" + "\n".join(errors)
