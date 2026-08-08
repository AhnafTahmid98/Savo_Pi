# Copyright 2026 Ahnaf Tahmid

"""Check XML files."""

from pathlib import Path

from ament_xmllint.main import main


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_xmllint() -> None:
    """Run XML lint checker independent of working directory."""
    rc = main(argv=[str(PACKAGE_ROOT / "package.xml")])
    assert rc == 0
