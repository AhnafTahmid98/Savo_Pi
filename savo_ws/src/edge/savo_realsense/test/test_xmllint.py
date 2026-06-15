# Copyright 2026 Ahnaf Tahmid

"""Check XML files."""

from ament_xmllint.main import main


def test_xmllint() -> None:
    """Run XML lint checker."""
    rc = main(argv=["package.xml"])
    assert rc == 0
