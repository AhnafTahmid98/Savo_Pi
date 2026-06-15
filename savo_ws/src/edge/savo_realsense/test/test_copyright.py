# Copyright 2026 Ahnaf Tahmid

"""Check project copyright headers."""

from pathlib import Path


def test_copyright() -> None:
    """Check source files contain the Robot Savo copyright header."""
    paths = []

    for root_name in ("savo_realsense", "test", "launch"):
        root = Path(root_name)
        if root.exists():
            paths.extend(root.rglob("*.py"))

    setup_py = Path("setup.py")
    if setup_py.exists():
        paths.append(setup_py)

    missing = []
    for path in sorted(paths):
        text = path.read_text(encoding="utf-8")
        header_area = "\n".join(text.splitlines()[:10])
        if "Copyright 2026 Ahnaf Tahmid" not in header_area:
            missing.append(str(path))

    assert not missing, "Missing copyright header:\n" + "\n".join(missing)
