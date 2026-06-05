from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_main_xacro_exists():
    path = PACKAGE_ROOT / "urdf" / "robot_savo.urdf.xacro"
    assert path.is_file()


def test_required_description_folders_exist():
    required_dirs = [
        "launch",
        "urdf",
        "urdf/macros",
        "config",
        "rviz",
        "meshes",
        "scripts",
        "test",
    ]

    for rel_path in required_dirs:
        assert (PACKAGE_ROOT / rel_path).is_dir()