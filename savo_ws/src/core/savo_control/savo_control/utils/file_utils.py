# -*- coding: utf-8 -*-

"""Small file helpers for Python CLI tools and tests."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import yaml


def read_text_file(path: str | Path) -> str:
    return Path(path).read_text(encoding="utf-8")


def write_text_file(path: str | Path, text: str) -> Path:
    file_path = Path(path)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    file_path.write_text(text, encoding="utf-8")
    return file_path


def read_json_file(path: str | Path) -> Any:
    return json.loads(read_text_file(path))


def write_json_file(path: str | Path, data: Any, *, indent: int = 2) -> Path:
    text = json.dumps(data, indent=indent, sort_keys=True)
    return write_text_file(path, f"{text}\n")


def read_yaml_file(path: str | Path) -> Any:
    data = yaml.safe_load(read_text_file(path))
    return {} if data is None else data


def write_yaml_file(path: str | Path, data: Any) -> Path:
    text = yaml.safe_dump(data, sort_keys=True)
    return write_text_file(path, text)


def ensure_directory(path: str | Path) -> Path:
    directory = Path(path)
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def ensure_parent_dir(path: str | Path) -> Path:
    file_path = Path(path)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    return file_path


def file_exists(path: str | Path) -> bool:
    return Path(path).is_file()


def directory_exists(path: str | Path) -> bool:
    return Path(path).is_dir()


def list_files(
    directory: str | Path,
    *,
    pattern: str = "*",
    recursive: bool = False,
) -> list[Path]:
    root = Path(directory)

    if not root.is_dir():
        return []

    iterator = root.rglob(pattern) if recursive else root.glob(pattern)
    return sorted(path for path in iterator if path.is_file())


def relative_to_root(path: str | Path, root: str | Path) -> str:
    return str(Path(path).resolve().relative_to(Path(root).resolve()))


def safe_unlink(path: str | Path) -> bool:
    file_path = Path(path)

    if not file_path.exists():
        return False

    if not file_path.is_file():
        return False

    file_path.unlink()
    return True


__all__ = [
    "directory_exists",
    "ensure_directory",
    "ensure_parent_dir",
    "file_exists",
    "list_files",
    "read_json_file",
    "read_text_file",
    "read_yaml_file",
    "relative_to_root",
    "safe_unlink",
    "write_json_file",
    "write_text_file",
    "write_yaml_file",
]
