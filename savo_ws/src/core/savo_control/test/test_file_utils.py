#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for shared file utilities."""

from __future__ import annotations

from pathlib import Path
import tempfile

from savo_control.utils import (
    directory_exists,
    ensure_directory,
    ensure_parent_dir,
    file_exists,
    list_files,
    read_json_file,
    read_text_file,
    read_yaml_file,
    relative_to_root,
    safe_unlink,
    write_json_file,
    write_text_file,
    write_yaml_file,
)


def test_write_and_read_text_file():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "logs" / "status.txt"

        written = write_text_file(path, "hello")

        assert written == path
        assert path.is_file()
        assert read_text_file(path) == "hello"


def test_write_text_file_creates_parent_directory():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "nested" / "dir" / "status.txt"

        write_text_file(path, "ok")

        assert path.is_file()
        assert path.parent.is_dir()
        assert read_text_file(path) == "ok"


def test_write_and_read_json_file():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "data" / "status.json"

        written = write_json_file(
            path,
            {
                "ok": True,
                "count": 3,
            },
        )

        assert written == path
        assert path.is_file()
        assert read_json_file(path) == {
            "ok": True,
            "count": 3,
        }


def test_write_json_file_adds_trailing_newline():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "status.json"

        write_json_file(path, {"ok": True})

        assert read_text_file(path).endswith("\n")


def test_write_and_read_yaml_file():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "config" / "params.yaml"

        written = write_yaml_file(
            path,
            {
                "rate": 30,
                "enabled": True,
            },
        )

        assert written == path
        assert path.is_file()
        assert read_yaml_file(path) == {
            "enabled": True,
            "rate": 30,
        }


def test_read_empty_yaml_file_returns_empty_dict():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "empty.yaml"
        path.write_text("", encoding="utf-8")

        assert read_yaml_file(path) == {}


def test_ensure_directory():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "logs" / "control"

        result = ensure_directory(path)

        assert result == path
        assert path.is_dir()
        assert directory_exists(path) is True


def test_ensure_parent_dir():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "logs" / "status.txt"

        result = ensure_parent_dir(path)

        assert result == path
        assert path.parent.is_dir()
        assert path.is_file() is False


def test_file_exists_and_directory_exists():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        file_path = root / "status.txt"
        dir_path = root / "logs"

        file_path.write_text("ok", encoding="utf-8")
        dir_path.mkdir()

        assert file_exists(file_path) is True
        assert file_exists(dir_path) is False
        assert file_exists(root / "missing.txt") is False

        assert directory_exists(dir_path) is True
        assert directory_exists(file_path) is False
        assert directory_exists(root / "missing") is False


def test_list_files_non_recursive():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)

        (root / "a.txt").write_text("a", encoding="utf-8")
        (root / "b.json").write_text("{}", encoding="utf-8")

        nested = root / "nested"
        nested.mkdir()
        (nested / "c.txt").write_text("c", encoding="utf-8")

        files = list_files(root, pattern="*.txt", recursive=False)

        assert files == [root / "a.txt"]


def test_list_files_recursive():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)

        (root / "a.txt").write_text("a", encoding="utf-8")
        (root / "b.json").write_text("{}", encoding="utf-8")

        nested = root / "nested"
        nested.mkdir()
        (nested / "c.txt").write_text("c", encoding="utf-8")

        files = list_files(root, pattern="*.txt", recursive=True)

        assert files == [
            root / "a.txt",
            nested / "c.txt",
        ]


def test_list_files_missing_directory_returns_empty_list():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)

        assert list_files(root / "missing", pattern="*.txt") == []


def test_relative_to_root():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        path = root / "logs" / "status.txt"

        path.parent.mkdir()
        path.write_text("ok", encoding="utf-8")

        assert relative_to_root(path, root) == "logs/status.txt"


def test_safe_unlink_removes_file():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "status.txt"
        path.write_text("ok", encoding="utf-8")

        assert safe_unlink(path) is True
        assert path.exists() is False
        assert safe_unlink(path) is False


def test_safe_unlink_rejects_directory():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "logs"
        path.mkdir()

        assert safe_unlink(path) is False
        assert path.is_dir()
