from pathlib import Path

import pytest
import yaml

from savo_lidar.utils.param_loader import (
    deep_merge_dicts,
    get_nested_value,
    load_merged_yaml,
    load_yaml_file,
    require_nested_value,
)


def test_load_yaml_file_reads_dictionary(tmp_path: Path):
    path = tmp_path / "config.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "lidar_driver_node": {
                    "ros__parameters": {
                        "backend": "dryrun",
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    data = load_yaml_file(path)

    assert data["lidar_driver_node"]["ros__parameters"]["backend"] == "dryrun"


def test_load_yaml_file_returns_empty_dict_for_empty_yaml(tmp_path: Path):
    path = tmp_path / "empty.yaml"
    path.write_text("", encoding="utf-8")

    assert load_yaml_file(path) == {}


def test_load_yaml_file_rejects_missing_file(tmp_path: Path):
    with pytest.raises(FileNotFoundError):
        load_yaml_file(tmp_path / "missing.yaml")


def test_load_yaml_file_rejects_non_dict_root(tmp_path: Path):
    path = tmp_path / "bad.yaml"
    path.write_text("- one\n- two\n", encoding="utf-8")

    with pytest.raises(ValueError):
        load_yaml_file(path)


def test_deep_merge_dicts_merges_nested_values():
    base = {
        "lidar_driver_node": {
            "ros__parameters": {
                "backend": "dryrun",
                "frame_id": "laser",
            }
        }
    }
    override = {
        "lidar_driver_node": {
            "ros__parameters": {
                "backend": "real",
            }
        }
    }

    merged = deep_merge_dicts(base, override)

    assert merged["lidar_driver_node"]["ros__parameters"]["backend"] == "real"
    assert merged["lidar_driver_node"]["ros__parameters"]["frame_id"] == "laser"


def test_load_merged_yaml_applies_later_files_as_override(tmp_path: Path):
    base = tmp_path / "base.yaml"
    override = tmp_path / "override.yaml"

    base.write_text(
        yaml.safe_dump(
            {
                "node": {
                    "ros__parameters": {
                        "backend": "dryrun",
                        "frame_id": "laser",
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    override.write_text(
        yaml.safe_dump(
            {
                "node": {
                    "ros__parameters": {
                        "backend": "real",
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    merged = load_merged_yaml(base, override)

    assert merged["node"]["ros__parameters"]["backend"] == "real"
    assert merged["node"]["ros__parameters"]["frame_id"] == "laser"


def test_get_nested_value_returns_value_from_dotted_key():
    data = {
        "lidar_driver_node": {
            "ros__parameters": {
                "backend": "dryrun",
            }
        }
    }

    assert (
        get_nested_value(
            data,
            "lidar_driver_node.ros__parameters.backend",
        )
        == "dryrun"
    )


def test_get_nested_value_returns_default_when_missing():
    assert get_nested_value({}, "missing.path", default="fallback") == "fallback"


def test_require_nested_value_returns_existing_value():
    data = {"a": {"b": {"c": 10}}}

    assert require_nested_value(data, "a.b.c") == 10


def test_require_nested_value_raises_when_missing():
    with pytest.raises(KeyError):
        require_nested_value({}, "a.b.c")