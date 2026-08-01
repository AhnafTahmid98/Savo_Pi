"""Verify RViz migration accounting and immutable imports."""

import hashlib
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_SRC = ROOT.parents[1]


def _digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_every_legacy_rviz_asset_is_accounted_for():
    manifest = yaml.safe_load(
        (ROOT / 'config/migration_manifest.yaml').read_text(encoding='utf-8')
    )['migrations']
    assert len(manifest) == 10
    assert len({entry['original'] for entry in manifest}) == 10
    for entry in manifest:
        destination = REPOSITORY_SRC / 'shared' / entry['destination']
        assert destination.is_file(), destination
        if entry['status'] == 'moved_unchanged':
            assert _digest(destination) == entry['original_sha256']
            assert _digest(destination) == entry['current_sha256']
        if entry['status'] == 'moved_and_corrected':
            assert _digest(destination) == entry['current_sha256']
            assert entry.get('correction')


def test_old_rviz_paths_are_compatibility_symlinks_only():
    legacy_directories = (
        REPOSITORY_SRC / 'shared/savo_description/rviz',
        REPOSITORY_SRC / 'core/savo_mapping/rviz',
    )
    for directory in legacy_directories:
        for path in directory.glob('*.rviz'):
            assert path.is_symlink(), path
            assert 'savo_observer/rviz' in str(path.readlink()), path
