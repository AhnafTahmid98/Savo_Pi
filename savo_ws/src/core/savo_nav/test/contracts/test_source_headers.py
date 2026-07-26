# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]

SOURCE_SUFFIXES = {
    '.cpp',
    '.hpp',
    '.py',
}

COPYRIGHT_TEXT = 'Copyright 2026 Ahnaf Tahmid'

LICENSE_TEXT = (
    'SPDX-License-Identifier: LicenseRef-Proprietary'
)


def project_source_files():
    return sorted(
        path
        for path in ROOT.rglob('*')
        if (
            path.is_file()
            and path.suffix in SOURCE_SUFFIXES
        )
    )


def test_project_sources_have_proprietary_header():
    files = project_source_files()

    assert files

    for path in files:
        lines = path.read_text(
            encoding='utf-8',
        ).splitlines()

        assert len(lines) >= 2, str(path)
        assert COPYRIGHT_TEXT in lines[0], str(path)
        assert LICENSE_TEXT in lines[1], str(path)
