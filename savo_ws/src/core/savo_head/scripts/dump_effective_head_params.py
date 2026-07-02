#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import sys

_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_ROOT))

from savo_head.tools.dump_effective_head_params import main


if __name__ == "__main__":
    raise SystemExit(main())
