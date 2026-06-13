#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Public exports for savo_base.ros: QoS profiles, params, adapters, topic contract.

from .adapters import *          # noqa: F401,F403
from .params import *            # noqa: F401,F403
from .qos_profiles import *      # noqa: F401,F403
from .topic_contract import *    # noqa: F401,F403

# Build a combined __all__ from submodules (safe fallback if any submodule
# does not define __all__).
__all__ = []

try:
    from .adapters import __all__ as _adapters_all
    __all__.extend(_adapters_all)
except Exception:
    pass

try:
    from .params import __all__ as _params_all
    __all__.extend(_params_all)
except Exception:
    pass

try:
    from .qos_profiles import __all__ as _qos_all
    __all__.extend(_qos_all)
except Exception:
    pass

try:
    from .topic_contract import __all__ as _topic_contract_all
    __all__.extend(_topic_contract_all)
except Exception:
    pass

# Remove duplicates while preserving order
_seen = set()
__all__ = [x for x in __all__ if not (x in _seen or _seen.add(x))]
del _seen

# Clean temporary names (if present)
for _tmp in (
    "_adapters_all",
    "_params_all",
    "_qos_all",
    "_topic_contract_all",
):
    if _tmp in globals():
        del globals()[_tmp]
del _tmp
