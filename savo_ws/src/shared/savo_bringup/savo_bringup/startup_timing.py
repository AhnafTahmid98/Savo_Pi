"""Canonical absolute TimerAction offsets for Core production startup.

These delays stagger process startup only; they do not establish readiness or
authorize motion. Edge startup has its own independent timing defaults.
"""

CORE_START_DELAYS = {
    "description": "0.0",
    "base": "5.0",
    "lidar": "10.0",
    "perception": "15.0",
    "control": "20.0",
    "localization": "30.0",
    "power": "35.0",
    "head": "40.0",
    "supervisor": "45.0",
    "location_lifecycle": "50.0",
    "navigation": "55.0",
    "mapping": "60.0",
}
