"""Compatibility alias for robot_bringup.launch.py."""

import importlib.util
from pathlib import Path


def generate_launch_description():
    """Delegate the legacy full-stack alias to the canonical launch."""
    target = Path(__file__).with_name("robot_bringup.launch.py")
    spec = importlib.util.spec_from_file_location("savo_robot_bringup", target)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load full bringup launch: {target}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()
