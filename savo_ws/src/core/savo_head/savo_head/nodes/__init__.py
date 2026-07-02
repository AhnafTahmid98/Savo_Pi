# -*- coding: utf-8 -*-

"""Python fallback ROS nodes for Robot Savo active head."""

from __future__ import annotations

from importlib import import_module
from typing import Callable, Final


HEAD_CONTROLLER_NODE: Final[str] = "head_controller_node"
HEAD_SCAN_NODE: Final[str] = "head_scan_node"
HEAD_TF_NODE: Final[str] = "head_tf_node"
HEAD_STATUS_NODE: Final[str] = "head_status_node"
APRILTAG_CONFIRM_NODE: Final[str] = "apriltag_confirm_node"

FALLBACK_NODE_MODULES: Final[dict[str, str]] = {
    HEAD_CONTROLLER_NODE: "savo_head.nodes.head_controller_node",
    HEAD_SCAN_NODE: "savo_head.nodes.head_scan_node",
    HEAD_TF_NODE: "savo_head.nodes.head_tf_node",
    HEAD_STATUS_NODE: "savo_head.nodes.head_status_node",
    APRILTAG_CONFIRM_NODE: "savo_head.nodes.apriltag_confirm_node",
}

FALLBACK_NODE_EXECUTABLES: Final[dict[str, str]] = {
    HEAD_CONTROLLER_NODE: "head_controller_node_py",
    HEAD_SCAN_NODE: "head_scan_node_py",
    HEAD_TF_NODE: "head_tf_node_py",
    HEAD_STATUS_NODE: "head_status_node_py",
    APRILTAG_CONFIRM_NODE: "apriltag_confirm_node_py",
}


def fallback_node_names() -> tuple[str, ...]:
    return tuple(FALLBACK_NODE_MODULES.keys())


def fallback_node_executables() -> tuple[str, ...]:
    return tuple(FALLBACK_NODE_EXECUTABLES.values())


def get_node_module_name(node_name: str) -> str:
    key = str(node_name).strip()
    if key not in FALLBACK_NODE_MODULES:
        valid = ", ".join(fallback_node_names())
        raise KeyError(f"unknown fallback node {node_name!r}; valid: {valid}")
    return FALLBACK_NODE_MODULES[key]


def get_node_executable_name(node_name: str) -> str:
    key = str(node_name).strip()
    if key not in FALLBACK_NODE_EXECUTABLES:
        valid = ", ".join(fallback_node_names())
        raise KeyError(f"unknown fallback node {node_name!r}; valid: {valid}")
    return FALLBACK_NODE_EXECUTABLES[key]


def load_node_main(node_name: str) -> Callable:
    module = import_module(get_node_module_name(node_name))
    main = getattr(module, "main", None)

    if not callable(main):
        raise RuntimeError(f"fallback node {node_name!r} does not expose callable main()")

    return main


__all__ = [
    "HEAD_CONTROLLER_NODE",
    "HEAD_SCAN_NODE",
    "HEAD_TF_NODE",
    "HEAD_STATUS_NODE",
    "APRILTAG_CONFIRM_NODE",
    "FALLBACK_NODE_MODULES",
    "FALLBACK_NODE_EXECUTABLES",
    "fallback_node_names",
    "fallback_node_executables",
    "get_node_module_name",
    "get_node_executable_name",
    "load_node_main",
]
