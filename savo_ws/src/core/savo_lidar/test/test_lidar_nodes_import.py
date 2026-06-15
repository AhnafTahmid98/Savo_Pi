# -*- coding: utf-8 -*-

import importlib


NODE_MODULES = [
    "savo_lidar.nodes.lidar_py_driver_node",
    "savo_lidar.nodes.lidar_filter_node",
    "savo_lidar.nodes.lidar_watchdog_node",
    "savo_lidar.nodes.lidar_health_node",
    "savo_lidar.nodes.lidar_state_publisher_node",
]


def test_node_modules_import_without_side_effects():
    for module_name in NODE_MODULES:
        module = importlib.import_module(module_name)

        assert module is not None


def test_lidar_py_driver_node_exports_main():
    module = importlib.import_module("savo_lidar.nodes.lidar_py_driver_node")

    assert hasattr(module, "main")
    assert callable(module.main)
    assert hasattr(module, "LidarDriverNode")


def test_lidar_filter_node_exports_main():
    module = importlib.import_module("savo_lidar.nodes.lidar_filter_node")

    assert hasattr(module, "main")
    assert callable(module.main)
    assert hasattr(module, "LidarFilterNode")


def test_lidar_watchdog_node_exports_main():
    module = importlib.import_module("savo_lidar.nodes.lidar_watchdog_node")

    assert hasattr(module, "main")
    assert callable(module.main)
    assert hasattr(module, "LidarWatchdogNode")


def test_lidar_health_node_exports_main():
    module = importlib.import_module("savo_lidar.nodes.lidar_health_node")

    assert hasattr(module, "main")
    assert callable(module.main)
    assert hasattr(module, "LidarHealthNode")


def test_lidar_state_publisher_node_exports_main():
    module = importlib.import_module("savo_lidar.nodes.lidar_state_publisher_node")

    assert hasattr(module, "main")
    assert callable(module.main)
    assert hasattr(module, "LidarStatePublisherNode")
