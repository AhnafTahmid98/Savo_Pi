#!/usr/bin/env python3
"""Print parameters from running Robot Savo LiDAR nodes."""

from __future__ import annotations

import argparse
import sys

import rclpy
from rcl_interfaces.srv import ListParameters

from savo_lidar.utils.logging import node_start_message


DEFAULT_NODES = (
    "/lidar_driver_node",
    "/lidar_filter_node",
    "/lidar_watchdog_node",
    "/lidar_health_node",
    "/lidar_state_publisher_node",
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Dump effective parameters from running savo_lidar nodes.",
    )
    parser.add_argument(
        "--nodes",
        nargs="*",
        default=list(DEFAULT_NODES),
        help="Node names to inspect.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="Timeout in seconds for each parameter service.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    rclpy.init()
    node = rclpy.create_node("savo_lidar_dump_effective_params")

    try:
        node.get_logger().info(
            node_start_message(
                "dump_effective_params",
                nodes=",".join(args.nodes),
                timeout_s=args.timeout,
            )
        )

        any_found = False

        for target_node in args.nodes:
            found = _dump_node_params(
                node=node,
                target_node=str(target_node),
                timeout_s=float(args.timeout),
            )
            any_found = any_found or found

        return 0 if any_found else 1

    finally:
        node.destroy_node()
        rclpy.shutdown()


def _dump_node_params(*, node, target_node: str, timeout_s: float) -> bool:
    service_name = f"{target_node}/list_parameters"
    client = node.create_client(ListParameters, service_name)

    if not client.wait_for_service(timeout_sec=timeout_s):
        print(f"\n[{target_node}] parameter service not available")
        return False

    request = ListParameters.Request()
    request.prefixes = []
    request.depth = 0

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)

    if not future.done() or future.result() is None:
        print(f"\n[{target_node}] parameter request timed out")
        return False

    names = sorted(future.result().result.names)

    print(f"\n[{target_node}]")
    if not names:
        print("  no parameters found")
        return True

    for name in names:
        value = node.get_parameter_or(name, None)
        # The helper node cannot directly read another node's values through
        # get_parameter_or; this script intentionally lists names only for now.
        _ = value
        print(f"  {name}")

    return True


if __name__ == "__main__":
    sys.exit(main())