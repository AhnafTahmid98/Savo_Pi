#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Dump effective parameters from running LiDAR ROS nodes."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient

from savo_lidar.utils.logging import node_start_message


try:
    import yaml

    _HAS_YAML = True
except Exception:
    yaml = None
    _HAS_YAML = False


DEFAULT_NODES = (
    "/lidar_driver_node",
    "/lidar_filter_node",
    "/lidar_watchdog_node",
    "/lidar_health_node",
    "/lidar_state_publisher_node",
)


def parameter_value_to_python(value: ParameterValue) -> Any:
    param_type = int(value.type)

    if param_type == ParameterType.PARAMETER_NOT_SET:
        return None

    if param_type == ParameterType.PARAMETER_BOOL:
        return bool(value.bool_value)

    if param_type == ParameterType.PARAMETER_INTEGER:
        return int(value.integer_value)

    if param_type == ParameterType.PARAMETER_DOUBLE:
        return float(value.double_value)

    if param_type == ParameterType.PARAMETER_STRING:
        return str(value.string_value)

    if param_type == ParameterType.PARAMETER_BYTE_ARRAY:
        return [int(item) for item in value.byte_array_value]

    if param_type == ParameterType.PARAMETER_BOOL_ARRAY:
        return [bool(item) for item in value.bool_array_value]

    if param_type == ParameterType.PARAMETER_INTEGER_ARRAY:
        return [int(item) for item in value.integer_array_value]

    if param_type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return [float(item) for item in value.double_array_value]

    if param_type == ParameterType.PARAMETER_STRING_ARRAY:
        return [str(item) for item in value.string_array_value]

    return None


class EffectiveParamDumper(Node):
    def __init__(self) -> None:
        super().__init__("savo_lidar_dump_effective_params")

    def wait_future(
        self,
        executor: SingleThreadedExecutor,
        future,
        *,
        timeout_s: float,
        label: str,
    ):
        end_s = time.monotonic() + max(0.0, float(timeout_s))

        while rclpy.ok() and not future.done():
            if time.monotonic() >= end_s:
                raise TimeoutError(f"timeout while waiting for {label}")

            executor.spin_once(timeout_sec=0.05)

        if not future.done():
            raise TimeoutError(f"future incomplete for {label}")

        exc = future.exception()
        if exc is not None:
            raise RuntimeError(f"{label} failed: {exc}")

        return future.result()

    def wait_for_services(
        self,
        client: AsyncParameterClient,
        executor: SingleThreadedExecutor,
        *,
        target_node: str,
        timeout_s: float,
    ) -> bool:
        end_s = time.monotonic() + max(0.0, float(timeout_s))

        while rclpy.ok():
            try:
                if client.wait_for_services(timeout_sec=0.2):
                    return True
            except TypeError:
                if client.wait_for_services(0.2):
                    return True

            executor.spin_once(timeout_sec=0.05)

            if time.monotonic() >= end_s:
                self.get_logger().warning(
                    f"parameter services not available for {target_node}"
                )
                return False

        return False

    def dump_node_params(
        self,
        executor: SingleThreadedExecutor,
        *,
        target_node: str,
        timeout_s: float,
        include_hidden: bool,
        batch_size: int,
    ) -> dict[str, Any] | None:
        client = AsyncParameterClient(self, target_node)

        if not self.wait_for_services(
            client,
            executor,
            target_node=target_node,
            timeout_s=timeout_s,
        ):
            return None

        list_future = client.list_parameters([], depth=100)
        list_result = self.wait_future(
            executor,
            list_future,
            timeout_s=timeout_s,
            label=f"list_parameters({target_node})",
        )

        names = sorted(list(list_result.result.names))

        if not include_hidden:
            names = [name for name in names if not name.startswith("_")]

        if not names:
            return {}

        output: dict[str, Any] = {}
        batch_size = max(1, int(batch_size))

        for start in range(0, len(names), batch_size):
            chunk = names[start : start + batch_size]
            get_future = client.get_parameters(chunk)
            get_result = self.wait_future(
                executor,
                get_future,
                timeout_s=timeout_s,
                label=f"get_parameters({target_node})",
            )

            values = list(get_result.values)

            if len(values) != len(chunk):
                raise RuntimeError(
                    f"{target_node} returned {len(values)} values for {len(chunk)} names"
                )

            for name, value in zip(chunk, values):
                output[name] = parameter_value_to_python(value)

        return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="dump_effective_params.py",
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
    parser.add_argument(
        "--include-hidden",
        action="store_true",
        help="Include hidden parameters.",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=64,
        help="Number of parameters to fetch per request.",
    )
    parser.add_argument(
        "--format",
        choices=("text", "json", "yaml"),
        default="text",
        help="Output format.",
    )
    parser.add_argument(
        "--ros2-yaml",
        action="store_true",
        help="Wrap output in ROS 2 parameter YAML format.",
    )
    parser.add_argument(
        "--out",
        type=str,
        default="",
        help="Optional output file.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    rclpy.init()
    node = EffectiveParamDumper()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info(
            node_start_message(
                "dump_effective_params",
                nodes=",".join(args.nodes),
                timeout_s=args.timeout,
            )
        )

        results: dict[str, Any] = {}
        found_any = False

        for target_node in args.nodes:
            params = node.dump_node_params(
                executor,
                target_node=str(target_node),
                timeout_s=float(args.timeout),
                include_hidden=bool(args.include_hidden),
                batch_size=int(args.batch_size),
            )

            if params is None:
                results[str(target_node)] = {
                    "available": False,
                    "parameters": {},
                }
                continue

            found_any = True
            results[str(target_node)] = {
                "available": True,
                "parameters": params,
            }

        rendered = render_output(
            results,
            output_format=str(args.format),
            ros2_yaml=bool(args.ros2_yaml),
        )

        if args.out:
            Path(args.out).write_text(rendered + "\n", encoding="utf-8")
        else:
            print(rendered)

        return 0 if found_any else 1

    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


def render_output(
    results: dict[str, Any],
    *,
    output_format: str,
    ros2_yaml: bool,
) -> str:
    if ros2_yaml:
        wrapped = {
            node_name: {
                "ros__parameters": data.get("parameters", {})
            }
            for node_name, data in results.items()
            if data.get("available", False)
        }
        return render_yaml(wrapped)

    if output_format == "json":
        return json.dumps(results, indent=2, sort_keys=False)

    if output_format == "yaml":
        return render_yaml(results)

    return render_text(results)


def render_text(results: dict[str, Any]) -> str:
    lines: list[str] = []

    for node_name, data in results.items():
        lines.append(f"\n[{node_name}]")

        if not data.get("available", False):
            lines.append("  parameter service not available")
            continue

        params = data.get("parameters", {})
        if not params:
            lines.append("  no parameters found")
            continue

        for name, value in params.items():
            lines.append(f"  {name}: {value}")

    return "\n".join(lines).lstrip()


def render_yaml(data: Any) -> str:
    if _HAS_YAML:
        return yaml.safe_dump(data, sort_keys=False, allow_unicode=True)

    return json.dumps(data, indent=2, sort_keys=False)


if __name__ == "__main__":
    sys.exit(main())
