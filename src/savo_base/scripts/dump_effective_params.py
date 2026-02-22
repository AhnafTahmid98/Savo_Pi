#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/scripts/dump_effective_params.py
-------------------------------------------------------
Professional ROS 2 Jazzy CLI utility to dump the *effective runtime parameters*
from a target node (default: /base_driver_node).

Why this is useful
------------------
- Confirms what parameters are actually active after launch/YAML overrides
- Helps debugging when multiple YAML files are merged
- Creates reproducible snapshots for logs / issue reports
- Verifies base_driver_node safety + motor config on real hardware

Features
--------
- Waits for parameter services
- Lists all parameters from target node
- Fetches values in batches
- Outputs YAML (default) or JSON
- Can save to file
- Can optionally include hidden/internal parameters (names starting with "_")
- Can wrap output in ROS2-compatible YAML structure:
    /base_driver_node:
      ros__parameters:
        ...

Examples
--------
# Print effective params from base_driver_node (YAML)
ros2 run savo_base dump_effective_params.py

# Dump another node
ros2 run savo_base dump_effective_params.py --node /safety_stop_node

# Save to file (ROS2 YAML wrapper)
ros2 run savo_base dump_effective_params.py --out /tmp/base_driver_effective.yaml --ros2-yaml

# JSON output
ros2 run savo_base dump_effective_params.py --format json
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter_client import AsyncParameterClient
from rcl_interfaces.msg import ParameterType, ParameterValue


# -----------------------------------------------------------------------------
# YAML helper (optional dependency)
# -----------------------------------------------------------------------------
try:
    import yaml  # PyYAML
    _HAS_YAML = True
except Exception:
    yaml = None
    _HAS_YAML = False


# -----------------------------------------------------------------------------
# ParameterValue conversion
# -----------------------------------------------------------------------------
def parameter_value_to_python(pv: ParameterValue) -> Any:
    """Convert rcl_interfaces/ParameterValue into native Python value."""
    t = int(pv.type)

    if t == ParameterType.PARAMETER_NOT_SET:
        return None
    if t == ParameterType.PARAMETER_BOOL:
        return bool(pv.bool_value)
    if t == ParameterType.PARAMETER_INTEGER:
        return int(pv.integer_value)
    if t == ParameterType.PARAMETER_DOUBLE:
        return float(pv.double_value)
    if t == ParameterType.PARAMETER_STRING:
        return str(pv.string_value)
    if t == ParameterType.PARAMETER_BYTE_ARRAY:
        # bytes are safer as list[int] in dumps
        return [int(x) for x in pv.byte_array_value]
    if t == ParameterType.PARAMETER_BOOL_ARRAY:
        return [bool(x) for x in pv.bool_array_value]
    if t == ParameterType.PARAMETER_INTEGER_ARRAY:
        return [int(x) for x in pv.integer_array_value]
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return [float(x) for x in pv.double_array_value]
    if t == ParameterType.PARAMETER_STRING_ARRAY:
        return [str(x) for x in pv.string_array_value]

    # Unknown future type
    return None


# -----------------------------------------------------------------------------
# ROS dumper node
# -----------------------------------------------------------------------------
class EffectiveParamDumper(Node):
    def __init__(self, helper_node_name: str = "dump_effective_params_cli") -> None:
        super().__init__(helper_node_name)

    def wait_future(
        self,
        executor: SingleThreadedExecutor,
        future,
        timeout_s: float,
        *,
        what: str,
    ):
        t_end = time.monotonic() + max(0.0, float(timeout_s))
        while rclpy.ok() and not future.done():
            if time.monotonic() >= t_end:
                raise TimeoutError(f"Timeout while waiting for {what}")
            executor.spin_once(timeout_sec=0.05)

        if not future.done():
            raise TimeoutError(f"Future incomplete for {what}")

        exc = future.exception()
        if exc is not None:
            raise RuntimeError(f"{what} failed: {exc}")

        return future.result()

    def wait_for_param_services(
        self,
        client: AsyncParameterClient,
        executor: SingleThreadedExecutor,
        *,
        timeout_s: float,
    ) -> None:
        """Wait until target node parameter services are available."""
        t_end = time.monotonic() + max(0.0, float(timeout_s))
        self.get_logger().info(f"Waiting for parameter services (timeout={timeout_s:.1f}s)...")

        while rclpy.ok():
            # AsyncParameterClient.wait_for_services() is sync-ish in some ROS2 distros,
            # so support both styles robustly.
            try:
                ready = client.wait_for_services(timeout_sec=0.2)
                if ready:
                    self.get_logger().info("Parameter services available.")
                    return
            except TypeError:
                # Some versions use positional arg
                ready = client.wait_for_services(0.2)
                if ready:
                    self.get_logger().info("Parameter services available.")
                    return

            executor.spin_once(timeout_sec=0.05)

            if time.monotonic() >= t_end:
                raise TimeoutError("Timed out waiting for target node parameter services.")

    def dump_node_params(
        self,
        executor: SingleThreadedExecutor,
        *,
        target_node: str,
        timeout_s: float = 5.0,
        include_hidden: bool = False,
        batch_size: int = 64,
    ) -> Dict[str, Any]:
        """
        Fetch effective runtime parameters from target node.
        Returns a flat dict: {param_name: python_value}
        """
        client = AsyncParameterClient(self, target_node)
        self.wait_for_param_services(client, executor, timeout_s=timeout_s)

        # 1) List parameter names (depth big enough for nested namespaces)
        list_future = client.list_parameters([], depth=100)
        list_result = self.wait_future(
            executor, list_future, timeout_s, what=f"list_parameters({target_node})"
        )

        names = list(list_result.result.names)
        names.sort()

        if not include_hidden:
            names = [n for n in names if not n.startswith("_")]

        if not names:
            self.get_logger().warn(f"No parameters found on target node: {target_node}")
            return {}

        self.get_logger().info(f"Found {len(names)} parameter(s) on {target_node}")

        # 2) Fetch values in batches
        out: Dict[str, Any] = {}
        bs = max(1, int(batch_size))

        for i in range(0, len(names), bs):
            chunk = names[i : i + bs]
            fut = client.get_parameters(chunk)
            result = self.wait_future(
                executor, fut, timeout_s, what=f"get_parameters[{i}:{i+len(chunk)}]({target_node})"
            )

            # result is GetParameters.Response
            values = list(result.values)
            if len(values) != len(chunk):
                raise RuntimeError(
                    f"Parameter service returned {len(values)} values for {len(chunk)} names."
                )

            for name, pv in zip(chunk, values):
                out[name] = parameter_value_to_python(pv)

        return out


# -----------------------------------------------------------------------------
# Formatting
# -----------------------------------------------------------------------------
def build_ros2_yaml_wrapper(target_node: str, params: Dict[str, Any]) -> Dict[str, Any]:
    # Preserve exact node key user passed (e.g. "/base_driver_node")
    return {
        target_node: {
            "ros__parameters": params
        }
    }


def render_yaml(data: Any) -> str:
    if _HAS_YAML:
        return yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
    # Fallback: JSON-looking string if PyYAML is unavailable
    return json.dumps(data, indent=2, ensure_ascii=False)


def render_json(data: Any) -> str:
    return json.dumps(data, indent=2, ensure_ascii=False, sort_keys=False)


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="dump_effective_params.py",
        description="Dump effective runtime parameters from a ROS2 node (default: /base_driver_node).",
    )
    p.add_argument(
        "--node",
        default="/base_driver_node",
        help="Target node name (default: /base_driver_node)",
    )
    p.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Timeout in seconds for service wait / calls (default: 5.0)",
    )
    p.add_argument(
        "--batch-size",
        type=int,
        default=64,
        help="get_parameters batch size (default: 64)",
    )
    p.add_argument(
        "--include-hidden",
        action="store_true",
        help="Include hidden/internal params (names starting with '_')",
    )
    p.add_argument(
        "--format",
        choices=["yaml", "json"],
        default="yaml",
        help="Output format (default: yaml)",
    )
    p.add_argument(
        "--ros2-yaml",
        action="store_true",
        help="Wrap output as ROS2 YAML file structure: <node>: {ros__parameters: ...}",
    )
    p.add_argument(
        "--out",
        type=str,
        default="",
        help="Optional file path to save output (prints to stdout as well unless --quiet)",
    )
    p.add_argument(
        "--quiet",
        action="store_true",
        help="Do not print dump content to stdout (use with --out)",
    )
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    if args.batch_size <= 0:
        print("ERROR: --batch-size must be > 0", file=sys.stderr)
        return 2
    if args.timeout <= 0.0:
        print("ERROR: --timeout must be > 0", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node: Optional[EffectiveParamDumper] = None
    executor: Optional[SingleThreadedExecutor] = None

    try:
        node = EffectiveParamDumper()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        params = node.dump_node_params(
            executor,
            target_node=str(args.node),
            timeout_s=float(args.timeout),
            include_hidden=bool(args.include_hidden),
            batch_size=int(args.batch_size),
        )

        payload: Any = params
        if args.ros2_yaml:
            payload = build_ros2_yaml_wrapper(str(args.node), params)

        if args.format == "json":
            text = render_json(payload)
        else:
            text = render_yaml(payload)

        # Save file if requested
        if args.out:
            out_path = Path(args.out).expanduser()
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(text, encoding="utf-8")
            node.get_logger().info(f"Saved effective params to: {out_path}")

        if not args.quiet:
            print(text)

        return 0

    except KeyboardInterrupt:
        return 130
    except Exception as e:
        print(f"[dump_effective_params.py] ERROR: {e}", file=sys.stderr)
        return 1
    finally:
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception:
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())