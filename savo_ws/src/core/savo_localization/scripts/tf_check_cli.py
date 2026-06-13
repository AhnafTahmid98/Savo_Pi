#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Check Robot Savo localization TF edges."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from typing import Any

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class TfEdgeResult:
    name: str
    parent_frame: str
    child_frame: str
    available: bool
    ok: bool
    message: str
    age_s: float | None = None
    translation_xyz: tuple[float, float, float] | None = None
    rotation_xyzw: tuple[float, float, float, float] | None = None


class TfCheckNode(Node):
    def __init__(self) -> None:
        super().__init__("tf_check_cli")

        self.buffer = Buffer()
        self.listener = TransformListener(
            self.buffer,
            self,
            spin_thread=False,
        )

    def check_edge(
        self,
        *,
        name: str,
        parent_frame: str,
        child_frame: str,
        timeout_s: float,
        max_age_s: float,
    ) -> TfEdgeResult:
        parent = normalize_frame_id(parent_frame)
        child = normalize_frame_id(child_frame)

        if not parent or not child:
            return TfEdgeResult(
                name=name,
                parent_frame=parent,
                child_frame=child,
                available=False,
                ok=False,
                message="frame id cannot be empty",
            )

        if parent == child:
            return TfEdgeResult(
                name=name,
                parent_frame=parent,
                child_frame=child,
                available=False,
                ok=False,
                message="parent and child frames cannot be the same",
            )

        try:
            transform = self.buffer.lookup_transform(
                parent,
                child,
                rclpy.time.Time(),
                timeout=Duration(seconds=timeout_s),
            )
        except TransformException as exc:
            return TfEdgeResult(
                name=name,
                parent_frame=parent,
                child_frame=child,
                available=False,
                ok=False,
                message=str(exc),
            )

        stamp = transform.header.stamp
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        now_s = float(self.get_clock().now().nanoseconds) * 1e-9
        age_s = max(0.0, now_s - stamp_s) if stamp_s > 0.0 else None

        ok = age_s is None or age_s <= max_age_s
        message = "TF healthy" if ok else f"TF stale: age_s={age_s:.3f}"

        t = transform.transform.translation
        q = transform.transform.rotation

        return TfEdgeResult(
            name=name,
            parent_frame=parent,
            child_frame=child,
            available=True,
            ok=ok,
            message=message,
            age_s=age_s,
            translation_xyz=(float(t.x), float(t.y), float(t.z)),
            rotation_xyzw=(float(q.x), float(q.y), float(q.z), float(q.w)),
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check Robot Savo localization TF transforms."
    )
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--imu-frame", default="imu_link")

    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--max-age", type=float, default=0.5)
    parser.add_argument("--warmup", type=float, default=0.5)

    parser.add_argument("--check-map-odom", action="store_true")
    parser.add_argument("--no-odom-base", action="store_true")
    parser.add_argument("--no-base-imu", action="store_true")

    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    if args.timeout <= 0.0:
        raise SystemExit("--timeout must be > 0.0")

    if args.max_age <= 0.0:
        raise SystemExit("--max-age must be > 0.0")

    if args.warmup < 0.0:
        raise SystemExit("--warmup must be >= 0.0")

    rclpy.init()
    node = TfCheckNode()

    try:
        warmup_tf_buffer(node, duration_s=args.warmup)

        results: list[TfEdgeResult] = []

        if args.check_map_odom:
            results.append(
                node.check_edge(
                    name="map_to_odom",
                    parent_frame=args.map_frame,
                    child_frame=args.odom_frame,
                    timeout_s=args.timeout,
                    max_age_s=args.max_age,
                )
            )

        if not args.no_odom_base:
            results.append(
                node.check_edge(
                    name="odom_to_base",
                    parent_frame=args.odom_frame,
                    child_frame=args.base_frame,
                    timeout_s=args.timeout,
                    max_age_s=args.max_age,
                )
            )

        if not args.no_base_imu:
            results.append(
                node.check_edge(
                    name="base_to_imu",
                    parent_frame=args.base_frame,
                    child_frame=args.imu_frame,
                    timeout_s=args.timeout,
                    max_age_s=args.max_age,
                )
            )

        report = build_report(
            results=results,
            timeout_s=args.timeout,
            max_age_s=args.max_age,
        )

        if args.json:
            print(json.dumps(report, indent=2, sort_keys=True))
        else:
            print_text_report(report)

        return 0 if report["ok"] else 1

    finally:
        node.destroy_node()
        rclpy.shutdown()


def warmup_tf_buffer(node: TfCheckNode, *, duration_s: float) -> None:
    deadline_s = time.monotonic() + duration_s

    while rclpy.ok() and time.monotonic() < deadline_s:
        rclpy.spin_once(node, timeout_sec=0.05)


def build_report(
    *,
    results: list[TfEdgeResult],
    timeout_s: float,
    max_age_s: float,
) -> dict[str, Any]:
    reasons: list[str] = []

    if not results:
        reasons.append("no TF edges selected for checking")

    for result in results:
        if result.ok:
            continue

        reasons.append(
            f"{result.name}: {result.parent_frame} -> {result.child_frame}: "
            f"{result.message}"
        )

    ok = not reasons

    return {
        "ok": ok,
        "status": "OK" if ok else "ERROR",
        "message": "TF checks passed" if ok else "TF checks failed",
        "reasons": reasons,
        "timeout_s": timeout_s,
        "max_age_s": max_age_s,
        "checked_count": len(results),
        "healthy_count": sum(1 for result in results if result.ok),
        "available_count": sum(1 for result in results if result.available),
        "results": {
            result.name: asdict(result)
            for result in results
        },
    }


def print_text_report(report: dict[str, Any]) -> None:
    print("Robot Savo TF Check")
    print("===================")
    print(f"Status: {report['status']}")
    print(f"Message: {report['message']}")
    print(f"Checked: {report['checked_count']}")
    print(f"Available: {report['available_count']}")
    print(f"Healthy: {report['healthy_count']}")
    print(f"Timeout: {report['timeout_s']:.3f}s")
    print(f"Max age: {report['max_age_s']:.3f}s")

    results = report.get("results", {})
    if isinstance(results, dict) and results:
        print("")
        print("Edges:")
        for name, result in results.items():
            if not isinstance(result, dict):
                continue

            parent = result.get("parent_frame", "")
            child = result.get("child_frame", "")
            available = result.get("available", False)
            ok = result.get("ok", False)
            age_s = result.get("age_s", None)

            age_text = "unknown" if age_s is None else f"{float(age_s):.3f}s"

            print(
                "  "
                f"{name}: {parent} -> {child} | "
                f"available={available} | "
                f"ok={ok} | "
                f"age={age_text}"
            )

            translation = result.get("translation_xyz")
            if isinstance(translation, list | tuple) and len(translation) == 3:
                print(
                    "    translation: "
                    f"x={float(translation[0]):.4f} "
                    f"y={float(translation[1]):.4f} "
                    f"z={float(translation[2]):.4f}"
                )

            print(f"    message: {result.get('message', '')}")

    if report["reasons"]:
        print("")
        print("Reasons:")
        for reason in report["reasons"]:
            print(f"  - {reason}")

    print("")
    print(f"Result: {'PASS' if report['ok'] else 'FAIL'}")


def normalize_frame_id(frame_id: str) -> str:
    return str(frame_id).strip().lstrip("/")


if __name__ == "__main__":
    sys.exit(main())