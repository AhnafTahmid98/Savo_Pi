#!/usr/bin/env python3
"""Create the immutable pre-real-test zero-byte source inventory.

Generated build/install/log trees and tool-owned worktrees are deliberately
excluded: they are outputs or repository copies, not authoritative sources.
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path


ROOTS = ("deploy", "docs", "savo_ws/src", "tools")

REQUIRED_DOCS = {
    "docs/testing/full_robot_test_plan.md",
    "docs/testing/base_test_plan.md",
    "docs/testing/perception_test_plan.md",
    "docs/testing/localization_test_plan.md",
    "docs/testing/speech_test_plan.md",
    "docs/testing/vo_test_plan.md",
    "docs/setup/ethernet_core_edge_setup.md",
    "docs/setup/time_sync.md",
    "docs/setup/audio_setup.md",
    "docs/setup/realsense_setup.md",
    "docs/architecture/network_architecture.md",
    "docs/architecture/savo_core_architecture.md",
    "docs/architecture/speech_intent_flow.md",
}

REQUIRED_DIAGNOSTICS = {
    "tools/diag/infra/diag_utils.py",
    "tools/diag/infra/run_all.sh",
    "tools/diag/infra/tf_tree_check.py",
    "tools/diag/safety/estop_test.py",
    "tools/diag/safety/safety_stop_test.py",
    "tools/diag/safety/cmd_vel_gate_test.py",
    "tools/diag/motion/odom_test.py",
    "tools/diag/motion/odom_calibration.py",
    "tools/diag/sensors/apriltag_test.py",
    "tools/diag/ui/head_pan_tilt_test.py",
    "tools/diag/ui/screen_ui_test.py",
    "tools/diag/voice/audio_mic_test.py",
    "tools/diag/voice/audio_speaker_test.py",
    "tools/diag/voice/asr_topic_test.py",
    "tools/diag/voice/tts_topic_test.py",
    "tools/diag/power/current_draw_logger.py",
}

REQUIRED_DEPLOYMENT = {
    "deploy/network/core_netplan.yaml",
    "deploy/network/edge_netplan.yaml",
    "deploy/network/chrony_core.conf",
    "deploy/network/chrony_edge.conf",
    "deploy/systemd/savo.service",
    "deploy/systemd/savo_mapping.service",
    "savo_ws/src/edge/savo_ui/systemd/savo-ui.service",
}


def package_for(path: str) -> str:
    parts = Path(path).parts
    for index, part in enumerate(parts):
        if part.startswith("savo_") and index >= 2:
            return part
    return parts[0]


def classify(path: str) -> tuple[str, str]:
    p = Path(path)
    if p.name == ".gitkeep" or "resource" in p.parts:
        return "KEEP_INTENTIONALLY_EMPTY", "ament/resource marker or directory marker"
    if p.name == "__init__.py":
        return "KEEP_INTENTIONALLY_EMPTY", "namespace-only Python package marker"
    if path in REQUIRED_DOCS:
        return "IMPLEMENT", "required pre-real-test operator documentation"
    if path in REQUIRED_DIAGNOSTICS:
        return "IMPLEMENT", "required safe diagnostic implementation"
    if path in REQUIRED_DEPLOYMENT:
        return "IMPLEMENT", "required deployment/runtime configuration"
    if "/savo_ui/include/savo_ui/" in path or "/savo_ui/src/" in path:
        return "REMOVE_AFTER_REFERENCE_PROOF", "inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate"
    if path.startswith("savo_ws/src/core/savo_mapping/"):
        return "REMOVE_AFTER_REFERENCE_PROOF", "legacy zero-byte mapping path outside active runtime targets"
    if path.startswith("savo_ws/src/shared/savo_perception/test/test_"):
        return "REMOVE_AFTER_REFERENCE_PROOF", "obsolete empty test superseded by active unit/contract tests"
    if path.startswith("savo_ws/src/shared/savo_bringup/config/") or path.startswith(
        "savo_ws/src/shared/savo_bringup/params/"
    ):
        return "REMOVE_AFTER_REFERENCE_PROOF", "legacy empty overlay superseded by package-owned configuration"
    if path.startswith("savo_ws/src/core/savo_localization/savo_localization/"):
        return "REPLACE", "empty compatibility implementation replaced by C++ health path and observer visualization"
    return "DEFER_WITH_REASON", "non-runtime placeholder not required for pre-real-test closure"


def flags(path: str, category: str) -> dict[str, bool]:
    suffix = Path(path).suffix
    runtime_type = suffix in {".cpp", ".hpp", ".py", ".sh", ".yaml", ".xml", ".service", ".conf"}
    is_ui_scaffold = "/savo_ui/include/savo_ui/" in path or "/savo_ui/src/" in path
    is_localization = "savo_localization/savo_localization/" in path
    installed = (
        is_ui_scaffold
        or path.startswith("savo_ws/src/core/savo_mapping/config/")
        or path.startswith("savo_ws/src/core/savo_mapping/launch/")
        or is_localization
    )
    launched = is_localization and "/nodes/" in path
    referenced = installed or launched or path in REQUIRED_DEPLOYMENT
    return {
        "installed": installed,
        "compiled": False,
        "launched": launched,
        "referenced": referenced,
        "tested": category == "REMOVE_AFTER_REFERENCE_PROOF" and "/test/" in path,
        "intentional_marker": category == "KEEP_INTENTIONALLY_EMPTY",
        "legacy_duplicate": category in {"REMOVE_AFTER_REFERENCE_PROOF", "REPLACE"},
        "required_implementation": category == "IMPLEMENT",
        "safe_deletion_candidate": category == "REMOVE_AFTER_REFERENCE_PROOF",
        "unknown": False,
        "runtime_type": runtime_type,
    }


def collect(repo: Path) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    for root_name in ROOTS:
        root = repo / root_name
        if not root.exists():
            continue
        for item in sorted(root.rglob("*")):
            if not item.is_file() or item.stat().st_size != 0:
                continue
            path = item.relative_to(repo).as_posix()
            category, reason = classify(path)
            records.append(
                {
                    "package": package_for(path),
                    "path": path,
                    "file_type": item.suffix.lstrip(".") or "marker",
                    "category": category,
                    **flags(path, category),
                    "reason": reason,
                }
            )
    return records


def markdown(records: list[dict[str, object]]) -> str:
    counts = Counter(str(record["category"]) for record in records)
    lines = [
        "# Pre-real-test zero-byte inventory",
        "",
        "This is the immutable RT-0 baseline captured before implementation. It scans",
        "authoritative `deploy`, `docs`, `savo_ws/src`, and `tools` trees. Generated",
        "`build`, `install`, `log`, `.git`, cache, attachment, and tool-owned worktree",
        "copies are intentionally excluded.",
        "",
        f"Total: **{len(records)}**",
        "",
        "| Category | Count |",
        "| --- | ---: |",
    ]
    for category in (
        "KEEP_INTENTIONALLY_EMPTY",
        "IMPLEMENT",
        "REPLACE",
        "REMOVE_AFTER_REFERENCE_PROOF",
        "DEFER_WITH_REASON",
    ):
        lines.append(f"| {category} | {counts[category]} |")
    lines.extend(
        [
            "",
            "| Path | Package | Type | Category | Installed | Launched | Reason |",
            "| --- | --- | --- | --- | --- | --- | --- |",
        ]
    )
    for record in records:
        lines.append(
            "| `{path}` | {package} | {file_type} | {category} | {installed} | "
            "{launched} | {reason} |".format(**record)
        )
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    repo = args.repo.resolve()
    output = (args.output_dir or repo / "docs/audits").resolve()
    output.mkdir(parents=True, exist_ok=True)
    records = collect(repo)
    payload = {
        "schema_version": 1,
        "audit": "pre_real_test_zero_byte_inventory",
        "scope": list(ROOTS),
        "excluded": [".git", "build", "install", "log", "caches", "tool-owned worktrees"],
        "total": len(records),
        "files": records,
    }
    (output / "pre_real_test_zero_byte_inventory.yaml").write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )
    (output / "pre_real_test_zero_byte_inventory.md").write_text(
        markdown(records), encoding="utf-8"
    )
    print(f"captured {len(records)} authoritative zero-byte files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
