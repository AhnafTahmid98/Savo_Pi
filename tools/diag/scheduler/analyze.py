#!/usr/bin/env python3
"""Offline, conservative correlation of trace-cmd report -t with ROS timing JSON."""
import argparse
from collections import defaultdict
from decimal import Decimal
import json
from pathlib import Path
import re

LINE = re.compile(r"\[(\d+)\].*?\s(\d+\.\d+):\s+(?:sched:)?(sched_\w+):\s+(.*)")


def parse_trace(lines):
    transitions = defaultdict(list)
    wakeups = defaultdict(list)
    cpus, losses, first, last, exits = set(), False, None, None, {}
    count = 0
    for line in lines:
        if re.search(r"LOST\s+\d+\s+EVENT|missed_events=[1-9]|dropped events", line, re.I):
            losses = True
        match = LINE.search(line)
        if not match:
            if "sched_switch:" in line or "sched_wakeup:" in line:
                raise ValueError("Unsupported timestamp format; use trace-cmd report -t -N --nodate, without --raw-ts")
            continue
        cpu, timestamp, event, payload = match.groups()
        stamp = int(Decimal(timestamp) * 1_000_000_000)
        cpus.add(int(cpu))
        first = stamp if first is None else min(first, stamp)
        last = stamp if last is None else max(last, stamp)
        count += 1
        if event == "sched_switch":
            previous = re.search(r"prev_pid=(\d+).*?prev_state=(\S+)", payload)
            following = re.search(r"next_pid=(\d+)", payload)
            if not previous or not following:
                raise ValueError("Unsupported sched_switch print format; use trace-cmd report -t -N")
            tid, state = int(previous[1]), previous[2]
            # R+ is preemption, D uninterruptible; remaining non-running states
            # retain their raw spelling for review rather than implying a lock cause.
            classification = "unknown" if state in ("X", "Z", "x") else (
                "runnable" if state.startswith("R") else (
                    "blocked_D" if "D" in state else "sleeping"))
            if tid:
                transitions[tid].append((stamp, classification, "switch_out:" + state))
            if int(following[1]):
                transitions[int(following[1])].append((stamp, "running", "switch_in"))
        elif event in ("sched_wakeup", "sched_wakeup_new"):
            match_pid = re.search(r"\bpid=(\d+)", payload)
            if match_pid:
                tid = int(match_pid[1])
                transitions[tid].append((stamp, "runnable", "wakeup"))
        elif event == "sched_process_exit":
            match_pid = re.search(r"\bpid=(\d+)", payload)
            if match_pid:
                exits[int(match_pid[1])] = stamp
                transitions[int(match_pid[1])].append((stamp, "unknown", "exit"))
    for tid in transitions:
        transitions[tid].sort(key=lambda item: item[0])
    return {"transitions": dict(transitions), "wakeups": dict(wakeups), "cpus": sorted(cpus),
            "loss_detected": losses, "first_ns": first, "last_ns": last,
            "event_count": count, "exits": exits}


def summarize_thread(trace, tid, start, end):
    if end <= start:
        raise ValueError("Window end must follow start")
    totals = dict.fromkeys(("running", "runnable", "sleeping", "blocked_D", "unknown"), 0)
    state, cursor, pending, woke = "unknown", start, None, False
    wake_latency, runnable_latency = [], []
    trace_end = trace["last_ns"] or start
    for stamp, next_state, cause in trace["transitions"].get(tid, []):
        if stamp > min(end, trace_end):
            break
        boundary = max(start, stamp)
        if boundary > cursor:
            totals[state] += boundary - cursor
            cursor = boundary
        if next_state == "runnable" and state != "runnable":
            pending, woke = stamp, cause == "wakeup"
        elif next_state == "running" and state == "runnable" and pending is not None:
            if start <= stamp <= end:
                latency = (stamp - pending) / 1e6
                runnable_latency.append(latency)
                if woke:
                    wake_latency.append(latency)
            pending, woke = None, False
        state = next_state
    covered_end = max(start, min(end, trace_end))
    if cursor < covered_end:
        totals[state] += covered_end - cursor
    totals["unknown"] += end - covered_end
    return {"state_ms": {key: value / 1e6 for key, value in totals.items()},
            "wakeup_to_run_ms": wake_latency,
            "max_runnable_to_run_ms": max(runnable_latency, default=None),
            "complete": not trace["loss_detected"] and totals["unknown"] == 0
                        and trace["cpus"] == [0, 1, 2, 3]}


def callback_windows(health, threshold_ms):
    result, seen = [], set()
    for item in health:
        now = item["evaluation_start_monotonic_ns"]
        for name, callback in item.get("timing", {}).get("callbacks", {}).items():
            stamp = callback.get("last_start_monotonic_ns", -1)
            if stamp < 0 or callback.get("count", 0) == 0:
                continue
            gap = callback.get("gap_s", -1)
            candidates = [("open_at_evaluation", stamp, now, 0)]
            if gap is not None and gap >= 0:
                candidates.append(("last_completed", stamp - round(gap * 1e9), stamp, 500))
            for kind, start, end, precision in candidates:
                key = (name, kind, start, end)
                if (end - start) / 1e6 < threshold_ms or key in seen:
                    continue
                seen.add(key)
                result.append({"callback": name, "kind": kind, "start_ns": start, "end_ns": end,
                    "precision_ns": precision, "evaluation_sequence": item["evaluation_sequence"],
                    "callback_count": callback["count"], "callback_timing_snapshot": callback})
    return result


def json_lines(path):
    with path.open() as stream:
        for line in stream:
            yield json.loads(line)


def evaluation_spans(health):
    starts = {value["evaluation_sequence"]: value["evaluation_start_monotonic_ns"]
              for value in health}
    spans = {}
    for value in health:
        completed = value.get("timing", {}).get("completed_evaluation", {})
        sequence = completed.get("sequence")
        if sequence not in starts or "total" not in completed:
            continue
        spans[sequence] = {**completed, "start_ns": starts[sequence],
            "end_ns": starts[sequence] + round(completed["total"]["wall_s"] * 1e9)}
    return list(spans.values())


def analyze(bundle, report, threshold_ms):
    with report.open() as stream:
        trace = parse_trace(stream)
    if not trace["event_count"]:
        raise ValueError("No scheduler events decoded; cannot analyze timing")
    rows = list(json_lines(bundle / "observer.jsonl"))
    health, coordinator = [], []
    for row in rows:
        if row["topic"] in ("/savo_localization/health", "/savo_bringup/core/startup_trace"):
            value = json.loads(row["data"])
            (health if row["topic"].endswith("/health") else coordinator).append(value)
    health.sort(key=lambda value: value["evaluation_start_monotonic_ns"])
    evaluations = evaluation_spans(health)
    windows = callback_windows(health, threshold_ms)
    inventory = {}
    for snapshot in json_lines(bundle / "proc.jsonl"):
        for process in snapshot["processes"]:
            for thread in process["threads"]:
                key = (process["pid"], thread["tid"], thread["start_ticks"])
                if key not in inventory:
                    inventory[key] = {"pid": key[0], "tid": key[1], "start_ticks": key[2],
                        "argv": process["argv"], "comm": thread["comm"],
                        "role": "process_main" if key[0] == key[1] else "unresolved_worker",
                        "first_seen_ns": snapshot["monotonic_ns"]}
                inventory[key]["last_seen_ns"] = snapshot["monotonic_ns"]
    reused = {value["tid"] for value in inventory.values()
              if sum(k[1] == value["tid"] for k in inventory) > 1}
    for window in windows:
        window["threads"] = []
        for thread in inventory.values():
            if thread["last_seen_ns"] < window["start_ns"] - 1_000_000_000 or (
                thread["first_seen_ns"] > window["end_ns"] + 1_000_000_000):
                continue
            evidence = summarize_thread(trace, thread["tid"], window["start_ns"], window["end_ns"])
            if thread["tid"] in reused:
                evidence["complete"] = False
                evidence["pid_reuse_warning"] = True
            window["threads"].append({**thread, **evidence})
        window["health_evaluations"] = [value for value in health
            if window["start_ns"] <= value["evaluation_start_monotonic_ns"] <= window["end_ns"]]
        window["completed_evaluations"] = [value for value in evaluations
            if value["start_ns"] <= window["end_ns"] and value["end_ns"] >= window["start_ns"]]
        window["coordinator_evaluations"] = [value for value in coordinator
            if window["start_ns"] <= value.get("evaluation_start_monotonic_ns", -1) <= window["end_ns"]]
    return {"trace": {key: value for key, value in trace.items() if key not in ("transitions", "wakeups")},
            "threshold_ms": threshold_ms, "inventory": list(inventory.values()), "windows": windows,
            "limitations": ["Scheduler running time includes IRQ interruptions; it is not thread CPU time.",
                "Worker role is unresolved unless separately verified; all process threads are included.",
                "Completed gap start is reconstructed from rounded gap_s (±500 ns).",
                "Open windows and completed windows can overlap: do not sum them.",
                "Snapshots omit intermediate callbacks; lifetime maxima cannot locate individual gaps.",
                "DDS reception timestamps are RMW-specific; zero/missing/clock jumps invalidate deltas.",
                "Check trace-cmd --stat and capture logs for loss; this parser only sees report loss markers."]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--report", type=Path, required=True, help="trace-cmd report -t -N output")
    parser.add_argument("--gap-ms", type=float, default=400)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.gap_ms <= 0:
        parser.error("gap threshold must be positive")
    result = analyze(args.bundle, args.report, args.gap_ms)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")
    print(f"{len(result['windows'])} callback windows; losses={result['trace']['loss_detected']}; "
          f"CPUs={result['trace']['cpus']}. Evidence only; no automatic root-cause claim.")


if __name__ == "__main__":
    main()
