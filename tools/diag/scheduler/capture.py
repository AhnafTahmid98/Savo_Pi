#!/usr/bin/env python3
"""Bounded Pi Core scheduler experiment. ROS runs as the invoking user."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
EVENTS = ("sched_switch", "sched_wakeup", "sched_wakeup_new",
          "sched_process_fork", "sched_process_exit", "sched_process_exec")
TARGETS = ("localization_health_node", "ekf_node", "bringup_readiness_node",
           "imu_node", "wheel_odom_node", "base_driver_node", "control_mode_manager_node",
           "supervisor_node", "observe.py", "rosbag2", "ros2 bag record")
BAG_TOPICS = ("/savo_bringup/core/startup_trace", "/imu/data",
              "/savo_localization/imu_state", "/savo_localization/health",
              "/savo_localization/wheel_odom_state", "/savo_bringup/core/startup_status",
              "/wheel/odom", "/odometry/filtered", "/tf_static", "/tf")


def validate_timing(duration, delay, trace_seconds):
    if not (10 <= trace_seconds <= 60 and 0 <= delay and
            delay + trace_seconds + 5 <= duration <= 180):
        raise ValueError("Require 10–60 s tracing, delay >= 0, and delay+trace+5 <= duration <= 180")


def launch_command(map_id):
    if not re.fullmatch(r"[a-z][a-z0-9_]*", map_id):
        raise ValueError("map ID must match [a-z][a-z0-9_]*")
    return ["ros2", "launch", "savo_bringup", "autonomous_mapping.launch.py",
            f"map_id:={map_id}", "autonomous_mapping_profile:=core_lidar_mapping_degraded",
            "base_profile:=dryrun_sim_motoroff.yaml", "base_driver_impl:=cpp",
            "control_startup_mode:=STOP", "supervisor_auto_arm:=false"]


def trace_command(output, instance, seconds):
    return ["sudo", "-n", "trace-cmd", "record", "-o", str(output / "scheduler.dat"),
            "--temp", str(output), "--compression", "none", "-B", instance,
            "-C", "mono", "-M", "f", "-b", "4096", "-s", "100000", "--no-filter",
            *[item for event in EVENTS for item in ("-e", "sched:" + event)],
            "sleep", str(seconds)]


def read(path):
    try:
        return Path(path).read_text(errors="replace")
    except (OSError, PermissionError):
        return None


def clocks():
    before = time.monotonic_ns()
    realtime = time.time_ns()
    after = time.monotonic_ns()
    return {"monotonic_ns": (before + after) // 2, "realtime_ns": realtime,
            "pair_uncertainty_ns": (after - before + 1) // 2}


def stat_fields(value):
    # comm may contain spaces and parentheses; fields after final ')' start at #3.
    end = value.rfind(")")
    fields = value[end + 2:].split()
    return {"comm": value[value.index("(") + 1:end], "state": fields[0],
            "ppid": int(fields[1]), "pgrp": int(fields[2]), "session": int(fields[3]),
            "utime_ticks": int(fields[11]),
            "stime_ticks": int(fields[12]), "priority": int(fields[15]),
            "nice": int(fields[16]), "start_ticks": int(fields[19]),
            "cpu": int(fields[36]), "rt_priority": int(fields[37]), "policy": int(fields[38])}


def processes():
    result = {}
    for directory in Path("/proc").glob("[0-9]*"):
        value = read(directory / "stat")
        if not value:
            continue
        try:
            result[int(directory.name)] = {**stat_fields(value),
                "argv": (read(directory / "cmdline") or "").rstrip("\0").split("\0")}
        except (ValueError, IndexError):
            continue  # exited during snapshot
    return result


def save_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


class Sampler:
    def __init__(self, output):
        self.output = output
        self.seen = set()
        self.previous = {}
        self.sample_count = 0
        self.hz = os.sysconf("SC_CLK_TCK")
        (output / "processes").mkdir()
        (output / "parameters").mkdir()

    def snapshot(self):
        self.sample_count += 1
        stamp = clocks()
        entries = []
        table = processes()
        for pid, process in table.items():
            command = " ".join(process["argv"])
            if not any(target in command for target in TARGETS):
                continue
            identity = (pid, process["start_ticks"])
            directory = Path(f"/proc/{pid}")
            if identity not in self.seen:
                self.seen.add(identity)
                metadata = {"pid": pid, **process, **stamp,
                    "maps": read(directory / "maps"), "status": read(directory / "status"),
                    "rmw_environment": [v for v in (read(directory / "environ") or "").split("\0")
                        if v.startswith(("RMW_", "ROS_", "CYCLONEDDS_", "FASTDDS_", "FASTRTPS_"))]}
                save_json(self.output / "processes" / f"{pid}-{identity[1]}.json", metadata)
                # Preserve generated launch parameter files before launch removes them.
                for index, arg in enumerate(process["argv"][:-1]):
                    if arg == "--params-file":
                        source = Path(process["argv"][index + 1])
                        if source.is_file():
                            data = source.read_bytes()
                            digest = hashlib.sha256(data).hexdigest()
                            (self.output / "parameters" / f"{digest}.yaml").write_bytes(data)
                            with (self.output / "parameter-index.jsonl").open("a") as stream:
                                stream.write(json.dumps({"pid": pid, "argv_index": index,
                                    "source": str(source), "sha256": digest}) + "\n")
            # Libraries may be dlopened after the first process snapshot.
            if self.sample_count % 10 == 0:
                (self.output / "processes" / f"{pid}-{identity[1]}.maps").write_text(
                    read(directory / "maps") or "process exited during maps read")
            threads = []
            for task in directory.glob("task/[0-9]*"):
                value = read(task / "stat")
                if not value:
                    continue
                try:
                    item = {"tid": int(task.name), **stat_fields(value),
                            "schedstat": read(task / "schedstat"), "wchan": read(task / "wchan"),
                            "status": read(task / "status")}
                except (ValueError, IndexError):
                    continue
                key = (pid, item["tid"], item["start_ticks"])
                ticks = item["utime_ticks"] + item["stime_ticks"]
                previous = self.previous.get(key)
                item["cpu_percent_one_core"] = (None if previous is None else
                    100 * (ticks - previous[1]) / self.hz /
                    ((stamp["monotonic_ns"] - previous[0]) / 1e9))
                self.previous[key] = (stamp["monotonic_ns"], ticks)
                threads.append(item)
            entries.append({"pid": pid, **process, "threads": threads,
                            "io": read(directory / "io")})
        return {**stamp, "processes": entries,
                "system": {name: read(path) for name, path in {
                    "stat": "/proc/stat", "loadavg": "/proc/loadavg", "meminfo": "/proc/meminfo",
                    "cpu_pressure": "/proc/pressure/cpu", "io_pressure": "/proc/pressure/io",
                    "diskstats": "/proc/diskstats", "temperature": "/sys/class/thermal/thermal_zone0/temp",
                    **{f"cpu{i}_khz": f"/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_cur_freq"
                       for i in range(4)}}.items()}}


def stop(process, grace=(15, 5, 5)):
    if process is None:
        return None
    outcome = {"pid": process.pid, "signals": [], "forced": False, "exit": process.poll()}

    def live_group():
        process.poll()  # reap our child, but its descendants may still be alive
        return [pid for pid, value in processes().items()
                if value["pgrp"] == process.pid and value["state"] not in ("Z", "X")]

    for sig, seconds in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), grace):
        if not live_group():
            outcome["exit"] = process.poll()
            return outcome
        try:
            os.killpg(process.pid, sig)
            outcome["signals"].append(sig.name)
            outcome["forced"] |= sig != signal.SIGINT
        except ProcessLookupError:
            outcome["exit"] = process.poll()
            return outcome
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if not live_group():
                outcome["exit"] = process.poll()
                return outcome
            time.sleep(.05)
    if live_group():
        raise RuntimeError(f"Owned process group {process.pid} still has live processes")
    outcome["exit"] = process.poll()
    return outcome


def recorder_finished(run, shutdown):
    return run == "A" or bool(shutdown and shutdown["exit"] == 0 and not shutdown["forced"])


def run_logged(command, output, timeout=10, env=None):
    with output.open("w") as stream:
        try:
            return subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                  timeout=timeout, env=env, check=False).returncode
        except subprocess.TimeoutExpired:
            stream.write("\nCAPTURE COMMAND TIMED OUT\n")
            return 124


def preflight():
    if os.geteuid() == 0:
        raise RuntimeError("Run as the normal ROS user, never sudo the runner")
    if (read("/sys/devices/system/cpu/online") or "").strip() != "0-3":
        raise RuntimeError("This experiment requires exactly four online CPUs (0-3)")
    for executable in ("ros2", "trace-cmd", "sudo"):
        if not shutil.which(executable):
            raise RuntimeError(f"Missing {executable}")
    subprocess.run(["sudo", "-n", "true"], check=True)
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from rclpy.utilities import get_rmw_implementation_identifier
    profile = Path(get_package_share_directory("savo_base")) / "config/profiles/dryrun_sim_motoroff.yaml"
    data = yaml.safe_load(profile.read_text())["/base_driver_node"]["ros__parameters"]
    if not (data["dryrun"] is True and data["board_backend"] == "dryrun" and data["max_duty"] == 0):
        raise RuntimeError("Installed motor-off profile is not motor-off")
    for process in processes().values():
        command = " ".join(process["argv"])
        if any(target in command for target in TARGETS if target != "observe.py"):
            raise RuntimeError(f"Existing experiment/robot process must be stopped first: {command}")
    return {"installed_motoroff_profile": str(profile), "motoroff_profile": profile.read_text(),
            "observer_rmw": get_rmw_implementation_identifier()}


def archive_configuration(output):
    from ament_index_python.packages import get_package_share_directory
    index = []
    for package in ("savo_bringup", "savo_localization", "savo_base", "savo_control",
                    "savo_power", "savo_safety", "savo_supervisor", "savo_nav", "savo_mapping"):
        try:
            share = Path(get_package_share_directory(package))
        except LookupError:
            index.append({"package": package, "missing": True})
            continue
        for subdir in ("launch", "config"):
            for path in sorted((share / subdir).rglob("*")):
                if not path.is_file() or "__pycache__" in path.parts:
                    continue
                data = path.read_bytes()
                destination = output / "installed" / package / path.relative_to(share)
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(data)
                index.append({"package": package, "source": str(path),
                              "sha256": hashlib.sha256(data).hexdigest()})
    # Middleware configuration can be inline XML or a path/URI. Preserve the
    # exact setting; copy readable local files, never fetch remote resources.
    for key in ("CYCLONEDDS_URI", "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
        value = os.environ.get(key, "")
        path = Path(value.removeprefix("file://")) if value and "<" not in value else None
        if path and path.is_file():
            destination = output / (key + ".xml")
            destination.write_bytes(path.read_bytes())
    save_json(output / "configuration-index.json", index)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", choices=("A", "B"), required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory; never reused")
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--duration", type=int, default=150)
    parser.add_argument("--trace-delay", type=int, default=80)
    parser.add_argument("--trace-seconds", type=int, default=60)
    parser.add_argument("--motor-power-isolated", action="store_true",
                        help="Operator has physically isolated motor power; keep logic/sensors powered")
    parser.add_argument("--print-plan", action="store_true", help="No ROS or sudo needed")
    args = parser.parse_args()
    validate_timing(args.duration, args.trace_delay, args.trace_seconds)
    command = launch_command(args.map_id)
    output = args.output.expanduser().resolve()
    instance = f"savo_sched_{os.getpid()}"
    trace_cmd = trace_command(output, instance, args.trace_seconds)
    if args.print_plan:
        print(json.dumps({"launch": command, "trace": trace_cmd,
                          "bag_topics": BAG_TOPICS if args.run == "B" else []}, indent=2))
        return
    if not args.motor_power_isolated:
        parser.error("Physically isolate motor power, then supply --motor-power-isolated")
    metadata = preflight()
    output.mkdir(parents=True, exist_ok=False)
    archive_configuration(output)
    env = dict(os.environ, ROS_LOG_DIR=str(output / "ros_logs"))
    metadata.update({"run": args.run, "launch_command": command, "trace_command": trace_cmd,
        "duration_s": args.duration, "trace_delay_s": args.trace_delay,
        "trace_seconds": args.trace_seconds, "clock": "CLOCK_MONOTONIC / ftrace mono",
        "environment": {k: v for k, v in env.items() if k.startswith(
            ("ROS_", "RMW_", "CYCLONEDDS_", "FASTRTPS_", "FASTDDS_", "AMENT_"))},
        "uname": list(os.uname()), "uid": os.getuid(), "clock_ticks": os.sysconf("SC_CLK_TCK"),
        "boot_id": read("/proc/sys/kernel/random/boot_id"), "start": clocks()})
    save_json(output / "manifest.json", metadata)
    run_logged(["git", "-C", str(REPO), "status", "--short", "--branch"], output / "git-status.txt")
    run_logged(["git", "-C", str(REPO), "rev-parse", "HEAD"], output / "git-head.txt")
    run_logged(["dpkg-query", "-W", "ros-jazzy-rmw*", "ros-jazzy-fastdds*", "ros-jazzy-cyclonedds*",
                "trace-cmd", "linux-image*"], output / "versions.txt")
    run_logged(["ros2", "launch", "savo_bringup", "autonomous_mapping.launch.py", "--show-args"],
               output / "launch-defaults.txt", env=env)
    children, handles = [], []
    trace = None
    observer = None
    recorder = None
    launch = None
    results = {"complete": False}

    def interrupted(*_):
        raise KeyboardInterrupt("capture interrupted")

    signal.signal(signal.SIGTERM, interrupted)

    def spawn(cmd, filename):
        handle = (output / filename).open("w")
        handles.append(handle)
        process = subprocess.Popen(cmd, stdout=handle, stderr=subprocess.STDOUT,
                                   env=env, start_new_session=True)
        children.append(process)
        return process

    try:
        observer = spawn([sys.executable, str(HERE / "observe.py"), "--output", str(output),
                          "--duration", str(args.duration + 20)], "observer.log")
        deadline = time.monotonic() + 10
        while not (output / "observer-ready").exists():
            if observer.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError("Observer did not become ready; inspect observer.log")
            time.sleep(.1)
        if args.run == "B":
            recorder = spawn(["ros2", "bag", "record", "--storage", "mcap", "--output", str(output / "bag"),
                              *BAG_TOPICS], "bag.log")
        launch = spawn(command, "launch.log")
        origin = time.monotonic()
        metadata["launch_start"] = clocks()
        save_json(output / "manifest.json", metadata)
        sampler = Sampler(output)
        with (output / "proc.jsonl").open("w", buffering=65536) as stream:
            next_sample = origin
            while time.monotonic() - origin < args.duration:
                if launch.poll() is not None or observer.poll() is not None:
                    raise RuntimeError("Launch/observer exited early; inspect logs")
                if recorder is not None and recorder.poll() is not None:
                    raise RuntimeError("Recorder exited early; inspect bag.log")
                elapsed = time.monotonic() - origin
                if trace is None and elapsed >= args.trace_delay:
                    trace = spawn(trace_cmd, "trace-cmd.log")
                    metadata["trace_request"] = clocks()
                    save_json(output / "manifest.json", metadata)
                if trace is not None and trace.poll() not in (None, 0):
                    raise RuntimeError("trace-cmd failed; inspect trace-cmd.log")
                stream.write(json.dumps(sampler.snapshot()) + "\n")
                next_sample += 1
                time.sleep(max(0, next_sample - time.monotonic()))
        if trace is None or trace.wait(timeout=15) != 0:
            raise RuntimeError("Scheduler trace did not complete")
        # Queries happen AFTER the critical window, equally in A and B.
        nodes = output / "nodes.txt"
        run_logged(["ros2", "node", "list", "--no-daemon"], nodes, timeout=5, env=env)
        for node in sorted(set(nodes.read_text().splitlines())):
            if node.startswith("/") and any(key in node for key in (
                "localization_health", "ekf", "bringup_readiness", "control_mode_manager", "base_driver", "supervisor")):
                run_logged(["ros2", "param", "dump", node],
                    output / "parameters" / (node.strip("/").replace("/", "_") + ".effective.yaml"),
                    timeout=3, env=env)
        results["complete"] = True
    except (KeyboardInterrupt, Exception) as error:
        results["error"] = repr(error)
        raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        # Trace command is independently bounded by sleep; never reset another user's tracing.
        # Stop the observer while publishers still exist, preserving endpoint QoS.
        cleanup_errors = []
        shutdowns = {}
        try:
            if observer:
                shutdowns[observer.pid] = stop(observer)
        except (OSError, RuntimeError) as error:
            cleanup_errors.append(str(error))
        for process in reversed(children):
            if process is not trace and process is not observer:
                try:
                    shutdowns[process.pid] = stop(process)
                except (OSError, RuntimeError) as error:
                    cleanup_errors.append(str(error))
        if trace is not None:
            try:
                trace.wait(timeout=args.trace_seconds + 20)
                results["trace_exit"] = trace.returncode
            except subprocess.TimeoutExpired:
                results["trace_cleanup_error"] = "trace-cmd did not exit; inspect owned instance " + instance
            if trace.poll() is not None:
                run_logged(["sudo", "-n", "trace-cmd", "reset", "-B", instance, "-d"],
                           output / "trace-cleanup.log")
            # Only this newly created directory's trace files; no recursive chown.
            for path in output.glob("*.dat*"):
                if path.is_file() and not path.is_symlink():
                    run_logged(["sudo", "-n", "chown", f"{os.getuid()}:{os.getgid()}", str(path)],
                               output / "trace-ownership.log")
        for handle in handles:
            handle.close()
        results["end"] = clocks()
        results["scheduler_readable"] = os.access(output / "scheduler.dat", os.R_OK)
        results["observer_exit"] = observer.returncode if observer else None
        results["cleanup_errors"] = cleanup_errors
        results["shutdowns"] = shutdowns
        results["recorder_shutdown"] = shutdowns.get(recorder.pid) if recorder else None
        results["launch_shutdown"] = shutdowns.get(launch.pid) if launch else None
        results["complete"] = bool(results["complete"] and results["scheduler_readable"]
                                   and results["observer_exit"] == 0 and results.get("trace_exit") == 0
                                   and not cleanup_errors
                                   and not any(value["forced"] for value in shutdowns.values())
                                   and results["launch_shutdown"]
                                   and results["launch_shutdown"]["exit"] in (0, -signal.SIGINT, 130)
                                   and recorder_finished(args.run, results["recorder_shutdown"]))
        save_json(output / "result.json", results)
    if not results["complete"]:
        raise RuntimeError("Incomplete capture; inspect result.json and logs")
    print(f"Capture saved: {output}. Verify coverage, safety and losses before interpreting it.")


if __name__ == "__main__":
    main()
