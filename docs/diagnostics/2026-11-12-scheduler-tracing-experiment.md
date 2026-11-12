# Pi Core scheduler tracing: matched recorder-off/on startup

**Prepared 2026-11-12; no Pi capture or root-cause claim yet.** Builds on [capture 01](2026-11-12-instrumented-startup-timing-01.md). The prior failure occurred about 124 seconds after launch: localization cached old IMU/wheel/filtered messages while other subscribers received newer messages. Long publication wall time with little thread CPU time does not distinguish blocking from runnable scheduling delay.

## Scope and controls

The new tools only observe timing and manage a bounded experiment. They do not change node source, parameter files, EKF frequency, freshness limits, confirmation counting, recovery policy or arming restrictions. No control/arming messages or service calls are sent.

The fixed launch repeats the prior capture's `core_lidar_mapping_degraded` profile and **existing** `base_profile:=dryrun_sim_motoroff.yaml`, C++ base driver, `control_startup_mode:=STOP`, `supervisor_auto_arm:=false`. The prior log explicitly shows `backend=dryrun dryrun=true max_duty=0`; selecting this same profile is essential for a comparable motor-off experiment. Other arguments retain installed launch defaults, which are archived. Supervisor startup remains at its existing default (false); if it is not running, auto-arm is not applicable to a running process. Do not enable it just for this experiment.

Physically isolate motor power, keeping Pi, sensor and logic power intact. This is an operator prerequisite, acknowledged by `--motor-power-isolated`; the script cannot verify wiring. Keep the robot stationary, with no mission client, teleop, autonomous action, arming request or other controller running. The observer aborts on non-STOP mode, a non-dryrun base, nonzero max duty or reported wheel duty. This check is diagnostic and may itself be delayed: it is not a replacement for physical isolation or existing motor safety.

A and B have identical launch, observer, trace duration/events/buffer, storage location, sensors, network peers and environment. Only B adds `ros2 bag record`. Unique map IDs/output directories prevent overwriting sessions. Preserve the original diagnostic bags. Use the same installed build for both runs; inspect the archived configuration hashes before comparison.

## Tools and overhead

- `tools/diag/scheduler/capture.py`: normal-user runner, default **150 s** startup, **60 s** trace beginning at **80 s**. Allowed capture ≤180 s, trace 10–60 s, with ≥5 s after the trace. Read-only parameter queries occur after the critical window. Shutdown and bounded query/flush time are additional to the capture duration.
- System-wide `sched_switch`, `sched_wakeup`, `sched_wakeup_new`, process fork/exec/exit in a unique tracing instance, **CPU mask `f` (0–3)**, **`mono` clock**. No PID/event filters, stack sampling, function tracing, affinity/priority changes or busy polling. `--no-filter` includes trace-cmd itself. Kernel buffers: 4096 KiB/core = 16 MiB; 100 ms drain interval; uncompressed output. On-disk size depends on context-switch rate; reserve at least 1 GiB. Duration is bounded, not a fixed file-size cap that could silently discard early events.
- A one-Hz `/proc` sampler records every current TID in the target processes: localization health (including its TF and DDS workers), EKF, coordinator, IMU, wheel odometry, control/base/supervisor, observer and recorder. It stores full argv, PID/TID/start ticks, state, CPU, scheduler policy/priority, affinity in status, CPU percent (100% = one core), schedstat, wchan, process I/O and system CPU/I/O pressure, disk statistics, temperature and frequencies. Thread names alone do **not** reliably identify TF/DDS roles. All process threads are covered even when their individual role is unresolved; fork/exit events retain evidence for threads too short-lived for one-Hz sampling.
- `observe.py`: same best-effort, volatile, depth-20 String subscriptions in A and B: localization health, startup trace, IMU/wheel producer health, mode state and base state. It retains messages in bounded RAM (32 MiB accounting limit), writing JSONL at shutdown. No raw IMU/odom/TF subscriptions in A. This reduces—but does not eliminate—observer fan-out/CPU overhead. B records the same ten topics as the previous MCAP capture, using installed rosbag defaults, without QoS overrides or compression flags.
- Loaded-library maps and selected environment identify the **actual** RMW in each process, not just the runner's environment. Installed launch/config files, generated `--params-file` contents and argv order are copied, plus post-window parameter dumps and endpoint QoS. Missing dumps/config files must be treated as missing evidence. The experiment does not assert full equivalence to capture 01, whose complete invocation/RMW environment was not preserved.

## Exact Pi Core commands

Run from the normal `savo` account on the Pi, with the current instrumented workspace already built. Do not run the whole runner with sudo. Install only diagnostic tooling if absent:

```bash
sudo apt-get install trace-cmd ros-jazzy-rosbag2-storage-mcap
cd ~/Savo_Pi
git status --short --branch
git log -3 --oneline
source /opt/ros/jazzy/setup.bash
source ~/Savo_Pi/savo_ws/install/setup.bash
cat /sys/devices/system/cpu/online
trace-cmd record --help
sudo trace-cmd list -C
```

Require `master`, CPUs `0-3`, the `mono` clock and the documented trace-cmd options (`-B`, `-M`, `--temp`, `--compression`, `--no-filter`). Do not run alongside another tracing experiment. Stop earlier ROS launches normally with Ctrl-C, then inspect for leftovers; the runner refuses known existing target processes. Do not use broad `pkill` commands.

```bash
pgrep -af 'localization_health_node|ekf_node|bringup_readiness_node|imu_node|wheel_odom_node|ros2 bag record'
python3 tools/diag/scheduler/capture.py --run A \
  --output ~/savo-diagnostics/scheduler_a_01 --map-id scheduler_a_01 --print-plan
sudo -v
python3 tools/diag/scheduler/capture.py --run A \
  --output ~/savo-diagnostics/scheduler_a_01 --map-id scheduler_a_01 \
  --motor-power-isolated
```

After complete shutdown, restore the same starting conditions and allow temperature/load to settle. Keep sensor/Edge/network state and background jobs the same. Then:

```bash
sudo -v
python3 tools/diag/scheduler/capture.py --run B \
  --output ~/savo-diagnostics/scheduler_b_01 --map-id scheduler_b_01 \
  --motor-power-isolated
```

`sudo -v` primes credentials; tracing and ownership changes use noninteractive `sudo -n`. A timeout/denial is a failed capture, not a reason to run ROS as root. Trace files are chowned only within the newly created capture directory; verify readability below. On interruption, the runner shuts down its ROS children, waits for the independently bounded trace, and preserves partial evidence. Do not interpret an interrupted run as a complete A/B result. If trace-cmd fails to exit/flush, inspect `result.json` and its named instance; do not reset unrelated tracing buffers.

The default window includes the previous incident. If launch stages move outside it, repeat **both** runs with the same `--trace-delay` and `--trace-seconds`, keeping the bound; do not change stage delays or health parameters to move the failure. Repeat pairs in **A–B–B–A** order with fresh names, or randomized order, before attributing a difference to recording. Do not rebuild, change RMW, QoS, CPU governor, services or sensor load between the pair.

## Verify before analyzing (no sudo required)

```bash
CAPTURE=~/savo-diagnostics/scheduler_a_01
cat "$CAPTURE/result.json"
cat "$CAPTURE/observer-summary.json"
test -r "$CAPTURE/scheduler.dat"
find "$CAPTURE" ! -user "$(id -un)" -ls
trace-cmd report -i "$CAPTURE/scheduler.dat" --cpus
trace-cmd report -i "$CAPTURE/scheduler.dat" --first-event
trace-cmd report -i "$CAPTURE/scheduler.dat" --last-event
trace-cmd dump -i "$CAPTURE/scheduler.dat" --clock
trace-cmd report -i "$CAPTURE/scheduler.dat" --stat > "$CAPTURE/scheduler-stats.txt"
trace-cmd report -i "$CAPTURE/scheduler.dat" --ts-check > /dev/null
trace-cmd report -t -N --nodate -i "$CAPTURE/scheduler.dat" > "$CAPTURE/scheduler.txt"
rg 'LOST|overrun|dropped|entries:' "$CAPTURE/scheduler-stats.txt" "$CAPTURE/trace-cmd.log" "$CAPTURE/scheduler.txt"
rg 'max_duty=0 backend=dryrun dryrun=true|control_startup_mode=STOP' "$CAPTURE/launch.log"
rg -n 'startup_mode:|auto_arm:|dryrun:|max_duty:|frequency:|confirmation' "$CAPTURE/parameters"
rg 'librmw_|libddsc|libfastdds|libfastrtps' "$CAPTURE/processes"
python3 tools/diag/scheduler/analyze.py "$CAPTURE" \
  --report "$CAPTURE/scheduler.txt" --gap-ms 400 \
  --output "$CAPTURE/callback-analysis.json"
```

Repeat with `CAPTURE=~/savo-diagnostics/scheduler_b_01`. The analyzer's output is created exclusively; choose a new filename for re-analysis. Require the stored clock to select `[mono]`, `complete=true`, zero observer/trace errors, mode/base messages present, both required timing topics present, and all four CPUs represented over the target interval. Check that trace output actually contains switch **and wakeup** events, including non-EKF processes. Reject strong scheduling claims if overrun, dropped/LOST events, unconsumed entries or unexplained timestamp reversal/coverage holes occur. Kernel buffer statistics can expose losses not printed as per-event markers. `complete` on a thread summary describes known state coverage under these trace assumptions, not an automatic root-cause verdict.

Verify all expected target processes and threads in `proc.jsonl` / analyzer inventory. Main TID equals PID for these standalone nodes. For TF/DDS workers use full process membership, thread comm, loaded RMW libraries and wchan; mark ambiguous workers unresolved. Names can be truncated or shared. One-Hz snapshots can miss short-lived threads: inspect `sched_process_fork/exec/exit` before claiming every worker's lifetime is mapped. Do not attach a debugger or sample user stacks during the measured window. If exact worker role is necessary, obtain a separate post-window thread backtrace and label it as noncontemporaneous evidence, rather than assigning a role from a name guess.

Check B's recorder completion and QoS in its log/endpoints and bag metadata:

```bash
ros2 bag info ~/savo-diagnostics/scheduler_b_01/bag
cat ~/savo-diagnostics/scheduler_b_01/bag.log
```

A must have no bag directory or recorder process. Compare `configuration-index.json`, `launch-defaults.txt`, `git-head.txt`, `versions.txt` and manifests; only run label, paths, map ID, timing anchors and B's recorder should differ intentionally. Generated parameter hashes containing map IDs may differ; compare their contents. Protect captured configuration/environment files as diagnostics; they may contain local paths or network addresses.

## Correlation and decision procedure

Use **monotonic nanoseconds** as the primary timeline: ftrace `mono`, application `evaluation_start_monotonic_ns`, callback `last_start_monotonic_ns`, and coordinator `receive_monotonic_ns`. Do not use wall-clock log order, bag order or a previous boot's offset to order decisions. The observer and proc sampler bracket realtime with two monotonic reads and store the midpoint/uncertainty. Inspect the realtime-minus-monotonic series for clock steps. DDS timestamps are system-clock values: convert with local paired offsets only when needed, and respect the deployed RMW semantics. Zero, negative/unavailable values and clock steps prevent trustworthy DDS latency attribution.

For **each** detected gap, fill one evidence row (overlapping open/completed windows are not independent incidents):

| Interval / evidence | Required observation |
| --- | --- |
| Callback, count, evaluation sequences, monotonic start/end | Use current `gap_s` and last callback start for the last completed gap; start precision ±500 ns because JSON rounds seconds. An unchanged count plus evaluation time gives an unfinished gap. Lifetime `max_gap_s` cannot locate a gap. Some intermediate callbacks are never exposed in snapshots. |
| Localization main and every TF/DDS candidate TID | Sum running/runnable/sleeping/D/unknown milliseconds. Switch-out R/R+ means runnable/preempted; wakeup begins runnable; switch-in ends scheduler latency. S means sleeping, D means uninterruptible—not a diagnosis of which lock/I/O operation. Unknown coverage cannot be filled by interpolation. |
| Wakeup→execution and preemption→execution | Inspect maxima and actual event intervals on all CPUs. Repeated wakeups must not reset the first pending wakeup. Identify which other tasks occupied CPUs while the target was runnable. Scheduler running intervals include IRQ time; compare with actual thread CPU durations rather than treating them as equal. |
| DDS reception→callback and source→reception | Use the *same exposed callback's* timestamp pair and `receive_to_callback_s`, not two lifetime maxima as though paired. Compare IMU producer publish counters/monotonic times, coordinator receipts and B's raw IMU/header/TF evidence. A deliberately lacks raw-topic recording. A late RMW timestamp is not proof of packets arriving late at the NIC. |
| Evaluation and publication wall/CPU | `timing.completed_evaluation.sequence` identifies the **previous completed evaluation**. Analyzer joins it to that sequence's start, including timing reported after the gap. High wall/low CPU plus runnable time supports contention; high wall/low CPU plus sleeping supports blocking/waiting. High CPU and execution supports application computation. Total evaluation bounds do not give exact internal publication start/end. |
| EKF and coordinator | Inspect all their TIDs over the same interval, EKF overrun logs, filtered callback timing and coordinator evaluation gaps/decision durations. Reconstruct localization receipt sequence/event order, count 1/2→2/2, failure and recovery order. Preserve cached-observation confirmation and terminal latch behavior. |
| A versus B | Compare matched-window gap counts/durations, state fractions, runnable latency distributions, CPU pressure, recorder CPU/I/O, temperatures/frequencies, and stage/decision timing. Normalize by window duration. Sequence holes may indicate observer loss; absence of a failure in one run is weak evidence. |

The analyzer provides per-window thread state totals, wakeup latencies, callback metadata, sequence-correct completed evaluations and coordinator snapshots. Inspect `proc.jsonl` for sampled CPU/I/O/pressure and all raw trace events for competing tasks. It deliberately does not invent unsampled callback histories or automatically select a root cause.

| Hypothesis | Evidence supporting it | Evidence against it / remaining ambiguity |
| --- | --- | --- |
| Linux scheduling contention | Main or DDS/TF worker is runnable for a large fraction of the gap; long wakeup/preemption-to-run delays, occupied CPUs, pressure and similar EKF/coordinator delays | Long S/D waits with little runnable time weaken a pure run-queue explanation; scheduling trace alone does not identify a userspace lock owner |
| Executor starvation/blocking | Main thread runs unrelated work or sleeps inside a long observed evaluation/publication while input DDS receive timestamps are already recent | Main timer progress, small callback/evaluation CPU and delayed reception/TF worker progress weaken executor-only attribution; exact callback scope requires extra tracepoints if snapshots cannot locate it |
| DDS/RMW receive-path delay | Producer/coordinator advance, localization source→receive grows, relevant receiver threads stall, little post-receive delay | Common scheduler delay can explain the same observation; timestamps do not measure wire arrival. RMW lock/transport details need targeted tracing |
| Recorder-induced amplification | Repeated B-only increase in delays with recorder CPU/I/O or reliable reader pressure, controlled temperatures/stages/config | One A/B pair, modest bandwidth or one MCAP chunk boundary proves neither causality nor absence of recorder impact. The common observer and trace also perturb timing |

**No application fix is proposed at this stage.** The smallest justified next action is this matched pair. If it isolates runnable contention, investigate the observed competing workload first. If it isolates sleeping in DDS/publication, the next experiment should narrowly trace that wait/lock or publication call. If DDS receipt is timely but executor dispatch is late, add narrowly scoped callback/dispatch instrumentation only after reviewing the traces. None of these outcomes alone authorizes changing executor type, QoS, scheduling priority, EKF rate, freshness thresholds, confirmation semantics or automatic recovery.

## PC validation and Pi limits

Offline unit tests and an isolated ROS 2 Jazzy observer smoke test are run before handoff; results are recorded in the final implementation notes below. Full ftrace privilege, Pi kernel trace clock/instance support, thermal/CPU behavior, actual thread roles, installed launch/config equivalence, safety telemetry, process cleanup and recording overhead require the Pi run. The previous diagnostic bag is unchanged.

### Implementation validation

- **19 offline/safety tests passed**; one opt-in ROS test skipped on the host without ROS. Coverage includes R/S/D scheduling states, CPU migration, repeated wakeups, trace boundaries/loss, terminal threads, rounded callback windows, sequence joins, motor-off checks, cleanup of orphaned descendants and failed/forced recorder shutdown.
- **1 Jazzy integration test passed** in a network-isolated `ros:jazzy` container with `rmw_fastrtps_cpp`, checking real MessageInfo timestamps and endpoint capture. An earlier smoke test caught and corrected Jazzy's dictionary-shaped MessageInfo access.
- Parser compatibility checked against **44,647 scheduler events** from the older trace. Sequence joins checked separately against **217 health reports / 216 completed evaluations** from capture 01, reproducing evaluation 187/188/189 publication wall times of 16.698/27.784/0.102 ms. These are different captures and were **not** correlated to one another.
- Independent review findings were fixed and rechecked. No existing application/configuration files changed, no build of unchanged C++ packages was necessary, and nothing was committed or pushed.

Reproduce PC checks:

```bash
cd ~/Savo_Pi
python3 -m pytest -q tools/diag/test/test_scheduler_experiment.py \
  tools/diag/test/test_scheduler_observer_ros.py \
  tools/diag/test/test_diagnostic_safety_contract.py
docker run --rm --network none --user "$(id -u):$(id -g)" \
  -e ROS_DOMAIN_ID=137 -e SAVO_ISOLATED_ROS_TEST=1 -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$PWD:/repo:ro" ros:jazzy bash -lc \
  'source /opt/ros/jazzy/setup.bash; python3 -m pytest -q -p no:cacheprovider /repo/tools/diag/test/test_scheduler_observer_ros.py'
```

Only opt into the publisher smoke test inside this isolated container. It publishes synthetic diagnostic state; it must not share the robot's live ROS graph.
