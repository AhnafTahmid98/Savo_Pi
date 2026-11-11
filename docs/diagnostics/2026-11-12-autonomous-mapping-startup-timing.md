# Autonomous mapping startup timing investigation

Date: 2026-11-12. Repository baseline: `0202e86` on `master`.
Evidence: `~/savo-diagnostics/savo_startup_20261112_004950/`.

## Finding

The immediate startup failure mechanism is established-dependency confirmation followed by a latched stage failure. A transient localization report marked IMU, wheel odometry, filtered odometry and odom TF stale. Two matching coordinator evaluations can confirm **the same cached health report**. Localization becoming healthy afterwards does not clear an already-failed startup stage.

The initiating timing fault is localized to the health consumer's view of the pipeline, but its precise cause is **not established**. Filtered odometry was reaching an independent subscriber while health had a much older callback observation. Publisher interruption alone cannot explain that discrepancy. Subscriber-specific DDS delivery, callback scheduling, blocking inside an earlier callback, and process/thread scheduling remain candidates. No executor, scheduling, freshness, EKF frequency or recovery-policy change is justified by this capture alone.

| Candidate | Supported conclusion |
| --- | --- |
| Publisher timing | EKF overruns are real; publisher stoppage alone does not explain fresh filtered odometry at the independent subscriber. Producer state-topic publication during the stall was not captured. |
| DDS delivery | Subscriber-specific delay remains possible; the original bundle has no middleware receive timestamps for health. |
| Callback starvation | Supported as delayed consumer observation, but executor starvation versus delivery/blocking cannot be separated yet. |
| Slow health evaluation | The failing evaluation appears fast (~0.3 ms up to its warning); prior callbacks/evaluations were not timed. |
| Scheduler contention | EKF runnable delays are present; trace filtering prevents conclusions about health/coordinator scheduling. |
| Coordinator handling | Exact cached-signature confirmation and terminal latching explain why a transient health fault can abort startup. Recovery receipt ordering in the original run is unobserved. |

## Baseline preserved

`git status --short --branch` initially showed clean `master...origin/master`. Inspected both recent commits before editing:

- `5385fde`: localization profile checkpoint.
- `0202e86`: existing rate/expectation experiments, profile freshness limits of 0.6 s, SLAM stabilization of 5 s, relaxed instantaneous rate condition for the already-debounced rate-only recovery classifier, and autonomous orchestrator constructor/interface traces.

The existing startup-only 12 s EKF rate recovery configuration is also retained. No commit, push, branch creation, motor-control edit, threshold adjustment, autonomous authorization change, EKF frequency change or automatic failure recovery was made by this investigation.

## Evidence and timeline

Times below use the captured epoch, not the filename or the PC clock. `topics.csv` monotonic arrival times were aligned using `clock_offset_s.txt` (`1794429056.001567125` s). This is a single offset measurement, not a clock-drift calibration.

| Event | Captured time / value | Evidence |
| --- | --- | --- |
| Health process | PID 8558 | `launch.log:55` |
| Health becomes operational | 1794437440.632851567 | `launch.log:63` |
| Stale health evaluation stamp | 1794437529.365484 | Embedded health JSON in failed status, `launch.log:392` |
| Evaluation start-to-start interval | 0.731598 s | Same health JSON; nominal health period 0.5 s |
| Health stale log | 1794437529.365784997 | `launch.log:322` |
| Health OK log | 1794437529.674614275 | `launch.log:325` |
| Stage gate reports latched mapping-runtime failure | 1794437541.556487806 | `launch.log:392`, startup status sequence 132, elapsed 14.408 s |
| EKF overruns | 78 reports, maximum 0.373329513 s | All `Failed to meet update rate` lines in launch log |

The stale snapshot and its log differ by approximately **0.301 ms**. Subject to the ROS clock remaining continuous, the failing evaluation's input building, decision and publications up to that log were fast. The 732 ms number measures the interval between evaluations, **not evaluation execution duration**. It is about 232 ms beyond the nominal 500 ms interval. It does not establish that the prior evaluation or another callback was fast.

Health changed from STALE to OK in approximately **308.8 ms**. The stage-gate error was logged considerably later; its timestamp is not the timestamp at which the coordinator first latched failure. The existing log lacks coordinator receipt and evaluation traces, so it cannot prove whether the OK callback reached that coordinator before or after confirmation. The cached failure payload and the source confirm the failure mechanism, but the exact receipt ordering remains to be captured.

At the failing health evaluation:

| Input | Health age | Independent monitor arrival age | Independent latest header age |
| --- | ---: | ---: | ---: |
| IMU | 958.433 ms | 26.747 ms | 28.371 ms |
| Wheel odometry | 607.724 ms | 19.749 ms | 52.329 ms |
| Filtered odometry | 656.656 ms | 14.989 ms | 114.560 ms |
| Odom to base TF | 791.710 ms | Not captured | Not captured |

Important distinction: health subscribes to `/savo_localization/imu_state` and `/savo_localization/wheel_odom_state`, while the independent monitor captured **raw** `/imu/data` and `/wheel/odom`. These are different streams. Fresh raw messages do not prove recent publication/delivery of producer-health snapshots. The filtered-odometry comparison is on the same `/odometry/filtered` topic and is stronger evidence of a consumer-path delay. The snapshot's producer publication-gap and rate values describe its cached producer report; they do not establish what the producer was doing throughout the subsequent stall.

All reported input rates were still valid in the stale snapshot. The existing rate-only recovery classifier correctly excludes this multi-source freshness/TF failure. Extending that grace would change safety policy and was not done.

## Scheduler and source-code interpretation

`trace-cmd report -i scheduler.dat` decoded 32,574 `sched_switch` and 12,073 `sched_wakeup` events over approximately monotonic 8382.175 to 8497.865 s. Every captured switch involves an `ekf_node` thread (including 8557, 8614, 8619). This is not a complete scheduler trace of localization health, its TF listener, DDS threads, or the coordinator.

For EKF main thread 8557, pairing captured switch-outs and switch-ins shows a maximum runnable (`R`) off-CPU interval of about **55.126 ms** (8474.665798–8474.720924). Sleeping (`S`) switch-outs are not proof of runnable CPU contention. These observations support scheduling pressure as a contributor to EKF timing, but cannot attribute the health gap to scheduler starvation or equate the 373 ms overrun with CPU execution time.

Relevant source paths inspected:

- `savo_localization/src/localization_health_node.cpp`: single-threaded `rclcpp::spin`; state snapshot parsing, filtered/VO callbacks, steady-clock receive ages, timers, JSON/diagnostics publishing, and TF lookups.
- `savo_localization/src/{localization_health_core,producer_health}.cpp`: immediate freshness/validity decisions, rate debouncing, producer-success versus consumer-receive age accounting.
- Localization profile, health configuration, producer nodes, and bringup launch/configuration references.
- `savo_bringup/src/bringup_contract.cpp`: exact loss-signature confirmation, bounded rate-only recovery and terminal stage failure latching.
- `savo_bringup/src/nodes/bringup_readiness_node.cpp`: 5 Hz staged evaluations, observation cache, eligibility constraints, status throttling and revalidation of completed stages.
- `startup_stage_gate_node.cpp`: consumer of the failed startup status; the stage gate's log can lag the original decision.
- Existing mapping-orchestrator startup tracing in `0202e86`.

The health TF listener uses its default dedicated spinning thread; the zero-timeout latest-transform lookup is not a configured wait for future TF. Therefore main-executor starvation alone is not a complete explanation for stale TF in the independently serviced buffer. Whole-process scheduling, TF delivery or buffer locking still need evidence.

Coordinator matching includes the full stage dependency reason, which can include the entire health JSON. A newly received unhealthy payload can reset matching even if its logical fault is unchanged. Conversely, two evaluations of one cached payload can confirm it. The implementation preserves both behaviors and makes them visible rather than redefining confirmation.

## Implemented diagnostics

### Localization health

Existing `/savo_localization/health` JSON gains additive fields:

- `evaluation_sequence`, `evaluation_start_monotonic_ns`, `evaluation_period_excess_s` (interval above the nominal period, not an exact timer-deadline measurement).
- `timing.completed_evaluation`: sequence-tagged wall and thread-CPU last/max durations for total evaluation, input construction, TF observations, core evaluation, serialization, publication, and transition logging. TF is included within input construction, so these phases must not all be summed.
- `timing.callbacks.{imu_state,wheel_state,filtered_odom,vo_odom}`: callback count, last monotonic start, last/max callback gap, last/max work wall/CPU duration, DDS source/received timestamps and source/receive-to-callback deltas, their lifetime maxima, and odometry header age at callback.

Completed evaluation timings appear in the **next** health report, identified by their own sequence. The first report has completed sequence zero and no completed sample. Maxima cover node lifetime. There are no per-sensor log lines, dynamic histories or additional subscriptions. Metadata is never used for health or arming decisions.

Zero/unsupported DDS timestamps produce JSON `null` delays; missing header/CPU timing is also explicit. DDS deltas assume timestamps share the system clock epoch. They are not compared to monotonic or simulated ROS time. Negative deltas are retained to expose skew; remote hosts require synchronized clocks. The heartbeat callback and middleware internals are not individually profiled; an unexplained gap may still require callback tracing. The RMW implementation defines where its receive timestamp is sampled, so `receive_to_callback_s` is not automatically a pure executor-queue measurement. Validate that interpretation on the Pi's RMW. CPU duration much smaller than wall duration indicates waiting/preemption, but does not distinguish those two causes by itself.

### Startup coordinator

New best-effort, volatile, depth-10 `/savo_bringup/<role>/startup_trace` publishes one compact JSON record per staged evaluation, independently of startup-status throttling. It includes:

- startup ID, evaluation sequence, monotonic start/gap, event order and elapsed time to the trace point;
- stage and lost-stage names, loss-signature generation/change, consecutive/required confirmation counts, blocking/confirmed/recovered flags;
- recovery eligibility/holding/expiry and the actual ready/failed stage decision, plus first failing evaluation;
- per-dependency receive sequence/order/time, age, DDS timestamps and current ready/failed/fresh state.

Comparing localization receive sequences across two trace records tells whether confirmation reused one callback observation. `dependencies_recovered=true` together with `decision_failed=true` exposes recovery after a terminal decision. Concise `STARTUP_TRACE` transition logs remain in launch output. The stream is observational: no state-reset, recovery, motor or authorization path consumes it.

`decision_elapsed_s` stops at the trace point; it does not measure trace/status publication or total coordinator callback duration. Trace freshness values are sampled at trace serialization, slightly later than the decision. Use event sequences and the decision fields for ordering.

## Verification

The ROS environment is a disposable existing `ros:jazzy` amd64 image. Repository is mounted read-only, with build/install/log outputs under `/tmp/savo-timing-build`. Missing JSON/YAML/lint build dependencies were installed only inside that container.

Final PC results: **3 packages built; colcon reports 805 tests, 0 errors, 0 failures, 5 skipped**. The five skips are the installed cppcheck 2.13.0 performance workaround, not skipped runtime scenarios. The GPIO-dependent `wheel_odom_node` target was not built because `lgpio` is absent on this PC image. The broader hardware-dependent location lifecycle integration test is intentionally not registered for an unspecified PC build role; this run does not claim that coverage.

Focused results: 41 localization-core gtests, 2 timing gtests, 32 bringup-contract gtests, 1 localization ROS runtime scenario and 2 coordinator ROS runtime scenarios passed. All other registered tests in the three packages ran as part of the same package suites. Build/test summaries and focused XML results are retained in [`../logs/validation/PC/startup_timing_20261112/`](../logs/validation/PC/startup_timing_20261112/).

The independent read-only code review found no Critical/Important issues or changed safety decisions. Both minor findings were addressed: missing peak timings now serialize as `null` and negative DDS skew maxima are preserved (assertions reproduced the issue before the fix); the wall-clock-sensitive runtime transient case was removed in favor of the deterministic C++ transient regression. The remaining ROS cases exercise cached confirmation, safety stop and recovery after terminal failure. The full suites were rerun after these changes.

Reproduce inside the prepared Jazzy environment (repository mounted at `/repo`; outputs under `/host-tmp`, corresponding to host `/tmp`):

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /host-tmp/savo-timing-build/log build \
  --base-paths /repo/savo_ws/src/shared/savo_msgs \
    /repo/savo_ws/src/shared/savo_bringup /repo/savo_ws/src/core/savo_localization \
  --build-base /host-tmp/savo-timing-build/build \
  --install-base /host-tmp/savo-timing-build/install \
  --executor sequential --cmake-force-configure --cmake-args -DBUILD_TESTING=ON
source /host-tmp/savo-timing-build/install/setup.bash
colcon --log-base /host-tmp/savo-timing-build/log test \
  --base-paths /repo/savo_ws/src/shared/savo_msgs \
    /repo/savo_ws/src/shared/savo_bringup /repo/savo_ws/src/core/savo_localization \
  --build-base /host-tmp/savo-timing-build/build \
  --install-base /host-tmp/savo-timing-build/install \
  --executor sequential --return-code-on-test-failure
colcon test-result --test-result-base /host-tmp/savo-timing-build/build --verbose
```

Force CMake configuration when first adding the coordinator test because that package discovers Python tests via a configure-time glob. The tests use synthetic topics in isolated ROS domains and start no motor nodes.

Added regression coverage:

- deterministic callback gap/work/CPU/delivery arithmetic, missing timestamps, clock skew and maxima;
- captured multi-source freshness failure followed by localization recovery;
- invalid data, frames, timestamps, diagnostics and pose/yaw jumps remaining fail-closed even during startup grace;
- transient loss resetting stability, exact-signature changes resetting confirmation, repeated cached observations confirming loss, and recovery not clearing terminal failure;
- multiple freshness/safety faults excluded from rate-only grace;
- actual ROS node tests for published timing JSON, stale input/recovery, coordinator callback/evaluation ordering, cached confirmation and safety-stop confirmation.

## Required Pi validation

1. Rebuild and review these changes before deployment; retain the current EKF/profile/safety parameters and normal autonomous authorization procedures.
2. Capture the health JSON, producer state snapshots, raw sensors, filtered odometry, `/tf`, `/tf_static`, startup status and new startup trace together. Attach the effective parameters, Git revision, RMW implementation/version, ROS/use-sim-time setting and startup command to the bundle.
3. Capture scheduling for the **whole relevant process/thread set**, including localization main/TF threads, coordinator and DDS threads, rather than filtering to EKF alone. Include wakeups and switches; record runqueue/CPU pressure and clock alignment without enabling costly per-message logs.
4. Correlate the next stall: fresh publisher/source timestamps with late DDS receipt suggest transport/reader delay; early DDS receipt with late callback suggests execution/take delay (subject to RMW timestamp semantics); large callback/evaluation wall time with little CPU suggests blocking or descheduling; high CPU time identifies actual expensive work. Use scheduler evidence to separate blocking from runnable contention.
5. Check that the same receive sequence is or is not confirmed twice, and whether the recovery receipt event precedes confirmation. Confirm all non-ready and safety cases still inhibit autonomous arming.
6. Compare instrumented and baseline startup overhead on the Pi. PC arithmetic/runtime tests do not establish Pi timing, DDS behavior, GPIO/I2C performance, motor safety on hardware or successful autonomous mapping startup.

The investigation does not claim the Pi startup timing fault is fixed. It supplies a supported failure mechanism, narrower hypotheses and instrumentation to resolve the remaining cause without weakening safety.
