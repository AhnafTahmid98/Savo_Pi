# Base state publisher C++ implementation plan

Goal: C++17 ROS 2 Jazzy aggregation with the existing Python contract, now the
launch default following Core Pi validation, with explicit Python rollback.
Scope: savo_base plus necessary launch forwarding; master, no commit/push/deploy.

Design: a ROS-independent state core with an explicit monotonic timestamp input,
RapidJSON streaming validation and cached serialized input data, and a thin rclcpp
adapter. Keep Python unchanged. Use a single launch Node selected at runtime,
with cpp as the default and py as an explicit rollback. The initial rollout used
py until Pi validation completed. ROS-clock timers preserve simulated-clock behavior.
A pure streaming approach avoids rebuilding nested input JSON on each summary;
rebuilding a DOM each tick adds allocations without changing the contract.

- [x] Add a differential harness against the original Python implementation;
      test startup, all fields, status priority, exact stale boundaries, toggles,
      malformed/duplicate/non-object JSON, Unicode, nonfinite values, clamps.
- [x] Implement state core and C++ ROS adapter; compile core locally with C++17.
- [x] Register build/dependencies/tests and forward launch selector through base
      wrappers and direct bringup callers. Test exactly one selected executable.
- [x] Add ROS runtime compatibility tests for topic types/QoS, heartbeat, parameters,
      timers and simulated time; skip clearly when ROS is unavailable.
- [x] Run package tests, source/lint checks and independent review. Document audit,
      Pi build/test commands and performance measurement limits.

Review focus: retained data after malformed input, stale safety-stop priority,
ROS time versus monotonic freshness, Python non-standard JSON numeric values,
and mutually exclusive launch selection including disabled state publishing.

Execution notes: implemented inline under the user's authorization, keeping all
changes unstaged on master. No branch/worktree/commit was created. Independent
review found oversized-number recursive-regex stack exhaustion and missing ST
ligature normalization; both were reproduced by failing differential tests and
fixed with iterative numeric scanning and explicit status mappings. ROS-only
execution was subsequently reported passing on Core Pi; it is not a claimed
local result.

Final local evidence is recorded in cpu_phase1.md. The third-party RapidJSON
internal stack UBSan finding is isolated by a header-specific test suppression;
SAVO code remains instrumented. Subsequent Core Pi evidence is recorded in
cpu_phase1.md: eight isolated runtime cases passed, zero colcon test-result
failures, CPU 13.33% to 2.75%, and RSS 81.3 to 42.9 MiB. That evidence authorizes
the C++ default promotion in all six launch files. Actual launch smoke tests of
the new default and Python rollback still require ROS on Core Pi; timing spikes
remain a separate investigation.
