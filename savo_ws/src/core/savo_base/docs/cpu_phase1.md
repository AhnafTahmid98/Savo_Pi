# Phase 1: optional C++ base state publisher

The Python executable remains unchanged and remains the launch default. The new
`base_state_publisher_node_cpp` is a C++17 `rclcpp` telemetry aggregator. Both use
ROS node name `base_state_publisher_node` and JSON `node` value
`base_state_publisher_node`. No driver, motor I/O, watchdog, control, or safety
implementation was changed. Do not run the two executables manually together on
the same topics; the launch selector provides exclusivity within one launch.

## Audited ROS contract

All endpoints use VOLATILE durability, KEEP_LAST history and depth 10.

| Direction | Parameter (default topic) | Message | Reliability |
|---|---|---|---|
| Input | `cmd_topic` (`/cmd_vel_safe`) | `geometry_msgs/msg/Twist` | RELIABLE |
| Input | `safety_stop_topic` (`/safety/stop`) | `std_msgs/msg/Bool` | BEST_EFFORT |
| Input | `slowdown_topic` (`/safety/slowdown_factor`) | `std_msgs/msg/Float32` | BEST_EFFORT |
| Input | `watchdog_topic` (`/savo_base/watchdog_state`) | `std_msgs/msg/String` | RELIABLE |
| Input | `base_state_topic` (`/savo_base/base_state`) | `std_msgs/msg/String` | RELIABLE |
| Input | `motor_board_status_topic` (`/savo_base/motor_board_status`) | `std_msgs/msg/String` | RELIABLE |
| Output | `summary_topic` (`/savo_base/state_summary`) | `std_msgs/msg/String` | RELIABLE |
| Output | `heartbeat_topic` (`/savo_base/state_heartbeat`) | `std_msgs/msg/Bool` | RELIABLE |

The optional motor-board input is disabled by default; all other inputs are
on. Disabled inputs remain in the summary, with null topic names, unseen state,
zero counts, stale=true and age=Infinity. Initial command is (0,0,0), stop=false,
slowdown=1.0, and driver JSON data=null.

### Parameters

Topic parameters are listed above. All other declared custom parameters:

| Parameter | Default | Effective minimum |
|---|---|---|
| `robot_name` | `Robot Savo` | — |
| `subscribe_cmd` | true | — |
| `subscribe_safety_stop` | true | — |
| `subscribe_slowdown` | true | — |
| `subscribe_watchdog` | true | — |
| `subscribe_base_state` | true | — |
| `subscribe_motor_board_status` | false | — |
| `publish_hz` | 5.0 | 0.5 |
| `heartbeat_hz` | 1.0 | 0.2 |
| `cmd_stale_s` | 0.30 | 0.05 |
| `safety_stale_s` | 0.50 | 0.05 |
| `watchdog_stale_s` | 1.50 | 0.10 |
| `base_state_stale_s` | 1.50 | 0.10 |
| `motor_board_status_stale_s` | 1.50 | 0.10 |
| `pretty_json` | false | — |

Both implementations read these settings at startup. Parameter service updates
are not wired to reconfigure running subscriptions, timers or cached settings,
matching the Python implementation. The ROS `use_sim_time` parameter still
controls publication timers. Freshness, uptime and the JSON timestamp use Linux
CLOCK_MONOTONIC, independently of `/clock`. Age is clamped to zero for a backwards
test clock; unseen age is Infinity. An input is stale when **age >= timeout**.

### Summary schema and behavior

Every Python summary field is retained:

- Top level: `node`, `robot_name`, `status_level`, `timestamp_monotonic_s`,
  `uptime_s`, `summary_pub_count`, `topics`, `command_path`,
  `driver_state_topics`, `diagnostics`.
- `topics.inputs`: all six input topic parameter names; `topics.outputs`:
  `summary_topic`, `heartbeat_topic`.
- `command_path.cmd_vel_safe`: `seen`, `count`, `stale`, `age_s`,
  `stale_timeout_s`, `vx`, `vy`, `wz`.
- `command_path.safety_stop` and `command_path.slowdown_factor`: `seen`, `count`,
  `stale`, `age_s`, `stale_timeout_s`, `value`.
- `driver_state_topics.watchdog_state`, `.base_state`, `.motor_board_status`:
  `seen`, `count`, `stale`, `age_s`, `stale_timeout_s`, `parse_error_count`, `data`.
- `diagnostics.notes`: the same three explanatory strings as Python.

Summary count starts at 1 and advances only after successful publish. Heartbeat
is always true, on its independent timer; it signals liveness, not base health.

Priority (first match wins):

1. Any previously received true stop => SAFETY_STOP, even if stale.
2. Fresh watchdog status ERROR, STALE, TRIPPED or FAULT => that status.
3. Fresh base status ERROR, FAULT, STALE, WARN or WARNING => that status,
   with WARNING normalized to WARN.
4. Enabled, stale command subscription => CMD_STALE.
5. Otherwise OK.

Status comparison is case-insensitive. Slowdown, motor-board faults, and missing
watchdog/base inputs do not independently change the overall status. This is a
port of the existing reporting logic, not a change to motor safety policy.

Valid JSON objects are retained; non-object JSON is wrapped as `{"_raw_value":...}`.
Malformed input increments receive/error counts and refreshes receive time, while
retaining previous valid data **and its status**. Repeated identical valid or
invalid messages still update those counters and timestamps. Unknown/nested
fields, Unicode, nonfinite values and large integers are retained. Integer input
limits follow the normal CPython 3.12 default of 4300 decimal digits.

Compatibility is at parsed JSON value level, not byte-for-byte wire encoding:
numeric spelling and duplicate-key serialization can differ. Duplicate keys
retain last-value semantics. Compact output and two-space pretty output are
supported. NaN/Infinity are intentionally retained because Python emits them;
this includes unseen ages. Consumers requiring strictly standard JSON already
need to handle the Python output's nonfinite values.

## Implementation and optimization

`include/savo_base/state/state_summary.hpp` and `src/state/state_summary.cpp`
contain transport-independent state/cache/serialization logic. The small adapter
in `src/nodes/base_state_publisher_node.cpp` declares all parameters and creates
subscriptions, publishers, ROS-clock timers and a single-threaded executor.

RapidJSON validates changed JSON inputs once and serializes them into cached
fragments. Repeated wire payloads skip parsing, including repeated malformed
payloads. Summary ticks stream fixed fields and cached fragments directly into a
reused message string; they neither parse input JSON nor rebuild a nested DOM.
Static topic metadata is serialized once. Pretty formatting also avoids parsing.
The rare oversized-number compatibility path uses iterative number validation to
avoid recursive-regex stack exhaustion. Motor and watchdog code is untouched.

This adds the rosdep `rapidjson` build/test dependency (Ubuntu `rapidjson-dev`);
it is a header-only dependency, with no new runtime library dependency.

## Selection and rollback

After building and sourcing the workspace:

```bash
ros2 launch savo_base base_bringup.launch.py \
  profile:=dryrun_sim_motoroff.yaml base_state_publisher_impl:=cpp
```

Use `base_state_publisher_impl:=py` to return to Python, or omit the argument.
`use_state_publisher:=false` starts neither implementation. Invalid selectors are
rejected. A single launch factory creates one Node, retaining the existing node
name and parameter overrides for both executables.

The selector is explicitly declared/forwarded in `base_safe_idle`,
`base_dryrun_test`, `base_teleop_test`, and the direct `savo_bringup`
`core_bringup`/`autonomous_mapping` callers. The base driver implementation and
all other launch defaults are unchanged.

## Local checks without ROS

Install development/test tools on a development PC if needed:
`g++`, `clang`, `rapidjson-dev`, `libgtest-dev`, `python3-pytest`, `python3-yaml`,
`python3-flake8`. Dependencies may instead be extracted under `/tmp`; set
`RAPIDJSON_INCLUDE_DIR` to that include directory. No headers are vendored.

From the repository root:

```bash
PYTHONPATH=savo_ws/src/core/savo_base \
  python3 -m pytest -q savo_ws/src/core/savo_base/test

# Optional alternate compiler and undefined-behavior instrumentation:
CXX=clang++ \
  CXXFLAGS='-fsanitize=undefined -fno-sanitize-recover=all -fsanitize-ignorelist=savo_ws/src/core/savo_base/test/rapidjson_ubsan.ignore' \
  PYTHONPATH=savo_ws/src/core/savo_base \
  python3 -m pytest -q savo_ws/src/core/savo_base/test/test_state_publisher_compatibility.py

python3 -m flake8 -j 1 --config savo_ws/src/core/savo_base/setup.cfg \
  savo_ws/src/core/savo_base \
  savo_ws/src/shared/savo_bringup/launch/core_bringup.launch.py \
  savo_ws/src/shared/savo_bringup/launch/autonomous_mapping.launch.py

g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -I savo_ws/src/core/savo_base/include \
  savo_ws/src/core/savo_base/test/test_driver_runtime.cpp \
  savo_ws/src/core/savo_base/src/state/base_state.cpp \
  savo_ws/src/core/savo_base/test/test_freenove_motor_board.cpp \
  savo_ws/src/core/savo_base/src/drivers/freenove_motor_board.cpp \
  -lgtest_main -lgtest -pthread -o /tmp/savo_base_existing_tests
/tmp/savo_base_existing_tests
```

The differential tests load the unchanged Python source, replace only unavailable
ROS transport with test doubles, and feed identical callbacks and deterministic
timestamps to a compiled C++ harness. They compare complete decoded summaries,
including all nested fields, counters and nonfinite values. Launch tests execute
the real selection factory and check wrapper forwarding/defaults.

## Required Raspberry Pi Jazzy validation

The PC cannot validate `rclcpp`, ament integration, DDS behavior or CPU reduction.
On the Pi, after reviewing the changes, use the existing workspace dependencies:

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
# Source an existing workspace overlay if dependencies such as savo_msgs are built there.
source install/setup.bash
rosdep install --from-paths src/core/savo_base --ignore-src -r -y
colcon build --packages-select savo_base savo_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select savo_base --event-handlers console_direct+
colcon test-result --verbose
```

For a clean workspace, build dependencies first (`--packages-up-to savo_base
savo_bringup` instead of `--packages-select`). The installed-node pytest module
runs both implementations separately on unique test topics and a local test DDS
domain; it starts no driver or motor nodes. It checks every custom parameter,
actual graph topic types/QoS, all-disabled subscriptions, enabled motor-board
input, summary/heartbeat rates, counters, stale stop priority, malformed-input
retention, and ROS-time pause with monotonic command expiry. It skips explicitly
when `rclpy` is unavailable. ROS timing tolerances should be checked under Pi load.

Then compare Python and C++ on the Pi with the same Release build, RMW, CPU
frequency settings, topic rates, input recording, parameter settings and summary
subscriber load. Start only one implementation at a time. A fixed replay of the
six input topics is preferable to comparing unrelated robot workloads. Warm up,
then record at least 60 seconds of process CPU and RSS, e.g. `pidstat -u -r -p PID
1 60`, using the selected publisher process PID. Confirm summary and heartbeat
rates and status behavior during each run. Stop the first process before changing
implementation. Do not infer improvement from PC compilation or a core-only
microbenchmark: the supplied 13.7% baseline includes ROS middleware/runtime costs.
Keep Python as the default until Pi build, behavior and CPU measurements pass.


## Review and local verification record (2026-11-10)

- `savo_base` package pytest: **51 passed, 1 skipped** (ROS runtime module).
- Package plus available `savo_bringup` contracts: **153 passed, 1 skipped**,
  excluding its existing ROS-only lifecycle test. An initial unfiltered run
  failed `test_production_launch_completes_full_location_lifecycle` because the
  `ros2` executable is absent; no lifecycle code was modified.
- Existing C++ driver/motor-board tests: **19 passed**, compiled directly with
  GCC and GTest; includes fault hold, recovery failures and fail-safe startup.
- Differential core tests under Clang UBSan: **31 passed** with the narrowly
  scoped `test/rapidjson_ubsan.ignore`. The unsuppressed run reported null-pointer
  arithmetic in RapidJSON 1.1 `internal/stack.h` during its lazy allocation. This
  is a remaining third-party sanitizer limitation; no SAVO source is suppressed.
- GCC and Clang C++17 core builds with `-Wall -Wextra -Werror -pedantic` passed.
- Package/changed-launch flake8, Python compilation, package XML parsing and
  `git diff --check` passed. Python fallback and motor/safety sources are unchanged.
- Independent review findings (oversized-number stack exhaustion and Unicode ST
  ligatures) were reproduced with failing tests, fixed, and verified passing.

Remaining validation: actual Jazzy/ament build and all eight installed-node ROS
cases on the Pi, CPU/RSS and timing measurements under representative load, and
consumer compatibility if a downstream client compares raw JSON text instead of
parsed values. Exotic malformed/Unicode/depth cases are not an exhaustive proof
of equivalence with every configurable CPython runtime. No deployment, commit,
push or merge was performed; Python remains the default.
