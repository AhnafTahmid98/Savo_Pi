# LEFT ToF channel 7 and Base I2C fault containment

## Scope and evidence

The operator isolated a physical TCA9548A channel-2 fault: swapping logical
LEFT/RIGHT made initialization failure follow physical channel 2. Moving the
same LEFT sensor to channel 7 restored valid readings and healthy ToF status.
Production defaults now use LEFT=7, RIGHT=3, bus=1, mux=0x70, VL53L1X=0x29.
The explicitly configured dry-run profile remains unchanged.

Two separate software defects were confirmed in Base: a recovery `stop()` could
throw out of the original exception handler, and fatal exceptions in `main()`
were logged but returned exit status 0. Neither fix establishes the cause of
the original Linux I2C controller lockup.

The operator reported that a complete power cycle restored the bus, followed
by successful Base-only, Base+BNO055, Base+BNO055+Power runs exceeding 90 s,
and a combined perception run without reproducing the controller timeout.
Neither channel 2 nor repeated STOP traffic is proven to have caused that
whole-bus lockup. No kernel, device-tree or I2C baud-rate change is included.

## Production changes

- Python/C++ ToF defaults, both real-node YAML sections, diagnostic configuration
  and topic documentation agree on LEFT=7. Both diagnostic CLIs already consume
  the shared Python constant; their default parsers are regression-tested.
- The Base callback uses a small callback-based helper, following the existing
  Freenove test seam pattern. Recovery STOP has its own exception handler.
  The primary error is retained separately from the recovery STOP error.
- After the first board fault, later callbacks only attempt STOP; they never
  retry a nonzero command. The error remains latched for the node lifetime,
  including after a successful brake retry. Operator investigation and explicit
  driver restart are required; this patch adds no automatic recovery/re-arm.
- Fault cycles set the requested duty to zero, force a blocked safety decision,
  and increment zero/trip counters even when STOP fails. These report the safe
  request, **not confirmation that the hardware received it**. A partially
  completed I2C write cannot be undone or made safe by software diagnostics.
- `diagnostics.last_recovery_stop_error` is additive. Existing
  `diagnostics.last_board_error` remains populated and causes status `ERROR`.
  Throttled logs expose both errors. Successful STOP retries do not erase them.
- Normal main exit returns 0; a caught fatal `std::exception` returns 1, logs
  the cause, and shuts down an active ROS context.

No changes to command routing, motor inversion/kinematics, physical thresholds,
VL53 initialization, sensor rates/timing, ultrasonic configuration, Supervisor,
mapping, navigation, TF or startup staging are included.

## Repeated STOP writes: inspected, not optimized

Every stopped/no-command Base loop calls `stop()`. At the configured 30 Hz,
four motor pairs each receive two five-byte PCA9685 channel writes: about
240 writes/s and 1,200 payload bytes/s, excluding I2C framing/address overhead.
STOP is active braking: PWM 4095 on both pair inputs, not zero PWM.
No caching, debounce or reduction of those writes was introduced. The first
failed operation can additionally cause one immediate recovery STOP attempt.

## Tests and limits (macOS, 2026-09-16)

Tests were made red before implementation: three Python ToF tests and one native
ToF-default test rejected channel 2. The extracted old Base exception/main logic
failed five of seven initial native tests (two passed). This was a test of the
old logic through the small helper seam, not a running ROS node.

Final local results:

| Check | Result |
| --- | --- |
| Both packages' Python tests | 55 passed |
| New Base runtime/main native gtests | 12 passed |
| Existing Freenove native gtests | 7 passed |
| ToF defaults + existing range-fusion native gtests | 17 passed |
| Both packages' flake8 | Passed |
| Both packages' Python compileall | Passed |
| All Base/perception YAML files | 21 parsed |
| Changed/new C++ files, ament-style uncrustify 0.78.1 | 8 passed |
| YAML semantic diff | Only LEFT channel 2 -> 7 |
| `git diff --check` | Passed |

Native tests used clang++ C++17 and GoogleTest 1.17.0. Existing Freenove tests
used a temporary macOS compile-only `linux/i2c-dev.h` containing `I2C_SLAVE`;
tests use dry-run/callback injection and do not open hardware. This is not a
Linux driver or ROS-node build. All temporary dependencies/binaries are outside
the repository under `/private/tmp`.

An attempted additional native build of the unchanged obstacle-cloud gtest
failed at `test_obstacle_cloud_filter.cpp:557`: Clang rejects the
`EXPECT_THROW(ObstacleCloudProcessingGate(std::numeric_limits<double>::infinity()), ...)`
expression as a declaration. That file has zero diff and was left untouched.
Its hardware-free Python/core contract tests are included in the 55 passes.
The complete package suite is therefore **not** claimed to pass locally.

Commands run for Python checks:

```bash
PYTHONPATH=savo_ws/src/core/savo_base:savo_ws/src/shared/savo_perception \
  python3 -m pytest savo_ws/src/core/savo_base/test \
  savo_ws/src/shared/savo_perception/test -q
python3 -m compileall -q savo_ws/src/core/savo_base savo_ws/src/shared/savo_perception
python3 -m flake8 --config savo_ws/src/core/savo_base/setup.cfg savo_ws/src/core/savo_base
python3 -m flake8 --config savo_ws/src/shared/savo_perception/setup.cfg savo_ws/src/shared/savo_perception
git diff --check
```

Flake8 was supplied by temporary dependencies via `PYTHONPATH` on this Mac.
The required `colcon build`, `colcon test` and `colcon test-result` commands were
attempted and each was unavailable (`command not found: colcon`, exit 127).
The ROS node build, full package tests and real hardware validation remain open.

## Core Pi build and package validation

Keep the robot stationary; do not launch a mission or any motion publisher.

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select savo_perception savo_base --symlink-install \
  --cmake-args -DSAVO_PERCEPTION_REQUIRE_VL53L1X_ULD=ON \
  -DSAVO_PERCEPTION_REQUIRE_LGPIO=ON
source install/setup.bash
colcon test --packages-select savo_perception savo_base --event-handlers console_direct+
colcon test-result --verbose
```

These existing production build flags require the real ToF ULD and ultrasonic
lgpio support to be present instead of silently building hardware-disabled
fallbacks. Do not proceed to hardware validation if build/tests fail.

## Manual stationary hardware verification

Secure the robot so no wheel can cause movement and keep independent motor
power isolation/emergency stop available. Stop other robot bringups, teleop,
navigation and mapping. Do not inject motion or simulate bus faults on hardware.

First check: normal perception launch, **no channel overrides**:

```bash
ros2 launch savo_perception perception_bringup.launch.py \
  driver_impl:=cpp use_ultrasonic:=true use_dashboard:=false
```

In another sourced terminal:

```bash
ros2 topic echo --once /savo_perception/range/left_m
ros2 topic echo --once /savo_perception/range/right_m
ros2 topic echo --once /savo_perception/tof_status
```

Expect finite valid LEFT/RIGHT distances, `state="OK"`, `worker_stale=false`,
and `driver_error=""`. Confirm the effective channels in the ToF status are
LEFT 7 and RIGHT 3. A probe acknowledgement alone does not prove SensorInit.

Before Base startup, ensure `/cmd_vel_safe` has **zero publishers** (a missing
topic is also expected with the other stacks stopped):

```bash
ros2 topic info /cmd_vel_safe
ros2 launch savo_base base_safe_idle.launch.py \
  profile:=real_robot_v1.yaml driver_impl:=cpp
```

This existing idle launch sends no motion commands and retains its default
watchdog/monitoring helpers. The driver's internal watchdog remains enabled.
Do not run this alongside another Base driver or a command-producing stack.

```bash
ros2 topic info /cmd_vel_safe
ros2 topic echo --once /savo_base/base_state
```

Observe repeatedly for at least 90 s: requested wheel duties remain zero,
zero/loop counters advance, and primary/recovery error fields stay empty.
No-command stale/blocked status is expected in this stationary test; it is not
evidence of a board fault. On any I2C error, use independent motor power
isolation; do not assume the requested STOP reached the PCA9685. Investigate
hardware before restarting, and ensure no fresh motion publisher exists before
any explicit restart. Do not clear Supervisor state or start autonomous mapping
as part of this verification.

## Exact changed-file inventory

Under `savo_ws/src/core/savo_base/`:

- `CMakeLists.txt`
- `include/savo_base/base_driver_node.hpp`
- `include/savo_base/base_state.hpp`
- `include/savo_base/driver_runtime.hpp` (new)
- `src/nodes/base_driver_node.cpp`
- `src/state/base_state.cpp`
- `test/test_driver_runtime.cpp` (new)

Under `savo_ws/src/shared/savo_perception/`:

- `CMakeLists.txt`
- `config/core/perception_core.yaml`
- `config/core/tof_mux.yaml`
- `config/diagnostics.yaml`
- `config/profiles/core_real_robot_v1.yaml`
- `include/savo_perception/constants.hpp`
- `savo_perception/constants.py`
- `savo_perception/nodes/vl53_mux_node_py.py`
- `savo_perception/ros/topic_contract.py`
- `test/contracts/test_range_safety_contracts.py`
- `test/unit/test_tof_defaults.cpp` (new)

Plus this new report: `docs/validation/tof_channel7_base_i2c.md`.
No commit, push, deployment or hardware command was performed.
