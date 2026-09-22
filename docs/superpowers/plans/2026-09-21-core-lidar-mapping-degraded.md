# Temporary Core LiDAR Mapping Degraded Profile Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Do not delegate this plan and do not commit or push.

**Goal:** Add an explicit temporary Core-only LiDAR autonomous-mapping profile that tolerates invalid optional ToFs without weakening the default production path.

**Architecture:** An allowlisted `autonomous_mapping_profile` selector atomically chooses one perception/Nav2 asset pair. Perception policy remains driven by required sensor membership, while a small pure range-health evaluator makes aggregate behavior directly testable in both C++ and Python. The degraded Nav2 file is a production clone with only three angular limits changed.

**Tech Stack:** ROS 2 Jazzy launch, C++17/rclcpp/gtest, Python/pytest, ROS parameter YAML, Nav2 DWB.

**Spec:** `docs/superpowers/specs/2026-09-21-core-lidar-mapping-degraded-design.md`

## Global Constraints

- `autonomous_mapping_profile` accepts only `production` and `core_lidar_mapping_degraded`; default is `production`.
- Production keeps both ToFs required and retains all existing thresholds and fail-closed behavior.
- Degraded mode has no required range sensors and makes both ToFs, depth, and ultrasonic optional.
- Because Jazzy cannot type an empty YAML sequence, degraded profiles encode the empty required set as `["__none__"]`; every C++ and Python policy node validates and normalizes it to an empty set before use.
- LEFT ToF remains TCA channel 7; RIGHT remains channel 3; TCA remains `0x70`; VL53L1X remains `0x29`.
- Do not modify `savo_ws/src/core/savo_base/`, motor behavior, linear Nav2 velocity, costmaps, or `/cmd_vel_safe` routing.
- The degraded Nav2 file changes only `max_vel_theta`, `acc_lim_theta`, and `decel_lim_theta` to `0.30`, `0.50`, and `-0.50`.
- The production runner explicitly pins production and rejects profile/file overrides.
- Dedicated autonomous mapping fixes `initial_scan360_required` to false and rejects attempts to enable it before nodes start.
- Launch, mission admission, start-pose capture, and SLAM initialization produce no physical base motion; the first navigation goal comes from frontier selection/handoff after mapping-local authority and NAV selection.
- Ordinary Nav2 path-following rotation remains enabled at the profile's configured yaw limits.
- No synthetic range values, commits, or pushes.

## Review Focus

- A user-supplied invalid selector must fail before any staged node starts; test the pure resolver and launch validation contract.
- A matching selector with a manually supplied file must not bypass atomic pairing; test that direct perception/Nav2 overrides are rejected.
- A freshly received NaN must be `ERROR`, not mislabeled `STALE`; test both the pure health model and Python fallback sample conversion.
- Optional valid ToF data may still contribute a real close obstacle; test only that invalid optional data does not trigger a required-sensor failure.
- Empty required-sensor configuration must load as a typed string array on ROS 2; validate the installed parameter YAML through `rclcpp::parameter_map_from_yaml_file` and normalize its sentinel to an empty effective set.

---

### Task 1: Lock the Production and Degraded Perception Contracts

**Files:**
- Create: `savo_ws/src/shared/savo_perception/config/profiles/core_lidar_mapping_degraded.yaml`
- Modify: `savo_ws/src/shared/savo_perception/test/contracts/test_range_safety_contracts.py`
- Modify: `savo_ws/src/shared/savo_perception/test/unit/test_range_fusion.cpp`

**Interfaces:**
- Consumes: existing `RangeFusionConfig.required_sensors` and production `core_real_robot_v1.yaml`.
- Produces: a complete degraded parameter file with ROS-typed `required_sensors: ["__none__"]`, normalized to an empty effective set, and optional `tof_left`, `tof_right`, `depth_front`, `ultrasonic_front` for both safety and range-health nodes.

- [ ] **Step 1: Add failing YAML contract tests**

Load both profile files with `yaml.safe_load`. Assert production requires exactly
`["tof_left", "tof_right"]`; degraded requires `[]`; degraded optional sensors
are the four named sensors for both C++ and Python node sections. Recursively
compare the two documents after replacing only those required/optional arrays,
so channels, addresses, thresholds, topics, and `/cmd_vel_safe` must match.

- [ ] **Step 2: Run the focused contract test and verify RED**

Run:
`PYTHONPATH=savo_ws/src/shared/savo_perception pytest -q savo_ws/src/shared/savo_perception/test/contracts/test_range_safety_contracts.py`

Expected: failure because `core_lidar_mapping_degraded.yaml` does not exist.

- [ ] **Step 3: Add failing production/degraded fusion tests**

Add Python and C++ cases equivalent to:

```cpp
auto degraded = RangeFusionConfig{};
degraded.required_sensors.clear();
auto snapshot = clear_snapshot();
snapshot.tof_left = invalid_sample("tof_left");
const auto result = fuse_range_snapshot(snapshot, degraded);
EXPECT_FALSE(result.decision.stop_required);
EXPECT_NE(result.decision.reason, "required_sensor_invalid");
EXPECT_TRUE(contains_sensor_name(result.invalid_sensors, "tof_left"));
```

Retain the existing production cases that expect `required_sensor_invalid` for
left and right ToFs. Add explicit NaN/infinity assertions for both configs.

- [ ] **Step 4: Create the degraded perception profile minimally**

Copy `core_real_robot_v1.yaml` exactly, then change only each safety/range-health
node's `required_sensors` and `optional_sensors` arrays. Keep the ToF driver,
topics, addresses, channels, invalid-value publication, safety gate, and all
thresholds unchanged.

- [ ] **Step 5: Run perception contract and fusion tests GREEN**

Run the Python contract test above and, in a ROS build, the `test_range_fusion`
gtest. Confirm production cases still fail closed and degraded cases retain
invalid sensors diagnostically without a required-sensor stop.

### Task 2: Make Required Range Health Directly Testable

**Files:**
- Create: `savo_ws/src/shared/savo_perception/include/savo_perception/range_health_policy.hpp`
- Modify: `savo_ws/src/shared/savo_perception/include/savo_perception/range_health_node.hpp`
- Modify: `savo_ws/src/shared/savo_perception/src/nodes/range_health_node.cpp`
- Modify: `savo_ws/src/shared/savo_perception/savo_perception/models/sensor_health.py`
- Modify: `savo_ws/src/shared/savo_perception/savo_perception/nodes/range_health_node_py.py`
- Modify: `savo_ws/src/shared/savo_perception/test/unit/test_range_fusion.cpp`
- Modify: `savo_ws/src/shared/savo_perception/test/contracts/test_range_safety_contracts.py`

**Interfaces:**
- Consumes: per-sensor `SensorHealth`, required sensor names, and below-minimum required-rate names.
- Produces: `RequiredRangeHealth`/Python dictionary containing `ok`, aggregate status, `stale_required_sensors`, and `error_required_sensors`.

- [ ] **Step 1: Write failing pure health-policy tests**

Construct a fresh invalid `tof_left` health entry. Assert an empty required list
returns overall OK while the sensor itself remains ERROR. Assert production's
`["tof_left", "tof_right"]` returns overall ERROR with `tof_left` in the error
list. Add the same tests for the Python evaluator.

- [ ] **Step 2: Run the focused tests and verify RED**

Run the perception contract test and the gtest target. Expected: missing
`evaluate_required_range_health` API.

- [ ] **Step 3: Implement the pure evaluator and use it in both nodes**

Implement an evaluator with ERROR precedence over STALE, matching the current
C++ behavior. Refactor node payload construction to use the evaluator without
changing topic names or payload field names. The Python evaluator must return:

```python
{
    "ok": not stale_required and not error_required,
    "status": "ERROR" if error_required else "STALE" if stale_required else "OK",
    "stale_required_sensors": stale_required,
    "error_required_sensors": error_required,
}
```

- [ ] **Step 4: Correct received-invalid timestamp semantics in Python fallbacks**

Keep initial never-received required samples stale, but make `_sample_from_value`
create a current-timestamp invalid `RangeSample` for NaN, infinity, and zero.
This preserves invalidity while reporting freshly received bad data as ERROR.

- [ ] **Step 5: Run focused range-health and fusion tests GREEN**

Confirm optional invalid ToFs are present with `status=ERROR`, aggregate degraded
health is OK, and production aggregate health is ERROR.

### Task 3: Add the Degraded Nav2 Configuration with an Exact Diff Guard

**Files:**
- Create: `savo_ws/src/core/savo_nav/config/nav2_live_mapping_core_lidar_degraded.yaml`
- Modify: `savo_ws/src/core/savo_nav/test/contracts/test_phase5_nav2_contracts.py`
- Modify: `savo_ws/src/core/savo_nav/test/contracts/test_live_mapping_navigation_contracts.py`

**Interfaces:**
- Consumes: production `nav2_live_mapping.yaml`.
- Produces: a live-mapping Nav2 file whose only leaf differences are the three angular limits.

- [ ] **Step 1: Write a failing recursive YAML-diff test**

Flatten both YAML documents to key paths and assert the differing path set is
exactly:

```python
{
    "controller_server.ros__parameters.FollowPath.max_vel_theta",
    "controller_server.ros__parameters.FollowPath.acc_lim_theta",
    "controller_server.ros__parameters.FollowPath.decel_lim_theta",
}
```

Assert degraded values are `0.30`, `0.50`, and `-0.50`, and production values
remain `0.55`, `1.00`, and `-1.00`.

- [ ] **Step 2: Run the focused Nav2 test and verify RED**

Run the two Nav2 contract files. Expected: degraded file missing.

- [ ] **Step 3: Copy production Nav2 YAML and change only angular limits**

Do not edit the production file. Preserve `/scan` observation sources, costmap
plugins, footprints, all linear velocity/acceleration limits, and controller
output routing.

- [ ] **Step 4: Run focused Nav2 tests GREEN**

Confirm the recursive diff contains exactly three leaves and existing
`/cmd_vel_nav` routing assertions still pass.

### Task 4: Add the Atomic Profile Resolver and Harden Launch/Runner Selection

**Files:**
- Create: `savo_ws/src/shared/savo_bringup/savo_bringup/autonomous_mapping_profiles.py`
- Modify: `savo_ws/src/shared/savo_bringup/launch/autonomous_mapping.launch.py`
- Modify: `savo_ws/src/shared/savo_bringup/test/test_autonomous_mapping_deployment_contract.py`
- Modify: `deploy/core/run_autonomous_mapping.sh`

**Interfaces:**
- Consumes: profile name.
- Produces: immutable `AutonomousMappingAssets(perception_config_filename, nav_params_filename)` for the two allowlisted profiles.

- [ ] **Step 1: Write failing resolver and launch contract tests**

Test `production` and `core_lidar_mapping_degraded` mappings; assert an unknown
name raises `ValueError`. Assert launch default is production, direct
`perception_config_file`/`nav_params_file` inputs are rejected, and selection
happens before staged actions. Assert the production runner rejects all three
override names and explicitly passes `autonomous_mapping_profile:=production`.

- [ ] **Step 2: Run bringup deployment tests and verify RED**

Run:
`PYTHONPATH=savo_ws/src/shared/savo_bringup pytest -q savo_ws/src/shared/savo_bringup/test/test_autonomous_mapping_deployment_contract.py`

Expected: missing resolver/profile selector assertions.

- [ ] **Step 3: Implement the pure allowlisted resolver**

Define only these mappings:

```python
production -> core_real_robot_v1.yaml + nav2_live_mapping.yaml
core_lidar_mapping_degraded -> core_lidar_mapping_degraded.yaml + nav2_live_mapping_core_lidar_degraded.yaml
```

Normalize surrounding whitespace only; do not accept aliases or case variants.

- [ ] **Step 4: Wire launch selection and reject direct file overrides**

Declare `autonomous_mapping_profile` with default `production`. Give the two
internal file arguments empty defaults. In the first `OpaqueFunction`, reject
non-empty direct overrides, resolve installed package-share paths from the
allowlist, set the two launch configurations, then perform all existing
geometry, STOP-mode, and map-id validation. Keep the production selection
behavior identical after resolution.

- [ ] **Step 5: Pin and harden the production runner**

Remove its direct perception-file injection, reject selector/perception/Nav2
override arguments, and append `autonomous_mapping_profile:=production`.

- [ ] **Step 6: Run bringup focused tests GREEN**

Confirm invalid names and mixed/manual file combinations are rejected and the
production runner cannot enter degraded mode.

### Task 5: Document the Temporary Operator Flow

**Files:**
- Modify: `savo_ws/src/shared/savo_bringup/README.md`

**Interfaces:**
- Consumes: the final selector and existing autonomous-mapping action workflow.
- Produces: exact degraded launch and pre-motion verification commands.

- [ ] **Step 1: Add a failing documentation contract**

Assert the README names the temporary profile, warns that it has no required
short-range sensors, shows `perception_use_ultrasonic:=false`, and says the
production runner cannot select it.

- [ ] **Step 2: Document launch and verification commands**

Document:

```bash
ros2 launch savo_bringup autonomous_mapping.launch.py \
  map_id:=<lowercase_map_id> \
  autonomous_mapping_profile:=core_lidar_mapping_degraded \
  perception_use_ultrasonic:=false
```

Before motion, require inspection of `/scan`, `/map`, TF/localization health,
Nav2 readiness, `/savo_perception/range_health`, `/savo_perception/tof_status`,
`/safety/stop`, and the `/cmd_vel_nav` to `/cmd_vel_safe` routing. Launch itself
must remain inert until the existing typed mission action is submitted.

- [ ] **Step 3: Run the documentation contract GREEN**

Confirm the command and safety warning remain covered by bringup tests.

### Task 6: Run Focused and Full Hardware-Free Verification

**Files:**
- No production file changes.

**Interfaces:**
- Consumes: completed working tree.
- Produces: captured pass/fail evidence and final diff report.

- [ ] **Step 1: Run focused package tests**

Run package-select builds/tests for `savo_perception`, `savo_nav`,
`savo_mapping`, and `savo_bringup`, followed by `colcon test-result --verbose`.

- [ ] **Step 2: Run the complete hardware-free workspace suite**

Run the repository's Core hardware-free build/test command, then the complete
workspace hardware-free test command documented by the deployment scripts.
Report every failure by name, including environment-only failures.

- [ ] **Step 3: Run lint/static checks**

Run package Flake8, C++ lint/static tests registered by colcon, YAML parsing,
Python compilation, and `git diff --check`.

- [ ] **Step 4: Prove scope and report the diff**

Run:

```bash
git status --short
git diff --stat
git diff -- savo_ws/src/core/savo_base
```

The base diff must be empty. Report exact files, both profile behaviors, all
results, the exact degraded command, pre-motion checks, and diff stat. Do not
commit or push.

### Task 7: Lock Stationary Autonomous-Mapping Startup

**Files:**
- Modify: `savo_ws/src/shared/savo_bringup/savo_bringup/autonomous_mapping_profiles.py`
- Modify: `savo_ws/src/shared/savo_bringup/launch/autonomous_mapping.launch.py`
- Modify: `deploy/core/run_autonomous_mapping.sh`
- Modify: `savo_ws/src/shared/savo_bringup/test/test_autonomous_mapping_deployment_contract.py`
- Modify: `savo_ws/src/core/savo_mapping/test/test_autonomous_mapping_mission.cpp`
- Modify: `savo_ws/src/core/savo_mapping/test/test_autonomous_mapping_orchestrator_contract.py`
- Modify: `savo_ws/src/core/savo_nav/test/contracts/test_live_mapping_navigation_contracts.py`

**Interfaces:**
- Consumes: the dedicated launch's initial Scan360 argument, mapping mission decisions, and guarded frontier goal handoff.
- Produces: a fail-closed fixed-false startup Scan360 contract and regression evidence that the first navigation goal is frontier-derived.

- [ ] **Step 1: Write and run the failing override-rejection test**

Exercise a pure validator with disabled boolean spellings and assert enabled or
malformed values are rejected. Assert the runner rejects any
`initial_scan360_required` override and the launch hardcodes false at the child
mapping boundary.

- [ ] **Step 2: Implement fixed-false startup Scan360 selection**

Validate before staged nodes, hardcode `initial_scan360_required` to false for
the child mapping launch, and base rotate-server startup only on the independent
final Scan360 setting.

- [ ] **Step 3: Add stationary mission and frontier-origin regressions**

Verify mission admission requests only session setup; stationary pose capture
precedes frontier-mode selection; no startup scan, return, Spin, synthetic goal,
or velocity dispatch occurs; frontier planning consumes current TF position and
the selected frontier alone reaches the Nav2 exploration action handoff.

- [ ] **Step 4: Preserve ordinary Nav2 turning**

Retain the exact three-leaf degraded Nav2 diff, assert the degraded maximum yaw
velocity remains nonzero, and retain the `RotateToGoal` path-following critic.

- [ ] **Step 5: Run focused and full hardware-free verification**

Run bringup, mapping, Nav2, perception, and deployment suites; syntax/static
checks; `git diff --check`; and prove the `savo_base` diff remains empty.

### Task 8: Make the Empty Required-Sensor Policy ROS-Typed

**Files:**
- Create: `savo_ws/src/shared/savo_perception/include/savo_perception/range_sensor_parameters.hpp`
- Create: `savo_ws/src/shared/savo_perception/test/unit/test_degraded_profile_parameters.cpp`
- Modify: `savo_ws/src/shared/savo_perception/config/profiles/core_lidar_mapping_degraded.yaml`
- Modify: both C++ and both Python range-policy nodes
- Modify: `savo_ws/src/shared/savo_perception/CMakeLists.txt`

**Interfaces:**
- Consumes: Jazzy YAML string-array parsing and the degraded required-sensor setting.
- Produces: `["__none__"]` as a typed transport value and an empty validated effective sensor list.

- [ ] **Step 1: Add failing transport-normalization tests**

Require the sentinel representation, empty effective normalization, and
fail-closed rejection when the sentinel is mixed with a real sensor.

- [ ] **Step 2: Normalize in every policy node**

Use one shared C++ helper and one shared Python helper before required sensors
reach range fusion, aggregate health, or diagnostics.

- [ ] **Step 3: Test with the ROS parameter parser**

Load the installed degraded profile with
`rclcpp::parameter_map_from_yaml_file`, assert all four node parameters are
typed string arrays, and assert normalization yields an empty effective set.
