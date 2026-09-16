# Autonomous mapping: mapping-local authority

## Ownership and compatibility

Old path: caller/bridge -> system Supervisor ACQUIRE -> autonomous action ->
independent system Supervisor CHECK -> mission execution -> RELEASE.

New path: explicit autonomous action (generation zero) -> orchestrator validates
direct mapping-local health and mission inputs -> acquires its own bound lease
-> selects NAV for navigation -> revalidates during execution -> terminal STOP
acknowledgement -> clears local authority. Launch never submits that action.
Existing Scan360 phases still use their existing AUTO mode, not an invented
navigation or motor path.

`mapping_supervisor_node` is read-only. Its new opt-in monitor consumes actual
base, control/mux/shaper, LiDAR, localization, perception, safety, per-source
power, Nav2 and SLAM evidence. Existing scan/map/odom/TF mapping readiness is
also required. No `/savo_supervisor/...` payload participates in this path.
Base battery and Core UPS are required. Edge UPS is required only when
`edge_ups_expected=true`; optional Edge/D435/VO absence does not block Core-only
mapping. Producers still own their sensor validity/rate and power thresholds.

Two additions are `std_msgs/msg/String` JSON schema version 1:

- `/savo_mapping/local_health`: `node`, `admission_ready`,
  `continuation_ready`, `semantic_ready`, `reason`.
- `/savo_mapping/autonomous/authority`: `node`, `mission_id`, `actor_id`,
  `request_id`, `map_id`, `map_revision`, `generation`, `require_semantic`,
  `active`, `coverage_allowed`, `semantic_allowed`.

Both new snapshot publishers/subscribers are reliable, volatile KeepLast(1),
with a 1.5-second steady-clock receive timeout. Volatile durability deliberately
prevents treating a retained pre-restart ready snapshot as new evidence.
Direct source subscriptions use reliable volatile KeepLast(10), compatible with
the existing reliable producers; their existing 1.0/1.5/2.0/2.5/3.0-second
per-source observation contracts are preserved. ROS stamps still describe
sensor/TF time; elapsed freshness uses `steady_clock`.

An environmental obstacle blocks new admission but permits an existing healthy
mission to remain NAV. `/safety/stop` and the unchanged downstream gate force
`/cmd_vel_safe` to zero. Any additional failed Nav dependency still blocks;
only control-mode permission and valid environmental motion interlocks are
excluded from pre-admission infrastructure checks. Critical/invalid/stale
required evidence revokes the mission, commands STOP, and cannot auto-resume.
LOW required power denies new admission but retains the existing continuation
policy. LiDAR WARN is not treated as nominal admission evidence.

`RunAutonomousMapping` retains wire layout and `CONTRACT_VERSION=3`:

- `authority_request_id`, actor, map/revision and semantic scope remain binding.
- `authority_generation=0` acquires a new mapping-local generation only after
  readiness passes. The orchestrator publishes the resulting generation.
- Nonzero supplied generations are rejected with
  `mapping_local_preacquired_generation_not_supported`. A system lease is not
  silently accepted or ignored. Clients that pre-acquired one must migrate to
  zero; the bridge is updated accordingly.
- Autonomous Coverage uses CHECK-only
  `/savo_mapping/autonomous/authorize_phase` (`AuthorizeOperation`) with the
  exact parent scope. Semantic children use
  `/savo_mapping/autonomous/authorize_location_operation`
  (`AuthorizeLocationOperation`) and parent-scoped authorization request IDs.
  They pin parent identity/generation and recheck it after asynchronous work.
  Paused semantic work does not grant motion permission.
- Standalone Coverage/location workflows keep `mapping_local_authority=false`
  and their existing Supervisor behavior. Legacy `supervisor_authorized` status
  fields and `RESULT_SUPERVISOR_DENIED` enum values remain compatible aliases;
  `authority_owner`/reason identify local mode, and the semantic response's
  legacy lifecycle field explicitly says `MAPPING_LOCAL`.

Only the dedicated autonomous launch defaults `start_supervisor=false`.
Normal system launch, Supervisor policy, auto-arm=false, and latch fix
`5081dc2e3a4c11acf95c28ef120c9f62b80528d2` remain unchanged. No hardware,
kinematics, TF ownership, Nav tuning, safety distances or velocity routing
changes are part of this patch.

## Mac verification

Native Clang C++17/gtest: 85 tests across local health, local authority, child
authority context, autonomous mission, AM-7 and Coverage policy. These are real
pure-C++ tests, not proof that the ROS nodes compile or DDS runtime works.

Hardware-free Python, run from each package directory:

| Package | Passed | Skipped |
| --- | ---: | ---: |
| savo_mapping | 174 | 0 |
| savo_bringup | 86 | 0 |
| savo_bridge | 16 | 0 |
| savo_msgs | 56 | 0 |
| savo_nav | 127 | 0 |
| savo_control | 752 | 0 |
| savo_localization | 378 | 0 |
| savo_perception | 44 | 0 |
| savo_power | 370 | 3 |
| savo_supervisor | 19 | 0 |

ROS runtime tests are excluded (`*runtime.py`). Nav launch tests requiring
`launch`/`action_msgs` and localization's `test_topic_contract.py` requiring
`geometry_msgs` are unavailable on this Mac. They must run on Jazzy below.
Package-local working directories matter for existing relative-path tests.
An initial repo-root aggregate attempt had collection/dependency/path failures;
the table records the corrected package-local available runs, not a claim that
the whole ROS suite passed. Three power tests are skipped by their existing
environment checks.

Changed C++ is checked with ROS Jazzy ament_cpplint (including mapping's existing
include-order/copyright filters) and uncrustify 0.78.1 with Jazzy's 0.78 config.
Changed Python compilation/flake8, YAML/XML parsing and `git diff --check` are
also required before commit. The independent agent review did not return a
verdict because of an execution quota; it is not represented as passed.

## Core Jazzy build and tests (operator commands; not run on this Mac)

Keep the robot physically secured and leave existing FastDDS/domain/discovery
settings intact. Build in a fresh shell, before sourcing the rebuilt overlay:

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to \
  savo_msgs savo_mapping savo_bringup savo_supervisor savo_nav savo_control \
  --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select \
  savo_msgs savo_mapping savo_bringup savo_supervisor savo_nav savo_control \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
```

On the host that builds the bridge (normally Edge), also build/test the changed
bridge with the updated `savo_msgs` overlay. This does not launch Edge hardware:

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to savo_bridge \
  --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select savo_bridge \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
```

Required ROS regressions include orchestrator no-Supervisor admission,
pause/resume/cancel, stale and critical local-health abort, environmental-stop
continuation, AM-7, semantic standalone compatibility, and bridge typed-action
submission/rejection. Real FastDDS QoS discovery and all changed ROS C++ node
builds remain Pi validation, not Mac-verified results.

## Core-only real-robot validation

Use the existing validated/locked geometry profile. Do not bypass geometry,
power or health checks to make admission pass. Start only the dedicated stack;
do not run normal bringup concurrently and do not launch a separate Supervisor.

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch savo_bringup autonomous_mapping.launch.py \
  map_id:=core_local_validation \
  start_supervisor:=false supervisor_auto_arm:=false \
  control_startup_mode:=STOP \
  start_head:=false start_location_lifecycle:=false \
  start_semantic_interruption:=false coverage_enabled:=false \
  initial_head_scan_required:=false final_head_scan_required:=false \
  initial_scan360_required:=false final_scan360_required:=false \
  localization_use_vo:=false perception_use_ultrasonic:=false \
  edge_ups_expected:=false
```

In a second sourced terminal, before any mission:

```bash
ros2 node list
ros2 service list
ros2 topic echo --once /savo_control/mode_state
ros2 topic echo --once /savo_mapping/local_health
ros2 topic echo --once /savo_mapping/autonomous/authority
ros2 topic echo --once /savo_nav/status
ros2 topic echo --once /savo_power/base/battery
ros2 topic echo --once /savo_power/core/ups
ros2 topic info --verbose /savo_mapping/local_health
ros2 node info /autonomous_mapping_orchestrator_node
```

Expected: no Supervisor node/service; STOP; local health `admission_ready=true`
once all genuine requirements are ready; authority inactive; Nav may report
only `control_mode_permission` as blocked. The orchestrator must have no
Supervisor service client/subscription. Do not interpret missing data as ready.

**The following explicit action authorizes robot motion.** Run it only in a
clear supervised test area with an accessible independent emergency stop,
after the above checks pass. This initial validation disables saving/release;
it does not claim a production map was approved.

```bash
ros2 action send_goal /savo_mapping/autonomous/run \
  savo_msgs/action/RunAutonomousMapping \
  "{contract_version: 3, mission_id: core_local_001, actor_id: operator_1, map_id: core_local_validation, map_revision: 1, strategy: 1, authority_request_id: core_local_request_001, authority_generation: 0, require_semantic: false, auto_save: false, require_quality_approval: false, mission_timeout: {sec: 120, nanosec: 0}}" \
  --feedback
```

Observe local authority active with nonzero generation, control NAV, and
correlated frontier handoff. NAV without fresh navigation velocity is stationary.
Cancel from another sourced terminal:

```bash
ros2 service call /savo_mapping/autonomous/control \
  savo_msgs/srv/ControlAutonomousMapping \
  "{contract_version: 2, mission_id: core_local_001, actor_id: operator_1, command: 3, reason: operator_validation_complete}"
ros2 topic echo --once /savo_control/mode_state
ros2 topic echo --once /savo_mapping/autonomous/authority
```

Expected terminal acknowledgement: STOP and inactive authority. In separate
controlled trials verify a valid environmental obstacle leaves NAV/authority
active while `/cmd_vel_safe` is zero; clearing it continues the same mission.
Validate stale/invalid required sources and critical power through ROS test
fixtures, not by spoofing healthy production topics or deliberately draining
the real battery. Those tests must abort and reach STOP with no automatic
restart. Repeat semantic/Coverage validation only with their actual optional
hardware and services present. No deployment or robot commands were executed
as part of the Mac implementation.

## Exact patch file inventory

The six existing Scan360/Coverage/TF immutable-asset contract tests have only
their expected hashes updated for the required mapping package/CMake/launch
changes. Their guarded hardware/Scan360/TF implementation files are untouched.
No temporary build, downloaded dependency, or scratch review artifact is included.

```text
savo_ws/src/core/savo_mapping/CMakeLists.txt
savo_ws/src/core/savo_mapping/README.md
savo_ws/src/core/savo_mapping/config/autonomous_mapping_orchestrator.yaml
savo_ws/src/core/savo_mapping/include/savo_mapping/autonomous_mapping_mission.hpp
savo_ws/src/core/savo_mapping/launch/autonomous_mapping.launch.xml
savo_ws/src/core/savo_mapping/launch/coverage_operation_orchestrator.launch.xml
savo_ws/src/core/savo_mapping/package.xml
savo_ws/src/core/savo_mapping/src/nodes/autonomous_mapping_orchestrator_node.cpp
savo_ws/src/core/savo_mapping/src/nodes/coverage_operation_orchestrator_node.cpp
savo_ws/src/core/savo_mapping/src/nodes/location_review_gateway_node.cpp
savo_ws/src/core/savo_mapping/src/nodes/mapped_location_registration_node.cpp
savo_ws/src/core/savo_mapping/src/nodes/mapping_supervisor_node.cpp
savo_ws/src/core/savo_mapping/src/workflow/autonomous_mapping_mission.cpp
savo_ws/src/core/savo_mapping/test/test_autonomous_mapping_am7_runtime.py
savo_ws/src/core/savo_mapping/test/test_autonomous_mapping_mission.cpp
savo_ws/src/core/savo_mapping/test/test_autonomous_mapping_orchestrator_contract.py
savo_ws/src/core/savo_mapping/test/test_autonomous_mapping_orchestrator_runtime.py
savo_ws/src/core/savo_mapping/test/test_autonomous_mapping_sequencer_runtime.py
savo_ws/src/core/savo_mapping/test/test_coverage_mapper_node_contract.py
savo_ws/src/core/savo_mapping/test/test_scan360_deployment_assets_contract.py
savo_ws/src/core/savo_mapping/test/test_scan360_mapper_node_contract.py
savo_ws/src/core/savo_mapping/test/test_scan360_orchestrator_contract.py
savo_ws/src/core/savo_mapping/test/test_scan360_rotate_action_binding_contract.py
savo_ws/src/core/savo_mapping/test/test_tf_pose_reader_contract.py
savo_ws/src/shared/savo_bridge/README.md
savo_ws/src/shared/savo_bridge/src/ros_command_dispatcher.cpp
savo_ws/src/shared/savo_bridge/test/contracts/test_command_boundary_contract.py
savo_ws/src/shared/savo_bridge/test/integration/test_ros_command_dispatcher.cpp
savo_ws/src/shared/savo_bringup/README.md
savo_ws/src/shared/savo_bringup/launch/autonomous_mapping.launch.py
savo_ws/src/shared/savo_bringup/launch/location_integration.launch.py
savo_ws/src/shared/savo_bringup/test/test_autonomous_mapping_deployment_contract.py
savo_ws/src/shared/savo_msgs/action/RunAutonomousMapping.action
savo_ws/src/shared/savo_msgs/test/test_autonomous_mapping_interfaces.py
docs/superpowers/plans/2026-09-14-mapping-local-authority.md
docs/validation/mapping_local_authority.md
savo_ws/src/core/savo_mapping/include/savo_mapping/local_mapping_authority.hpp
savo_ws/src/core/savo_mapping/include/savo_mapping/local_mapping_health.hpp
savo_ws/src/core/savo_mapping/include/savo_mapping/mapping_phase_authority.hpp
savo_ws/src/core/savo_mapping/src/core/local_mapping_health.cpp
savo_ws/src/core/savo_mapping/test/test_local_mapping_authority.cpp
savo_ws/src/core/savo_mapping/test/test_local_mapping_health.cpp
savo_ws/src/core/savo_mapping/test/test_mapping_local_authority_contract.py
savo_ws/src/core/savo_mapping/test/test_mapping_phase_authority.cpp
```
