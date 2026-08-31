# savo_supervisor

Robot Savo's global monitor-and-permission authority. The package never commands motors, selects frontiers, plans paths, saves maps, or writes semantic locations. Package-owned nodes remain responsible for those operations. `savo_supervisor` observes their health/readiness contracts, publishes a truthful robot-wide capability model, and grants or revokes permission for guarded operations.

## Implemented scope

### Phase 1 — core readiness

The C++ supervisor integrates:

- `savo_base`
- `savo_control`
- `savo_perception`
- `savo_lidar`
- `savo_localization`
- `savo_power`
- authoritative `/safety/stop` and `/safety/slowdown_factor`

It publishes core health, safety and motion capabilities and fails closed for missing, stale, malformed or unhealthy required inputs.

### Phase 2 — mission authority, mapping and navigation

The supervisor additionally observes:

- `/savo_mapping/status`
- `/savo_nav/status` and `/savo_nav/heartbeat`
- `/savo_head/status`
- `/savo_locations/status` and `/savo_locations/heartbeat`
- autonomous-mapping, Scan360, Coverage and AprilTag-confirmation action availability

Package-specific payloads are normalized only at the supervisor boundary. Mapping remains authoritative for SLAM readiness, `savo_nav` for goal acceptance, `savo_head` for camera/AprilTag evidence readiness, and `savo_locations` for persistent semantic storage.

## Global services

### `/savo_supervisor/authorize_operation`

Typed stateful mission authority using `savo_msgs/srv/AuthorizeOperation`.

Commands:

- `CHECK` — read-only permission/revalidation check
- `ACQUIRE` — acquire ownership of an exclusive major operation
- `PAUSE` — explicitly pause an owned operation
- `RESUME` — resume only after dependencies are healthy again
- `RELEASE` — release operation ownership

Exclusive operations:

- manual control
- manual mapping
- autonomous mapping
- Scan360
- Coverage
- navigate-to-pose
- navigate-to-location

Semantic registration, candidate review and arrival confirmation are guarded but non-exclusive sub-operations.

Ownership is protected by operation type, request ID, actor ID and authority generation. A runtime dependency loss changes an active operation to `REVOKED`. Dependency recovery alone never restarts motion; the owner must explicitly call `RESUME`.

### `/savo_supervisor/update_map_context`

Stores map identity and approval metadata using `savo_msgs/srv/UpdateMapContext`:

- no active map
- live mapping map/session
- approved saved-map release

The service does not load a map. The mapping/release/bringup workflow remains responsible for loading and promoting artifacts. Navigation authorization requires an approved saved release by default, including matching `map_id`, `map_revision` and `map_release_id`.

### `/savo_supervisor/authorize_location_operation`

The existing location gateway remains available. Phase 2 routes its mapping/navigation/arrival checks through the global mission-capability policy before applying the location-specific policy.

## High-level operating modes

The state summary reports one global mode:

- `BOOTING`
- `IDLE`
- `MANUAL`
- `MAPPING`
- `NAVIGATE`
- `RECOVERY`
- `ESTOP`
- `ERROR`
- `SHUTTING_DOWN`

These are global mission modes and do not replace `savo_control`'s internal execution modes.

## Published capability model

Core capabilities:

```text
core_health_ready
core_safety_ready
core_motion_ready
can_manual_drive
can_rotate
can_start_geometric_mapping
```

Mission capabilities:

```text
mapping_available
navigation_ready
head_ready
locations_read_ready
locations_write_ready
semantic_mapping_ready
can_start_manual_mapping
can_start_autonomous_mapping
can_run_scan360
can_run_coverage
can_navigate
can_register_location
can_review_location
can_confirm_arrival
```

Full autonomous semantic mapping requires healthy core motion, mapping readiness, Nav readiness, the autonomous mapping endpoint, head/camera/pose readiness, AprilTag confirmation availability and writable healthy `savo_locations` storage.

Saved-map navigation requires healthy motion and safety, Nav goal acceptance, and an approved matching saved-map release.

## Executor integration

Phase 2 connects the permission contract to real executors:

- `savo_mapping/autonomous_mapping_orchestrator_node` acquires authority before starting, periodically rechecks it during motion phases, pauses on revocation, explicitly resumes, and releases on every terminal path.
- `savo_nav/navigate_to_location_node` acquires an approved-map navigation lease before Nav2 execution, rechecks through the existing location authorization loop, cancels on revocation, and releases on every finish path.

The supervisor still publishes no velocity or navigation goal.

## Outputs

- `/savo_supervisor/state_summary` — transient-local JSON schema version 2
- `/savo_supervisor/heartbeat` — supervisor liveness
- `/savo_supervisor/health` — aggregate and per-component diagnostics
- `/savo_supervisor/events` — health, recovery, authority and map-context transitions

The state summary includes `mission`, `mission_capabilities`, and `map_context` sections.

## Launch

```bash
ros2 launch savo_supervisor supervisor.launch.py
```

The launch loads `supervisor.yaml` and `location_authorization.yaml`.

## Build and test

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select savo_msgs savo_supervisor savo_mapping savo_nav \
  --symlink-install \
  --event-handlers console_direct+

source install/setup.bash

colcon test \
  --packages-select savo_msgs savo_supervisor savo_mapping savo_nav \
  --event-handlers console_direct+

colcon test-result --verbose
```

Phase 2 mission-authority runtime fixture:

```bash
src/shared/savo_supervisor/test/runtime/run_phase2_mission_authority_test.sh
```

It validates acquisition, conflict rejection, dependency revocation, no automatic resume, explicit resume, release, approved saved-map navigation, and safety-stop revocation.

## Phase 3 — edge integration and startup authority

Phase 3 observes the edge command and perception path without turning an optional edge loss into a false core-motor fault:

- `savo_bridge` state, readiness and heartbeat
- RealSense camera health
- `savo_speech` readiness and heartbeat
- visual-odometry health
- configured UI node visibility in the ROS graph

The default startup policy requires truthful core readiness, known safety and a healthy core-edge bridge. RealSense and VO are observed as optional capabilities by default. Speech and UI monitoring default off to match production Edge bringup; when explicitly enabled, their loss degrades the published edge model without independently faulting local core safety.

### `/savo_supervisor/manage_system_state`

Typed startup and lifecycle authority using `savo_msgs/srv/ManageSystemState`:

- `STATUS`
- `ARM`
- `DISARM`
- `BEGIN_SHUTDOWN`
- `CLEAR_FAULT_LATCH`

Production startup is disarmed by default. Motion-capable operations are denied until an explicit `ARM` request succeeds. A required core fault after arming disarms the system and latches the fault. The latch is atomically persisted and survives supervisor restart; dependency recovery alone does not clear it.

Remote-origin actors such as `savo_bridge`, SavoMind, the operator app and `savo_speech` require a healthy bridge command path. Bridge loss revokes an active remote lease while preserving local core supervision. Recovery never resumes a mission automatically.

System-state mutations are accepted only from configured system-actor prefixes. This is a supervisor-level authorization boundary, not cryptographic identity; production DDS access control remains a deployment responsibility.

Controlled shutdown publishes a shutdown intent, blocks new missions and changes the global mode to `SHUTTING_DOWN`. The supervisor does not directly call the operating-system poweroff command.

### Phase 3 outputs

- `/savo_supervisor/system_ready`
- `/savo_supervisor/remote_commands_ready`
- `/savo_supervisor/shutdown_requested`

The JSON state adds `edge_capabilities`, `edge_components` and `system_authority` sections.

### Production service

After building the workspace, install and start the hardened systemd unit:

```bash
ros2 run savo_supervisor install_core_runtime.sh
```

The unit starts with automatic arming disabled and persists state under:

```text
/var/lib/robot_savo/supervisor/system_state.json
```

Request controlled shutdown intent with:

```bash
ros2 run savo_supervisor request_controlled_shutdown.sh operator_requested_shutdown
```

### Validation

Fixture validation:

```bash
src/shared/savo_supervisor/test/runtime/run_phase3_edge_startup_test.sh
src/shared/savo_supervisor/test/runtime/run_phase3_fault_persistence_test.sh
```

Non-motion real-robot preflight:

```bash
ros2 run savo_supervisor real_robot_preflight.sh
```

The complete staged hardware procedure is installed at `share/savo_supervisor/docs/phase3_real_robot_validation.md`. Phase 3 is considered real-hardware validated only after its physical robot stages pass.
