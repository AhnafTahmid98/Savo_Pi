# savo_supervisor

Robot Savo's monitor-and-permission authority. The package never commands motors, plans paths, saves maps, or writes semantic locations. It observes package-owned contracts, evaluates freshness and health, publishes a truthful global readiness model, and authorizes guarded operations.

## Phase 1 scope

The C++ supervisor now integrates the core motion stack:

- `savo_base`
- `savo_control`
- `savo_perception`
- `savo_lidar`
- `savo_localization`
- `savo_power`

Package-specific payloads are normalized only at the supervisor boundary. The source packages retain ownership of their detailed health policies.

## Inputs

| Component | Required contracts |
|---|---|
| Base | `/savo_base/base_state` |
| Control | `/savo_control/control_status`, `/savo_control/twist_mux/status`, `/savo_control/cmd_vel_shaper/status` |
| Perception | `/savo_perception/range_health`, `/savo_perception/safety_state`, `/savo_perception/heartbeat` |
| LiDAR | `/savo_lidar/state`, `/savo_lidar/heartbeat` |
| Localization | `/savo_localization/health`, `/savo_localization/state_summary`, `/savo_localization/heartbeat` |
| Power | `/savo_power/status`, `/savo_power/health` |
| Authoritative safety | `/safety/stop`, `/safety/slowdown_factor` |

Every required stream is checked for first acquisition, freshness, malformed payloads, clock regressions, reported readiness, and recovery.

## Outputs

- `/savo_supervisor/state_summary` — transient-local JSON schema version 2
- `/savo_supervisor/heartbeat` — supervisor liveness, always published
- `/savo_supervisor/health` — aggregate and per-component diagnostics
- `/savo_supervisor/events` — state and recovery transitions
- `/savo_supervisor/authorize_location_operation` — permission-only location service

The state summary contains:

```text
lifecycle
health
safety
component summaries
core_health_ready
core_safety_ready
core_motion_ready
can_manual_drive
can_rotate
can_start_geometric_mapping
```

## Safety behavior

- Missing, stale, or malformed authoritative safety data blocks global readiness after startup grace.
- An active safety stop does **not** falsely mark the supervisor process as failed.
- A safety stop keeps the supervisor `RUNNING` but `DEGRADED`, and all motion capabilities become false.
- Slowdown keeps safe motion capabilities available while global health is degraded.
- Location motion authorization requires a known safety state by default.

## Capability policy

`can_manual_drive` and `can_rotate` require available, non-inhibited base and control execution, ready perception/localization/power, and known non-stop safety. LiDAR is configurable for these capabilities and is not required by default.

`can_start_geometric_mapping` additionally requires healthy, nominal LiDAR and—by default—nominal power. A low battery may still permit controlled manual movement while preventing a new mapping mission. A component cannot be configured as both disabled and required.

## Launch

```bash
ros2 launch savo_supervisor supervisor.launch.py
```

The launch loads both `supervisor.yaml` and `location_authorization.yaml`.

## Build and test

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select savo_supervisor \
  --symlink-install \
  --event-handlers console_direct+

source install/setup.bash

colcon test \
  --packages-select savo_supervisor \
  --event-handlers console_direct+

colcon test-result --verbose
```

## Next phases

Phase 2 adds high-level mission authority, mapping/navigation readiness, semantic dependencies, and active-map context. Phase 3 adds edge-computer integration, system-service supervision, deployment hardening, and real-hardware fault validation.
