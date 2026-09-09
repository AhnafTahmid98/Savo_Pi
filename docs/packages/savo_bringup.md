# savo_bringup

## Purpose

Distributed Core/Edge launch composition with package-owned health.

## Deployment

Built on both Pis. `robot_bringup.launch.py` selects `host_role:=core|edge`; deploy wrappers default to `safe_idle`, `lidar_only`, geometry required, D435 voxel unvalidated, and Core `STOP`.

## Responsibilities

Validate role/mode/profile, compose package launches, apply geometry/D435 profile gates, and expose supported manual mapping, autonomous mapping, live/saved-map navigation, and diagnostics compositions.

## Non-responsibilities and authority boundaries

Does not implement component behavior, override readiness, grant mission authority, approve releases, or command motion.

## Package structure

Python launch graphs, C++ readiness contract/node, configuration, and contract/runtime tests.

## Runtime components

### `bringup_readiness_node`

Retained as a diagnostic/test utility. Production Core, Edge, and autonomous-mapping launch paths do not use it to release processes.

## Runtime data flow

`role + mode + profile -> package launch selection`; each package publishes its own health, while Supervisor and mission owners retain authority.

## ROS interfaces

### Published topics

When the retained diagnostic node is launched explicitly, under `/savo_bringup/{core|edge}` (or configured namespace): `state` (`String`), `ready` (`Bool`), `heartbeat` (`UInt64`), and diagnostics.

### Subscribed topics

The retained diagnostic node can subscribe to configured base, control, safety, LiDAR, perception, localization, power, supervisor, navigation, bridge, RealSense, VO, speech, UI, and obstacle-cloud state topics according to role/profile. Production launch composition does not use those subscriptions to release processes.

### Services

No package-owned public service.

### Actions

No package-owned action; launch graphs expose actions owned by mapping/nav/head/control.

## TF ownership

None; launches description, localization, SLAM/AMCL owners without duplicating them.

## Parameters and configuration

Modes: `safe_idle`, `manual`, `manual_mapping`, `autonomous_mapping`, `saved_map_navigation`, `diagnostics`. Profiles: `bench`, `lidar_only`, `lidar_d435_voxel`, `production`. Production defaults include `d435_voxel_validated=false`; the retained diagnostic has its own timeout and freshness parameters.

## Launch files

`robot_bringup`, `core_bringup`, and `edge_bringup` are production. Mode compositions include `manual_mapping`, `autonomous_mapping`, `saved_map_navigation`, `live_mapping_navigation`, `diagnostics_bringup`, sensors/localization/location integration. Older `bringup`, `savo_full`, and `savo_*` entries are compatibility compositions, not role truth.

## Persistent state and runtime files

Does not own data; passes Core map/location/supervisor paths under `/var/lib/robot_savo` and relies on deploy storage preparation.

## Hardware ownership

None.

## Dependencies

### Internal Robot Savo dependencies

All production role packages through launch includes; role arrays, not folder names, are authoritative.

### External ROS/system dependencies

ROS launch, lifecycle/Nav2/SLAM components selected by compositions.

## Safety behavior

Invalid role/mode/profile, provisional geometry in motion profiles, unvalidated D435 voxel selection, or missing map release fails closed. Package health and Supervisor authority enforce runtime eligibility. Safe-idle and STOP are defaults.

## Failure and degraded behavior

Optional Edge speech/UI/cloud can remain absent. Required package-health and Supervisor contracts remain fail closed; no Edge status grants Core motion.

## Startup and shutdown behavior

Each host starts its graph independently. Deploy wrappers validate role/storage and use launch shutdown to tear down children.

## Build

Use Core or Edge role build script.

## Run

`bash deploy/core/run_core.sh` or `bash deploy/edge/run_edge.sh`.

## Validation and testing

`validate_full_bringup.sh` plus package launch/contract/runtime tests.

## Current validation status

Implemented and source-validated; target builds and two-Pi safe-idle integration remain required.

## Known limitations and remaining validation

Geometry lock and D435 voxel hardware gate remain closed; saved-map/autonomous modes require their package-owned release/authority gates.

## Change-control considerations

Role membership, defaults, required-component policy, path forwarding, and includes require cross-role regression.

## Related documentation

- [Implementation README](../../savo_ws/src/shared/savo_bringup/README.md)
- [Full bringup closure](../FULL_BRINGUP_CLOSURE_2026-08-01.md)
- [Production startup](../deployment/production_startup.md)
- [Bringup test plan](../testing/bringup_test_plan.md)
- [Ownership matrix](package_ownership_matrix.md)
