# UI Test Plan

## Objective

Verify accurate, fresh, read-only Edge presentation on the 800×480 framebuffer/touch UI without creating robot command authority.

## Scope

Classic `ui_node`, selectable v2 path, framebuffer/touch/rendering, safety/control/navigation/mapping/location/speech/power/bringup state, stale/disconnect behavior, optional preview, performance, and shutdown.

## Test ownership

UI maintainer owns UI-001–004; Edge/display integrator owns UI-005–007.

## Safety classification

All tests are `STATIC`, `UNIT`, `PC`, `HARDWARE-NON-ACTUATING`, `INTEGRATION`, or `RECOVERY` / `NO-MOTION`.

## Preconditions

Robot remains STOP; exact UI variant/profile recorded; Edge display/touch permissions correct; known fresh and stale telemetry fixtures; optional camera privacy approved.

## Required hardware

None for source/preview tests. Edge 800×480 display/framebuffer and touch device for physical validation.

## Required software / configuration

UI config/profile/assets/launch, framebuffer/input dependencies, current `savo_msgs`. Selector must default to the approved production variant.

## Interfaces under test

Read-only subscriptions documented in [the package page](../packages/savo_ui.md), local rendering/preview output, framebuffer and touch input. No service/action client or motion/mission publisher is allowed.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| UI-001 | T0 `STATIC` | Verify expected 800×480 resolution, selector/default/profile, installed assets and source subscribers against package docs. |
| UI-002 | T0 `SOURCE-CONTRACT` | Repository search and package tests prove production UI creates no command publisher, service/action client, shell execution, map/location approval, or authority mutation. Legacy retired-topic observation is documented and non-authoritative. |
| UI-003 | T1 `UNIT`/`PC` | Build/tests pass for render/freshness/state parsing, launch/config, classic and selected v2 read-only contracts. Generate preview frames without hardware. |
| UI-004 | T2 `TARGET-NON-HARDWARE` | Feed recorded/synthetic fresh, stale, malformed, missing, and disconnected states; safety overlay dominates and no stale state is represented current. |
| UI-005 | T3/T4 `HARDWARE-NON-ACTUATING` | Validate startup, framebuffer resolution/colors/text/assets, touch calibration/page selection, optional preview policy, CPU/memory/render rate, and clean restoration/shutdown. Touch changes presentation only. |
| UI-006 | T5 `INTEGRATION` | On distributed safe-idle, display Core/Edge readiness plus safety, control, navigation, mapping, locations, speech, and power accurately; disconnect each source/network and verify explicit stale/offline state. |
| UI-007 | T6/T7 `FAULT-INJECTION`/`RECOVERY` | Stop/restart UI, remove/recover a telemetry source, and reconnect network; robot authority is unchanged, stale state persists until fresh samples, and no duplicate framebuffer owner remains. |

## Pass criteria

Rendering is legible/correct; all required states and staleness are truthful; touch is local presentation only; resource use is recorded; shutdown is clean; no mutation authority exists.

## Blocked criteria

Display/touch device, permissions, selected variant decision, current target build, or approved performance threshold unavailable.

## Failure criteria

UI sends a robot command, hides stale/unsafe state, reports false readiness, wrong resolution/touch mapping, crashes repeatedly, or leaves display/input unusable.

## Abort criteria

Stop UI on electrical/display overheating, input loop, privacy exposure, severe resource starvation affecting robot processes, or any unexpected command publication.

## Evidence to retain

Commit/variant/profile, source authority scan, tests, screenshots of fresh/stale/safety states, touch checklist, CPU/memory/render measurements, disconnect/restart logs, and reviewer.

## Regression triggers

Subscriptions/types, freshness/safety priority, UI selector/default, framebuffer/touch device, assets/layout, camera preview, service/systemd ownership, or any new publisher/client.

## Current validation status

Source tests cover read-only contracts. Current framebuffer, touch, integrated freshness, variant selection, and target performance require Edge validation.

## Related documentation

- [UI package](../packages/savo_ui.md)
- [Edge architecture](../architecture/savo_edge_architecture.md)
- [Observer plan](observer_test_plan.md)

