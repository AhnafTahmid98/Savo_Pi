# Robot Savo autonomous mapping mission contract

## Ownership

- `savo_mapping` owns mission lifecycle, strategy selection, completion, saving,
  verification and final mission result.
- `savo_nav` owns validation and execution of each admitted exploration goal.
- `savo_control` and `savo_perception` remain final movement and safety
  authorities.
- `savo_supervisor` authorizes the operating mode; it does not choose frontiers
  or save maps.

## Public interfaces

- `/savo_mapping/autonomous/run` uses `RunAutonomousMapping.action` and starts
  exactly one mission.
- `/savo_mapping/autonomous/control` uses `ControlAutonomousMapping.srv` and
  supports pause, resume and cancel.
- `/savo_mapping/autonomous/status` publishes
  `AutonomousMappingStatus.msg` as retained typed state.
- `/savo_mapping/frontier_explorer/typed_status` publishes
  `FrontierExplorationStatus.msg`; AM-2 completion detection consumes only
  this typed planner evidence and never parses the legacy JSON status.

The public mission contract never carries a Nav2 pose or path. Exploration goal
selection remains internal to `savo_mapping`, and all movement is forwarded
through the existing guarded `savo_nav` exploration handoff.

AM-7 adds movement internally without adding poses or paths to the public
mission request: Coverage uses the supervisor-gated public Coverage operation
services and return-to-start uses the guarded
`/savo_nav/navigation/navigate_to_pose` action. Raw Nav2 actions remain private.

## AM-1 behavior

AM-1 supports the frontier strategy and establishes mission lifecycle control.
A safety or readiness loss latches a pause. Any active exploration goal is
canceled before the workflow returns to monitor-only mode. Resuming requires an
explicit operator or authorized caller request.

AM-1 defers completion and persistence to later autonomous-mapping phases.

## AM-2 completion boundary

Stable completion requires distinct planner evaluations, a configured stable
observation duration, no pending or active exploration goal, fresh typed
frontier evidence, mapping readiness, and no active safety stop. A new
selectable frontier revokes completion-pending and returns the mission to
exploration.

When `auto_save=false`, the mission may complete after entering monitor-only
mode, with `map_saved=false` and an explicit manual-save-required reason.

## AM-3 persistence boundary

When `auto_save=true`, the orchestrator calls only the public
`/savo_mapping/map_session/save` service. The map-session manager owns the
slam_toolbox save and pose-graph serialization calls, atomic staging commit,
manifest creation and overwrite policy. The orchestrator then verifies the
committed session through the shared saved-map contract. It reports success
only when verification passes and returns `map_saved=true`.

AM-3 does not automatically approve navigation handoff or create/promote a
production release. Quality evaluation and operator approval remain separate
later phases.

## AM-7 post-frontier sequence

The mission remains `STRATEGY_FRONTIER`. Once frontier completion is stable and
the workflow is safely monitor-only, it runs:

```text
CoveragePending -> Coverage -> ReturningToStart
  -> FinalScan360 -> FinalHeadScan -> Saving -> Verifying
```

Coverage planning is explicitly requested and correlated by new plan and map
generations. Empty plans are not successful unless the planner explicitly
marks a valid no-op. Execution is approved, canceled, and reset only through
the public supervisor-authorized Coverage operation boundary.

Return uses the captured `start_pose_map`, a normalized map-frame quaternion,
the guarded navigation action, and a final fresh map-to-base proximity check.
Final scans use new operation generations so initial or conditional scan
completion cannot satisfy the final sequence. Timeouts, stale feedback,
rejection, cancellation, failed proximity verification, and exhausted retry
limits fail closed with existing typed mission results.
