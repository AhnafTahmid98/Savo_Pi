# Mapping and Navigation Architecture

Mapping creates reviewed, immutable map releases; navigation consumes only a verified active release. Both run on Core and remain subordinate to supervisor permission, readiness, control, and safety.

## Mapping lifecycle

```text
RunAutonomousMapping goal / manual mapping
       -> SLAM owns map -> odom
       -> exploration / coverage / Scan360
       -> session save + hashes + quality report
       -> operator review
       -> transactional production release
       -> /var/lib/robot_savo/maps/production/active_map.yaml
```

`savo_mapping` owns session orchestration, frontier/coverage selection, scan requests, saving, quality evaluation, review records, and release transaction recovery. The typed mission entry is `/savo_mapping/autonomous/run` (`RunAutonomousMapping`). Map sessions are stored under `/var/lib/robot_savo/maps/sessions`; production releases and journals use `maps/production` and `maps/release_transactions`.

## Navigation lifecycle

Production launch validates the active-map contract, manifest identity, frame `map`, artifact existence/hashes, and release state before Nav2 admission. `savo_nav` owns map context, readiness, goal admission, recovery coordination, and public navigation adapters. Public actions include `/savo_nav/navigation/navigate_to_pose`, `/savo_nav/exploration/navigate_to_pose`, `/savo_nav/coverage/execute_path`, and `/savo_nav/locations/navigate`; raw Nav2 actions remain internal integration points.

Saved-map navigation uses AMCL for `map -> odom`. Mapping uses SLAM. The two owners are mutually exclusive. All resulting velocity commands enter the NAV/AUTO lane in `savo_control`, then pass the Core safety gate and base watchdog.

## Locations and authority

`savo_locations` owns the SQLite semantic registry and candidate/release lifecycle. Registration and map release require operator review; supervisor authorizes context but does not approve content. Bridge/SavoMind may request bounded operations and observe results, not fabricate approvals or bypass readiness.

Missing map context, stale readiness, failed quality, incomplete transaction, authority revocation, cancellation, localization loss, or safety stop must reject/cancel and fail closed. Source and runtime contract tests cover many lifecycle transitions. Physical validation remains for map quality, coverage, recovery, cancellation, localization transitions, route clearance, footprint/inflation tuning, rollback, and repeated release activation.
