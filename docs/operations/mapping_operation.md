# Mapping Operation

## Purpose and classification

> [!WARNING]
> Manual and autonomous mapping can move the robot. Use a trained operator,
> spotter, clear site, verified stop control, and approved current-source scope.

Mapping creates session evidence. A saved map is not an approved production
map, and map creation is not release promotion.

## Common preconditions

- Pre-operation inspection, storage preflight, Core safety/localization/TF, and
  supervisor authority pass.
- Geometry is locked; SLAM is the sole `map -> odom` publisher.
- AMCL/saved-map navigation is not running concurrently.
- A unique `<map-id>`, revision plan, operator identity, and session evidence
  location have been assigned.
- `/var/lib/robot_savo/maps/sessions`, `production`, and
  `release_transactions` are writable with adequate free space.

Maintainers can run the non-destructive storage check on Core:

```bash
sudo bash deploy/common/preflight_storage_check.sh
```

## Manual mapping

The installed mapping unit is fail-closed. A maintainer must have deliberately
set `SAVO_ENABLE_MAPPING_SERVICE=true` in the deployed environment and created
`/etc/robot-savo/enable-mapping-service`. Operators must not create those gates
ad hoc.

1. Confirm Core is in `STOP` and the mapping service gate was released for this
   session.
2. Start and observe the installed unit:

   ```bash
   sudo systemctl start savo_mapping.service
   systemctl status savo_mapping.service --no-pager
   ros2 topic echo --once /savo_mapping/readiness
   ros2 topic echo --once /savo_mapping/session_state
   ```

3. Confirm SLAM map updates and TF continuity before moving.
4. Drive only with the [manual procedure](manual_drive_procedure.md). Maintain
   overlap and avoid moving objects, glass/reflection traps, and wheel slip.
5. Monitor mapping state, safety, localization, storage, and loop closures.
6. Stop driving and return control to `STOP` before save/review work.
7. Use the deployed mapping control surface to save the session through
   `/savo_mapping/map_session/save`; there is no routine shell save CLI.
8. Wait for artifact verification and quality results. Retain the session
   directory and logs even if quality fails.

## Autonomous mapping

The production entry is the typed `/savo_mapping/autonomous/run` action through
an approved bridge/operator client. This repository does not ship a routine
operator CLI for constructing that action goal. Do not call internal exploration,
coverage, Scan360, or Nav2 interfaces directly.

1. Verify autonomous mapping readiness and supervisor permission.
2. Submit a unique mission and map identity through the approved client.
3. Monitor `/savo_mapping/autonomous/status`, safety, localization, map quality,
   storage, and command cancellation capability.
4. Pause/resume/cancel only through the typed mission control surface.
5. At completion, verify map saved, hashes/artifacts, quality, semantic checks,
   return-to-start/scans, and terminal mission result.
6. Review the release explicitly. Approval requires mission/map/revision,
   review generation, actor identity, decision, and reason through the typed
   review service; no current operator CLI is shipped for map release review.
7. Confirm the joint map/location release transaction and active-map contract
   before considering the map production-ready.
8. Back up state after an approved release.

## Stop and failure handling

Cancel and return to `STOP` for SLAM loss, TF discontinuity, bad loop closure,
stale safety/localization, repeated navigation recovery, power/storage fault,
unexpected motion, or lost cancellation. Failed save, quality, review, release,
or rollback remains unreleased and requires maintainer evidence review. Never
edit `active_map.yaml` or map artifacts by hand.

Related: [mapping/navigation architecture](../architecture/mapping_navigation_architecture.md),
[mapping test plan](../testing/mapping_test_plan.md), and
[state administration](map_and_location_administration.md).
