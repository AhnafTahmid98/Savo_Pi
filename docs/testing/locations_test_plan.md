# Locations Test Plan

## Objective

Verify the authoritative SQLite candidate/catalog/release lifecycle, operator-only approval, map/geometry association, persistence, recovery, and named lookup using isolated data for destructive tests.

## Scope

Schema/migrations, normalization/duplicates, candidate create/inspect/approve/reject, operator identity, approved records, release prepare/verify/commit/rollback, active context, backup/restore/corruption, and supported enable/disable behavior. No physical navigation is authorized here.

## Test ownership

Locations maintainer owns LCT-001–007; mapping/navigation/operator integration owners own LCT-008–010.

## Safety classification

LCT-001–010 are `STATIC`, `UNIT`, `PC`, `TARGET-NON-HARDWARE`, `PERSISTENT-STATE`, `FAULT-INJECTION`, `RECOVERY`, or `INTEGRATION` / `NO-MOTION`. Production data must never be destructively tested.

## Preconditions

Temporary or copied SQLite/release root, current schema/policy, known map/revision/geometry digest fixtures, named operator test identity, backups, and STOP.

## Required hardware

None. Physical tag capture and navigation are external integration stages after this plan.

## Required software / configuration

SQLite3; storage/location policy; `savo_msgs`; production directory provisioning; backup/restore tools. Production paths are inspected read-only unless explicitly commissioning a fresh environment.

## Interfaces under test

Status/snapshot/heartbeat/events and all typed location/candidate/storage/release services documented in [the package page](../packages/savo_locations.md). `NavigateToLocation` belongs to nav; `RegisterMappedLocation` belongs to mapping.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| LCT-001 | T0 `STATIC` | Verify schema/policy, production paths, deployment-created parent policy, WAL/foreign keys/integrity, map/revision/tag/approach rules, service ownership, and operator/SavoMind boundary. |
| LCT-002 | T1 `UNIT`/`PC` | Build/tests pass for normalization/validation, schema/migration/integrity, catalog/registry, persistence/mutations, ROS read/write/hardening, recovery and release contracts. |
| LCT-003 | T2 `TARGET-NON-HARDWARE` | Create isolated database; verify schema/version, empty/bootstrap status, permissions policy, restart persistence and current read snapshot. |
| LCT-004 | T2 `PERSISTENT-STATE` | Register candidates with map/revision/tag/pose evidence; test duplicate ID/tag/name/alias, stale revision, missing approach pose, ambiguous lookup and mismatch rejection. |
| LCT-005 | T2 `PERSISTENT-STATE` | List/inspect candidates; approve/reject with authorized operator identity and reasons; verify events, immutable audit fields, enabled/disabled behavior, and no SavoMind approval route. |
| LCT-006 | T2 `PERSISTENT-STATE` | Resolve approved name/alias/ID under matching context; pending/rejected/disabled/mismatched/ambiguous records fail closed. |
| LCT-007 | T7 `PERSISTENT-STATE`/`RECOVERY` | Prepare/verify/commit/rollback a location release associated with exact map/revision and geometry context used by current contract; validate digest/active identity and restart. |
| LCT-008 | T6 `FAULT-INJECTION` | On copied data, simulate write failure, busy DB, truncated/corrupt/newer schema and release mismatch; writes latch unavailable, last committed reads behave as documented, explicit fresh-connection recovery is required. |
| LCT-009 | T7 `RECOVERY` | Backup and restore isolated state using documented tooling; verify database/release identities/digests, ownership and rollback. Delete/retire is N/A unless current implemented service/policy supports it; `set_enabled` is tested instead. |
| LCT-010 | T5 `INTEGRATION` | Mapping supplies a real candidate, authorized operator reviews it, release transaction completes, and nav resolves the approved approach pose under the same active map/location release. Physical navigation remains NAV plan scope. |

## Pass criteria

All mutations are atomic/authorized/audited; duplicates and context mismatches reject; only approved enabled records resolve; release identity/digests persist; corruption/write fault fails closed; backup/restore is verified.

## Blocked criteria

No isolated storage, missing approved map/geometry context for release integration, unavailable operator authorization, or unsupported delete/retire behavior.

## Failure criteria

Production data modified by a negative test, unauthorized approval, duplicate/ambiguous/context-invalid record accepted, corrupt state trusted, uncommitted write visible, or release mismatch active.

## Abort criteria

Stop immediately if a command targets `/var/lib/robot_savo` production data unintentionally, backup identity is unknown, filesystem is failing, or authorization boundary is bypassed.

## Evidence to retain

Commit/schema/config, isolated paths, pre/post database/release hashes, service requests/results, operator identity, event journal, integrity output, backup/restore/corruption logs, map/geometry/location release identities.

## Regression triggers

Schema/migration/storage path, normalization/identity/duplicates, map/pose/tag policy, service/message contract, operator authorization, write recovery, release digest/transaction/backup.

## Current validation status

Extensive unit/contract/runtime tests exist. Current target permissions, physical candidate/operator flow, joint release, restart, backup/restore and named navigation integration require evidence.

## Related documentation

- [Locations package](../packages/savo_locations.md)
- [Map/location administration](../operations/map_and_location_administration.md)
- [Mapping plan](mapping_test_plan.md)
- [Navigation plan](navigation_test_plan.md)

