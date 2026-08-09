# Robot Savo Operations

These runbooks are for trained Robot Savo operators and maintainers after the
robot has been installed and commissioned. They do not replace deployment,
engineering tests, site safety rules, or physical emergency controls.

## Roles and authority

| Role | Permitted work |
| --- | --- |
| Operator | Inspections, approved startup, STOP, released missions, routine shutdown |
| Maintainer | Services, logs, backups, restore, hardware inspection, controlled recovery |
| Developer | Source, ROS interface debugging, builds, configuration and protocol changes |
| Safety reviewer | Return-to-service decision after safety-relevant or critical incidents |

`safe_idle` is the production startup mode. `STOP` is the Core control state
that selects no motion command lane. Neither state replaces physical isolation.
Motion remains subject to supervisor permission, operation-owner admission,
`savo_control`, the `savo_perception` gate, and the independent base watchdog.

## Reading paths

### Daily operator

1. [Quick start](operator_quick_start.md)
2. [Pre-operation inspection](pre_operation_inspection.md)
3. [Startup and shutdown](startup_and_shutdown.md)
4. [Manual drive](manual_drive_procedure.md) or
   [navigation](navigation_operation.md)

### Mapping operator

1. [Pre-operation inspection](pre_operation_inspection.md)
2. [Startup and shutdown](startup_and_shutdown.md)
3. [Mapping operation](mapping_operation.md)
4. [Map and location administration](map_and_location_administration.md)

### Maintainer

- [Troubleshooting](troubleshooting.md)
- [Log collection](log_collection.md)
- [Maintenance schedule](maintenance_schedule.md)
- [Backup, restore, and rollback](backup_restore_and_rollback.md)

### Incident response

1. [Emergency stop and recovery](emergency_stop_and_recovery.md)
2. [Incident response](incident_response.md)
3. [Incident report template](incident_report_template.md)

## Motion classification

The manual-drive, mapping, and navigation runbooks can cause motion. Location
approval is non-motion, but arrival confirmation may move the head. Startup is
non-motion only while `safe_idle` and `STOP` are confirmed. All other pages are
normally non-motion maintenance or evidence procedures.

Stop immediately for unexpected motion, loss of stop capability, stale safety
state, localization/TF discontinuity during autonomous motion, power fault,
failed cancellation, or any person/property risk. Use physical isolation first
when software response is too slow or uncertain.

See the [current system status](../status/current_system_status.md),
[safety architecture](../architecture/safety_architecture.md), and
[real-robot acceptance checklist](../testing/real_robot_acceptance_checklist.md)
for release gates. Prior physical baseline evidence does not replace regression
of the current source and configuration.
