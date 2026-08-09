# Diagnostics and Observability

Diagnostics expose evidence; they do not grant control authority. Core, Edge, and the workstation have distinct views.

## Signals

Packages publish combinations of retained state, health, readiness, heartbeat counters, and `diagnostic_msgs/DiagnosticArray`. Bringup aggregates required dependencies without replacing package-owned health. The Core/Edge readiness namespaces prevent collisions. TF tools, topic introspection, Chrony, systemd journals, and deployment health scripts supplement ROS diagnostics.

`savo_ui` provides an optional local 800 x 480 read-only view. `savo_observer` provides read-only telemetry/RViz/browser tooling on a workstation. `savo_bridge` exposes a bounded snapshot to SavoMind. None may publish mutation interfaces or infer freshness from a retained sample alone.

## Failure interpretation

A heartbeat proves recent process publication, not semantic health. A health string must be checked for expected schema/state and freshness. Bringup treats missing/stale required inputs as waiting then blocked; optional unhealthy inputs become degraded. Safety, control, localization, navigation, and operation owners retain their own stricter fail-closed decisions.

Core persistent logs default under `/var/log/robot_savo`; Edge uses its ROS log location unless overridden by service configuration. Log rotation is deployed for Core paths. Evidence records should include revision, host, role, mode/profile, command, timestamps/time-sync state, result, and artifact location.

Source validators check observer read-only behavior, required topics/assets, role launch, and aggregate pre-real readiness. Target validation must verify diagnostic QoS, stale detection, clock jumps, process/link loss, disk pressure, log rotation, UI freshness labels, observer disconnect, and that no diagnostic/presentation path can mutate robot state.
