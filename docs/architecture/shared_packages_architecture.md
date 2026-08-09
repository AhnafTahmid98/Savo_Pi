# Shared Packages Architecture

Shared source placement means reusable contract or role-specific implementation, not automatic deployment on every host.

| Package | Core | Edge | Workstation | Architectural ownership |
| --- | :---: | :---: | :---: | --- |
| `savo_msgs` | Yes | Yes | Dependency | Generated interfaces only |
| `savo_description` | Yes | Yes | Dependency | Geometry, URDF, fixed-frame contract |
| `savo_bringup` | Yes | Yes | No | Role/mode/profile orchestration and readiness |
| `savo_perception` | Yes | Yes | No | Core safety gate; optional Edge cloud filter |
| `savo_power` | Yes | Yes | Observe | Per-host monitors; Core aggregate |
| `savo_supervisor` | Yes | No | Observe | Core permission and persistent system state |
| `savo_bridge` | No | Yes | No | Typed SavoMind/ROS boundary |
| `savo_observer` | No | No | Yes | Read-only operator tooling |

Interfaces in `savo_msgs` carry data but do not own behavior. The producer/consumer packages retain validation, freshness, cancellation, and authority checks. Shared description is a hard cross-host frame contract; runtime TF must still have exactly one publisher per transform.

Core and Edge use different launch branches for perception and power. Core range fusion may stop motion; Edge obstacle-cloud filtering cannot replace the Core near-field gate. Edge UPS telemetry crosses ROS to the Core aggregate, but each UPS is on its host's independent I2C bus.

No shared package may become a shortcut around the ownership matrix. Role changes require deployment-array, dependency, launch, test, status, and documentation updates together. Validation is provided by package contract tests and role validators; distributed runtime behavior still requires two-host regression.
