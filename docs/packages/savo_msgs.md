# savo_msgs

## Purpose

Generated cross-package ROS 2 interface definitions. It has no runtime behavior or authority.

## Deployment

Built on Core and Edge and wherever consumers such as observer are built.

## Responsibilities

Version and generate stable messages, services, and actions for status, AprilTags, semantic locations, supervisor authority, and autonomous mapping.

## Non-responsibilities and authority boundaries

No node, persistence, validation policy, authorization, motion, TF, or hardware ownership. Producers/servers remain authoritative.

## Package structure

`msg/`, `srv/`, `action/`, CMake/manifest generation rules, and interface contract tests.

## Runtime components

No executable or node.

## Runtime data flow

Interfaces are compiled into type support used by other packages.

## ROS interfaces

### Messages

| Message | Purpose / primary ownership |
| --- | --- |
| `AprilTagObservation` | Head detector evidence consumed by confirmation/mapping/nav |
| `AutonomousMappingStatus` | Mapping-owned contract-v4 mission state |
| `FrontierExplorationStatus` | Typed frontier/exhaustion evidence from mapping |
| `IntentResult` | Legacy/compatibility parsed intent result; current SavoMind command authority crosses `savo_bridge` |
| `LocationCandidate` | Pending/reviewed candidate evidence |
| `LocationEvent` | Persistent registry mutation event |
| `LocationRecord` | Approved semantic destination and approach/confirmation poses |
| `NavState` | Navigation state summary |
| `RobotStatus` | Robot-wide status summary |
| `SemanticInterruptionStatus` | Mapping semantic interruption state |

### Services

| Service | Purpose / primary server |
| --- | --- |
| `ApproveLocation`, `RejectLocationCandidate`, `ReviewLocationCandidate` | Operator-reviewed candidate mutation; locations/mapping gateway |
| `RegisterLocationCandidate`, `GetLocationCandidate`, `ListLocationCandidates` | Candidate registry lifecycle |
| `GetLocation`, `ListLocations`, `ResolveLocation`, `SetLocationEnabled` | Semantic catalog query/administration |
| `PrepareLocationRelease`, `VerifyLocationRelease`, `CommitLocationRelease`, `RollbackLocationRelease` | AM-8 location release transaction |
| `RecoverLocationStorage` | Explicit integrity-checked locations storage recovery |
| `AuthorizeOperation`, `AuthorizeLocationOperation`, `UpdateMapContext`, `ManageSystemState` | Supervisor authority/system state |
| `ControlAutonomousMapping`, `ReviewAutonomousMappingRelease` | Mapping pause/resume/cancel/status and operator release decision |
| `SubmitSemanticLocation` | Operator semantic data for active mapping interruption |

### Actions

| Action | Purpose / server |
| --- | --- |
| `ConfirmAprilTag` | Head confirmation for registration/arrival |
| `ExecuteCoveragePath` | Nav guarded coverage execution requested by mapping |
| `NavigateToLocation` | Nav semantic destination workflow |
| `RegisterMappedLocation` | Mapping candidate construction/persistence workflow |
| `RotateToHeading` | Control bounded yaw maneuver |
| `RunAutonomousMapping` | Mapping contract-v3 Supervisor-bound autonomous mission |

### Published topics

Not applicable; definitions do not publish.

### Subscribed topics

Not applicable.

## TF ownership

None.

## Parameters and configuration

None. Contract versions/constants live in `.msg/.srv/.action` definitions.

## Launch files

None.

## Persistent state and runtime files

None; generated build artifacts only.

## Hardware ownership

None.

## Dependencies

### Internal Robot Savo dependencies

Used across head, mapping, nav, locations, supervisor, bridge, control, UI/observer.

### External ROS/system dependencies

`rosidl_default_generators/runtime`, builtin/std/geometry/nav message packages.

## Safety behavior

Interfaces encode result codes, generations, context, timeouts, and evidence but do not enforce them. Server implementations must reject invalid/stale requests.

## Failure and degraded behavior

Type/version mismatch prevents build or communication; no fallback schema is invented.

## Startup and shutdown behavior

Not applicable.

## Build

Built before consumers by role scripts.

## Run

No runtime command.

## Validation and testing

Tests check registration, constants/fields, and AprilTag, mapping, coverage, location, semantic interruption, and supervisor contracts.

## Current validation status

Implemented and source-validated through interface tests; integration validity depends on current producers/consumers.

## Known limitations and remaining validation

`IntentResult` contains planning-era wording and is not the current generic SavoMind authority path; removal/migration requires consumer audit.

## Change-control considerations

Interface field/constants are compatibility changes requiring all role consumers to rebuild and contract-version review.

## Related documentation

- [Implementation README](../../savo_ws/src/shared/savo_msgs/README.md)
- [ROS topic contracts](../architecture/ros2_topic_contracts.md)
- [Ownership matrix](package_ownership_matrix.md)
