# Test Evidence Guidelines

## Purpose

This document defines the minimum evidence needed to support Robot Savo verification claims. A procedure, test binary, checklist mark, or historical result is not evidence that the current revision passed unless an execution record ties the result to that revision and environment.

## Result vocabulary

Use only these execution results:

| Result | Exact meaning |
| --- | --- |
| `PASS` | The required test was executed and met every defined acceptance criterion. |
| `FAIL` | The required test was executed and one or more acceptance criteria were not met. |
| `BLOCKED` | The test could not validly execute because a prerequisite, dependency, measurement, authorization, environment, hardware state, or earlier gate is missing. |
| `NOT RUN` | The test is in scope but has not yet been executed. |
| `N/A` | The test is formally outside the stated scope and the reason is documented. |

Do not use PASS for test existence, compilation alone, a partial stage, or “no failure was noticed.” Do not use BLOCKED to conceal an observed failure.

## Validation status

Keep maturity separate from execution result:

`Implemented`, `Source-validated`, `PC-validated`, `Target-validated`, `Hardware-validated`, `Integration-validated`, `Acceptance-validated`, `Blocked`, and `Deferred`.

Traceability may use evidence record labels `AUTOMATED_PASS`, `SOURCE_PASS`, `PC_PASS`, `TARGET_PASS`, `HARDWARE_PASS`, `INTEGRATION_PASS`, `ACCEPTANCE_PASS`, `BLOCKED`, `NOT_RUN`, and `HISTORICAL_BASELINE`. `HISTORICAL_BASELINE` never becomes current-source `HARDWARE_PASS` without applicable regression.

## Test identifiers

IDs are stable and use these families:

| Area | Prefix | Area | Prefix |
| --- | --- | --- | --- |
| Bringup | `BRG` | Base | `BAS` |
| Control | `CTL` | Description | `DSC` |
| Perception | `PER` | LiDAR | `LID` |
| Localization | `LOC` | Head | `HED` |
| RealSense | `RLS` | Visual odometry | `VO` |
| Speech | `SPH` | UI | `UI` |
| Bridge | `BRD` | Power | `PWR` |
| Supervisor | `SUP` | Locations | `LCT` |
| Mapping | `MAP` | Navigation | `NAV` |
| Observer | `OBS` | System/acceptance | `SYS` |

Use three digits, for example `BAS-006`. Do not recycle an obsolete ID for a different test. Mark retired IDs in the owning plan rather than renumbering later cases.

## Required execution metadata

Every run record must contain:

- test run ID and every test ID;
- date/time/timezone and, for machine correlation, UTC time;
- operator and reviewer where required;
- repository commit and working-tree state;
- host/hostname, OS, ROS distribution and middleware/domain;
- hardware revision and relevant serial/device identity;
- geometry profile/revision/canonical digest;
- configuration files, overrides, robot mode and bringup profile;
- map and location release IDs/digests where relevant;
- exact command or procedure and classification/risk;
- expected result and each acceptance criterion;
- observed result, measurements and result vocabulary value;
- logs, ROS bag, screenshots/photos/video or other artifacts as applicable;
- deviations, aborts/incidents and their references;
- post-test state and reviewer/final decision.

Redact secrets and unnecessary speech/image content. A redaction must not remove the identity, timing, configuration, result, or reason required to assess the test.

## Good evidence

- complete terminal log containing command, timestamps and exit status;
- `colcon test-result --verbose` tied to commit and host;
- ROS bag with topic/type/clock context and a small index/manifest;
- timestamped diagnostic dump or systemd journal;
- measurement CSV with units, instrument identity/calibration and reference;
- photo/video showing physical behavior, test setup and revision/config reference;
- screenshot with accompanying run record and state identity;
- immutable map/location release manifest and digests;
- completed [test result form](test_result_template.md) reviewed against attached artifacts.

## Weak or invalid evidence

The following cannot support production PASS by themselves:

```text
it worked
tested yesterday
seemed fine
screenshot without revision
video without configuration
test file exists
build once passed on an unknown host
```

Partial logs, hand-edited success text, copied historical output, or a bag without commit/config/time identity are also insufficient.

## Artifact naming

Use UTC and a stable run ID:

```text
YYYYMMDDTHHMMSSZ_<run-id>_<host>_<test-id-or-scope>_<artifact>.<ext>
```

Examples:

```text
20260809T011500Z_RUN-024_core_BRG-001_validator.txt
20260809T013100Z_RUN-024_observer_DSC-007_tf-tree.pdf
```

Do not rename historical evidence merely to match this convention; reference it with its original identity.

## Storage strategy

Small, reviewed, durable text/CSV/manifests/screenshots may live under `docs/logs/` or `docs/assets/testing/` when they are appropriate for Git. Do not add secrets, full runtime databases, huge ROS bags, raw audio/video, model data or generated build trees.

Large/sensitive artifacts belong in controlled external test storage. The Git record must retain an evidence manifest containing archive URI/record ID, cryptographic digest, size, content type, retention/access policy, and reviewer. External references must remain resolvable for the release retention period.

## Evidence integrity and review

1. Start a run record before executing the first test.
2. Capture exact repository/config/hardware identity.
3. Preserve raw output before summarizing it.
4. Hash immutable release artifacts and externally archived evidence.
5. Record deviations and aborts as they happen.
6. Have an independent reviewer sign motion, safety, release, persistent-state recovery and acceptance records.
7. Link the evidence record from traceability; do not paste unsupported PASS into the matrix.

## Historical hardware evidence

Record older physical evidence as `HISTORICAL_BASELINE` with the baseline revision if known. State which current changes touch the verified path and which regression IDs must run. If identity is incomplete, it remains contextual engineering evidence, not a current acceptance result.

## Privacy and security

- Store only speech transcripts/audio/images needed for the approved test.
- Never capture environment files, secrets, credentials or private keys in logs.
- Restrict bridge/socket and incident artifacts that contain peer identities or security details.
- Reference production databases read-only; use copied/isolated data for fault tests.

