# Robot Savo Incident Report

## Identification

| Field | Entry |
| --- | --- |
| Incident ID | |
| Date/time/timezone | |
| Location | |
| Severity category | Operational anomaly / Safety-relevant / Critical |
| Report author | |
| Operators/witnesses present | |

## System identity

| Field | Entry |
| --- | --- |
| Robot asset and hardware revision | |
| Core hostname, commit, active install | |
| Edge hostname, commit, active install | |
| SavoMind version | |
| Geometry profile/digest/state | |
| Robot mode and bringup profile | |
| Active map ID/revision/release | |
| Location release | |
| ROS domain/RMW and time-sync state | |

## Event

**Description and timeline:**

**Expected behavior:**

**Observed behavior:**

| Question | Answer/evidence |
| --- | --- |
| Physical motion involved? | |
| Person/property contact or near miss? | |
| Physical emergency control used? | |
| Software STOP/external stop response? | |
| Mission cancel response? | |
| Active command source/control mode? | |
| Sensor/safety status? | |
| Localization and TF state? | |
| Supervisor/authority/fault state? | |
| Power/UPS/shutdown-request state? | |
| Core–Edge network and clock state? | |
| Unexpected process/service restart? | |

## Evidence

- Service journals:
- ROS graph/topic-owner capture:
- Diagnostics/readiness snapshots:
- ROS bag and topic selection:
- Map/location/release artifacts:
- Kernel/hardware/network/time logs:
- Bridge/SavoMind evidence without secrets:
- Photos/video:
- Evidence hashes and storage location:

## Containment and investigation

**Immediate containment:**

**Suspected cause:**

**Confirmed root cause and supporting evidence:**

**Contributing factors:**

## Corrective action and validation

- Code/config changes and review links:
- Hardware/state changes:
- Calibration/geometry changes:
- Required regression tests:
- Regression results/evidence:
- Residual risk and operating restrictions:

## Return-to-service sign-off

| Decision | Entry |
| --- | --- |
| Release / Restricted release / Reject | |
| Reviewer and role | |
| Safety reviewer, if required | |
| Decision date/time/timezone | |
| Approved source/install/state identity | |
| Restrictions and expiry/review trigger | |
| Signatures/record references | |

An incident is not closed merely because the robot restarts or the symptom is
not reproduced.
