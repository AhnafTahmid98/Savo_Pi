# Failure and Abort Criteria

## Purpose

These rules apply to every Robot Savo test. An owning test plan may add stricter conditions but may not weaken these controls.

## Immediate physical abort

Immediately command STOP when available, use the physical emergency-stop/isolation path, and remove actuator power when safe for any of the following:

- uncommanded motion or motion in the wrong direction;
- inability to stop, cancel, or quench actuator output;
- emergency-stop failure or uncertainty;
- person/property hazard or entry into the exclusion zone;
- motor/servo stall, collision, mechanical obstruction or cable tension;
- smoke, arcing, excessive heat, electrical smell, swelling or unsafe battery event;
- robot shifting/falling from stands or test fixture failure;
- unsafe speed/acceleration/route divergence;
- loss of required safety sensing during an actuating test.

Do not resume merely because the symptom disappears.

## Controlled software abort

End the active procedure safely, request STOP/cancel, prevent new operations and preserve state when any of these occurs:

- localization loss, jump, divergence or duplicate/discontinuous TF;
- safety sensor, readiness, authority, heartbeat or command source becomes stale/unexpected;
- command appears on an unapproved lane or persists after timeout;
- supervisor authority/map context/release/geometry mismatch;
- repeated or unbounded recovery, backend/lifecycle loss or cancellation timeout;
- time synchronization failure affecting sensor correlation;
- duplicate node/hardware/TF owner;
- corrupted or unexpectedly modified persistent state;
- wrong peer/credential, malformed command acceptance or security-boundary failure;
- production data is targeted by a destructive fault test.

Escalate to immediate physical abort if motion or physical safety is uncertain.

## Test failure

Use `FAIL` when the test executed but any acceptance criterion was not met. A safe end state does not convert a failure to BLOCKED. Examples include wrong frame/type/sign, measured behavior outside an approved limit, stale data shown current, denied STOP, unexpected publisher, hash mismatch or unsuccessful recovery.

Every FAIL records observed facts, safe terminal state, evidence, affected requirement, issue/incident reference as applicable, corrective action and prerequisite regression.

## Test block

Use `BLOCKED` only when valid execution cannot start or complete because a prerequisite, dependency, environment, authorization, hardware state, measurement or earlier gate is absent. Record the exact blocker and owner. Do not create a simulated PASS for a blocked hardware criterion.

Use `NOT RUN` when an in-scope test simply has not been attempted. Use `N/A` only with a formal scope reason.

## Mandatory post-abort sequence

1. **STOP** — request the supported STOP/cancel path if it is trustworthy; use physical emergency stop when needed.
2. **Isolate** — remove actuator/servo/power/network/command authority appropriate to the hazard.
3. **Verify** — confirm `/cmd_vel_safe` and actuator outputs are zero, missions are terminal, and the area is safe.
4. **Preserve evidence** — keep logs, bags, state files, correlation IDs, device/config identity, photos/video and timestamps. Do not restart first if it would destroy evidence.
5. **Record state** — complete the abort/incident section in the result record; create an incident report when safety/security/data integrity was affected.
6. **Diagnose** — identify the cause and scope; do not change production behavior merely to make the test pass.
7. **Correct** — review the change, update controlled configuration/documentation and choose regression from the matrix.
8. **Retest prerequisites** — rerun every invalidated lower gate before the failed test.
9. **Explicitly reauthorize** — motion, destructive state tests and return to service require the named operator/reviewer approval again.

## Stop and evidence priority

Human safety and electrical isolation take precedence over evidence preservation. Once safe, preserve rather than clean up. Do not delete a failed session/database/log, reset a latch, power-cycle, restart nodes or overwrite active release context until the evidence owner approves.

## Return to service

An abort or safety-critical FAIL removes the affected operating scope from service. Return requires:

- cause and impact documented;
- corrective change reviewed;
- required R-level regression completed with evidence;
- every safety prerequisite PASS;
- open deviations explicitly accepted by authorized reviewer;
- final safe state and operating envelope recorded;
- explicit return-to-service approval, never inferred from process restart.

