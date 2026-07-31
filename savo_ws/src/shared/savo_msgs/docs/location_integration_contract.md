# Robot Savo typed location integration contract

This contract is locked by LOC-3P-C2-C5.

## Ownership

- `savo_head` owns camera-derived AprilTag observations and stable visual confirmation.
- `savo_mapping` owns map georeferencing and candidate construction.
- `savo_locations` is the only persistent location authority.
- `savo_supervisor` grants or denies permission and performs no location operation itself.
- `savo_nav` owns semantic navigation orchestration and Nav2 handoff.

No package other than `savo_locations` may maintain an authoritative location database.

## Locked graph

| Interface | Owner | Consumer |
|---|---|---|
| `/savo_head/apriltag/observations` | `savo_head` detector bridge | head confirmation action |
| `/savo_head/apriltag/confirm` | `savo_head` | mapping and navigation |
| `/savo_mapping/locations/register` | `savo_mapping` | operator or SavoMind workflow |
| `/savo_supervisor/authorize_location_operation` | `savo_supervisor` | mapping and navigation |
| `/savo_locations/candidates/register` | `savo_locations` | mapping |
| `/savo_locations/candidates/list` | `savo_locations` | operator CLI/app (read-only) |
| `/savo_locations/candidates/get` | `savo_locations` | mapping gateway and operator CLI (read-only) |
| `/savo_mapping/locations/review` | `savo_mapping` | operator CLI/app |
| `/savo_locations/resolve` | `savo_locations` | navigation |
| `/savo_nav/locations/navigate` | `savo_nav` | operator or SavoMind workflow |
| `/savo_nav/navigation/navigate_to_pose` | validated navigation gateway | semantic navigation |

## Registration transaction boundary

1. Supervisor authorizes candidate registration.
2. Head confirms fresh, stable AprilTag evidence.
3. Mapping builds and validates a candidate.
4. Mapping calls `savo_locations/candidates/register`.
5. Only the successful SQLite commit creates a candidate event.

Cancellation is accepted before persistence begins. Once the persistent registration request is in flight, cancellation is rejected until the commit acknowledgement is known.

## Candidate review transaction boundary

1. An operator tool lists or inspects candidates through read-only `savo_locations` services.
2. Approval or rejection is sent only to `/savo_mapping/locations/review` with the expected candidate revision.
3. Mapping reloads the authoritative candidate and rejects stale or terminal state before authorization.
4. Supervisor authorizes the matching non-motion operation using the stored map context.
5. Mapping forwards exactly one approval or rejection mutation to `savo_locations`.
6. Only the successful SQLite snapshot and event commit produces a completed review result.

The keyboard fallback is `ros2 run savo_mapping location_review_cli`. It never calls raw approval or rejection services directly.

## Navigation safety boundary

1. Navigation resolves the query through `savo_locations`.
2. The location must be approved, enabled and in the requested map context.
3. Supervisor authorizes motion before dispatch and periodically while motion is active.
4. Only `LocationRecord.approach_pose` is forwarded to the validated navigation gateway.
5. `tag_pose_map` is evidence and is never a motion target.
6. Authority loss, cancellation or timeout requests downstream navigation cancellation.

## Arrival confirmation

After Nav2 succeeds, the base remains at the approved approach pose. When confirmation is required, `savo_nav` requests `CONFIRM_ARRIVAL` from `savo_head` using the saved tag family and ID. A wrong, stale or unstable observation cannot produce semantic navigation success.

## Failure semantics

Every action returns a typed result code and deterministic reason. Dependency unavailability, supervisor denial, invalid map context, disabled locations, downstream rejection, timeout and cancellation fail closed. A navigation success flag and an arrival-confirmation flag are reported separately.

## Production lifecycle launch

`ros2 launch savo_bringup location_integration.launch.py` is the supported
cross-package orchestration boundary. It starts the registry, supervisor,
AprilTag observer and confirmation action, mapped-location registration,
authorized candidate review gateway, and semantic navigation. It does not
start a fake navigation server and does not replace the normal Nav2 or guarded
navigation bringup.

The launch exposes bounded overrides for the persistent database path,
supervisor localization requirement, head observation thresholds, downstream
service timeouts, the validated navigation action name, and arrival
confirmation timeout. Production defaults remain fail-closed; test overrides
are explicit launch arguments.

`ros2 run savo_bringup run_location_lifecycle_runtime` exercises the production
launch with synthetic observations and a fake Nav2 server. It proves the happy
path from fresh AprilTag evidence through candidate registration, authorized
review, alias resolution, approach-pose navigation, and saved-tag arrival
confirmation. The deeper `savo_nav` location integration smoke remains the
failure-path and persistence-restart test.
