# savo_bridge

`savo_bridge` is the native C++ security and compatibility boundary between
Robot SAVO ROS 2 and SavoMind. The package is stored under `src/shared` but one
production instance runs on `savo-edge`. DDS already connects the core and edge
computers; this package is not a network bridge between the Pis.

## Authority boundary

The bridge never owns motors, safety, mapping, navigation, locations, or
supervision. It validates a closed command protocol and then calls only the
public typed interfaces owned by those packages. It has no generic ROS
topic/service/action escape hatch.

Allowed command families are:

- emergency `stop` and active-action cancellation;
- bounded single-axis `teleop_nudge`;
- navigation to an approved named location and navigation cancellation;
- read-only navigation status;
- autonomous mapping start with `auto_save=true` and
  `require_quality_approval=true`;
- autonomous mapping pause, resume, cancel, and typed Scan360 request;
- read-only mapping status including save, verification, location review, and
  AM-8 release progress;
- read-only supervisor status.

Manual mapping start and standalone save/release mutation are intentionally not
exposed because the current workspace has no bridge-safe typed authority for
them. Operator approval is never accepted from SavoMind.

## Fail-closed map context

The production profile and launch defaults use an empty map ID and revision 0.
Navigation remains blocked until a fresh `/savo_nav/map_context/status` reports
a synchronized active map, positive revision, and real `map_release_id`. The
service and runner do not invent `saved_map` or revision 1.

## Unix-socket command contract

The protected Unix socket uses strict bounded JSON with:

- protocol and message type validation;
- command ID, source, agent, priority, issuance time, and expiry;
- duplicate-key and unknown-field rejection;
- UID authentication through `SO_PEERCRED`;
- duplicate/replay handling;
- accepted/rejected acknowledgement;
- dispatch evidence and typed result details;
- bounded timeouts and cancellation correlation.

STOP has the highest priority. Teleoperation remains bounded. Navigation accepts
location IDs, not arbitrary poses. Autonomous mapping cannot disable saving or
quality approval.

## Observation and snapshot

The bridge observes configured topics, tracks monotonic freshness, records ROS
graph evidence, and atomically writes the schema-v2 snapshot using a
same-directory temporary file, `fsync`, rename, and parent-directory `fsync`.
SavoMind must treat unavailable or stale required observations as blocked.

Production paths:

```text
/run/savo_bridge/command.sock
/run/savo_bridge/snapshot.json
```

The systemd template creates `/run/savo_bridge` with group
`savomind-bridge`, mode `0770`, and a `0660` command socket.

## Build and test

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to savo_bridge --symlink-install
source install/setup.bash
colcon test --packages-select savo_bridge --ctest-args --output-on-failure
colcon test-result --verbose
```

## Edge installation

Installation is inert unless `--start` is supplied:

```bash
sudo "$SAVO_WORKSPACE/install/savo_bridge/lib/savo_bridge/install_edge_runtime.sh" \
  --user "$USER" \
  --workspace "$SAVO_WORKSPACE"
```

Add `--start` only during the guarded edge deployment gate. The real active map
context must come from the core navigation/release pipeline.
