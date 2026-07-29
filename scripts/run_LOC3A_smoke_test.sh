#!/usr/bin/env bash

set +e
set -o pipefail

ROOT="$HOME/Savo_Pi"
WS="$ROOT/savo_ws"
RUNTIME_ROOT="$ROOT/runtime/savo_locations_smoke"

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$RUNTIME_ROOT/LOC3A_$STAMP"

DB_FILE="$RUN_DIR/locations.db"
NODE_LOG="$RUN_DIR/savo_locations_node.log"
REPORT="$RUN_DIR/smoke_report.log"

mkdir -p "$RUN_DIR"

NODE_PID=""

cleanup()
{
  if [ -n "$NODE_PID" ] &&
     kill -0 "$NODE_PID" 2>/dev/null
  then
    kill -INT "$NODE_PID" 2>/dev/null
    sleep 2

    if kill -0 "$NODE_PID" 2>/dev/null
    then
      kill -TERM "$NODE_PID" 2>/dev/null
      sleep 1
    fi
  fi
}

trap cleanup EXIT INT TERM

exec > >(tee "$REPORT") 2>&1

echo "=================================================="
echo "Robot Savo — LOC-3A Smoke Test"
echo "=================================================="
echo "Run directory : $RUN_DIR"
echo "Database      : $DB_FILE"
echo "Node log      : $NODE_LOG"
echo "Report        : $REPORT"
echo

cd "$WS" || {
  echo "FAIL: workspace missing: $WS"
  exit 1
}

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

source /opt/ros/jazzy/setup.bash

if [ ! -f install/setup.bash ]; then
  echo "FAIL: workspace install/setup.bash is missing"
  exit 1
fi

source install/setup.bash

# Keep the smoke test isolated from the robot network.
export ROS_DOMAIN_ID=96
export ROS_LOCALHOST_ONLY=1

echo "ROS_DOMAIN_ID      : $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY : $ROS_LOCALHOST_ONLY"
echo

if ! ros2 pkg prefix savo_locations >/dev/null 2>&1; then
  echo "FAIL: savo_locations is not available in this environment"
  exit 1
fi

echo "Package prefix:"
ros2 pkg prefix savo_locations
echo

echo "Starting isolated LOC-3A registry node..."

stdbuf -oL -eL \
  ros2 run \
    savo_locations \
    savo_locations_node \
    --ros-args \
    -p database_path:="$DB_FILE" \
    >"$NODE_LOG" 2>&1 &

NODE_PID=$!

echo "Node PID: $NODE_PID"
echo

NODE_READY=0

for attempt in $(seq 1 40)
do
  if ! kill -0 "$NODE_PID" 2>/dev/null
  then
    echo "FAIL: node exited during startup"
    echo
    echo "=== Node log ==="
    cat "$NODE_LOG"
    exit 1
  fi

  if ros2 node list 2>/dev/null |
     grep -Fxq "/savo_locations"
  then
    NODE_READY=1
    break
  fi

  sleep 0.5
done

if [ "$NODE_READY" -ne 1 ]; then
  echo "FAIL: /savo_locations did not appear"
  echo
  echo "=== Node log ==="
  cat "$NODE_LOG"
  exit 1
fi

echo "PASS: /savo_locations is running"
echo

echo "=== Node information ==="

timeout 10 \
  ros2 node info /savo_locations

NODE_INFO_RESULT=$?

if [ "$NODE_INFO_RESULT" -ne 0 ]; then
  echo "FAIL: could not inspect /savo_locations"
  exit 1
fi

echo
echo "=== Expected read services ==="

EXPECTED_SERVICES=(
  "/savo_locations/resolve"
  "/savo_locations/get"
  "/savo_locations/list"
)

SERVICE_FAILURES=0

for service in "${EXPECTED_SERVICES[@]}"
do
  FOUND=0

  for attempt in $(seq 1 20)
  do
    if ros2 service list 2>/dev/null |
       grep -Fxq "$service"
    then
      FOUND=1
      break
    fi

    sleep 0.25
  done

  if [ "$FOUND" -eq 1 ]; then
    echo "PASS: $service"
  else
    echo "FAIL: $service is missing"
    SERVICE_FAILURES=$(
      (SERVICE_FAILURES + 1)
    )
  fi
done

echo
echo "=== Forbidden LOC-3A write services ==="

FORBIDDEN_SERVICES=(
  "/savo_locations/candidates/register"
  "/savo_locations/candidates/approve"
  "/savo_locations/set_enabled"
)

WRITE_SERVICE_FAILURES=0

for service in "${FORBIDDEN_SERVICES[@]}"
do
  if ros2 service list 2>/dev/null |
     grep -Fxq "$service"
  then
    echo "FAIL: write service unexpectedly exposed: $service"
    WRITE_SERVICE_FAILURES=$(
      (WRITE_SERVICE_FAILURES + 1)
    )
  else
    echo "PASS: not exposed: $service"
  fi
done

if [ "$SERVICE_FAILURES" -ne 0 ] ||
   [ "$WRITE_SERVICE_FAILURES" -ne 0 ]
then
  echo
  echo "FAIL: service contract verification failed"
  exit 1
fi

echo
echo "=== Status topic ==="

timeout 10 \
  ros2 topic echo \
    --once \
    --qos-reliability reliable \
    --qos-durability transient_local \
    /savo_locations/status \
    std_msgs/msg/String

STATUS_RESULT=$?

echo
echo "=== Snapshot topic ==="

timeout 10 \
  ros2 topic echo \
    --once \
    --qos-reliability reliable \
    --qos-durability transient_local \
    /savo_locations/snapshot \
    std_msgs/msg/String

SNAPSHOT_RESULT=$?

echo
echo "=== Heartbeat topic ==="

timeout 10 \
  ros2 topic echo \
    --once \
    /savo_locations/heartbeat \
    std_msgs/msg/UInt64

HEARTBEAT_RESULT=$?

echo
echo "=== Resolve service transport ==="

timeout 10 \
  ros2 service call \
    /savo_locations/resolve \
    savo_msgs/srv/ResolveLocation \
    "{
      query: 'loc3a-smoke-missing',
      enforce_map_context: false,
      map_id: '',
      map_revision: 0
    }"

RESOLVE_RESULT=$?

echo
echo "=== Get service transport ==="

timeout 10 \
  ros2 service call \
    /savo_locations/get \
    savo_msgs/srv/GetLocation \
    "{}"

GET_RESULT=$?

echo
echo "=== List service transport ==="

timeout 10 \
  ros2 service call \
    /savo_locations/list \
    savo_msgs/srv/ListLocations \
    "{}"

LIST_RESULT=$?

echo
echo "=== Permanent SQLite database inspection ==="

python3 - "$DB_FILE" <<'PY'
from pathlib import Path
import sqlite3
import sys


database = Path(sys.argv[1])

if not database.is_file():
    print(f"FAIL: database was not created: {database}")
    raise SystemExit(1)

connection = sqlite3.connect(str(database))

try:
    integrity = connection.execute(
        "PRAGMA integrity_check"
    ).fetchone()[0]

    user_version = connection.execute(
        "PRAGMA user_version"
    ).fetchone()[0]

    tables = [
        row[0]
        for row in connection.execute(
            """
            SELECT name
            FROM sqlite_master
            WHERE type = 'table'
            ORDER BY name
            """
        ).fetchall()
    ]

    print(f"Database path : {database}")
    print(f"Integrity     : {integrity}")
    print(f"Schema       : {user_version}")
    print(f"Tables       : {tables}")

    if integrity != "ok":
        raise SystemExit(1)

finally:
    connection.close()
PY

DATABASE_RESULT=$?

echo
echo "=== Node log ==="
cat "$NODE_LOG"

echo
echo "=================================================="
echo "LOC-3A smoke-test result summary"
echo "=================================================="
echo "status topic    : $STATUS_RESULT"
echo "snapshot topic  : $SNAPSHOT_RESULT"
echo "heartbeat topic : $HEARTBEAT_RESULT"
echo "resolve service : $RESOLVE_RESULT"
echo "get service     : $GET_RESULT"
echo "list service    : $LIST_RESULT"
echo "database check  : $DATABASE_RESULT"
echo

FINAL_RESULT=0

for result in \
  "$STATUS_RESULT" \
  "$SNAPSHOT_RESULT" \
  "$HEARTBEAT_RESULT" \
  "$RESOLVE_RESULT" \
  "$GET_RESULT" \
  "$LIST_RESULT" \
  "$DATABASE_RESULT"
do
  if [ "$result" -ne 0 ]; then
    FINAL_RESULT=1
  fi
done

if [ "$FINAL_RESULT" -eq 0 ]; then
  echo "LOC-3A SMOKE TEST: PASS"
else
  echo "LOC-3A SMOKE TEST: FAIL"
fi

echo
echo "Permanent run artifacts:"
echo "  $RUN_DIR"
echo

exit "$FINAL_RESULT"
