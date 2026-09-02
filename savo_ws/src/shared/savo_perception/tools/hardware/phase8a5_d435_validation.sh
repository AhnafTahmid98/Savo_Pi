#!/usr/bin/env bash

set -eo pipefail
set -E

# Phase 8A5 is an operator-driven, stationary hardware acceptance test.
# It intentionally does not source or run any Robot Savo motion, navigation,
# recovery, lifecycle, or command-authority executable.

EXPECTED_SERIAL="801212070967"
EXPECTED_FIRMWARE="5.16.0.1"
EXPECTED_HOST="savo-edge"
RAW_TOPIC="/camera/camera/depth/color/points"
FILTERED_TOPIC="/savo_perception/obstacles/points"
HEALTH_TOPIC="/savo_perception/obstacle_cloud/health"
STATUS_TOPIC="/savo_perception/obstacle_cloud/status"
HEARTBEAT_TOPIC="/savo_perception/obstacle_cloud/heartbeat"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
ROS_DOMAIN_ID="${SAVO_PHASE8A5_ROS_DOMAIN_ID:-85}"
ROS2CLI_NO_DAEMON=1
PYTHONDONTWRITEBYTECODE=1

export ROS_DOMAIN_ID
export ROS2CLI_NO_DAEMON
export PYTHONDONTWRITEBYTECODE

WORKSPACE="${SAVO_WORKSPACE:-${HOME}/Savo_Pi/savo_ws}"
PERCEPTION_DIR="$WORKSPACE/src/shared/savo_perception"
REALSENSE_DIR="$WORKSPACE/src/edge/savo_realsense"
DESCRIPTION_DIR="$WORKSPACE/src/shared/savo_description"
FILTER_CONFIG="$PERCEPTION_DIR/config/edge/obstacle_cloud_filter.yaml"
REALSENSE_CONFIG="$REALSENSE_DIR/config/realsense_pointcloud_camera.yaml"
HARDWARE_FIXTURE="$PERCEPTION_DIR/test/fixtures/obstacle_cloud_filter_hardware_fixture.py"
SCRIPT_PATH="/tmp/savo_perception_phase8a5_d435_validation.sh"

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="/tmp/savo_perception_phase8a5_${RUN_STAMP}"
ROS_LOG_DIR="$RUN_DIR/ros_log"
REPORT_FILE="$RUN_DIR/hardware_report.txt"
EVIDENCE_DIR="$RUN_DIR/evidence"

export ROS_LOG_DIR

PROCESS_NAMES=()
PROCESS_PIDS=()
PROCESS_PGIDS=()
PROCESS_ACTIVE=()
PROCESS_TOKENS=()
LAST_PROCESS_INDEX=""
LAST_FOREGROUND_STATUS=0
LAST_FIXTURE_ALLOWED_FAILURES=0
FIXTURE_ALLOWED_FAILURE_CODES=""
CAMERA_INDEX=""
FILTER_INDEX=""
DESCRIPTION_INDEX=""
BACKUPS_CREATED=0
SOURCE_CLEANUP_ALLOWED=0
SUCCESS=0
FAIL_REASON=""
RESIDUAL_RESULT="NOT_RUN"
AUTHORITY_RESULT="NOT_RUN"
STALE_RESULT="NOT_RUN"
RECOVERY_RESULT="NOT_RUN"
SELF_FILTER_RESULT="NOT_RUN"
PERCEPTION_TOTALS="NOT_RUN"
REALSENSE_TOTALS="NOT_RUN"
D435_FIRMWARE="unknown"
USB_MODE="unknown"
USB_SPEED_MBPS="unknown"
RAW_FRAME="unknown"
TF_CHAIN="unknown"
ORIGINAL_SELF_FILTER="unknown"
FINAL_SELF_FILTER="unknown"

mkdir -p "$ROS_LOG_DIR" "$EVIDENCE_DIR"
: >"$REPORT_FILE"

timestamp()
{
  date --iso-8601=seconds
}

report()
{
  printf '%s\n' "$*" | tee -a "$REPORT_FILE"
}

section()
{
  report ""
  report "=== $* ==="
}

fail()
{
  FAIL_REASON="$*"
  report "BLOCKER: $*"
  exit 1
}

require_file()
{
  [[ -f "$1" ]] || fail "Required file is missing: $1"
}

require_directory()
{
  [[ -d "$1" ]] || fail "Required directory is missing: $1"
}

require_command()
{
  command -v "$1" >/dev/null 2>&1 ||
    fail "Required command is unavailable: $1"
}

group_is_alive()
{
  local process_group="$1"

  [[ -n "$process_group" ]] &&
    ps -e -o pgid=,stat= |
      awk -v group="$process_group" '
        $1 == group && $2 !~ /^Z/ {
          found = 1
        }
        END {
          exit(found ? 0 : 1)
        }
      '
}

group_is_owned()
{
  local process_group="$1"
  local process_token="$2"
  local member_pid
  local member_count=0
  local token_line="SAVO_PHASE8A5_PROCESS_TOKEN=$process_token"
  local member_pids=()

  mapfile -t member_pids < <(
    ps -e -o pid=,pgid=,stat= |
      awk -v group="$process_group" '
        $2 == group && $3 !~ /^Z/ {
          print $1
        }
      '
  )

  for member_pid in "${member_pids[@]}"; do
    if [[ ! -r "/proc/$member_pid/environ" ]]; then
      if kill -0 "$member_pid" 2>/dev/null; then
        return 1
      fi
      continue
    fi

    if ! grep -zFxq "$token_line" "/proc/$member_pid/environ"
    then
      return 1
    fi

    ((member_count += 1))
  done

  ((member_count > 0))
}

wait_for_group_exit()
{
  local process_group="$1"
  local attempts="$2"
  local attempt

  for ((attempt = 0; attempt < attempts; ++attempt)); do
    if ! group_is_alive "$process_group"; then
      return 0
    fi

    sleep 0.2
  done

  return 1
}

start_tracked_process()
{
  local process_name="$1"
  local log_file="$2"
  local process_id
  local process_group
  local process_index
  local process_token

  shift 2

  process_token="${RUN_STAMP}_${#PROCESS_PIDS[@]}_${RANDOM}_${RANDOM}"

  SAVO_PHASE8A5_PROCESS_TOKEN="$process_token" \
    setsid "$@" >>"$log_file" 2>&1 &
  process_id=$!

  # Record ownership immediately. Even if the leader exits before the PGID
  # lookup, the EXIT trap can still identify and clean descendants inherited
  # from this setsid group.
  PROCESS_NAMES+=("$process_name")
  PROCESS_PIDS+=("$process_id")
  PROCESS_PGIDS+=("$process_id")
  PROCESS_ACTIVE+=("1")
  PROCESS_TOKENS+=("$process_token")
  process_index="$((${#PROCESS_PIDS[@]} - 1))"
  LAST_PROCESS_INDEX="$process_index"

  sleep 0.25

  if ! kill -0 "$process_id" 2>/dev/null; then
    wait "$process_id" 2>/dev/null || true
    report "Process log: $log_file"
    sed -n '1,240p' "$log_file" | tee -a "$REPORT_FILE"
    fail "$process_name exited during startup"
  fi

  process_group="$(
    ps -o pgid= -p "$process_id" 2>/dev/null |
      tr -d '[:space:]'
  )"

  [[ -n "$process_group" ]] ||
    fail "Could not resolve process group for $process_name"

  [[ "$process_group" == "$process_id" ]] ||
    fail "$process_name did not start in its own setsid process group"

  PROCESS_PGIDS[$process_index]="$process_group"

  group_is_owned "$process_group" "$process_token" ||
    fail "Could not prove process-group ownership for $process_name"

  report "Started $process_name pid=$process_id pgid=$process_group"
}

stop_tracked_process()
{
  local process_index="$1"
  local process_name="${PROCESS_NAMES[$process_index]}"
  local process_id="${PROCESS_PIDS[$process_index]}"
  local process_group="${PROCESS_PGIDS[$process_index]}"
  local process_token="${PROCESS_TOKENS[$process_index]}"

  if [[ "${PROCESS_ACTIVE[$process_index]}" != "1" ]]; then
    return 0
  fi

  if group_is_alive "$process_group"; then
    if ! group_is_owned "$process_group" "$process_token"; then
      report \
        "Refusing to signal unverified/reused pgid=$process_group for $process_name"
      return 1
    fi

    report "Stopping $process_name pgid=$process_group with SIGINT"
    kill -INT -- "-$process_group" 2>/dev/null || true

    if ! wait_for_group_exit "$process_group" 40; then
      if ! group_is_owned "$process_group" "$process_token"; then
        report \
          "Refusing SIGTERM for unverified/reused pgid=$process_group"
        return 1
      fi

      report "Escalating $process_name pgid=$process_group to SIGTERM"
      kill -TERM -- "-$process_group" 2>/dev/null || true
    fi

    if ! wait_for_group_exit "$process_group" 25; then
      if ! group_is_owned "$process_group" "$process_token"; then
        report \
          "Refusing SIGKILL for unverified/reused pgid=$process_group"
        return 1
      fi

      report "Escalating $process_name pgid=$process_group to SIGKILL"
      kill -KILL -- "-$process_group" 2>/dev/null || true
    fi

    if ! wait_for_group_exit "$process_group" 15; then
      report "Residual process group after SIGKILL: $process_group"
      return 1
    fi
  fi

  wait "$process_id" 2>/dev/null || true
  PROCESS_ACTIVE[$process_index]="0"
  report "Stopped $process_name pgid=$process_group"
}

run_tracked_foreground()
{
  local process_name="$1"
  local log_file="$2"
  local process_index
  local process_id
  local process_group
  local process_status

  shift 2
  start_tracked_process "$process_name" "$log_file" "$@"
  process_index="$LAST_PROCESS_INDEX"
  process_id="${PROCESS_PIDS[$process_index]}"
  process_group="${PROCESS_PGIDS[$process_index]}"

  set +e
  wait "$process_id"
  process_status=$?
  set -e

  if group_is_alive "$process_group"; then
    stop_tracked_process "$process_index" || true
  else
    PROCESS_ACTIVE[$process_index]="0"
  fi

  LAST_FOREGROUND_STATUS="$process_status"

  if [[ $process_status -ne 0 ]]; then
    report "Process log: $log_file"
    sed -n '1,320p' "$log_file" | tee -a "$REPORT_FILE"
  fi

  return 0
}

cleanup_all_processes()
{
  local process_index
  local cleanup_status=0

  for ((
    process_index = ${#PROCESS_PIDS[@]} - 1;
    process_index >= 0;
    --process_index
  )); do
    stop_tracked_process "$process_index" || cleanup_status=1
  done

  return "$cleanup_status"
}

audit_tracked_processes()
{
  local process_index
  local process_group
  local process_token
  local residual=0

  for ((process_index = 0;
    process_index < ${#PROCESS_PGIDS[@]};
    ++process_index)); do
    process_group="${PROCESS_PGIDS[$process_index]}"
    process_token="${PROCESS_TOKENS[$process_index]}"

    if group_is_alive "$process_group"; then
      if group_is_owned "$process_group" "$process_token"; then
        report \
          "Residual tracked group: ${PROCESS_NAMES[$process_index]} pgid=$process_group"
        residual=1
      else
        report \
          "Tracked pgid=$process_group is live but is not owned by Phase 8A5; treating it as a reused/unrelated ID and leaving it untouched"
      fi
    fi
  done

  if [[ $residual -eq 0 ]]; then
    RESIDUAL_RESULT="PASS"
    report "Residual Phase 8A5 process audit: PASS"
    return 0
  fi

  RESIDUAL_RESULT="FAIL"
  return 1
}

clean_source_artifacts()
{
  local source_root

  for source_root in \
    "$PERCEPTION_DIR" \
    "$REALSENSE_DIR" \
    "$WORKSPACE/install/savo_perception" \
    "$WORKSPACE/install/savo_realsense"
  do
    [[ -d "$source_root" ]] || continue

    find "$source_root" -type f \
      \( -name '*.pyc' -o -name '*.pyo' \) -delete

    find "$source_root" -depth -type d \
      \( -name '__pycache__' -o -name '.pytest_cache' \) \
      -exec rm -rf -- {} +
  done
}

assert_no_source_artifacts()
{
  local artifact_root
  local artifact_roots=()
  local unexpected

  for artifact_root in \
    "$PERCEPTION_DIR" \
    "$REALSENSE_DIR" \
    "$WORKSPACE/install/savo_perception" \
    "$WORKSPACE/install/savo_realsense"
  do
    if [[ -e "$artifact_root" ]]; then
      artifact_roots+=("$artifact_root")
    fi
  done

  if ! unexpected="$(
    find "${artifact_roots[@]}" \
      \( \
        -type f \( -name '*.pyc' -o -name '*.pyo' \) \
        -o -type d \( -name '__pycache__' -o -name '.pytest_cache' \) \
        -o -type f \
          \( \
            -name '*.bak' \
            -o -name '*.orig' \
            -o -name '*~' \
            -o -name '*.before_*' \
          \) \
      \) -print
  )"
  then
    report "Could not audit source/install cache and backup artifacts"
    return 1
  fi

  if [[ -n "$unexpected" ]]; then
    report "$unexpected"
    return 1
  fi

  report "No source/install cache or backup artifacts: PASS"
}

on_exit()
{
  local incoming_status=$?
  local final_status="$incoming_status"
  local cleanup_status=0

  trap - EXIT ERR INT TERM HUP
  set +e

  cleanup_all_processes
  cleanup_status=$?

  if [[ $SOURCE_CLEANUP_ALLOWED -eq 1 ]]; then
    clean_source_artifacts
    assert_no_source_artifacts || cleanup_status=1
  fi

  audit_tracked_processes || cleanup_status=1

  if [[ $final_status -eq 0 && $cleanup_status -ne 0 ]]; then
    final_status=1
    FAIL_REASON="cleanup or residual-process audit failed"
  fi

  if [[ $final_status -ne 0 ]]; then
    report ""
    report "Phase 8A5 is NOT complete."

    if [[ -n "$FAIL_REASON" ]]; then
      report "Exact blocker: $FAIL_REASON"
    else
      report "Exact blocker: validation exited with status $final_status"
    fi
  fi

  report "Evidence directory: $RUN_DIR"
  report "Final exit status: $final_status"
  read -r -p "Press Enter to finish..."
  exit "$final_status"
}

on_signal()
{
  local signal_status="$1"
  FAIL_REASON="validation interrupted by signal"
  exit "$signal_status"
}

on_error()
{
  local error_status=$?
  local error_line="$1"
  local error_command="$2"

  if [[ -z "$FAIL_REASON" ]]; then
    FAIL_REASON="command failed at line $error_line with status $error_status: $error_command"
  fi

  return "$error_status"
}

trap on_exit EXIT
trap 'on_error "$LINENO" "$BASH_COMMAND"' ERR
trap 'on_signal 130' INT
trap 'on_signal 143' TERM
trap 'on_signal 129' HUP

validate_workspace_layout()
{
  section "Repository and workspace inspection"

  require_directory "$WORKSPACE"
  require_directory "$PERCEPTION_DIR"
  require_directory "$REALSENSE_DIR"
  require_directory "$DESCRIPTION_DIR"
  require_file "$PERCEPTION_DIR/package.xml"
  require_file "$REALSENSE_DIR/package.xml"
  require_file "$FILTER_CONFIG"
  require_file "$REALSENSE_CONFIG"
  require_file "$HARDWARE_FIXTURE"
  require_file "$SCRIPT_PATH"

  grep -q '<name>savo_perception</name>' "$PERCEPTION_DIR/package.xml" ||
    fail "Unexpected savo_perception package.xml"

  grep -q '<name>savo_realsense</name>' "$REALSENSE_DIR/package.xml" ||
    fail "Unexpected savo_realsense package.xml"

  report "Workspace: $WORKSPACE"
  report "Perception package: $PERCEPTION_DIR"
  report "RealSense package: $REALSENSE_DIR"
  report "Description/TF owner: $DESCRIPTION_DIR"
}

stationary_safety_gate()
{
  local confirmation

  section "Stationary robot safety gate"
  report "Motor power must be disabled or disconnected where practical."
  report "The robot must be secured and remain stationary."
  report "Do not run Nav2, teleoperation, recovery, or publish movement goals."
  read -r -p \
    "Type STATIONARY to confirm the robot is secured and motor motion is disabled: " \
    confirmation

  [[ "$confirmation" == "STATIONARY" ]] ||
    fail "Operator did not confirm the stationary safety gate"
}

hardware_preflight()
{
  local host_name
  local architecture
  local summary_file="$EVIDENCE_DIR/rs_enumerate_summary.txt"
  local detail_file="$EVIDENCE_DIR/rs_enumerate_detail.txt"
  local d435_count
  local serial_count
  local d435_line
  local sys_serial_file
  local usb_device_dir
  local bus_number
  local device_number
  local usb_node
  local memory_available_kib
  local disk_available_kib

  section "8A5.1 hardware and USB preflight"

  host_name="$(hostname -s)"
  architecture="$(uname -m)"

  [[ "$host_name" == "$EXPECTED_HOST" ]] ||
    fail "Phase 8A5 must run on $EXPECTED_HOST; current host is $host_name"

  case "$architecture" in
    aarch64|arm64)
      ;;
    *)
      fail "Phase 8A5 requires ARM64; current architecture is $architecture"
      ;;
  esac

  require_file "$ROS_SETUP"
  # shellcheck disable=SC1090
  source "$ROS_SETUP"

  [[ "${ROS_DISTRO:-}" == "jazzy" ]] ||
    fail "Expected ROS_DISTRO=jazzy; actual value is ${ROS_DISTRO:-unset}"

  require_command rs-enumerate-devices
  require_command lsusb
  require_command udevadm
  require_command colcon
  require_command ros2
  require_command python3
  require_command setsid
  require_command timeout
  require_command ament_flake8
  require_command ament_uncrustify
  require_command ament_cpplint

  rs-enumerate-devices -s | tee "$summary_file"
  rs-enumerate-devices | tee "$detail_file"
  lsusb | tee "$EVIDENCE_DIR/lsusb.txt"
  lsusb -t | tee "$EVIDENCE_DIR/lsusb_tree.txt"

  d435_count="$(
    grep -Ec 'Intel RealSense D435([[:space:]]|$)' "$summary_file" ||
      true
  )"

  [[ "$d435_count" -eq 1 ]] ||
    fail "Expected exactly one D435; rs-enumerate-devices found $d435_count"

  serial_count="$(
    grep -Ec "(^|[[:space:]])${EXPECTED_SERIAL}([[:space:]]|$)" \
      "$summary_file" ||
      true
  )"

  [[ "$serial_count" -eq 1 ]] ||
    fail "Required D435 serial $EXPECTED_SERIAL is missing or duplicated"

  d435_line="$(
    grep -E 'Intel RealSense D435([[:space:]]|$)' "$summary_file"
  )"

  grep -Eq \
    "(^|[[:space:]])${EXPECTED_SERIAL}([[:space:]]|$)" \
    <<<"$d435_line" ||
    fail \
      "The sole D435 row is not the required serial $EXPECTED_SERIAL: $d435_line"

  D435_FIRMWARE="$(
    awk -v serial="$EXPECTED_SERIAL" '
      {
        for (field = 1; field <= NF; ++field) {
          if ($field == serial && field < NF) {
            print $(field + 1)
            exit
          }
        }
      }
    ' "$summary_file"
  )"

  if [[ -z "$D435_FIRMWARE" ]]; then
    D435_FIRMWARE="$(
      awk -v serial="$EXPECTED_SERIAL" '
        index($0, serial) {
          found_serial = 1
          next
        }
        found_serial && tolower($0) ~ /firmware version/ {
          value = $0
          sub(/^[^:]*:[[:space:]]*/, "", value)
          gsub(/[[:space:]]+$/, "", value)
          print value
          exit
        }
      ' "$detail_file"
    )"
  fi

  [[ -n "$D435_FIRMWARE" ]] ||
    fail "Could not parse D435 firmware from: $d435_line"

  [[ "$D435_FIRMWARE" == "$EXPECTED_FIRMWARE" ]] ||
    fail \
      "D435 firmware mismatch: expected $EXPECTED_FIRMWARE, got $D435_FIRMWARE"

  mapfile -t sys_serial_files < <(
    find /sys/bus/usb/devices -maxdepth 2 -type f -name serial \
      -exec grep -l -x "$EXPECTED_SERIAL" {} \; 2>/dev/null
  )

  [[ ${#sys_serial_files[@]} -eq 1 ]] ||
    fail \
      "Expected one kernel USB device for serial $EXPECTED_SERIAL; found ${#sys_serial_files[@]}"

  sys_serial_file="${sys_serial_files[0]}"
  usb_device_dir="$(dirname "$sys_serial_file")"
  require_file "$usb_device_dir/speed"
  require_file "$usb_device_dir/busnum"
  require_file "$usb_device_dir/devnum"

  USB_SPEED_MBPS="$(tr -d '[:space:]' <"$usb_device_dir/speed")"

  if ! awk -v speed="$USB_SPEED_MBPS" \
    'BEGIN { exit !((speed + 0) >= 5000) }'
  then
    fail \
      "D435 is not on USB 3.x: negotiated kernel speed=${USB_SPEED_MBPS} Mb/s"
  fi

  USB_MODE="USB 3.x (${USB_SPEED_MBPS} Mb/s)"
  bus_number="$(tr -d '[:space:]' <"$usb_device_dir/busnum")"
  device_number="$(tr -d '[:space:]' <"$usb_device_dir/devnum")"

  [[ "$bus_number" =~ ^[0-9]+$ ]] ||
    fail "Malformed kernel USB bus number: $bus_number"

  [[ "$device_number" =~ ^[0-9]+$ ]] ||
    fail "Malformed kernel USB device number: $device_number"

  printf -v usb_node \
    '/dev/bus/usb/%03d/%03d' \
    "$((10#$bus_number))" \
    "$((10#$device_number))"

  [[ -r "$usb_node" && -w "$usb_node" ]] ||
    fail "Insufficient read/write permissions for D435 USB node: $usb_node"

  udevadm info --query=property --path="$usb_device_dir" |
    tee "$EVIDENCE_DIR/udev_d435.txt"

  memory_available_kib="$(
    awk '/^MemAvailable:/ { print $2; exit }' /proc/meminfo
  )"

  [[ -n "$memory_available_kib" ]] ||
    fail "Could not determine available memory"

  ((memory_available_kib >= 1048576)) ||
    fail "Less than 1 GiB of memory is available"

  disk_available_kib="$(
    df -Pk "$WORKSPACE" |
      awk 'NR == 2 { print $4 }'
  )"

  [[ -n "$disk_available_kib" ]] ||
    fail "Could not determine workspace disk availability"

  ((disk_available_kib >= 2097152)) ||
    fail "Less than 2 GiB of workspace disk space is available"

  ros2 pkg prefix realsense2_camera |
    tee "$EVIDENCE_DIR/realsense2_camera_prefix.txt"

  report "Host: $host_name"
  report "Architecture: $architecture"
  report "ROS distribution: $ROS_DISTRO"
  report "D435 serial: $EXPECTED_SERIAL"
  report "D435 firmware: $D435_FIRMWARE"
  report "D435 USB mode: $USB_MODE"
  report "D435 kernel node: $usb_node"
  report "Available memory: $memory_available_kib KiB"
  report "Workspace disk available: $disk_available_kib KiB"
}

create_required_backups()
{
  local perception_backup
  local realsense_backup
  local script_backup

  section "Required timestamped backups"

  perception_backup="/tmp/savo_perception_before_phase8a5_${RUN_STAMP}.tar.gz"
  realsense_backup="/tmp/savo_realsense_before_phase8a5_${RUN_STAMP}.tar.gz"
  script_backup="/tmp/savo_perception_phase8a5_d435_validation.sh.before_${RUN_STAMP}"

  tar -C "$(dirname "$PERCEPTION_DIR")" \
    -czf "$perception_backup" \
    "$(basename "$PERCEPTION_DIR")"

  tar -C "$(dirname "$REALSENSE_DIR")" \
    -czf "$realsense_backup" \
    "$(basename "$REALSENSE_DIR")"

  cp -p "$SCRIPT_PATH" "$script_backup"

  tar -tzf "$perception_backup" >/dev/null
  tar -tzf "$realsense_backup" >/dev/null
  sha256sum \
    "$perception_backup" \
    "$realsense_backup" \
    "$script_backup" |
    tee "$EVIDENCE_DIR/backup_sha256.txt"

  BACKUPS_CREATED=1
  SOURCE_CLEANUP_ALLOWED=1
  report "Perception backup: $perception_backup"
  report "RealSense backup: $realsense_backup"
  report "Validation-script backup: $script_backup"
}

validate_source_syntax()
{
  section "8A5.2 source syntax, lint, and contracts"

  bash -n "$SCRIPT_PATH"

  python3 - "$PERCEPTION_DIR" "$REALSENSE_DIR" "$HARDWARE_FIXTURE" <<'PY'
import ast
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


roots = tuple(Path(argument) for argument in sys.argv[1:3])
fixture = Path(sys.argv[3])

for root in roots:
    ET.parse(root / 'package.xml')

    for path in sorted(root.rglob('*.py')):
        if any(
            part in {'.pytest_cache', '__pycache__'}
            for part in path.parts
        ):
            continue

        source = path.read_text(encoding='utf-8')
        ast.parse(source, filename=str(path))
        compile(source, str(path), 'exec')

    for path in sorted(root.rglob('*.yaml')):
        with path.open('r', encoding='utf-8') as stream:
            yaml.safe_load(stream)

fixture_tree = ast.parse(
    fixture.read_text(encoding='utf-8'),
    filename=str(fixture),
)

for node in ast.walk(fixture_tree):
    if isinstance(node, ast.Attribute) and node.attr == 'create_publisher':
        raise SystemExit(
            'Hardware fixture must not call create_publisher().'
        )

for forbidden_name in (
    'StaticTransformBroadcaster',
    'TransformBroadcaster',
):
    if forbidden_name in fixture.read_text(encoding='utf-8'):
        raise SystemExit(
            f'Hardware fixture contains forbidden TF publisher: '
            f'{forbidden_name}'
        )
PY

  mapfile -d '' perception_python_files < <(
    find "$PERCEPTION_DIR" \
      -type f -name '*.py' \
      ! -path '*/__pycache__/*' \
      ! -path '*/.pytest_cache/*' \
      -print0
  )

  mapfile -d '' realsense_python_files < <(
    find "$REALSENSE_DIR" \
      -type f -name '*.py' \
      ! -path '*/__pycache__/*' \
      ! -path '*/.pytest_cache/*' \
      -print0
  )

  ament_flake8 \
    --config "$PERCEPTION_DIR/setup.cfg" \
    "${perception_python_files[@]}"

  ament_flake8 \
    --config "$REALSENSE_DIR/setup.cfg" \
    "${realsense_python_files[@]}"

  mapfile -d '' cpp_files < <(
    find "$PERCEPTION_DIR" "$REALSENSE_DIR" \
      -type f \
      \( -name '*.cpp' -o -name '*.hpp' \) \
      -print0
  )

  [[ ${#cpp_files[@]} -gt 0 ]] ||
    fail "No C++ files were found for format/lint validation"

  ament_uncrustify "${cpp_files[@]}"
  ament_cpplint \
    --filters=-build/include_order,-legal/copyright \
    "${cpp_files[@]}"

  python3 -m pytest \
    -p no:cacheprovider \
    -q \
    "$PERCEPTION_DIR/test/contracts/test_phase8a_obstacle_cloud_contracts.py"

  python3 -m pytest \
    -p no:cacheprovider \
    -q \
    "$REALSENSE_DIR/test/test_pointcloud_profile.py" \
    "$REALSENSE_DIR/test/test_serial_binding.py"

  python3 - \
    "$REALSENSE_CONFIG" \
    "$FILTER_CONFIG" \
    "$EXPECTED_SERIAL" <<'PY'
from pathlib import Path
import sys

import yaml


realsense_path = Path(sys.argv[1])
filter_path = Path(sys.argv[2])
expected_serial = sys.argv[3]

with realsense_path.open('r', encoding='utf-8') as stream:
    config = yaml.safe_load(stream)

params = config['/camera/camera']['ros__parameters']

expected = {
    'serial_no': expected_serial,
    'depth_module.depth_profile': '848x480x30',
    'rgb_camera.color_profile': '640x480x30',
    'align_depth.enable': True,
    'enable_sync': True,
    'pointcloud__neon_.enable': True,
}

for key, value in expected.items():
    if params.get(key) != value:
        raise SystemExit(
            f'RealSense parameter mismatch: {key}='
            f'{params.get(key)!r}, expected {value!r}'
        )

with filter_path.open('r', encoding='utf-8') as stream:
    filter_config = yaml.safe_load(stream)

filter_params = filter_config[
    'obstacle_cloud_filter_node'
]['ros__parameters']

expected_filter = {
    'input_topic': '/camera/camera/depth/color/points',
    'output_topic': '/savo_perception/obstacles/points',
    'output_frame': 'base_link',
    'health_topic': '/savo_perception/obstacle_cloud/health',
    'status_topic': '/savo_perception/obstacle_cloud/status',
    'heartbeat_topic': '/savo_perception/obstacle_cloud/heartbeat',
    'min_range_m': 0.20,
    'max_range_m': 3.00,
    'min_height_m': 0.05,
    'max_height_m': 1.60,
    'voxel_size_m': 0.05,
    'self_filter_enabled': True,
    'max_output_points': 100000,
    'transform_timeout_s': 0.10,
    'max_processing_hz': 10.0,
    'stale_timeout_s': 0.75,
}

for key, value in expected_filter.items():
    if filter_params.get(key) != value:
        raise SystemExit(
            f'Obstacle-cloud parameter mismatch: {key}='
            f'{filter_params.get(key)!r}, expected {value!r}'
        )
PY

  clean_source_artifacts
  assert_no_source_artifacts ||
    fail "Source cache cleanup failed after direct validation"

  report "Package syntax checks: PASS"
  report "Python lint: PASS"
  report "C++ format/lint: PASS"
  report "Phase 8A and RealSense point-cloud contracts: PASS"
}

safe_remove_package_build()
{
  local package_name="$1"
  local build_path="$WORKSPACE/build/$package_name"
  local install_path="$WORKSPACE/install/$package_name"

  [[ "$build_path" == "$WORKSPACE/build/"* ]] ||
    fail "Refusing unsafe build cleanup target: $build_path"

  [[ "$install_path" == "$WORKSPACE/install/"* ]] ||
    fail "Refusing unsafe install cleanup target: $install_path"

  rm -rf -- "$build_path" "$install_path"
}

verify_installed_executables()
{
  local executable

  for executable in \
    camera_health_node \
    camera_topic_monitor_node \
    depth_front_min_node \
    camera_health_node_py \
    camera_topic_monitor_node_py \
    depth_front_min_node_py \
    realsense_smoke_test_cli \
    dump_effective_realsense_params
  do
    [[ -x \
      "$WORKSPACE/install/savo_realsense/lib/savo_realsense/$executable" ]] ||
      fail "Missing installed savo_realsense executable: $executable"
  done

  for executable in \
    cmd_vel_safety_gate \
    obstacle_cloud_filter_node \
    range_health_node \
    safety_stop_node \
    ultrasonic_node \
    vl53_mux_node
  do
    [[ -x \
      "$WORKSPACE/install/savo_perception/lib/savo_perception/$executable" ]] ||
      fail "Missing installed savo_perception executable: $executable"
  done

  ros2 pkg executables savo_realsense |
    tee "$EVIDENCE_DIR/savo_realsense_executables.txt"

  ros2 pkg executables savo_perception |
    tee "$EVIDENCE_DIR/savo_perception_executables.txt"
}

parse_test_totals()
{
  local result_directory="$1"

  python3 - "$result_directory" <<'PY'
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


root_directory = Path(sys.argv[1])

if not root_directory.is_dir():
    raise SystemExit(f'Missing test-result directory: {root_directory}')

totals = {
    'tests': 0,
    'errors': 0,
    'failures': 0,
    'skipped': 0,
}
files = 0


def count(element, attribute, path, default=None):
    raw = element.attrib.get(attribute)

    if raw is None:
        if default is not None:
            return default
        raise SystemExit(
            f'Missing {attribute!r} count in {path}'
        )

    try:
        value = int(raw)
    except ValueError as error:
        raise SystemExit(
            f'Malformed {attribute!r} count in {path}'
        ) from error

    if value < 0:
        raise SystemExit(
            f'Negative {attribute!r} count in {path}'
        )

    return value


for path in sorted(root_directory.rglob('*.xml')):
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError) as error:
        raise SystemExit(
            f'Malformed XML test result {path}: {error}'
        ) from error

    summaries = [root]

    if 'tests' not in root.attrib:
        summaries = list(root.findall('testsuite'))

    if not summaries:
        raise SystemExit(
            f'No test-suite summary in {path}'
        )

    files += 1

    for summary in summaries:
        totals['tests'] += count(
            summary,
            'tests',
            path,
        )
        totals['errors'] += count(
            summary,
            'errors',
            path,
        )
        totals['failures'] += count(
            summary,
            'failures',
            path,
        )
        totals['skipped'] += count(
            summary,
            'skipped',
            path,
            default=count(
                summary,
                'disabled',
                path,
                default=0,
            ),
        )

if files == 0 or totals['tests'] == 0:
    raise SystemExit('No nonempty XML test results were found')

print(
    f"tests={totals['tests']} "
    f"errors={totals['errors']} "
    f"failures={totals['failures']} "
    f"skipped={totals['skipped']}"
)

if totals['errors'] or totals['failures']:
    raise SystemExit(1)
PY
}

clean_build_and_test_packages()
{
  section "8A5.2 clean package builds and complete tests"

  # shellcheck disable=SC1090
  source "$ROS_SETUP"

  if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
    # Source non-target workspace dependencies before removing the two targets.
    # shellcheck disable=SC1090
    source "$WORKSPACE/install/setup.bash"
  fi

  safe_remove_package_build savo_realsense
  safe_remove_package_build savo_perception

  cd "$WORKSPACE"

  colcon --log-base "$RUN_DIR/colcon_build_realsense" build \
    --packages-select savo_realsense \
    --symlink-install \
    --event-handlers console_direct+

  # shellcheck disable=SC1090
  source "$WORKSPACE/install/setup.bash"

  colcon --log-base "$RUN_DIR/colcon_build_perception" build \
    --packages-select savo_perception \
    --symlink-install \
    --event-handlers console_direct+

  # shellcheck disable=SC1090
  source "$WORKSPACE/install/setup.bash"
  verify_installed_executables

  colcon --log-base "$RUN_DIR/colcon_test_realsense" test \
    --packages-select savo_realsense \
    --event-handlers console_direct+ \
    --return-code-on-test-failure

  colcon test-result \
    --test-result-base \
    "$WORKSPACE/build/savo_realsense/test_results" \
    --verbose |
    tee "$EVIDENCE_DIR/savo_realsense_test_result_verbose.txt"

  REALSENSE_TOTALS="$(
    parse_test_totals \
      "$WORKSPACE/build/savo_realsense/test_results"
  )"

  colcon --log-base "$RUN_DIR/colcon_test_perception" test \
    --packages-select savo_perception \
    --event-handlers console_direct+ \
    --return-code-on-test-failure

  colcon test-result \
    --test-result-base \
    "$WORKSPACE/build/savo_perception/test_results" \
    --verbose |
    tee "$EVIDENCE_DIR/savo_perception_test_result_verbose.txt"

  PERCEPTION_TOTALS="$(
    parse_test_totals \
      "$WORKSPACE/build/savo_perception/test_results"
  )"

  clean_source_artifacts
  assert_no_source_artifacts ||
    fail "Source cache artifacts remain after package tests"

  git -C "$WORKSPACE/src" diff --check

  report "savo_realsense test totals: $REALSENSE_TOTALS"
  report "savo_perception test totals: $PERCEPTION_TOTALS"
  report "Skipped tests are reported and accepted only with zero errors/failures."
}

topic_has_publisher()
{
  local topic_name="$1"
  local topic_info

  topic_info="$(ros2 topic info "$topic_name" 2>/dev/null || true)"
  grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$topic_info"
}

node_exists_exact()
{
  local node_name="$1"

  ros2 node list 2>/dev/null |
    grep -Fxq "$node_name"
}

wait_for_topic_publisher()
{
  local topic_name="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))

  while ((SECONDS < deadline)); do
    if topic_has_publisher "$topic_name"; then
      return 0
    fi

    sleep 0.5
  done

  return 1
}

wait_for_no_topic_publisher()
{
  local topic_name="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))

  while ((SECONDS < deadline)); do
    if ! topic_has_publisher "$topic_name"; then
      return 0
    fi

    sleep 0.5
  done

  return 1
}

validate_active_camera_configuration()
{
  local parameter_dump="$EVIDENCE_DIR/active_camera_parameters_$(date +%H%M%S).yaml"

  ros2 param dump /camera/camera |
    tee "$parameter_dump"

  python3 - "$parameter_dump" "$EXPECTED_SERIAL" <<'PY'
from pathlib import Path
import sys

import yaml


path = Path(sys.argv[1])
expected_serial = sys.argv[2]

with path.open('r', encoding='utf-8') as stream:
    document = yaml.safe_load(stream)

if not isinstance(document, dict):
    raise SystemExit('Active camera parameter dump is not a mapping')

node = document.get('/camera/camera')
if not isinstance(node, dict):
    raise SystemExit(
        'Active parameter dump does not contain /camera/camera'
    )

parameters = node.get('ros__parameters')
if not isinstance(parameters, dict):
    raise SystemExit(
        'Active /camera/camera parameters are missing'
    )

expected = {
    'serial_no': expected_serial,
    'enable_color': True,
    'enable_depth': True,
    'depth_module.depth_profile': '848x480x30',
    'rgb_camera.color_profile': '640x480x30',
    'align_depth.enable': True,
    'enable_sync': True,
    'pointcloud__neon_.enable': True,
    'publish_tf': True,
}

for name, expected_value in expected.items():
    actual_value = parameters.get(name)
    if actual_value != expected_value:
        raise SystemExit(
            f'Active RealSense parameter {name}={actual_value!r}; '
            f'expected {expected_value!r}'
        )
PY

  report "Active D435 serial/profile/alignment/point-cloud parameters: PASS"
}

start_camera()
{
  local camera_log="$RUN_DIR/realsense_$(date +%H%M%S).log"

  if topic_has_publisher "$RAW_TOPIC" ||
    node_exists_exact /camera/camera
  then
    fail \
      "A non-script RealSense publisher/node is already active; stop it cleanly before Phase 8A5 so the script does not create or kill a duplicate"
  fi

  start_tracked_process \
    "RealSense D435 publisher" \
    "$camera_log" \
    ros2 launch \
    savo_realsense \
    realsense_pointcloud.launch.py \
    config_file:="$REALSENSE_CONFIG" \
    serial_no:="$EXPECTED_SERIAL"

  CAMERA_INDEX="$LAST_PROCESS_INDEX"

  wait_for_topic_publisher "$RAW_TOPIC" 40 ||
    fail "Raw RealSense PointCloud2 publisher did not appear"

  ros2 topic info "$RAW_TOPIC" --verbose |
    tee "$EVIDENCE_DIR/raw_topic_info_$(date +%H%M%S).txt"

  ros2 param get /camera/camera serial_no |
    tee "$EVIDENCE_DIR/active_camera_serial_$(date +%H%M%S).txt" |
    grep -Fq "$EXPECTED_SERIAL" ||
    fail "Active /camera/camera serial_no is not $EXPECTED_SERIAL"

  validate_active_camera_configuration

  report "Active RealSense publisher is serial-bound to $EXPECTED_SERIAL"
}

ensure_production_robot_description()
{
  local node_list
  local description_log="$RUN_DIR/robot_description.log"

  node_list="$(ros2 node list 2>/dev/null || true)"

  if grep -Fxq '/robot_state_publisher' <<<"$node_list"; then
    report "Using existing production /robot_state_publisher"
    return 0
  fi

  report \
    "No robot_state_publisher was visible; starting the production savo_description launch"

  start_tracked_process \
    "production robot_state_publisher" \
    "$description_log" \
    ros2 launch \
    savo_description \
    description.launch.py \
    use_sim_time:=false \
    use_transmissions:=false \
    use_gazebo:=false

  DESCRIPTION_INDEX="$LAST_PROCESS_INDEX"
  sleep 2

  ros2 node list |
    grep -Fxq '/robot_state_publisher' ||
    fail "Production robot_state_publisher did not start"
}

start_filter()
{
  local filter_log="$RUN_DIR/obstacle_cloud_filter.log"

  if topic_has_publisher "$FILTERED_TOPIC" ||
    topic_has_publisher "$HEALTH_TOPIC" ||
    topic_has_publisher "$STATUS_TOPIC" ||
    topic_has_publisher "$HEARTBEAT_TOPIC" ||
    node_exists_exact /obstacle_cloud_filter_node
  then
    fail \
      "A non-script obstacle-cloud filter is already active; stop it cleanly before Phase 8A5 so the script does not create or kill a duplicate"
  fi

  start_tracked_process \
    "production obstacle cloud filter" \
    "$filter_log" \
    ros2 launch \
    savo_perception \
    obstacle_cloud_filter.launch.py \
    config_file:="$FILTER_CONFIG" \
    use_sim_time:=false

  FILTER_INDEX="$LAST_PROCESS_INDEX"

  wait_for_topic_publisher "$FILTERED_TOPIC" 20 ||
    fail "Filtered obstacle-cloud publisher did not appear"

  wait_for_topic_publisher "$STATUS_TOPIC" 10 ||
    fail "Obstacle-cloud status publisher did not appear"

  wait_for_topic_publisher "$HEARTBEAT_TOPIC" 10 ||
    fail "Obstacle-cloud heartbeat publisher did not appear"

  ros2 topic info "$FILTERED_TOPIC" --verbose |
    tee "$EVIDENCE_DIR/filtered_topic_info.txt"
}

sample_process_group_resources()
{
  local label="$1"
  local process_index="$2"
  local process_group="${PROCESS_PGIDS[$process_index]}"
  local output_file="$EVIDENCE_DIR/resources_${label}.txt"
  local sample

  : >"$output_file"

  for sample in 1 2 3 4 5; do
    ps -e -o pid=,pgid=,pcpu=,pmem=,rss=,etimes=,comm= |
      awk \
        -v group="$process_group" \
        -v label="$label" \
        -v sample="$sample" '
          $2 == group {
            cpu += $3
            memory_percent += $4
            rss_kib += $5
            processes += 1
          }
          END {
            printf(
              "sample=%d label=%s pgid=%s processes=%d "
              "cpu_percent=%.2f memory_percent=%.2f rss_kib=%d\n",
              sample,
              label,
              group,
              processes,
              cpu,
              memory_percent,
              rss_kib
            )
          }
        ' |
      tee -a "$output_file"

    sleep 1
  done
}

run_fixture()
{
  local mode="$1"
  local label="$2"
  local duration="$3"
  local output_json="$4"
  local fixture_log="$RUN_DIR/fixture_${label}.log"
  local allowed_failure_summary=""
  local fixture_summary=""
  local timeout_seconds

  LAST_FIXTURE_ALLOWED_FAILURES=0
  shift 4
  timeout_seconds="$(
    awk -v duration="$duration" \
      'BEGIN { printf("%d", duration + 45) }'
  )"

  run_tracked_foreground \
    "hardware fixture: $label" \
    "$fixture_log" \
    timeout \
    --signal=INT \
    --kill-after=8s \
    "${timeout_seconds}s" \
    python3 \
    "$HARDWARE_FIXTURE" \
    "$mode" \
    --config "$FILTER_CONFIG" \
    --duration "$duration" \
    --report "$output_json" \
    "$@"

  if [[ "$LAST_FOREGROUND_STATUS" -ne 0 ]]; then
    if [[ -f "$output_json" ]] &&
      fixture_summary="$(
        python3 - "$output_json" <<'PY'
import json
from pathlib import Path
import sys


document = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
failures = document.get('validation', {}).get('failures', [])

if not failures:
    raise SystemExit(1)

print(
    '; '.join(
        f"{failure.get('code', 'unknown')}: "
        f"{failure.get('detail', 'no detail')}"
        for failure in failures[:8]
    )
)
PY
      )"
    then
      report "Fixture validation failures: $fixture_summary"
    else
      fixture_summary="fixture did not produce a readable failure report"
    fi

    if [[ "$LAST_FOREGROUND_STATUS" -eq 1 ]] &&
      [[ -n "$FIXTURE_ALLOWED_FAILURE_CODES" ]] &&
      [[ -f "$output_json" ]] &&
      allowed_failure_summary="$(
        python3 - \
          "$output_json" \
          "$FIXTURE_ALLOWED_FAILURE_CODES" <<'PY'
import json
from pathlib import Path
import sys


document = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
allowed = {
    code
    for code in sys.argv[2].split(',')
    if code
}
failures = document.get('validation', {}).get('failures', [])
observed = {
    str(failure.get('code', 'unknown'))
    for failure in failures
}

if not failures or not observed.issubset(allowed):
    raise SystemExit(1)

print(','.join(sorted(observed)))
PY
      )"
    then
      LAST_FIXTURE_ALLOWED_FAILURES=1
      report \
        "Original self-filter evidence requires tuning; allowed failure codes: $allowed_failure_summary"
    else
      fail \
        "hardware fixture $label failed: $fixture_summary (exit status $LAST_FOREGROUND_STATUS)"
    fi
  fi

  require_file "$output_json"
  python3 -m json.tool "$output_json" |
    tee "$EVIDENCE_DIR/${label}_pretty.json" >/dev/null

  if [[ "$LAST_FIXTURE_ALLOWED_FAILURES" -eq 1 ]]; then
    report \
      "Hardware fixture $label: SELF-TUNING EVIDENCE COLLECTED (not a pass)"
  else
    report "Hardware fixture $label: PASS"
  fi
}

operator_scene_ready()
{
  local title="$1"
  local instructions="$2"
  local confirmation

  section "$title"
  report "$instructions"
  report "Keep the robot stationary and keep hands clear of the camera."
  read -r -p "Type READY when the scene is safely arranged: " confirmation

  [[ "$confirmation" == "READY" ]] ||
    fail "Operator did not confirm scene readiness for $title"
}

run_physical_scene()
{
  local pass_label="$1"
  local scene_name="$2"
  local title="$3"
  local instructions="$4"
  local output_json="$EVIDENCE_DIR/scene_${pass_label}_${scene_name}.json"
  local confirmation

  operator_scene_ready "$title" "$instructions"
  run_fixture \
    scene \
    "scene_${pass_label}_${scene_name}" \
    8 \
    "$output_json" \
    --scene "$scene_name"

  python3 - "$output_json" <<'PY' |
    tee -a "$REPORT_FILE"
import json
from pathlib import Path
import sys


document = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
scene = document.get('scene', {})
selected = scene.get('selected_roi', {})
paired = scene.get('paired_roi_evidence', {})

print(
    'Scene evidence: '
    f"selected={scene.get('selected')!r} "
    f"raw={selected.get('raw')} "
    f"filtered={selected.get('output')} "
    f"same_stamp_pairs={paired.get('pairs')!r} "
    f"qualifying_pairs={paired.get('qualifying_pairs')!r}"
)
PY

  read -r -p \
    "Type CONFIRMED to attest that the robot stayed still, the instructed target remained in place, and the printed same-stamp evidence matches the expected retained/rejected behavior: " \
    confirmation

  [[ "$confirmation" == "CONFIRMED" ]] ||
    fail "Operator did not confirm the completed physical scene: $title"
}

run_physical_scene_suite()
{
  local pass_label="$1"

  section "8A5.6 physical scene suite ($pass_label)"

  run_physical_scene \
    "$pass_label" \
    clear \
    "Test A — clear floor and open space" \
    "Clear movable obstacles from the D435 view. Leave normal floor and open room visible."

  run_physical_scene \
    "$pass_label" \
    front \
    "Test B — front obstacle" \
    "Place a stable object 0.50–0.80 m directly in front of the robot (+X)."

  run_physical_scene \
    "$pass_label" \
    left \
    "Test C1 — left obstacle" \
    "Place the stable obstacle on the robot's left side; expected base_link coordinate is +Y."

  run_physical_scene \
    "$pass_label" \
    right \
    "Test C2 — right obstacle" \
    "Place the stable obstacle on the robot's right side; expected base_link coordinate is -Y."

  run_physical_scene \
    "$pass_label" \
    low \
    "Test D1 — near-floor height" \
    "Arrange a safe target whose visible points include z below 0.05 m in base_link."

  run_physical_scene \
    "$pass_label" \
    normal \
    "Test D2 — normal obstacle height" \
    "Arrange a stable obstacle with visible points between z=0.05 m and z=1.60 m."

  run_physical_scene \
    "$pass_label" \
    high \
    "Test D3 — above maximum height" \
    "Arrange a safe visible target whose points include z above 1.60 m."

  run_physical_scene \
    "$pass_label" \
    near \
    "Test E1 — inside minimum range" \
    "Where safe for the camera, present a target with horizontal range below 0.20 m. Do not touch the lens or housing."

  run_physical_scene \
    "$pass_label" \
    mid \
    "Test E2 — accepted range" \
    "Place a target between 0.20 m and 3.00 m horizontal range."

  run_physical_scene \
    "$pass_label" \
    far \
    "Test E3 — beyond maximum range" \
    "Where the room permits, present a visible target beyond 3.00 m."
}

read_self_filter_values()
{
  python3 - "$FILTER_CONFIG" <<'PY'
from pathlib import Path
import sys

import yaml


path = Path(sys.argv[1])

with path.open('r', encoding='utf-8') as stream:
    params = yaml.safe_load(stream)[
        'obstacle_cloud_filter_node'
    ]['ros__parameters']

keys = (
    'self_min_x_m',
    'self_max_x_m',
    'self_min_y_m',
    'self_max_y_m',
    'self_min_z_m',
    'self_max_z_m',
)

print(' '.join(f'{key}={params[key]}' for key in keys))
PY
}

validate_self_filter_numbers()
{
  python3 - "$@" <<'PY'
import math
import sys


try:
    values = tuple(float(value) for value in sys.argv[1:])
except ValueError as error:
    raise SystemExit(f'Invalid self-filter number: {error}') from error

if len(values) != 6:
    raise SystemExit('Exactly six self-filter values are required.')

if not all(math.isfinite(value) for value in values):
    raise SystemExit('Self-filter values must be finite.')

min_x, max_x, min_y, max_y, min_z, max_z = values

if not (min_x < max_x and min_y < max_y and min_z < max_z):
    raise SystemExit('Every self-filter minimum must be below its maximum.')

if any(abs(value) > 5.0 for value in values):
    raise SystemExit('A self-filter bound exceeds the 5 m safety sanity limit.')
PY
}

apply_self_filter_tuning()
{
  local tune_backup
  local rationale
  local min_x
  local max_x
  local min_y
  local max_y
  local min_z
  local max_z

  section "Self-filter tuning"
  report \
    "Tune only from measured robot-body extents; never tune merely to improve a metric."
  read -r -p "Explain why the measured extents require tuning: " rationale

  [[ -n "$rationale" ]] ||
    fail "A measured self-filter tuning rationale is required"

  read -r -p "New self_min_x_m: " min_x
  read -r -p "New self_max_x_m: " max_x
  read -r -p "New self_min_y_m: " min_y
  read -r -p "New self_max_y_m: " max_y
  read -r -p "New self_min_z_m: " min_z
  read -r -p "New self_max_z_m: " max_z

  validate_self_filter_numbers \
    "$min_x" "$max_x" "$min_y" "$max_y" "$min_z" "$max_z" ||
    fail "Proposed self-filter values are invalid"

  tune_backup="/tmp/obstacle_cloud_filter.yaml.before_self_filter_${RUN_STAMP}"
  cp -p "$FILTER_CONFIG" "$tune_backup"

  sed -E -i \
    -e "s/^    self_min_x_m:.*/    self_min_x_m: $min_x/" \
    -e "s/^    self_max_x_m:.*/    self_max_x_m: $max_x/" \
    -e "s/^    self_min_y_m:.*/    self_min_y_m: $min_y/" \
    -e "s/^    self_max_y_m:.*/    self_max_y_m: $max_y/" \
    -e "s/^    self_min_z_m:.*/    self_min_z_m: $min_z/" \
    -e "s/^    self_max_z_m:.*/    self_max_z_m: $max_z/" \
    "$FILTER_CONFIG"

  python3 - \
    "$tune_backup" \
    "$FILTER_CONFIG" \
    "$min_x" \
    "$max_x" \
    "$min_y" \
    "$max_y" \
    "$min_z" \
    "$max_z" <<'PY'
from copy import deepcopy
import math
from pathlib import Path
import sys

import yaml


backup_path = Path(sys.argv[1])
current_path = Path(sys.argv[2])
expected_values = tuple(float(value) for value in sys.argv[3:])
allowed_names = (
    'self_min_x_m',
    'self_max_x_m',
    'self_min_y_m',
    'self_max_y_m',
    'self_min_z_m',
    'self_max_z_m',
)


def load(path):
    with path.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


before = load(backup_path)
after = load(current_path)
before_for_comparison = deepcopy(before)
before_params = before_for_comparison[
    'obstacle_cloud_filter_node'
]['ros__parameters']
after_params = after[
    'obstacle_cloud_filter_node'
]['ros__parameters']

for name, expected in zip(allowed_names, expected_values):
    actual = float(after_params[name])
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-12):
        raise SystemExit(
            f'Self-filter edit did not apply {name}: '
            f'actual={actual}, expected={expected}'
        )
    before_params[name] = after_params[name]

if before_for_comparison != after:
    raise SystemExit(
        'Self-filter tuning changed content outside the six allowed bounds'
    )
PY

  FINAL_SELF_FILTER="$(read_self_filter_values)"
  report "Self-filter tuning rationale: $rationale"
  report "Self-filter YAML backup: $tune_backup"
  report "Original self-filter: $ORIGINAL_SELF_FILTER"
  report "Final self-filter: $FINAL_SELF_FILTER"

  python3 - "$FILTER_CONFIG" <<'PY'
from pathlib import Path
import sys

import yaml


with Path(sys.argv[1]).open('r', encoding='utf-8') as stream:
    yaml.safe_load(stream)
PY

  python3 -m pytest \
    -p no:cacheprovider \
    -q \
    "$PERCEPTION_DIR/test/contracts/test_phase8a_obstacle_cloud_contracts.py"

  stop_tracked_process "$FILTER_INDEX" ||
    fail "Could not stop filter for self-filter configuration reload"

  wait_for_no_topic_publisher "$FILTERED_TOPIC" 10 ||
    fail "Filtered publisher remained after stopping the script-owned filter"

  start_filter
}

evaluate_self_filter()
{
  local pass_label="$1"
  local body_output_json="$EVIDENCE_DIR/scene_${pass_label}_self_body.json"
  local output_json="$EVIDENCE_DIR/scene_${pass_label}_self.json"
  local decision
  local tuning_required=0

  if [[ "$pass_label" == "original" ]]; then
    FIXTURE_ALLOWED_FAILURE_CODES="scene_rejection,self_nearby_retention"
  else
    FIXTURE_ALLOWED_FAILURE_CODES=""
  fi

  operator_scene_ready \
    "8A5.7 self-filter body measurement ($pass_label)" \
    "Clear movable objects from the broad chassis-candidate envelope and leave the visible chassis/body unobstructed. This first window records body-candidate extents without a deliberately adjacent test object."

  run_fixture \
    scene \
    "scene_${pass_label}_self_body" \
    10 \
    "$body_output_json" \
    --scene self_body

  if [[ "$LAST_FIXTURE_ALLOWED_FAILURES" -eq 1 ]]; then
    tuning_required=1
  fi

  operator_scene_ready \
    "8A5.7 self-filter nearby-obstacle retention ($pass_label)" \
    "Without moving the robot, place one stable legitimate obstacle immediately outside the configured self-filter box (roughly 0.35–0.65 m forward, safely inside the valid range/height band). The fixture must observe same-stamp self-box rejection and nearby-obstacle retention."

  run_fixture \
    scene \
    "scene_${pass_label}_self" \
    10 \
    "$output_json" \
    --scene self

  if [[ "$LAST_FIXTURE_ALLOWED_FAILURES" -eq 1 ]]; then
    tuning_required=1
  fi

  FIXTURE_ALLOWED_FAILURE_CODES=""

  python3 - "$body_output_json" "$output_json" <<'PY' |
    tee -a "$REPORT_FILE"
import json
from pathlib import Path
import sys


body = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
nearby = json.loads(Path(sys.argv[2]).read_text(encoding='utf-8'))
body_scene = body.get('scene', {})
nearby_scene = nearby.get('scene', {})

print(
    'Measured clear-window body-candidate extents: '
    f"{body_scene.get('raw_body_candidate_extents_m')}"
)
print(
    'Configured self-box rejection evidence: '
    f"{nearby_scene.get('paired_roi_evidence')}"
)
print(
    'Controlled nearby-obstacle retention evidence: '
    f"{nearby_scene.get('self_nearby_obstacle_retention')}"
)
PY

  if [[ "$tuning_required" -eq 1 ]]; then
    report \
      "The original provisional bounds failed a self-specific acceptance check. KEEP is not permitted; tuning from measured physical evidence is required."
    read -r -p \
      "Type TUNE to correct the provisional self-filter bounds, or anything else to stop: " \
      decision
  else
    read -r -p \
      "Type KEEP only if the printed measured extents cover the visible chassis and the controlled nearby obstacle remains in the filtered cloud, or TUNE if measured bounds require correction: " \
      decision
  fi

  case "$decision" in
    KEEP)
      if [[ "$tuning_required" -eq 1 ]]; then
        fail \
          "KEEP is invalid because the original provisional self-filter bounds failed a self-specific acceptance check"
      fi

      SELF_FILTER_RESULT="PASS"
      FINAL_SELF_FILTER="$(read_self_filter_values)"
      ;;
    TUNE)
      if [[ "$pass_label" == "tuned" ]]; then
        fail \
          "Self-filter still needs tuning after the required full repeat; record measurements and correct the physical model before rerunning"
      fi

      apply_self_filter_tuning
      run_physical_scene_suite tuned
      evaluate_self_filter tuned
      ;;
    *)
      fail "Self-filter evaluation was not accepted with KEEP or TUNE"
      ;;
  esac
}

run_comprehensive_observation()
{
  local label="$1"
  local duration="$2"
  local output_json="$EVIDENCE_DIR/observe_${label}.json"

  section "8A5.3–8A5.5 real cloud, TF, and filter observation ($label)"

  run_fixture \
    observe \
    "observe_${label}" \
    "$duration" \
    "$output_json"

  RAW_FRAME="$(
    python3 - "$output_json" <<'PY'
import json
from pathlib import Path
import sys


report = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
print(report.get('raw_frame', 'unknown'))
PY
  )"

  TF_CHAIN="$(
    python3 - "$output_json" <<'PY'
import json
from pathlib import Path
import sys


report = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
chain = report.get('tf_chain', 'unknown')

if isinstance(chain, list):
    chain = ' -> '.join(str(frame) for frame in chain)

print(chain)
PY
  )"

  report "Observed raw cloud frame: $RAW_FRAME"
  report "Observed TF chain: $TF_CHAIN"
  python3 -m json.tool "$output_json" |
    tee -a "$REPORT_FILE"
}

run_stale_and_recovery_validation()
{
  local stale_json="$EVIDENCE_DIR/stale_input.json"
  local recovery_json="$EVIDENCE_DIR/observe_recovery.json"

  section "8A5.8 real stale-input behavior"
  report \
    "Stopping only the script-owned RealSense publisher group; the filter remains running."

  stop_tracked_process "$CAMERA_INDEX" ||
    fail "Could not stop the script-owned RealSense publisher"

  wait_for_no_topic_publisher "$RAW_TOPIC" 15 ||
    fail "Raw topic still has a publisher after controlled RealSense stop"

  sleep 1

  run_fixture \
    stale \
    stale_input \
    5 \
    "$stale_json" \
    --stale-phase outage

  STALE_RESULT="PASS"
  report "Real stale-input detection: PASS"

  section "8A5.8 RealSense recovery"
  start_camera

  run_fixture \
    observe \
    observe_recovery \
    15 \
    "$recovery_json"

  RECOVERY_RESULT="PASS"
  report "RealSense publication/filter recovery without filter restart: PASS"
}

authority_audit()
{
  local filter_node_info="$EVIDENCE_DIR/filter_node_info.txt"
  local node_list_file="$EVIDENCE_DIR/node_list.txt"
  local forbidden_pattern

  section "8A5.9 topic-authority audit"

  forbidden_pattern='(/cmd_vel($|_)|/cmd_vel_safe|/cmd_vel_recovery|/cmd_vel_nav|/savo_control/mode_cmd|/savo_control/recovery_request|navigate_to_pose|navigate_through_poses|follow_waypoints|lifecycle|recovery)'

  ros2 node list |
    sort -u |
    tee "$node_list_file"

  ros2 node info /obstacle_cloud_filter_node |
    tee "$filter_node_info"

  if grep -E "$forbidden_pattern" "$filter_node_info"; then
    fail "Obstacle-cloud filter has a forbidden control/navigation endpoint"
  fi

  python3 - "$HARDWARE_FIXTURE" <<'PY'
import ast
from pathlib import Path
import sys


path = Path(sys.argv[1])
source = path.read_text(encoding='utf-8')
tree = ast.parse(source, filename=str(path))

for node in ast.walk(tree):
    if isinstance(node, ast.Attribute) and node.attr == 'create_publisher':
        raise SystemExit(
            'Hardware fixture contains create_publisher().'
        )

    if isinstance(node, ast.Call):
        function = node.func

        if (
            isinstance(function, ast.Name)
            and function.id in {
                'Publisher',
                'StaticTransformBroadcaster',
                'TransformBroadcaster',
            }
        ):
            raise SystemExit(
                f'Hardware fixture constructs forbidden '
                f'publisher {function.id}.'
            )
PY

  AUTHORITY_RESULT="PASS"
  report "Filter and hardware fixture authority audit: PASS"
  report \
    "Unrelated robot endpoints, if present, were not treated as publishers owned by Phase 8A5."
}

print_hardware_report()
{
  local final_observation="$EVIDENCE_DIR/observe_final.json"
  local scene_front="$EVIDENCE_DIR/scene_original_front.json"
  local scene_left="$EVIDENCE_DIR/scene_original_left.json"
  local scene_right="$EVIDENCE_DIR/scene_original_right.json"
  local scene_self="$EVIDENCE_DIR/scene_original_self_body.json"

  if [[ -f "$EVIDENCE_DIR/scene_tuned_front.json" ]]; then
    scene_front="$EVIDENCE_DIR/scene_tuned_front.json"
    scene_left="$EVIDENCE_DIR/scene_tuned_left.json"
    scene_right="$EVIDENCE_DIR/scene_tuned_right.json"
    scene_self="$EVIDENCE_DIR/scene_tuned_self_body.json"
  fi

  section "Concise Phase 8A5 hardware report"
  report "D435 serial: $EXPECTED_SERIAL"
  report "D435 firmware: $D435_FIRMWARE"
  report "USB mode: $USB_MODE"
  report "Raw cloud frame: $RAW_FRAME"
  report "TF chain to base_link: $TF_CHAIN"

  python3 - \
    "$final_observation" \
    "$scene_front" \
    "$scene_left" \
    "$scene_right" \
    "$scene_self" <<'PY' | tee -a "$REPORT_FILE"
import json
from pathlib import Path
import sys


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


observation = load(sys.argv[1])
front = load(sys.argv[2])
left = load(sys.argv[3])
right = load(sys.argv[4])
self_scene = load(sys.argv[5])

streams = observation.get('streams', {})
raw = streams.get('raw', {})
filtered = streams.get('output', {})
counts = observation.get('point_counts', {})
raw_counts = counts.get('raw', {})
filtered_counts = counts.get('output', {})
timestamps = observation.get('timestamps', {})
latency = timestamps.get('receipt_latency_ms', {})
stamp_age = timestamps.get('stamp_age', {}).get('raw', {})
status = observation.get('status', {})

raw_average = raw_counts.get('average')
filtered_average = filtered_counts.get('average')
reduction = None

if (
    isinstance(raw_average, (int, float))
    and raw_average > 0
    and isinstance(filtered_average, (int, float))
):
    reduction = 100.0 * (1.0 - filtered_average / raw_average)

print(
    'Raw cloud rate: '
    f"{raw.get('average_rate_hz', 'unknown')} Hz "
    f"(minimum={raw.get('minimum_instantaneous_rate_hz', 'unknown')} Hz, "
    f"maximum={raw.get('maximum_instantaneous_rate_hz', 'unknown')} Hz, "
    f"max_gap={raw.get('maximum_gap_s', 'unknown')} s)"
)
print(
    'Filtered cloud rate: '
    f"{filtered.get('average_rate_hz', 'unknown')} Hz "
    f"(minimum={filtered.get('minimum_instantaneous_rate_hz', 'unknown')} Hz, "
    f"maximum={filtered.get('maximum_instantaneous_rate_hz', 'unknown')} Hz, "
    f"max_gap={filtered.get('maximum_gap_s', 'unknown')} s)"
)
print(
    'Point counts: '
    f"raw_average={raw_average} "
    f"filtered_average={filtered_average} "
    f"reduction={reduction}%"
)
print(
    'Timestamp/latency: '
    f"raw_age_mean={stamp_age.get('average_s', 'unknown')} s "
    f"receipt_latency_mean={latency.get('average', 'unknown')} ms "
    f"receipt_latency_max={latency.get('maximum', 'unknown')} ms"
)
print(
    'Status counters: '
    f"{status.get('latest_status', status)}"
)


def print_scene(label, document):
    scene = document.get('scene', {})
    selected = scene.get('selected_roi', {})
    filtered_roi = selected.get('output', {})
    paired = scene.get('paired_roi_evidence', {})
    print(
        f'{label} coordinate validation: '
        f"filtered_min={filtered_roi.get('minimum_xyz')} "
        f"filtered_max={filtered_roi.get('maximum_xyz')} "
        f"filtered_centroid={filtered_roi.get('weighted_centroid_xyz')} "
        f"same_stamp_pairs={paired.get('pairs')} "
        f"qualifying_pairs={paired.get('qualifying_pairs')}"
    )


print_scene('Front', front)
print_scene('Left', left)
print_scene('Right', right)
body_extents = self_scene.get('scene', {}).get(
    'raw_body_candidate_extents_m',
    {},
)
print(
    'Measured robot-body candidate extents: '
    f"minimum_xyz={body_extents.get('minimum_xyz')} "
    f"maximum_xyz={body_extents.get('maximum_xyz')} "
    f"contributing_points={body_extents.get('contributing_points')}"
)
PY

  report "Original self-filter: $ORIGINAL_SELF_FILTER"
  report "Final self-filter: $FINAL_SELF_FILTER"
  report "Self-filter evaluation: $SELF_FILTER_RESULT"
  report "Stale detection: $STALE_RESULT"
  report "Recovery: $RECOVERY_RESULT"
  report "Authority audit: $AUTHORITY_RESULT"
  report "Residual-process audit: $RESIDUAL_RESULT"
  report "savo_realsense package tests: $REALSENSE_TOTALS"
  report "savo_perception package tests: $PERCEPTION_TOTALS"

  awk '
    {
      for (field = 1; field <= NF; ++field) {
        split($field, pair, "=")
        if (pair[1] == "cpu_percent") {
          cpu += pair[2]
        } else if (pair[1] == "rss_kib") {
          rss += pair[2]
        }
      }
      samples += 1
    }
    END {
      if (samples > 0) {
        printf(
          "Camera process-group resources: "
          "average_cpu_percent=%.2f average_rss_kib=%.0f\n",
          cpu / samples,
          rss / samples
        )
      }
    }
  ' "$EVIDENCE_DIR/resources_camera.txt" |
    tee -a "$REPORT_FILE"

  awk '
    {
      for (field = 1; field <= NF; ++field) {
        split($field, pair, "=")
        if (pair[1] == "cpu_percent") {
          cpu += pair[2]
        } else if (pair[1] == "rss_kib") {
          rss += pair[2]
        }
      }
      samples += 1
    }
    END {
      if (samples > 0) {
        printf(
          "Filter process-group resources: "
          "average_cpu_percent=%.2f average_rss_kib=%.0f\n",
          cpu / samples,
          rss / samples
        )
      }
    }
  ' "$EVIDENCE_DIR/resources_filter.txt" |
    tee -a "$REPORT_FILE"
}

finalize_success()
{
  section "8A5.10 cleanup and residual-process audit"

  cleanup_all_processes ||
    fail "One or more script-owned process groups resisted cleanup"

  audit_tracked_processes ||
    fail "A script-owned Phase 8A5 process group remains"

  clean_source_artifacts
  assert_no_source_artifacts ||
    fail "Source/install cache artifacts remain"

  print_hardware_report

  [[ "$STALE_RESULT" == "PASS" ]] ||
    fail "Stale-input validation did not pass"

  [[ "$RECOVERY_RESULT" == "PASS" ]] ||
    fail "Recovery validation did not pass"

  [[ "$AUTHORITY_RESULT" == "PASS" ]] ||
    fail "Authority audit did not pass"

  [[ "$RESIDUAL_RESULT" == "PASS" ]] ||
    fail "Residual-process audit did not pass"

  [[ "$SELF_FILTER_RESULT" == "PASS" ]] ||
    fail "Self-filter evaluation did not pass"

  SUCCESS=1
  report ""
  report "Phase 8A5 real D435 hardware validation completed successfully"
  report "Phase 8A obstacle-cloud producer stationary acceptance is complete"
  report "Phase 8B Nav2 validation is tracked separately in savo_nav"
  report "Full production D435 validation remains gated"
}

main()
{
  report "Robot Savo Phase 8A5 D435 hardware validation"
  report "Started: $(timestamp)"
  report "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

  validate_workspace_layout
  hardware_preflight
  stationary_safety_gate
  create_required_backups
  validate_source_syntax
  clean_build_and_test_packages

  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  # shellcheck disable=SC1090
  source "$WORKSPACE/install/setup.bash"

  start_camera
  ensure_production_robot_description
  start_filter

  run_comprehensive_observation initial 20
  sample_process_group_resources camera "$CAMERA_INDEX"
  sample_process_group_resources filter "$FILTER_INDEX"

  ORIGINAL_SELF_FILTER="$(read_self_filter_values)"
  FINAL_SELF_FILTER="$ORIGINAL_SELF_FILTER"

  run_physical_scene_suite original
  evaluate_self_filter original
  authority_audit
  run_stale_and_recovery_validation
  run_comprehensive_observation final 20
  finalize_success
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
