#!/usr/bin/env bash
set -Eeuo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
output_dir="${SAVO_DIAG_OUTPUT_DIR:-$repo_root/log/diag/$(date -u +%Y%m%dT%H%M%SZ)}"
allow_motion=false

while (($#)); do
  case "$1" in
    --output-dir)
      output_dir="${2:-}"
      shift 2
      ;;
    --allow-motion)
      allow_motion=true
      shift
      ;;
    *)
      echo "Usage: $0 [--output-dir DIR] [--allow-motion]" >&2
      exit 2
      ;;
  esac
done

if [[ -z "$output_dir" ]]; then
  echo "Output directory must not be empty" >&2
  exit 2
fi

mkdir -p "$output_dir"
export PYTHONPATH="$repo_root${PYTHONPATH:+:$PYTHONPATH}"

nonmoving=(
  tools/diag/safety/estop_test.py
  tools/diag/safety/safety_stop_test.py
  tools/diag/safety/cmd_vel_gate_test.py
  tools/diag/motion/odom_test.py
  tools/diag/sensors/apriltag_test.py
  tools/diag/ui/screen_ui_test.py
  tools/diag/voice/audio_mic_test.py
  tools/diag/voice/asr_topic_test.py
  tools/diag/voice/tts_topic_test.py
  tools/diag/power/current_draw_logger.py
  tools/diag/infra/tf_tree_check.py
)

saw_failure=false
saw_blocked=false
for script in "${nonmoving[@]}"; do
  name="$(basename "$script" .py)"
  set +e
  python3 "$repo_root/$script" --output "$output_dir/$name.json"
  code=$?
  set -e
  case "$code" in
    0) ;;
    1) saw_failure=true ;;
    2) saw_blocked=true ;;
    *)
      echo "Unexpected diagnostic exit code $code from $script" >&2
      saw_failure=true
      ;;
  esac
done

if $allow_motion; then
  cat >&2 <<'MSG'
Moving diagnostics are never started by run_all.sh.
Run each approved movement diagnostic individually with a physical safety operator,
its explicit motion opt-in, and the required physical preconditions.
MSG
fi

echo "Diagnostic results: $output_dir"
if $saw_failure; then
  exit 1
fi
if $saw_blocked; then
  exit 2
fi
exit 0
