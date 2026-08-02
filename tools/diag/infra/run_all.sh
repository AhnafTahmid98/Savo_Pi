#!/usr/bin/env bash
set -Eeuo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
output_dir="${SAVO_DIAG_OUTPUT_DIR:-$repo_root/log/diag/$(date -u +%Y%m%dT%H%M%SZ)}"
allow_motion=false
while (($#)); do
  case "$1" in
    --output-dir) output_dir="${2:-}"; shift 2 ;;
    --allow-motion) allow_motion=true; shift ;;
    *) echo "Usage: $0 [--output-dir DIR] [--allow-motion]" >&2; exit 2 ;;
  esac
done
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
result=0
for script in "${nonmoving[@]}"; do
  name="$(basename "$script" .py)"
  python3 "$repo_root/$script" --output "$output_dir/$name.json" || result=$?
done
if $allow_motion; then
  echo "Moving diagnostics are not automated: a physical safety operator must run each one individually." >&2
  result=2
fi
echo "Diagnostic results: $output_dir"
exit "$result"
