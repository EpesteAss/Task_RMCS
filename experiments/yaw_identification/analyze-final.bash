#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
profile="${1:-final}"
if [[ "$profile" != "final" && "$profile" != "wide" ]]; then
  echo "Usage: bash analyze-final.bash [final|wide]" >&2
  exit 2
fi

latest_run() {
  local mode="$1"
  [[ -d "control_data/control-${mode}-${profile}" ]] || return 0
  find "control_data/control-${mode}-${profile}" -mindepth 2 -maxdepth 2 \
    -name feedback.csv -type f -print | sort | tail -n 1
}

baseline="$(latest_run baseline)"
tuned="$(latest_run tuned)"
if [[ -z "$baseline" || -z "$tuned" ]]; then
  echo "Need one recorded baseline-${profile} and tuned-${profile} run first." >&2
  exit 1
fi
printf 'Baseline: %s\nTuned: %s\n' "$baseline" "$tuned"
python3 compare_control.py --baseline "$baseline" --tuned "$tuned" \
  --out "control_analysis_${profile}_latest"
