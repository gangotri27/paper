#!/bin/bash
# Corridor planning sweep WITHOUT the top barrier (walls + bottom plate only).
# Usage:  run_sweep_nobarrier.sh <6dof|5dof> <goals_file> [trials_per_goal] [out_dir]
# Needs the matching move_group running in another terminal, and /opt/ros/jazzy sourced (no venv).
# scene_builder_options.py and planning_benchmark_runner_v2.py must sit in the same folder as this script.
# Override the widths with:  WIDTHS="0.13 0.14" bash run_sweep_nobarrier.sh ...
CONFIG=${1:?config 6dof|5dof}
GOALS=${2:?goals file}
TRIALS=${3:-10}
OUT=${4:-$HOME/runs_nobarrier}
HERE="$(cd "$(dirname "$0")" && pwd)"
WIDTHS=${WIDTHS:-"0.115 0.12 0.125 0.13 0.135 0.14 0.15 0.16 0.18 0.20"}
mkdir -p "$OUT"
for W in $WIDTHS; do
  DB="$OUT/planning_${CONFIG}_w${W}.db"
  if [ -s "$DB" ]; then echo "SKIP $DB exists"; continue; fi
  GAP=$(python3 -c "print(round(2*$W-0.14,4))")
  echo "=== $CONFIG  wall_y=$W  clear gap=$GAP m  (walls + plate, NO barrier) ==="
  python3 "$HERE/scene_builder_options.py" --wall-y "$W" --top-barrier false --bottom-plate true > /dev/null || { echo "scene builder failed"; exit 1; }
  python3 "$HERE/planning_benchmark_runner_v2.py" --ros-args \
    -p config_type:=$CONFIG -p goals_file:=$GOALS -p trials_per_goal:=$TRIALS \
    -p db_path:=$DB -p planning_time:=10.0 -p wall_y:=$W \
    -p run_id:=${CONFIG}_nobarrier_w${W}_$(date +%Y%m%d_%H%M)
done