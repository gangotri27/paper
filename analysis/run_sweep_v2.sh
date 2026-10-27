#!/bin/bash
# Usage:  run_sweep_v2.sh <6dof|5dof> <goals_file> [trials_per_goal]
# Needs move_group for that arm already running in ANOTHER terminal (see instructions).
# Run it from a shell that sourced /opt/ros/jazzy/setup.bash and ~/new_ws_review/install/setup.bash.
CONFIG=${1:?config 6dof|5dof}
GOALS=${2:?goals file}
TRIALS=${3:-10}
OUT=$HOME/runs_v2
mkdir -p "$OUT"
for W in 0.12 0.14 0.16 0.18 0.20; do
  DB="$OUT/planning_${CONFIG}_w${W}.db"
  if [ -s "$DB" ]; then echo "SKIP $DB exists"; continue; fi
  echo "=== $CONFIG  wall_y=$W  clear gap=$(python3 -c "print(round(2*$W-0.14,2))") m ==="
  ros2 run omx_benchmark scene_builder_variable --ros-args -p wall_y:=$W > /dev/null 2>&1
  sleep 2
  python3 "$(dirname "$0")/planning_benchmark_runner_v2.py" --ros-args \
    -p config_type:=$CONFIG -p goals_file:=$GOALS -p trials_per_goal:=$TRIALS \
    -p db_path:=$DB -p planning_time:=10.0 -p wall_y:=$W -p run_id:=${CONFIG}_w${W}_$(date +%Y%m%d_%H%M)
done