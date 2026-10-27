#!/bin/bash
# Complete planning benchmark execution script

set -e  # Exit on error

# Configuration
NUM_GOALS=20
TRIALS_PER_GOAL=10
PLANNING_TIME=10.0
SEED=42

# Create timestamp for this run
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RESULTS_DIR=~/planning_benchmark_${TIMESTAMP}
mkdir -p $RESULTS_DIR

echo "========================================================================"
echo "PLANNING BENCHMARK - $(date)"
echo "========================================================================"
echo "Results directory: $RESULTS_DIR"
echo "Configuration:"
echo "  Goals per robot: $NUM_GOALS"
echo "  Trials per goal: $TRIALS_PER_GOAL"
echo "  Planning timeout: ${PLANNING_TIME}s"
echo "  Random seed: $SEED"
echo "========================================================================"

# Source workspaces
echo -e "\n[1/5] Sourcing workspaces..."
source ~/omx_ws/install/setup.bash
source ~/new_ws/install/setup.bash

# Build if needed
echo -e "\n[2/5] Building packages..."
cd ~/new_ws
colcon build --packages-select omx_benchmark --symlink-install
source install/setup.bash

# Run 5-DOF benchmark
echo -e "\n[3/5] Running 5-DOF planning benchmark..."
ros2 launch omx_benchmark planning_benchmark_5dof.launch.py \
    num_goals:=$NUM_GOALS \
    trials_per_goal:=$TRIALS_PER_GOAL \
    planning_time:=$PLANNING_TIME \
    seed:=$SEED \
    db_path:=$RESULTS_DIR/planning_5dof.db \
    goals_file:=$RESULTS_DIR/goals_5dof.json

# Run 6-DOF benchmark
echo -e "\n[4/5] Running 6-DOF planning benchmark..."
ros2 launch omx_benchmark planning_benchmark_6dof.launch.py \
    num_goals:=$NUM_GOALS \
    trials_per_goal:=$TRIALS_PER_GOAL \
    planning_time:=$PLANNING_TIME \
    seed:=$SEED \
    db_path:=$RESULTS_DIR/planning_6dof.db \
    goals_file:=$RESULTS_DIR/goals_6dof.json

# Analyze results
echo -e "\n[5/5] Analyzing results..."
python3 ~/new_ws/src/omx_benchmark/omx_benchmark/planning_analyzer.py \
    --db-5dof $RESULTS_DIR/planning_5dof.db \
    --db-6dof $RESULTS_DIR/planning_6dof.db \
    --output-dir $RESULTS_DIR/analysis

echo -e "\n========================================================================"
echo "BENCHMARK COMPLETE"
echo "========================================================================"
echo "Results saved to: $RESULTS_DIR"
echo "Analysis saved to: $RESULTS_DIR/analysis"
echo ""
echo "Quick summary:"
sqlite3 $RESULTS_DIR/planning_5dof.db "SELECT '5-DOF: ' || COUNT(*) || ' trials, ' || SUM(success) || ' successes, ' || ROUND(AVG(success)*100,1) || '% success rate' FROM planning_results;" 2>/dev/null || echo "5-DOF: No data"
sqlite3 $RESULTS_DIR/planning_6dof.db "SELECT '6-DOF: ' || COUNT(*) || ' trials, ' || SUM(success) || ' successes, ' || ROUND(AVG(success)*100,1) || '% success rate' FROM planning_results;" 2>/dev/null || echo "6-DOF: No data"
echo "========================================================================"