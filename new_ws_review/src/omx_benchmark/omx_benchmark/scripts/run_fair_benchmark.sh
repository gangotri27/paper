#!/bin/bash
# Complete Fair Benchmark Execution Script
# Compares 5-DOF vs 6-DOF with proper experimental design

set -e  # Exit on error

# Configuration
NUM_GOALS=20
TRIALS_PER_GOAL=10
PLANNING_TIME=10.0
SEED=42

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Create timestamp for this run
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RESULTS_DIR=~/fair_benchmark_${TIMESTAMP}
mkdir -p $RESULTS_DIR

echo -e "${GREEN}========================================================================"
echo "FAIR 5-DOF vs 6-DOF BENCHMARK"
echo "========================================================================"
echo -e "${NC}Results directory: $RESULTS_DIR"
echo "Configuration:"
echo "  Goals per robot: $NUM_GOALS"
echo "  Trials per goal: $TRIALS_PER_GOAL"
echo "  Total trials per robot: $((NUM_GOALS * TRIALS_PER_GOAL))"
echo "  Planning timeout: ${PLANNING_TIME}s"
echo "  Random seed: $SEED"
echo -e "${GREEN}========================================================================${NC}"
echo ""

# Step 1: Source workspaces
echo -e "${YELLOW}[1/6] Sourcing workspaces...${NC}"
source ~/omx_ws/install/setup.bash
source ~/new_ws/install/setup.bash
echo -e "${GREEN}  ✓ Workspaces sourced${NC}"
echo ""

# Step 2: Build the benchmark package
echo -e "${YELLOW}[2/6] Building benchmark package...${NC}"
cd ~/new_ws
colcon build --packages-select omx_benchmark --symlink-install
source install/setup.bash
echo -e "${GREEN}  ✓ Build complete${NC}"
echo ""

# Step 3: Generate task-space goals
echo -e "${YELLOW}[3/6] Generating task-space goals (requires wrist roll)...${NC}"
ros2 run omx_benchmark task_goal_generator --ros-args \
    -p num_goals:=$NUM_GOALS \
    -p output_file_5dof:=$RESULTS_DIR/goals_5dof.json \
    -p output_file_6dof:=$RESULTS_DIR/goals_6dof.json \
    -p seed:=$SEED

# Verify goals were generated
if [ ! -f "$RESULTS_DIR/goals_5dof.json" ] || [ ! -f "$RESULTS_DIR/goals_6dof.json" ]; then
    echo -e "${RED}  ✗ Goal generation failed${NC}"
    exit 1
fi
echo -e "${GREEN}  ✓ Goals generated${NC}"
echo ""

# Step 4: Run 5-DOF benchmark
echo -e "${YELLOW}[4/6] Running 5-DOF benchmark...${NC}"
echo "  This will take approximately $((NUM_GOALS * TRIALS_PER_GOAL * 10 / 60)) minutes"
ros2 launch ~/new_ws/src/omx_benchmark/launch/planning_benchmark_5dof.launch.py \
    num_goals:=$NUM_GOALS \
    trials_per_goal:=$TRIALS_PER_GOAL \
    planning_time:=$PLANNING_TIME \
    seed:=$SEED \
    db_path:=$RESULTS_DIR/planning_5dof.db \
    goals_file:=$RESULTS_DIR/goals_5dof.json

echo -e "${GREEN}  ✓ 5-DOF benchmark complete${NC}"
echo ""

# Step 5: Run 6-DOF benchmark
echo -e "${YELLOW}[5/6] Running 6-DOF benchmark...${NC}"
echo "  This will take approximately $((NUM_GOALS * TRIALS_PER_GOAL * 10 / 60)) minutes"
ros2 launch ~/new_ws/src/omx_benchmark/launch/planning_benchmark_6dof.launch.py \
    num_goals:=$NUM_GOALS \
    trials_per_goal:=$TRIALS_PER_GOAL \
    planning_time:=$PLANNING_TIME \
    seed:=$SEED \
    db_path:=$RESULTS_DIR/planning_6dof.db \
    goals_file:=$RESULTS_DIR/goals_6dof.json

echo -e "${GREEN}  ✓ 6-DOF benchmark complete${NC}"
echo ""

# Step 6: Analyze results
echo -e "${YELLOW}[6/6] Analyzing results...${NC}"
python3 ~/new_ws/src/omx_benchmark/omx_benchmark/planning_analyzer.py \
    --db-5dof $RESULTS_DIR/planning_5dof.db \
    --db-6dof $RESULTS_DIR/planning_6dof.db \
    --output-dir $RESULTS_DIR/analysis

echo -e "${GREEN}  ✓ Analysis complete${NC}"
echo ""

# Summary
echo -e "${GREEN}========================================================================"
echo "BENCHMARK COMPLETE"
echo "========================================================================"
echo -e "${NC}Results saved to: $RESULTS_DIR"
echo ""
echo "Quick summary:"
echo "5-DOF:"
sqlite3 $RESULTS_DIR/planning_5dof.db "SELECT '  Trials: ' || COUNT(*) || ', Successes: ' || SUM(success) || ', Rate: ' || ROUND(AVG(success)*100,1) || '%' FROM planning_results;" 2>/dev/null || echo "  No data"
echo "6-DOF:"
sqlite3 $RESULTS_DIR/planning_6dof.db "SELECT '  Trials: ' || COUNT(*) || ', Successes: ' || SUM(success) || ', Rate: ' || ROUND(AVG(success)*100,1) || '%' FROM planning_results;" 2>/dev/null || echo "  No data"
echo ""
echo "Detailed analysis:"
echo "  $RESULTS_DIR/analysis/analysis_report.txt"
echo "  $RESULTS_DIR/analysis/figure1_success_rates.png"
echo "  $RESULTS_DIR/analysis/figure2_planning_times.png"
echo "  $RESULTS_DIR/analysis/figure3_failure_modes.png"
echo -e "${GREEN}========================================================================${NC}"