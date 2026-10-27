#!/bin/bash
# Run corridor width sweep with NO BACK WALL and RViz disabled

set -e

RESULTS_DIR=~/width_sweep_results_$(date +%Y%m%d_%H%M%S)
mkdir -p $RESULTS_DIR

echo "========================================================"
echo "CORRIDOR WIDTH SWEEP (NO BACK WALL, RViz DISABLED)"
echo "========================================================"
echo "Results: $RESULTS_DIR"
echo ""

# Widths to test: from 0.12m to 0.20m
WALL_Y_VALUES=(0.12 0.14 0.16 0.18 0.20)

for WALL_Y in "${WALL_Y_VALUES[@]}"; do
    CORRIDOR_WIDTH=$(echo "2 * $WALL_Y" | bc)
    echo ""
    echo "========================================================"
    echo "TESTING WIDTH: ${CORRIDOR_WIDTH}m (walls at ±${WALL_Y}m)"
    echo "========================================================"
    
    WIDTH_DIR=$RESULTS_DIR/width_${CORRIDOR_WIDTH}m
    mkdir -p $WIDTH_DIR
    
    # --- 6-DOF Benchmark ---
    echo "📌 Running 6-DOF..."
    source ~/new_ws/install/setup.bash
    
    # Launch robot (with RViz disabled for headless operation)
    ros2 launch ~/new_ws/src/omx_6dof_moveit/launch/demo.launch.py use_rviz:=false &
    ROBOT_PID=$!
    sleep 20
    
    # Build scene with current width
    ros2 run omx_benchmark scene_builder_variable --ros-args -p wall_y:=$WALL_Y
    
    # Run benchmark
    ros2 run omx_benchmark planning_benchmark_runner --ros-args \
        -p config_type:=6dof \
        -p goals_file:=/home/ubuntu/benchmark_test/goals_6dof_corrected.json \
        -p trials_per_goal:=5 \
        -p db_path:=$WIDTH_DIR/planning_6dof.db \
        -p planning_time:=10.0
    
    kill $ROBOT_PID
    sleep 5
    
    # --- 5-DOF Benchmark ---
    echo "📌 Running 5-DOF..."
    source ~/omx_ws/install/setup.bash
    
    # Launch 5-DOF robot WITHOUT RViz (FIXES THE CRASH!)
    ros2 launch ~/omx_ws/src/omx_5dof_moveit/launch/demo.launch.py use_rviz:=false &
    ROBOT_PID=$!
    sleep 20
    
    # Build scene (use the fixed scene builder)
    source ~/new_ws/install/setup.bash
    ros2 run omx_benchmark scene_builder_variable --ros-args -p wall_y:=$WALL_Y
    
    # Run benchmark
    ros2 run omx_benchmark planning_benchmark_runner --ros-args \
        -p config_type:=5dof \
        -p goals_file:=/home/ubuntu/benchmark_test/goals_5dof_all.json \
        -p trials_per_goal:=5 \
        -p db_path:=$WIDTH_DIR/planning_5dof.db \
        -p planning_time:=10.0
    
    kill $ROBOT_PID
    sleep 5
    
    echo "✅ Completed width ${CORRIDOR_WIDTH}m"
done

echo ""
echo "========================================================"
echo "WIDTH SWEEP COMPLETE"
echo "Results saved to: $RESULTS_DIR"
echo "========================================================"