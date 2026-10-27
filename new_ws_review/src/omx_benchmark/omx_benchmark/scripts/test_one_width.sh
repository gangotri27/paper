#!/bin/bash
# Test one specific width

WALL_Y=$1

if [ -z "$WALL_Y" ]; then
    echo "Usage: ./test_one_width.sh <wall_y>"
    echo "Example: ./test_one_width.sh 0.18"
    exit 1
fi

CORRIDOR_WIDTH=$(echo "2 * $WALL_Y" | bc)
echo "========================================================"
echo "Testing width ${CORRIDOR_WIDTH}m (walls at ±${WALL_Y}m)"
echo "========================================================"

# Launch 5-DOF robot
echo "📌 Launching 5-DOF robot..."
source ~/omx_ws/install/setup.bash
ros2 launch ~/omx_ws/src/omx_5dof_moveit/launch/demo.launch.py use_rviz:=false &
ROBOT_PID=$!
sleep 20

# Build scene with CORRECT width
echo "📌 Building scene with walls at ±${WALL_Y}m..."
source ~/new_ws/install/setup.bash
ros2 run omx_benchmark scene_builder_fixed --ros-args -p wall_y:=$WALL_Y
sleep 2

# Generate goals with CORRECT width
echo "📌 Generating 5-DOF goals for width ${WALL_Y}..."
ros2 run omx_benchmark generate_5dof_goals --ros-args -p wall_y:=$WALL_Y

# Clean up
echo "📌 Cleaning up..."
kill $ROBOT_PID
sleep 5

echo ""
echo "✅ Done! Check: /home/ubuntu/benchmark_test/goals_5dof_w${WALL_Y}.json"