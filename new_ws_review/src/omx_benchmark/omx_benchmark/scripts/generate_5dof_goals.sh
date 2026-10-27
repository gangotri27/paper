#!/bin/bash
# Generate 5-DOF goals for all corridor widths using 5-DOF robot

echo "========================================================"
echo "GENERATING 5-DOF GOALS FOR ALL WIDTHS"
echo "========================================================"

# Widths to test
WIDTHS=(0.12 0.14 0.16 0.18 0.20)

for WALL_Y in "${WIDTHS[@]}"; do
    CORRIDOR_WIDTH=$(echo "2 * $WALL_Y" | bc)
    echo ""
    echo "========================================================"
    echo "WIDTH: ${CORRIDOR_WIDTH}m (walls at ±${WALL_Y}m)"
    echo "========================================================"
    
    # Step 1: Launch 5-DOF robot (we need its IK service!)
    echo "📌 Launching 5-DOF robot..."
    source ~/omx_ws/install/setup.bash
    ros2 launch ~/omx_ws/src/omx_5dof_moveit/launch/demo.launch.py use_rviz:=false &
    ROBOT_PID=$!
    sleep 20
    
    # Step 2: Build scene with current width (using tools from new_ws)
    echo "📌 Building scene..."
    source ~/new_ws/install/setup.bash
    ros2 run omx_benchmark scene_builder_fixed --ros-args -p wall_y:=$WALL_Y
    sleep 2
    
    # Step 3: Generate 5-DOF goals (still using tools from new_ws)
    echo "📌 Generating 5-DOF goals..."
    ros2 run omx_benchmark generate_5dof_goals --ros-args -p wall_y:=$WALL_Y
    
    # Step 4: Kill robot
    echo "📌 Cleaning up..."
    kill $ROBOT_PID
    sleep 5
done

echo ""
echo "========================================================"
echo "✅ ALL DONE! Check these files:"
ls -la /home/ubuntu/benchmark_test/goals_5dof_w*.json
echo "========================================================"