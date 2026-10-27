#!/usr/bin/env python3
"""
Generate TRUE 5-DOF goals for each width WITH CORRECT SCENE
"""
import subprocess
import time
import os

# Widths to test
widths = [0.12, 0.14, 0.16, 0.18, 0.20]

for wall_y in widths:
    corridor_width = 2 * wall_y
    print(f"\n{'='*60}")
    print(f"Generating for width {corridor_width:.2f}m (walls at ±{wall_y}m)")
    print(f"{'='*60}")
    
    # Step 1: Kill any running robot
    os.system("pkill -f ros2")
    time.sleep(2)
    
    # Step 2: Launch robot with CORRECT scene width
    print("\n📌 Launching robot with correct scene...")
    robot_process = subprocess.Popen([
        "ros2", "launch", 
        "/home/ubuntu/omx_ws/src/omx_5dof_moveit/launch/demo.launch.py",
        "use_rviz:=false"
    ])
    time.sleep(20)  # Wait for robot to initialize
    
    # Step 3: Build scene with CURRENT width
    print(f"\n📌 Building scene with walls at ±{wall_y}m...")
    scene_cmd = [
        "ros2", "run", "omx_benchmark", "scene_builder_fixed",
        "--ros-args", "-p", f"wall_y:={wall_y}"
    ]
    subprocess.run(scene_cmd)
    time.sleep(5)  # Wait for scene to apply
    
    # Step 4: Generate goals for this width
    print(f"\n📌 Generating TRUE 5-DOF goals...")
    gen_cmd = [
        "python3", 
        "/home/ubuntu/new_ws/src/omx_benchmark/omx_benchmark/scripts/convert_6dof_to_5dof.py",
        "--wall_y", str(wall_y)
    ]
    subprocess.run(gen_cmd)
    
    # Step 5: Kill robot
    robot_process.terminate()
    time.sleep(5)

print("\n✅ All widths processed!")