#!/usr/bin/env python3
"""
Simple planning test without constraints
"""

import rclpy
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from task_success_benchmark.multi_stage_planner import MultiStagePlanner
import time

def main():
    rclpy.init()
    
    print("\n" + "="*70)
    print("SIMPLE PLANNING TEST")
    print("="*70)
    
    planner = MultiStagePlanner(robot_type='6dof')
    
    # Test different poses with increasing complexity
    test_poses = [
        ("Very close pose", 0.10, 0.0, 0.10, 0.0),
        ("Simple forward", 0.15, 0.0, 0.15, 0.0),
        ("Your pick pose", 0.20, -0.10, 0.15, 0.0),
        ("With small roll", 0.20, 0.0, 0.20, 0.2),
        ("With 45° roll", 0.20, 0.0, 0.20, 0.785),
    ]
    
    for name, x, y, z, roll in test_poses:
        print(f"\n{name}:")
        print(f"  Pose: x={x:.2f}, y={y:.2f}, z={z:.2f}, roll={roll:.2f}rad")
        
        # Reset to start
        planner.reset_to_start()
        
        # Try to plan
        traj, error = planner.plan_to_pose((x, y, z, roll), seed=12345)
        
        if error == 1:
            print(f"  ✓ SUCCESS!")
            if traj and traj.joint_trajectory.points:
                print(f"    Path length: {len(traj.joint_trajectory.points)} waypoints")
        else:
            error_map = {
                -2: "PLANNING_FAILED",
                -3: "INVALID_MOTION_PLAN",
                -7: "TIMED_OUT", 
                -10: "START_STATE_IN_COLLISION",
                -12: "GOAL_IN_COLLISION",
                -14: "GOAL_CONSTRAINTS_VIOLATED",
                -31: "NO_IK_SOLUTION",
                99999: "GOAL_REJECTED"
            }
            error_name = error_map.get(error, f"UNKNOWN_{error}")
            print(f"  ✗ FAILED: {error_name}")
    
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
