#!/usr/bin/env python3
"""
Simple script to test if a pose is reachable
"""

import rclpy
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from task_success_benchmark.multi_stage_planner import MultiStagePlanner
import time

def main():
    rclpy.init()
    
    # Test different poses
    test_poses = [
        (0.20, -0.10, 0.25, 0.0),   # approach_pick
        (0.20, -0.10, 0.15, 0.0),   # pick (lower)
        (0.20, 0.10, 0.15, 0.0),    # place
        (0.20, 0.10, 0.25, 0.0),    # approach_place
        (0.20, -0.10, 0.30, 0.0),   # higher approach
    ]
    
    print("\n" + "="*60)
    print("POSE REACHABILITY TEST")
    print("="*60)
    
    # Test 6-DOF
    print("\n[ Testing 6-DOF Robot ]")
    planner_6dof = MultiStagePlanner(robot_type='6dof')
    
    for i, pose in enumerate(test_poses):
        print(f"\nPose {i+1}: x={pose[0]:.2f}, y={pose[1]:.2f}, z={pose[2]:.2f}, roll={pose[3]:.1f}")
        
        # Reset to start state before each test
        planner_6dof.reset_to_start()
        
        # Try to plan
        trajectory, error_code = planner_6dof.plan_to_pose(pose, seed=12345)
        
        if error_code == 1:
            print(f"  ✓ REACHABLE (success)")
        else:
            # Map error codes
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
            error_name = error_map.get(error_code, f"UNKNOWN_{error_code}")
            print(f"  ✗ NOT REACHABLE: {error_name}")
    
    planner_6dof.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
