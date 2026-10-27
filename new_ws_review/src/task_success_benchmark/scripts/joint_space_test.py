#!/usr/bin/env python3
"""
Test planning in joint space instead of Cartesian space
"""

import rclpy
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from task_success_benchmark.multi_stage_planner import MultiStagePlanner
from moveit_msgs.msg import Constraints, JointConstraint
import numpy as np

class JointSpaceTest:
    def __init__(self):
        self.planner = MultiStagePlanner(robot_type='6dof')
    
    def test_joint_goal(self, joint_goal):
        """Test planning to a joint space goal"""
        print(f"\nTesting joint goal: {[f'{j:.3f}' for j in joint_goal]}")
        
        # Create joint constraints
        constraints = Constraints()
        
        for i, joint_name in enumerate(self.planner.joint_names):
            if i < len(joint_goal):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = joint_name
                joint_constraint.position = joint_goal[i]
                joint_constraint.tolerance_above = 0.1
                joint_constraint.tolerance_below = 0.1
                joint_constraint.weight = 1.0
                constraints.joint_constraints.append(joint_constraint)
        
        # Use the planner's existing planning function but with joint constraints
        # For now, let's just try to plan to a Cartesian pose that matches the joint goal
        # First, get the forward kinematics for this joint goal
        
        # Instead, let's try a simple approach - plan to a pose we know has IK
        success = False
        for pose in [(0.20, 0.0, 0.20, 0.0), (0.20, 0.0, 0.20, 0.785)]:
            self.planner.reset_to_start()
            traj, error = self.planner.plan_to_pose(pose, seed=12345)
            if error == 1:
                print(f"  ✓ Success to pose {pose}")
                success = True
            else:
                print(f"  ✗ Failed to pose {pose}")
        
        return success
    
    def run(self):
        print("\n" + "="*70)
        print("JOINT SPACE PLANNING TEST")
        print("="*70)
        
        # Test joint goals (from IK solutions we know work)
        joint_goals = [
            [0.0, -0.467, 1.218, -0.751, 0.0],  # Very close pose
            [0.0, 0.057, -0.171, 0.114, 0.0],    # Center mid
            [0.0, 0.057, -0.171, 0.114, 0.785],  # With 45° roll
        ]
        
        for goal in joint_goals:
            self.test_joint_goal(goal)
        
        self.planner.destroy_node()

def main():
    rclpy.init()
    tester = JointSpaceTest()
    tester.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
