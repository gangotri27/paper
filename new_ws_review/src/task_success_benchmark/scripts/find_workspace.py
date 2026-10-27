#!/usr/bin/env python3
"""
Find the actual reachable workspace by testing various positions
"""

import rclpy
import sys
import os
import numpy as np
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from task_success_benchmark.multi_stage_planner import MultiStagePlanner

class WorkspaceFinder:
    def __init__(self, robot_type='6dof'):
        self.planner = MultiStagePlanner(robot_type=robot_type)
        
    def test_pose(self, x, y, z, roll=0.0):
        """Test if a pose is reachable"""
        self.planner.reset_to_start()
        traj, error = self.planner.plan_to_pose((x, y, z, roll), seed=12345)
        return error == 1
    
    def find_reachable_region(self):
        """Test a grid of positions to find reachable area"""
        print("\n" + "="*70)
        print("REACHABLE WORKSPACE MAPPING")
        print("="*70)
        
        # Test positions in a grid
        x_range = np.arange(0.1, 0.4, 0.05)
        y_range = np.arange(-0.2, 0.21, 0.05)
        z_fixed = 0.2  # Fixed height for testing
        
        print(f"\nTesting at height z={z_fixed}m:")
        print("   y→  ", end="")
        for y in y_range:
            print(f"{y:6.2f}", end="")
        print()
        
        for x in x_range:
            print(f"x={x:.2f}: ", end="")
            for y in y_range:
                reachable = self.test_pose(x, y, z_fixed)
                marker = "✓" if reachable else "·"
                print(f"   {marker}  ", end="")
            print()
        
        # Test different heights
        print("\n\nTesting along center line (x=0.2, y=0):")
        z_range = np.arange(0.1, 0.4, 0.03)
        print("z (m)  | No Roll | Roll 45° | Roll 90°")
        print("-------|---------|----------|---------")
        
        for z in z_range:
            no_roll = self.test_pose(0.2, 0.0, z, 0.0)
            roll45 = self.test_pose(0.2, 0.0, z, 0.785)
            roll90 = self.test_pose(0.2, 0.0, z, 1.571)
            
            print(f"{z:.2f}   |   {'✓' if no_roll else '·'}    |    {'✓' if roll45 else '·'}    |   {'✓' if roll90 else '·'}")

def main():
    rclpy.init()
    finder = WorkspaceFinder('6dof')
    try:
        finder.find_reachable_region()
    finally:
        finder.planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
