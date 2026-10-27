#!/usr/bin/env python3
"""
Find the actual reachable workspace by testing various positions
Stores results and shows summary at the end
"""

import rclpy
import sys
import os
import numpy as np
import csv
from datetime import datetime
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from task_success_benchmark.multi_stage_planner import MultiStagePlanner

class WorkspaceFinder:
    def __init__(self, robot_type='6dof'):
        self.planner = MultiStagePlanner(robot_type=robot_type)
        self.results = {
            'grid': [],  # Store grid test results
            'height': []  # Store height test results
        }
        
        # Create results directory
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.results_dir = f"/home/ubuntu/new_ws/src/task_success_benchmark/results/workspace_{self.timestamp}"
        os.makedirs(self.results_dir, exist_ok=True)
        
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
        print("\nTesting positions... (this will take a few minutes)")
        
        # Test positions in a grid
        x_range = np.arange(0.1, 0.41, 0.05)  # 0.10 to 0.40
        y_range = np.arange(-0.25, 0.26, 0.05)  # -0.25 to 0.25
        z_fixed = 0.2  # Fixed height for testing
        
        print(f"\n📊 Testing grid at height z={z_fixed}m:")
        print("   y→ ", end="")
        for y in y_range:
            print(f"{y:6.2f}", end="")
        print("\n" + "-"*60)
        
        # Store grid results
        grid_results = []
        
        for x in x_range:
            row = []
            print(f"x={x:.2f}: ", end="")
            for y in y_range:
                reachable = self.test_pose(x, y, z_fixed)
                marker = "✓" if reachable else "·"
                print(f"   {marker}  ", end="")
                
                # Store result
                row.append({
                    'x': x,
                    'y': y,
                    'z': z_fixed,
                    'reachable': reachable
                })
            print()
            grid_results.extend(row)
        
        self.results['grid'] = grid_results
        
        # Save grid results to CSV
        grid_csv = os.path.join(self.results_dir, 'grid_results.csv')
        with open(grid_csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['x', 'y', 'z', 'reachable'])
            writer.writeheader()
            writer.writerows(grid_results)
        
        print(f"\n✅ Grid results saved to: {grid_csv}")
        
        # Test different heights along center line
        print("\n" + "="*70)
        print("📊 Testing heights along center line (x=0.2, y=0):")
        z_range = np.arange(0.1, 0.41, 0.03)  # 0.10 to 0.40
        print("\n   z (m)  | No Roll | Roll 45° | Roll 90°")
        print("   -------|---------|----------|---------")
        
        height_results = []
        
        for z in z_range:
            no_roll = self.test_pose(0.2, 0.0, z, 0.0)
            roll45 = self.test_pose(0.2, 0.0, z, 0.785)
            roll90 = self.test_pose(0.2, 0.0, z, 1.571)
            
            print(f"   {z:.2f}   |    {'✓' if no_roll else '·'}    |     {'✓' if roll45 else '·'}     |    {'✓' if roll90 else '·'}")
            
            height_results.append({
                'z': z,
                'no_roll': no_roll,
                'roll45': roll45,
                'roll90': roll90
            })
        
        self.results['height'] = height_results
        
        # Save height results to CSV
        height_csv = os.path.join(self.results_dir, 'height_results.csv')
        with open(height_csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['z', 'no_roll', 'roll45', 'roll90'])
            writer.writeheader()
            writer.writerows(height_results)
        
        print(f"\n✅ Height results saved to: {height_csv}")
        
        # Generate summary
        self.print_summary()
    
    def print_summary(self):
        """Print a summary of all results"""
        print("\n" + "="*70)
        print("📈 REACHABILITY SUMMARY")
        print("="*70)
        
        # Calculate statistics
        total_tests = len(self.results['grid'])
        reachable_tests = sum(1 for r in self.results['grid'] if r['reachable'])
        
        print(f"\n📊 Grid Test Statistics (z=0.2m):")
        print(f"   Total positions tested: {total_tests}")
        print(f"   Reachable positions: {reachable_tests}")
        print(f"   Success rate: {reachable_tests/total_tests*100:.1f}%")
        
        # Find reachable region bounds
        if reachable_tests > 0:
            reachable_x = [r['x'] for r in self.results['grid'] if r['reachable']]
            reachable_y = [r['y'] for r in self.results['grid'] if r['reachable']]
            
            print(f"\n📍 Reachable Region Bounds (z=0.2m):")
            print(f"   X range: {min(reachable_x):.2f} to {max(reachable_x):.2f} m")
            print(f"   Y range: {min(reachable_y):.2f} to {max(reachable_y):.2f} m")
        
        # Height test summary
        print(f"\n📊 Height Test Summary (x=0.2, y=0):")
        print(f"   {'Height':>6} | {'No Roll':>8} | {'Roll45°':>8} | {'Roll90°':>8}")
        print(f"   {'-'*6}-|-{'-'*8}-|-{'-'*8}-|-{'-'*8}")
        
        for r in self.results['height']:
            print(f"   {r['z']:5.2f}  |   {'Yes' if r['no_roll'] else 'No ':>5}   |   {'Yes' if r['roll45'] else 'No ':>5}   |   {'Yes' if r['roll90'] else 'No ':>5}")
        
        # Find best height for each task
        best_no_roll = max([r for r in self.results['height'] if r['no_roll']], key=lambda x: x['z'], default=None)
        best_roll45 = max([r for r in self.results['height'] if r['roll45']], key=lambda x: x['z'], default=None)
        best_roll90 = max([r for r in self.results['height'] if r['roll90']], key=lambda x: x['z'], default=None)
        
        print(f"\n🏆 Maximum Reachable Heights:")
        if best_no_roll:
            print(f"   No Roll:  z = {best_no_roll['z']:.2f} m")
        if best_roll45:
            print(f"   Roll 45°: z = {best_roll45['z']:.2f} m")
        if best_roll90:
            print(f"   Roll 90°: z = {best_roll90['z']:.2f} m")
        
        # Save summary to file
        summary_file = os.path.join(self.results_dir, 'summary.txt')
        with open(summary_file, 'w') as f:
            f.write("REACHABLE WORKSPACE SUMMARY\n")
            f.write("="*50 + "\n\n")
            f.write(f"Timestamp: {self.timestamp}\n")
            f.write(f"Robot Type: {self.planner.robot_type}\n\n")
            f.write(f"Grid Test (z=0.2m): {reachable_tests}/{total_tests} positions reachable\n")
            f.write(f"Success rate: {reachable_tests/total_tests*100:.1f}%\n\n")
            
            if reachable_tests > 0:
                f.write(f"X range: {min(reachable_x):.2f} to {max(reachable_x):.2f} m\n")
                f.write(f"Y range: {min(reachable_y):.2f} to {max(reachable_y):.2f} m\n\n")
            
            f.write("Height Test Results:\n")
            for r in self.results['height']:
                f.write(f"z={r['z']:.2f}: NoRoll={r['no_roll']}, Roll45={r['roll45']}, Roll90={r['roll90']}\n")
        
        print(f"\n✅ Full summary saved to: {summary_file}")
        print(f"\n📁 All results saved in: {self.results_dir}")

def main():
    rclpy.init()
    finder = WorkspaceFinder('6dof')
    try:
        finder.find_reachable_region()
    except KeyboardInterrupt:
        print("\n\n⚠️ Test interrupted by user")
        finder.print_summary()  # Print summary even if interrupted
    finally:
        finder.planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
