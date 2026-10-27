#!/usr/bin/env python3
"""
Test 5-DOF IK feasibility in FREE SPACE (no obstacles)
This proves whether failure is kinematic or environmental
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import numpy as np
import sys
import time

class FiveDofIKTester(Node):
    def __init__(self):
        super().__init__('five_dof_ik_tester')
        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('IK service not available!')
            self.get_logger().error('Make sure 5-DOF robot is running:')
            self.get_logger().error('  source ~/omx_ws/install/setup.bash')
            self.get_logger().error('  ros2 launch ~/omx_ws/src/omx_5dof_moveit/launch/demo.launch.py use_rviz:=false')
            sys.exit(1)
        
        # Test poses (same as your goals)
        self.test_rolls = [0, 15, 30, 45, 60, 75, 90]
        self.x = 0.30
        self.y = 0.0
        self.z = 0.15
        
        self.get_logger().info('✓ IK tester initialized')
        
    def test_ik(self, roll_deg):
        """Try IK with multiple seeds (since 'attempts' parameter doesn't exist in ROS2)"""
        
        # Multiple seed states to try
        seeds = [
            [0.0, -0.646, 0.0, 0.0],  # Your start state
            [0.0, -0.5, 0.2, -0.1],
            [0.0, -0.3, 0.4, -0.2],
            [0.5, -0.4, 0.1, -0.1],
            [-0.5, -0.4, 0.1, -0.1],
            [0.0, -0.2, 0.5, -0.3],
            [0.3, -0.5, 0.3, -0.2],
        ]
        
        for seed_idx, seed in enumerate(seeds):
            pose = Pose()
            pose.position.x = self.x
            pose.position.y = self.y
            pose.position.z = self.z
            
            roll_rad = np.radians(roll_deg)
            pose.orientation.x = np.sin(roll_rad/2)
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = np.cos(roll_rad/2)
            
            request = GetPositionIK.Request()
            request.ik_request.group_name = 'arm'
            request.ik_request.pose_stamped.header = Header()
            request.ik_request.pose_stamped.header.frame_id = 'world'
            request.ik_request.pose_stamped.pose = pose
            request.ik_request.robot_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4']
            request.ik_request.robot_state.joint_state.position = seed
            request.ik_request.timeout.sec = 1  # 1 second timeout
            
            # Note: 'attempts' parameter does NOT exist in ROS2 - removed!
            
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() and future.result().error_code.val == 1:
                solution = list(future.result().solution.joint_state.position)
                return True, solution[:4], seed_idx
        
        return False, None, -1
    
    def run_tests(self):
        print("\n" + "="*70)
        print("5-DOF IK FEASIBILITY TEST (FREE SPACE)")
        print("="*70)
        print(f"Target position: ({self.x}, {self.y}, {self.z})")
        print(f"Testing {len(self.test_rolls)} orientations with {7} seeds each")
        print("-"*70)
        
        results = {}
        
        for roll in self.test_rolls:
            success, solution, seed_idx = self.test_ik(roll)
            results[roll] = success
            
            if success:
                print(f"✓ Roll {roll:2d}°: IK SUCCESSFUL (seed {seed_idx})")
                print(f"  Solution: j1={solution[0]:6.3f}, j2={solution[1]:6.3f}, j3={solution[2]:6.3f}, j4={solution[3]:6.3f}")
            else:
                print(f"✗ Roll {roll:2d}°: IK FAILED (all seeds)")
        
        print("="*70)
        print("\n📊 SUMMARY:")
        successful = sum(1 for r in results.values() if r)
        print(f"  Successful IK solutions: {successful}/{len(self.test_rolls)}")
        
        if successful == 0:
            print("\n✅ This confirms: The 5-DOF robot CANNOT achieve these poses")
            print("   regardless of environmental constraints.")
            print("   This is a KINEMATIC limitation, not a planning issue.")
        else:
            print("\n⚠ Some poses are kinematically feasible in free space!")
            print("   The 0% success rate in planning is due to environment constraints.")
        
        return results

def main():
    rclpy.init()
    tester = FiveDofIKTester()
    results = tester.run_tests()
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()