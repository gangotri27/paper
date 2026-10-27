#!/usr/bin/env python3
"""
Final Goal Generator - Tests roll angles at the only reachable position (y=0.0)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import json
import os

class FinalGoalGenerator(Node):
    def __init__(self):
        super().__init__('final_goal_generator')
        
        self.declare_parameter('output_file', '/tmp/final_goals.json')
        self.output_file = os.path.expanduser(self.get_parameter('output_file').value)
        
        self.get_logger().info('Final Goal Generator initialized')
        self.get_logger().info(f'  Output: {self.output_file}')
        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('IK service not available')
        else:
            self.get_logger().info('✓ IK service connected')
    
    def create_pose(self, x, y, z, roll_deg):
        """Create a pose with given roll"""
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        
        # Convert roll to quaternion
        roll_rad = float(np.radians(roll_deg))
        qx = np.sin(roll_rad/2)
        qw = np.cos(roll_rad/2)
        
        pose.orientation.x = float(qx)
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = float(qw)
        
        return pose
    
    def solve_ik(self, pose, seed):
        """Solve IK for the pose"""
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.pose_stamped.header = Header()
        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.robot_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5_roll']
        request.ik_request.robot_state.joint_state.position = seed
        request.ik_request.timeout.sec = 1
        
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None and future.result().error_code.val == 1:
            return list(future.result().solution.joint_state.position)
        return None
    
    def generate_goals(self):
        """Generate goals at the only reachable position with various rolls"""
        goals = []
        poses_saved = []
        
        # The ONLY reachable position from your data
        x = 0.20
        y = 0.00
        z = 0.15
        
        # Test increasing roll angles
        roll_angles = [0, 15, 30, 45, 60, 75, 90]
        
        # Try different seeds to find solutions
        seeds = [
            [0.0, 0.0, 0.0, 0.0, 0.0],           # neutral
            [0.0, 0.3, 0.2, -0.1, 0.0],          # raised
            [0.0, 0.2, 0.3, -0.1, 1.57],          # pre-rolled
        ]
        
        self.get_logger().info(f'\nTesting roll angles at ({x}, {y}, {z})...')
        
        for roll in roll_angles:
            self.get_logger().info(f'\nRoll {roll}°:')
            pose = self.create_pose(x, y, z, roll)
            
            found = False
            for seed_idx, seed in enumerate(seeds):
                solution = self.solve_ik(pose, seed)
                if solution:
                    goals.append(solution)
                    poses_saved.append({
                        'roll': roll,
                        'position': [x, y, z],
                        'solution': solution
                    })
                    self.get_logger().info(f'  ✓ Found with seed {seed_idx}')
                    self.get_logger().info(f'    j5 = {solution[4]:.2f} rad ({np.degrees(solution[4]):.1f}°)')
                    found = True
                    break
            
            if not found:
                self.get_logger().info(f'  ✗ No solution found')
        
        # Save to file
        os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
        
        with open(self.output_file, 'w') as f:
            json.dump({
                'num_goals': len(goals),
                'goals': goals,
                'poses': poses_saved
            }, f, indent=2)
        
        self.get_logger().info(f'\nSaved {len(goals)} goals to {self.output_file}')
        
        # Summary
        self.get_logger().info('\n=== SUMMARY ===')
        for pose in poses_saved:
            self.get_logger().info(f'Roll {pose["roll"]}°: ✓ (j5={pose["solution"][4]:.2f})')
        
        return len(goals)

def main(args=None):
    rclpy.init(args=args)
    
    generator = FinalGoalGenerator()
    generator.generate_goals()
    
    generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
