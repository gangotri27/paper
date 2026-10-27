#!/usr/bin/env python3
"""
True 5-DOF Goal Generator - Accepts wall_y parameter
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import json
import numpy as np
import sys
import os

class True5DOFGoalGenerator(Node):
    def __init__(self):
        super().__init__('true_5dof_goal_generator')
        
        # Get wall_y from command line
        self.wall_y = 0.12  # default
        for i, arg in enumerate(sys.argv):
            if arg == '--wall_y' and i+1 < len(sys.argv):
                self.wall_y = float(sys.argv[i+1])
        
        self.output_file = f'/home/ubuntu/benchmark_test/goals_5dof_true_w{self.wall_y}.json'
        
        self.corridor_x = 0.30
        self.target_z = 0.15
        self.target_y = 0.0
        
        self.get_logger().info(f'🎯 Generating for width {2*self.wall_y:.2f}m (walls at ±{self.wall_y}m)')
        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.ik_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('✓ IK service connected')
    
    def create_pose(self, roll_deg):
        pose = Pose()
        pose.position.x = float(self.corridor_x)
        pose.position.y = float(self.target_y)
        pose.position.z = float(self.target_z)
        
        roll_rad = float(np.radians(roll_deg))
        pose.orientation.x = np.sin(roll_rad/2)
        pose.orientation.w = np.cos(roll_rad/2)
        
        return pose
    
    def solve_ik_with_multiple_seeds(self, pose):
        seeds = [
            [0.0, -0.646, 0.0, 0.0],
            [0.0, -0.3, 0.5, -0.2],
            [0.0, -0.5, 0.8, -0.4],
            [0.5, -0.4, 0.3, -0.1],
            [-0.5, -0.4, 0.3, -0.1],
        ]
        
        for seed_idx, seed in enumerate(seeds):
            request = GetPositionIK.Request()
            request.ik_request.group_name = 'arm'
            request.ik_request.pose_stamped.header = Header()
            request.ik_request.pose_stamped.header.frame_id = 'world'
            request.ik_request.pose_stamped.pose = pose
            request.ik_request.robot_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4']
            request.ik_request.robot_state.joint_state.position = seed
            request.ik_request.timeout.sec = 2
            
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            
            if future.result() is not None and future.result().error_code.val == 1:
                solution = list(future.result().solution.joint_state.position)
                return solution[:4]
        
        return None
    
    def generate_goals(self):
        goals = []
        poses_info = []
        roll_angles = [0, 15, 30, 45, 60, 75, 90]
        
        self.get_logger().info(f'\n📌 Testing at x={self.corridor_x}m, y={self.target_y}m, z={self.target_z}m')
        
        for roll in roll_angles:
            self.get_logger().info(f'\n  Roll {roll}°:')
            pose = self.create_pose(roll)
            solution = self.solve_ik_with_multiple_seeds(pose)
            
            if solution:
                goals.append(solution)
                poses_info.append({
                    'roll': roll,
                    'solution': [float(x) for x in solution]
                })
                self.get_logger().info(f'    ✓ Found: j2={solution[1]:.3f}, j3={solution[2]:.3f}, j4={solution[3]:.3f}')
            else:
                self.get_logger().info(f'    ✗ No solution')
        
        output_data = {
            'robot_type': '5dof',
            'wall_y': self.wall_y,
            'num_goals': len(goals),
            'goals': goals,
            'poses_info': poses_info
        }
        
        with open(self.output_file, 'w') as f:
            json.dump(output_data, f, indent=2)
        
        self.get_logger().info(f'\n✅ Saved {len(goals)} goals to {self.output_file}')
        return len(goals)

def main(args=None):
    rclpy.init(args=args)
    generator = True5DOFGoalGenerator()
    generator.generate_goals()
    generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()