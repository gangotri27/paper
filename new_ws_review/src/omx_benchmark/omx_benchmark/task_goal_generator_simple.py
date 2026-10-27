#!/usr/bin/env python3
"""
Simplified Task Goal Generator with Multiple Seeds
"""
import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import json
import os
import random

class SimpleTaskGoalGenerator(Node):
    def __init__(self):
        super().__init__('simple_task_goal_generator')
        
        self.declare_parameter('num_goals', 5)
        self.declare_parameter('output_file', '/tmp/goals.json')
        self.declare_parameter('seed', 42)
        
        self.num_goals = self.get_parameter('num_goals').value
        self.output_file = os.path.expanduser(self.get_parameter('output_file').value)
        self.seed = self.get_parameter('seed').value
        
        np.random.seed(self.seed)
        random.seed(self.seed)
        
        self.get_logger().info(f'Simple Task Goal Generator initialized')
        self.get_logger().info(f'  Output: {self.output_file}')
        
        # IK client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('IK service not available')
        else:
            self.get_logger().info('✓ IK service connected')
    
    def create_pose(self, x, y, z, roll_deg):
        """Create a pose with given position and roll"""
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
    
    def create_seed_state(self, seed_type):
        """Create different seed states to try"""
        if seed_type == "neutral":
            return [0.0, 0.0, 0.0, 0.0, 0.0]  # All zeros
        elif seed_type == "raised":
            return [0.0, 0.5, -0.3, 0.0, 0.0]  # Arm raised
        elif seed_type == "reached_out":
            return [0.0, 0.3, 0.5, -0.2, 0.0]  # Arm reaching forward
        elif seed_type == "twisted":
            return [0.0, 0.2, 0.3, -0.1, 1.57]  # Wrist twisted
        else:  # random
            return [
                random.uniform(-1.0, 1.0),
                random.uniform(-0.5, 1.0),
                random.uniform(-0.5, 1.0),
                random.uniform(-0.5, 0.5),
                random.uniform(-1.57, 1.57)
            ]
    
    def solve_ik(self, pose, seed_state):
        """Solve IK for the pose with given seed"""
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        
        # Set up header
        request.ik_request.pose_stamped.header = Header()
        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.pose = pose
        
        # Set seed state
        request.ik_request.robot_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5_roll']
        request.ik_request.robot_state.joint_state.position = seed_state
        
        # Set timeout
        request.ik_request.timeout.sec = 1
        
        # Send request
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None and future.result().error_code.val == 1:
            return list(future.result().solution.joint_state.position)
        return None
    
    def generate_goals(self):
        """Generate goals at different roll angles using multiple seeds"""
        goals = []
        poses_saved = []
        
        # Fixed position inside corridor
        x = 0.3
        y = 0.0
        z = 0.15
        
        # Test different roll angles
        roll_angles = [0, 30, 45, 60, 90]
        
        # Different seed types to try
        seed_types = ["neutral", "raised", "reached_out", "twisted", "random"]
        
        self.get_logger().info('Generating goals by trying multiple seeds...')
        
        for roll in roll_angles:
            self.get_logger().info(f'\nTesting roll {roll}°...')
            pose = self.create_pose(x, y, z, roll)
            
            found_solution = False
            for seed_type in seed_types:
                seed_state = self.create_seed_state(seed_type)
                self.get_logger().info(f'  Trying seed: {seed_type}')
                
                solution = self.solve_ik(pose, seed_state)
                
                if solution is not None:
                    goals.append(solution)
                    poses_saved.append({
                        'roll': roll,
                        'position': [float(x), float(y), float(z)],
                        'seed_type': seed_type,
                        'solution': solution
                    })
                    self.get_logger().info(f'  ✓ FOUND solution with seed {seed_type}!')
                    self.get_logger().info(f'    Joints: j1={solution[0]:.2f}, j2={solution[1]:.2f}, j3={solution[2]:.2f}, j4={solution[3]:.2f}, j5={solution[4]:.2f}')
                    found_solution = True
                    break
                else:
                    self.get_logger().info(f'    ✗ No solution')
            
            if not found_solution:
                self.get_logger().info(f'  ✗ No solution found with any seed')
            
            if len(goals) >= self.num_goals:
                break
        
        # Save to file
        os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
        
        with open(self.output_file, 'w') as f:
            json.dump({
                'num_goals': len(goals),
                'goals': goals,
                'poses': poses_saved,
                'seed': self.seed
            }, f, indent=2)
        
        self.get_logger().info(f'\nSaved {len(goals)} goals to {self.output_file}')
        
        # Summary
        self.get_logger().info('\n=== SUMMARY ===')
        for pose in poses_saved:
            self.get_logger().info(f'Roll {pose["roll"]}°: Found with {pose["seed_type"]} seed')
        
        return len(goals)

def main(args=None):
    rclpy.init(args=args)
    
    generator = SimpleTaskGoalGenerator()
    generator.generate_goals()
    
    generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()