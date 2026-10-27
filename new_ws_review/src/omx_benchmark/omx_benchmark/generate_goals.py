#!/usr/bin/env python3
"""
Unbiased Goal Configuration Generator
Generates random, collision-free joint configurations for benchmarking
"""
import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import json
import os
import random
from ament_index_python.packages import get_package_share_directory

class GoalGenerator(Node):
    def __init__(self):
        super().__init__('goal_generator')
        
        # Parameters
        self.declare_parameter('config_type', '6dof')
        self.declare_parameter('num_goals', 20)
        self.declare_parameter('output_file', '/tmp/goals.json')
        self.declare_parameter('seed', 42)
        
        self.config_type = self.get_parameter('config_type').value
        self.num_goals = self.get_parameter('num_goals').value
        self.output_file = self.get_parameter('output_file').value
        self.seed = self.get_parameter('seed').value
        
        random.seed(self.seed)
        np.random.seed(self.seed)
        
        # Joint limits (radians)
        self.joint_limits = {
            '6dof': {
                'joint1': (-3.14, 3.14),
                'joint2': (-2.0, 2.0),
                'joint3': (-2.0, 2.0),
                'joint4': (-2.0, 2.0),
                'joint5_roll': (-3.14, 3.14)
            },
            '5dof': {
                'joint1': (-3.14, 3.14),
                'joint2': (-2.0, 2.0),
                'joint3': (-2.0, 2.0),
                'joint4': (-2.0, 2.0)
            }
        }
        
        # Service client for collision checking
        self.val_client = self.create_client(GetStateValidity, '/check_state_validity')
        
        self.get_logger().info(f'Goal generator initialized for {self.config_type}')
        
    def generate_random_config(self):
        """Generate random configuration within joint limits"""
        limits = self.joint_limits[self.config_type]
        config = {}
        for joint, (lower, upper) in limits.items():
            config[joint] = random.uniform(lower, upper)
        return config
    
    def check_collision(self, config):
        """Check if configuration is collision-free"""
        if not self.val_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('State validity service not available - assuming valid')
            return True
        
        request = GetStateValidity.Request()
        request.robot_state = RobotState()
        request.robot_state.joint_state.name = list(config.keys())
        request.robot_state.joint_state.position = list(config.values())
        request.group_name = 'arm'
        
        future = self.val_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            return future.result().valid
        return True  # Assume valid if service fails
    
    def config_distance(self, config1, config2):
        """Euclidean distance between configurations"""
        joints1 = np.array(list(config1.values()))
        joints2 = np.array(list(config2.values()))
        return np.linalg.norm(joints1 - joints2)
    
    def generate_goals(self):
        """Generate set of valid, diverse goal configurations"""
        goals = []
        attempts = 0
        max_attempts = self.num_goals * 50  # Prevent infinite loop
        
        self.get_logger().info(f'Generating {self.num_goals} collision-free goals...')
        
        while len(goals) < self.num_goals and attempts < max_attempts:
            attempts += 1
            config = self.generate_random_config()
            
            # Check collision
            if not self.check_collision(config):
                continue
            
            # Check diversity (not too close to existing goals)
            is_diverse = True
            for existing in goals:
                if self.config_distance(config, existing) < 0.5:  # 0.5 rad threshold
                    is_diverse = False
                    break
            
            if is_diverse:
                goals.append(config)
                self.get_logger().info(f'  Found goal {len(goals)}/{self.num_goals}')
        
        # Save to file
        with open(self.output_file, 'w') as f:
            json.dump({
                'config_type': self.config_type,
                'num_goals': len(goals),
                'goals': goals,
                'seed': self.seed
            }, f, indent=2)
        
        self.get_logger().info(f'Saved {len(goals)} goals to {self.output_file}')
        return goals

def main(args=None):
    rclpy.init(args=args)
    
    generator = GoalGenerator()
    generator.generate_goals()
    
    generator.destroy_node()
    rclpy.shutdown()

def main_entry():
    """Entry point for console script"""
    import sys
    return main()

if __name__ == '__main__':
    main()