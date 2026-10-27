#!/usr/bin/env python3
"""
Task Goal Generator for Constrained Corridor
Generates end-effector poses that REQUIRE wrist roll to achieve
"""
import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R
import json
import os
import time

class TaskGoalGenerator(Node):
    def __init__(self):
        super().__init__('task_goal_generator')
        
        # Parameters
        self.declare_parameter('num_goals', 20)
        self.declare_parameter('output_file_5dof', '/tmp/goals_5dof.json')
        self.declare_parameter('output_file_6dof', '/tmp/goals_6dof.json')
        self.declare_parameter('seed', 42)
        
        self.num_goals = self.get_parameter('num_goals').value
        self.output_5dof = os.path.expanduser(self.get_parameter('output_file_5dof').value)
        self.output_6dof = os.path.expanduser(self.get_parameter('output_file_6dof').value)
        self.seed = self.get_parameter('seed').value
        
        np.random.seed(self.seed)
        
        self.get_logger().info('Task Goal Generator initialized')
        self.get_logger().info(f'  Output 5-DOF: {self.output_5dof}')
        self.get_logger().info(f'  Output 6-DOF: {self.output_6dof}')
        
        # Create IK service clients
        self.ik_client_5dof = self.create_client(GetPositionIK, '/compute_ik')
        self.ik_client_6dof = self.create_client(GetPositionIK, '/compute_ik')
        
        # Wait for services (with timeout)
        self.get_logger().info('Waiting for IK services...')
        
        if not self.ik_client_5dof.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('5-DOF IK service not available')
        else:
            self.get_logger().info('✓ 5-DOF IK service connected')
            
        if not self.ik_client_6dof.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('6-DOF IK service not available')
        else:
            self.get_logger().info('✓ 6-DOF IK service connected')
    
    def create_corridor_pose(self, y_offset=0.0, roll_angle=90):
        """
        Create a pose inside the corridor
        Args:
            y_offset: slight left/right offset (-0.1 to 0.1)
            roll_angle: how much wrist roll required (degrees)
        """
        pose = Pose()
        
        # Position: inside corridor (based on your scene)
        pose.position.x = 0.3
        pose.position.y = y_offset
        pose.position.z = 0.15  # Below top barrier
        
        # Orientation: requires roll about x-axis
        # Convert degrees to radians
        roll_rad = np.radians(roll_angle)
        quat = R.from_euler('xyz', [roll_rad, 0, 0]).as_quat()
        
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        return pose
    
    def solve_ik_5dof(self, pose, seed_state):
        """Solve IK for 5-DOF robot"""
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.robot_state.joint_state.name = list(seed_state.keys())
        request.ik_request.robot_state.joint_state.position = list(seed_state.values())
        request.ik_request.timeout.sec = 1
        # Note: 'attempts' parameter is not available in ROS2 - removed
        
        future = self.ik_client_5dof.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None and future.result().error_code.val == 1:
            return list(future.result().solution.joint_state.position)
        return None
    
    def solve_ik_6dof(self, pose, seed_state):
        """Solve IK for 6-DOF robot"""
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.robot_state.joint_state.name = list(seed_state.keys())
        request.ik_request.robot_state.joint_state.position = list(seed_state.values())
        request.ik_request.timeout.sec = 1
        # Note: 'attempts' parameter is not available in ROS2 - removed
        
        future = self.ik_client_6dof.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None and future.result().error_code.val == 1:
            return list(future.result().solution.joint_state.position)
        return None
    
    def generate_goals(self):
        """Generate goals that REQUIRE wrist roll"""
        goals_5dof = []
        goals_6dof = []
        poses_metadata = []
        
        # Seed state (your -37° start pose - NEUTRAL wrist for 6-DOF)
        seed_5dof = {
            'joint1': 0.0,
            'joint2': -0.6458,  # -37 degrees in radians
            'joint3': 0.0,
            'joint4': 0.0
        }
        
        seed_6dof = {
            'joint1': 0.0,
            'joint2': -0.6458,  # -37 degrees
            'joint3': 0.0,
            'joint4': 0.0,
            'joint5_roll': 0.0   # NEUTRAL - no unfair advantage
        }
        
        # Generate poses with varying difficulty
        # Roll angles from 0° to 90° in steps of 15°
        roll_angles = [0, 15, 30, 45, 60, 75, 90]
        y_offsets = [-0.1, -0.05, 0, 0.05, 0.1]
        
        self.get_logger().info('Generating task-space goals...')
        self.get_logger().info(f'  Roll angles: {roll_angles}')
        self.get_logger().info(f'  Y offsets: {y_offsets}')
        self.get_logger().info('')
        
        total_combinations = len(roll_angles) * len(y_offsets)
        self.get_logger().info(f'Testing {total_combinations} pose combinations')
        
        for roll in roll_angles:
            for y in y_offsets:
                pose = self.create_corridor_pose(y_offset=y, roll_angle=roll)
                
                # Log the pose being tested
                self.get_logger().info(f'Testing pose: y={y:.2f}, roll={roll}°')
                
                # Solve IK for both robots
                sol_5dof = self.solve_ik_5dof(pose, seed_5dof)
                sol_6dof = self.solve_ik_6dof(pose, seed_6dof)
                
                if sol_5dof is not None and sol_6dof is not None:
                    # Both robots have solutions - keep this goal
                    goals_5dof.append(sol_5dof)
                    goals_6dof.append(sol_6dof)
                    
                    poses_metadata.append({
                        'y_offset': float(y),
                        'roll_angle': float(roll),
                        'pose': {
                            'position': [pose.position.x, pose.position.y, pose.position.z],
                            'orientation': [pose.orientation.x, pose.orientation.y,
                                          pose.orientation.z, pose.orientation.w]
                        }
                    })
                    
                    self.get_logger().info(
                        f'  ✓ FOUND - 5DOF: j1={sol_5dof[0]:.2f}, j2={sol_5dof[1]:.2f}, j3={sol_5dof[2]:.2f}, j4={sol_5dof[3]:.2f} | '
                        f'6DOF: j1={sol_6dof[0]:.2f}, j2={sol_6dof[1]:.2f}, j3={sol_6dof[2]:.2f}, j4={sol_6dof[3]:.2f}, j5={sol_6dof[4]:.2f}'
                    )
                else:
                    self.get_logger().info(f'  ✗ No solution for both robots')
                
                if len(goals_5dof) >= self.num_goals:
                    break
            if len(goals_5dof) >= self.num_goals:
                break
        
        # Create output directories if needed
        os.makedirs(os.path.dirname(os.path.abspath(self.output_5dof)), exist_ok=True)
        os.makedirs(os.path.dirname(os.path.abspath(self.output_6dof)), exist_ok=True)
        
        # Save 5-DOF goals
        with open(self.output_5dof, 'w') as f:
            json.dump({
                'config_type': '5dof',
                'num_goals': len(goals_5dof),
                'goals': goals_5dof,
                'poses_metadata': poses_metadata,
                'seed': self.seed
            }, f, indent=2)
        
        # Save 6-DOF goals
        with open(self.output_6dof, 'w') as f:
            json.dump({
                'config_type': '6dof',
                'num_goals': len(goals_6dof),
                'goals': goals_6dof,
                'poses_metadata': poses_metadata,
                'seed': self.seed
            }, f, indent=2)
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('GOAL GENERATION COMPLETE')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Generated {len(goals_5dof)} paired goals')
        self.get_logger().info(f'Saved 5-DOF goals to: {self.output_5dof}')
        self.get_logger().info(f'Saved 6-DOF goals to: {self.output_6dof}')
        self.get_logger().info('')
        
        # Summary by roll angle
        self.get_logger().info('Summary by roll angle:')
        for roll in roll_angles:
            count = sum(1 for p in poses_metadata if p['roll_angle'] == roll)
            self.get_logger().info(f'  Roll {roll}°: {count} goals')
        
        return len(goals_5dof)

def main(args=None):
    rclpy.init(args=args)
    
    generator = TaskGoalGenerator()
    success = generator.generate_goals()
    
    generator.destroy_node()
    rclpy.shutdown()
    
    if success > 0:
        exit(0)
    else:
        exit(1)

if __name__ == '__main__':
    main()