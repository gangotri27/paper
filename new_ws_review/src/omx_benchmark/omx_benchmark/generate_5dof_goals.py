#!/usr/bin/env python3
"""
Generate TRUE 5-DOF goals using 5-DOF robot's IK service
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

class FiveDofGoalGenerator(Node):
    def __init__(self):
        super().__init__('five_dof_goal_generator')
        
        # DECLARE PARAMETER FIRST!
        self.declare_parameter('wall_y', 0.12)
        
        # NOW get the parameter value
        self.wall_y = self.get_parameter('wall_y').value
        
        self.output_file = f'/home/ubuntu/benchmark_test/goals_5dof_w{self.wall_y}.json'
        self.corridor_x = 0.30
        self.target_z = 0.15
        self.target_y = 0.0
        
        self.get_logger().info(f'🎯 Generating 5-DOF goals for width {2*self.wall_y:.2f}m (walls at ±{self.wall_y}m)')
        self.get_logger().info(f'Output file: {self.output_file}')
        
        # IMPORTANT: This uses the 5-DOF robot's IK service!
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('❌ 5-DOF IK service not available!')
            self.get_logger().error('Make sure 5-DOF robot is running:')
            self.get_logger().error('  source ~/omx_ws/install/setup.bash')
            self.get_logger().error('  ros2 launch ~/omx_ws/src/omx_5dof_moveit/launch/demo.launch.py use_rviz:=false')
            sys.exit(1)
        self.get_logger().info('✓ Connected to 5-DOF IK service')
    
    def create_pose(self, roll_deg):
        pose = Pose()
        pose.position.x = float(self.corridor_x)
        pose.position.y = float(self.target_y)
        pose.position.z = float(self.target_z)
        
        roll_rad = float(np.radians(roll_deg))
        pose.orientation.x = np.sin(roll_rad/2)
        pose.orientation.w = np.cos(roll_rad/2)
        return pose
    
    def solve_ik(self, pose):
        # Try multiple seeds for 5-DOF
        seeds = [
            [0.0, -0.646, 0.0, 0.0],  # Your start state
            [0.0, -0.5, 0.3, -0.1],
            [0.0, -0.3, 0.5, -0.2],
            [0.5, -0.4, 0.2, -0.1],
            [-0.5, -0.4, 0.2, -0.1],
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
            
            if future.result() and future.result().error_code.val == 1:
                solution = list(future.result().solution.joint_state.position)
                self.get_logger().info(f'    Found with seed {seed_idx}')
                return solution[:4]  # Return only 4 joints
        
        return None
    
    def generate(self):
        goals = []
        poses_info = []
        roll_angles = [0, 15, 30, 45, 60, 75, 90]
        
        self.get_logger().info(f'Testing {len(roll_angles)} orientations...')
        
        for roll in roll_angles:
            pose = self.create_pose(roll)
            self.get_logger().info(f'\n📌 Trying roll {roll}°...')
            solution = self.solve_ik(pose)
            
            if solution:
                goals.append(solution)
                poses_info.append({
                    'roll': roll,
                    'j1': float(solution[0]),
                    'j2': float(solution[1]),
                    'j3': float(solution[2]),
                    'j4': float(solution[3])
                })
                self.get_logger().info(f'  ✓ FOUND: j2={solution[1]:.3f}, j3={solution[2]:.3f}, j4={solution[3]:.3f}')
            else:
                self.get_logger().info(f'  ✗ No solution')
        
        # Save results
        output = {
            'robot_type': '5dof',
            'wall_y': self.wall_y,
            'num_goals': len(goals),
            'goals': goals,
            'poses_info': poses_info,
            'note': 'Generated using 5-DOF robot IK service'
        }
        
        # Ensure directory exists
        os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
        
        with open(self.output_file, 'w') as f:
            json.dump(output, f, indent=2)
        
        self.get_logger().info(f'\n✅ Saved {len(goals)} goals to {self.output_file}')
        return len(goals)

def main(args=None):
    rclpy.init(args=args)
    generator = FiveDofGoalGenerator()
    generator.generate()
    generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()