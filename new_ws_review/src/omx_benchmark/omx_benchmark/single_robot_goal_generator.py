#!/usr/bin/env python3
"""
Generate goals for a single robot type
"""
import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import json
import os
import sys

class SingleRobotGoalGenerator(Node):
    def __init__(self):
        super().__init__('single_robot_goal_generator')
        
        self.declare_parameter('robot_type', '6dof')
        self.declare_parameter('output_file', '/tmp/goals.json')
        
        self.robot_type = self.get_parameter('robot_type').value
        self.output_file = os.path.expanduser(self.get_parameter('output_file').value)
        
        self.get_logger().info(f'Generating goals for {self.robot_type}')
        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.ik_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('✓ IK service connected')
    
    def create_pose(self, x, y, z, roll_deg):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        
        roll_rad = float(np.radians(roll_deg))
        qx = np.sin(roll_rad/2)
        qw = np.cos(roll_rad/2)
        
        pose.orientation.x = float(qx)
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = float(qw)
        
        return pose
    
    def solve_ik(self, pose):
        if self.robot_type == '6dof':
            seed = {'joint1':0.0, 'joint2':-0.646, 'joint3':0.0, 'joint4':0.0, 'joint5_roll':0.0}
        else:
            seed = {'joint1':0.0, 'joint2':-0.646, 'joint3':0.0, 'joint4':0.0}
        
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.pose_stamped.header = Header()
        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.robot_state.joint_state.name = list(seed.keys())
        request.ik_request.robot_state.joint_state.position = list(seed.values())
        request.ik_request.timeout.sec = 1
        
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None and future.result().error_code.val == 1:
            return list(future.result().solution.joint_state.position)
        return None
    
    def generate_goals(self):
        goals = []
        poses_info = []
        
        x, y, z = 0.20, 0.00, 0.15
        roll_angles = [0, 15, 30, 45, 60, 75, 90]
        
        for roll in roll_angles:
            pose = self.create_pose(x, y, z, roll)
            solution = self.solve_ik(pose)
            
            if solution:
                goals.append(solution)
                poses_info.append({'roll': roll, 'solution': solution})
                self.get_logger().info(f'Roll {roll}°: ✓')
            else:
                self.get_logger().info(f'Roll {roll}°: ✗')
        
        with open(self.output_file, 'w') as f:
            json.dump({
                'robot_type': self.robot_type,
                'num_goals': len(goals),
                'goals': goals,
                'poses': poses_info
            }, f, indent=2)
        
        self.get_logger().info(f'Saved {len(goals)} goals to {self.output_file}')
        return len(goals)

def main(args=None):
    rclpy.init(args=args)
    
    generator = SingleRobotGoalGenerator()
    n = generator.generate_goals()
    
    generator.destroy_node()
    rclpy.shutdown()
    
    print(f"\nGenerated {n} goals for {generator.robot_type}")

if __name__ == '__main__':
    main()
