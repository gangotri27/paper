#!/usr/bin/env python3
"""
Generate goals based on END-EFFECTOR POSES, not joint angles
This ensures fair comparison between 5-DOF and 6-DOF
"""
import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import json
import os

class PoseGoalGenerator(Node):
    def __init__(self):
        super().__init__('pose_goal_generator')
        
        self.declare_parameter('output_file_5dof', '/tmp/goals_5dof_pose.json')
        self.declare_parameter('output_file_6dof', '/tmp/goals_6dof_pose.json')

        self.declare_parameter('skip_5dof', False)
        self.declare_parameter('skip_6dof', False)

        self.skip_5dof = self.get_parameter('skip_5dof').value
        self.skip_6dof = self.get_parameter('skip_6dof').value
        
        self.output_5dof = os.path.expanduser(self.get_parameter('output_file_5dof').value)
        self.output_6dof = os.path.expanduser(self.get_parameter('output_file_6dof').value)
        
        self.get_logger().info('Pose Goal Generator initialized')
        
        # IK clients for both robots
        self.ik_client_5dof = self.create_client(GetPositionIK, '/compute_ik')
        self.ik_client_6dof = self.create_client(GetPositionIK, '/compute_ik')
        
        # Wait for services
        # Wait for services
        if not self.skip_6dof:
            self.ik_client_6dof.wait_for_service(timeout_sec=10.0)
            self.get_logger().info('✓ 6-DOF IK service connected')
            
        if not self.skip_5dof:
            self.ik_client_5dof.wait_for_service(timeout_sec=10.0)
            self.get_logger().info('✓ 5-DOF IK service connected')
    
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
    
    def solve_ik(self, pose, robot_type, seed):
        """Solve IK for given pose on specific robot"""
        client = self.ik_client_6dof if robot_type == '6dof' else self.ik_client_5dof
        
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.pose_stamped.header = Header()
        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.robot_state.joint_state.name = list(seed.keys())
        request.ik_request.robot_state.joint_state.position = list(seed.values())
        request.ik_request.timeout.sec = 1
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None and future.result().error_code.val == 1:
            return list(future.result().solution.joint_state.position)
        return None
    
    def generate_goals(self):
        """Generate goals based on end-effector poses"""
        goals_5dof = []
        goals_6dof = []
        poses_info = []
        
        # The position we know is reachable
        x = 0.20
        y = 0.00
        z = 0.15
        
        # Test different roll angles
        roll_angles = [0, 15, 30, 45, 60, 75, 90]
        
        # Seed states (your -37° start pose)
        seed_5dof = {'joint1':0.0, 'joint2':-0.646, 'joint3':0.0, 'joint4':0.0}
        seed_6dof = {'joint1':0.0, 'joint2':-0.646, 'joint3':0.0, 'joint4':0.0, 'joint5_roll':0.0}
        
        self.get_logger().info('\nGenerating pose-based goals...')
        self.get_logger().info(f'Position: ({x}, {y}, {z})')
        
        for roll in roll_angles:
            pose = self.create_pose(x, y, z, roll)
            
            # Solve for both robots
            sol_6dof = self.solve_ik(pose, '6dof', seed_6dof)
            sol_5dof = self.solve_ik(pose, '5dof', seed_5dof)
            
            pose_info = {
                'roll': roll,
                'position': [x, y, z],
                'pose': {
                    'position': [pose.position.x, pose.position.y, pose.position.z],
                    'orientation': [pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w]
                }
            }
            
            if sol_6dof:
                goals_6dof.append(sol_6dof)
                pose_info['6dof_solution'] = sol_6dof
                self.get_logger().info(f'Roll {roll}°: 6-DOF ✓')
            else:
                self.get_logger().info(f'Roll {roll}°: 6-DOF ✗')
            
            if sol_5dof:
                goals_5dof.append(sol_5dof)
                pose_info['5dof_solution'] = sol_5dof
                self.get_logger().info(f'         5-DOF ✓')
            else:
                self.get_logger().info(f'         5-DOF ✗')
            
            poses_info.append(pose_info)
        
        # Save 6-DOF goals
        with open(self.output_6dof, 'w') as f:
            json.dump({
                'num_goals': len(goals_6dof),
                'goals': goals_6dof,
                'poses': poses_info,
                'robot': '6dof'
            }, f, indent=2)
        
        # Save 5-DOF goals
        with open(self.output_5dof, 'w') as f:
            json.dump({
                'num_goals': len(goals_5dof),
                'goals': goals_5dof,
                'poses': poses_info,
                'robot': '5dof'
            }, f, indent=2)
        
        self.get_logger().info(f'\nSaved {len(goals_6dof)} 6-DOF goals')
        self.get_logger().info(f'Saved {len(goals_5dof)} 5-DOF goals')
        
        return len(goals_6dof), len(goals_5dof)

def main(args=None):
    rclpy.init(args=args)
    
    generator = PoseGoalGenerator()
    n6, n5 = generator.generate_goals()
    
    generator.destroy_node()
    rclpy.shutdown()
    
    print(f"\nSUMMARY:")
    print(f"  6-DOF: {n6} achievable poses")
    print(f"  5-DOF: {n5} achievable poses")

if __name__ == '__main__':
    main()
