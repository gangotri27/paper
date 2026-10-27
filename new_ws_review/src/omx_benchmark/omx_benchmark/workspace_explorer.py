#!/usr/bin/env python3
"""
Workspace Explorer - Find what positions are actually reachable
"""
import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import json

class WorkspaceExplorer(Node):
    def __init__(self):
        super().__init__('workspace_explorer')
        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('IK service not available')
        else:
            self.get_logger().info('✓ IK service connected')
    
    def create_pose(self, x, y, z, roll_deg=0):
        """Create a pose"""
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        
        # Neutral orientation (no roll)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        
        return pose
    
    def check_reachable(self, x, y, z, seed_state=None):
        """Check if a position is reachable"""
        if seed_state is None:
            seed_state = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        pose = self.create_pose(x, y, z)
        
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.pose_stamped.header = Header()
        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.robot_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5_roll']
        request.ik_request.robot_state.joint_state.position = seed_state
        request.ik_request.timeout.sec = 1
        
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None and future.result().error_code.val == 1:
            return True, list(future.result().solution.joint_state.position)
        return False, None
    
    def explore_workspace(self):
        """Test a grid of positions"""
        self.get_logger().info('Exploring workspace...')
        self.get_logger().info('Format: x, y, z → Reachable?')
        
        # Test different x positions
        x_values = [0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4]
        y_values = [-0.2, -0.1, 0.0, 0.1, 0.2]
        z_values = [0.05, 0.1, 0.15, 0.2, 0.25, 0.3]
        
        reachable_points = []
        
        for x in x_values:
            for y in y_values:
                for z in z_values:
                    reachable, solution = self.check_reachable(x, y, z)
                    status = "✓" if reachable else "✗"
                    self.get_logger().info(f'({x:.2f}, {y:.2f}, {z:.2f}) → {status}')
                    
                    if reachable:
                        reachable_points.append({
                            'x': x, 'y': y, 'z': z,
                            'solution': solution
                        })
        
        self.get_logger().info(f'\nFound {len(reachable_points)} reachable points')
        
        # Save results
        with open('/tmp/reachable_points.json', 'w') as f:
            json.dump(reachable_points, f, indent=2)
        
        return reachable_points

def main(args=None):
    rclpy.init(args=args)
    
    explorer = WorkspaceExplorer()
    explorer.explore_workspace()
    
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()