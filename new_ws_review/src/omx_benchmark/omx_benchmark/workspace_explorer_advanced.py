#!/usr/bin/env python3
"""
Advanced Workspace Explorer - Tests positions near walls
"""
import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import json

class AdvancedWorkspaceExplorer(Node):
    def __init__(self):
        super().__init__('advanced_workspace_explorer')
        
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
        
        # Neutral orientation
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        
        return pose
    
    def check_reachable(self, x, y, z):
        """Check if a position is reachable"""
        seed = [0.0, 0.0, 0.0, 0.0, 0.0]
        pose = self.create_pose(x, y, z)
        
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
            return True
        return False
    
    def explore_corridor(self):
        """Test positions across the corridor width"""
        self.get_logger().info('Exploring corridor positions...')
        
        # Fixed x and z (inside corridor)
        x = 0.20
        z = 0.15
        
        # Test across corridor width (walls at ±0.12)
        y_values = [-0.15, -0.12, -0.10, -0.08, -0.05, -0.02, 0.0, 
                    0.02, 0.05, 0.08, 0.10, 0.12, 0.15]
        
        self.get_logger().info(f'Testing positions at x={x}, z={z}')
        self.get_logger().info('Walls at y = ±0.12')
        self.get_logger().info('')
        
        results = []
        for y in y_values:
            reachable = self.check_reachable(x, y, z)
            status = "✓" if reachable else "✗"
            wall_status = "INSIDE" if abs(y) < 0.12 else "OUTSIDE"
            self.get_logger().info(f'y={y:+.3f} ({wall_status:7s}) → {status}')
            results.append({'y': y, 'reachable': reachable})
        
        # Summary
        self.get_logger().info('')
        self.get_logger().info('=== SUMMARY ===')
        reachable_inside = [r for r in results if abs(r['y']) < 0.12 and r['reachable']]
        self.get_logger().info(f'Reachable positions inside corridor: {len(reachable_inside)}')
        
        # Save results
        with open('/tmp/corridor_explorer.json', 'w') as f:
            json.dump(results, f, indent=2)

def main(args=None):
    rclpy.init(args=args)
    
    explorer = AdvancedWorkspaceExplorer()
    explorer.explore_corridor()
    
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
