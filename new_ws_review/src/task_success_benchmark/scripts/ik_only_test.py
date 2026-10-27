#!/usr/bin/env python3
"""
Test IK solutions without planning
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, Constraints
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np

class IKTest(Node):
    def __init__(self):
        super().__init__('ik_test')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')
        
        self.get_logger().info('✓ Connected to IK service')
    
    def quaternion_from_euler(self, roll, pitch=0.0, yaw=0.0):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q
    
    def check_ik(self, x, y, z, roll=0.0):
        request = GetPositionIK.Request()
        
        # Create IK request
        ik_request = PositionIKRequest()
        ik_request.group_name = 'arm'
        ik_request.pose_stamped.header.frame_id = 'world'
        ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik_request.pose_stamped.pose.position = Point(x=x, y=y, z=z)
        ik_request.pose_stamped.pose.orientation = self.quaternion_from_euler(roll)
        
        # Set timeout (attempts not available in this version)
        ik_request.timeout.sec = 1
        
        # Disable collision checking
        ik_request.avoid_collisions = False
        
        # Add empty constraints
        ik_request.constraints = Constraints()
        
        request.ik_request = ik_request
        
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            response = future.result()
            if response.error_code.val == 1:  # SUCCESS
                return True, response.solution.joint_state.position
        
        return False, None
    
    def test_poses(self):
        print("\n" + "="*70)
        print("IK ONLY TEST (No Planning, No Collision Checking)")
        print("="*70)
        
        test_poses = [
            ("Very close", 0.10, 0.0, 0.10, 0.0),
            ("Simple forward", 0.15, 0.0, 0.15, 0.0),
            ("Pick pose", 0.20, -0.10, 0.15, 0.0),
            ("Place pose", 0.20, 0.10, 0.15, 0.0),
            ("Center low", 0.20, 0.0, 0.15, 0.0),
            ("Center mid", 0.20, 0.0, 0.20, 0.0),
            ("Center high", 0.20, 0.0, 0.25, 0.0),
            ("Roll 11.5°", 0.20, 0.0, 0.20, 0.2),
            ("Roll 45°", 0.20, 0.0, 0.20, 0.785),
            ("Roll 90°", 0.20, 0.0, 0.20, 1.571),
        ]
        
        for name, x, y, z, roll in test_poses:
            print(f"\n{name}:")
            print(f"  Pose: x={x:.2f}, y={y:.2f}, z={z:.2f}, roll={roll:.2f}rad ({np.degrees(roll):.1f}°)")
            
            success, solution = self.check_ik(x, y, z, roll)
            if success:
                print(f"  ✓ IK SUCCESS")
                joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5_roll']
                for i, (jname, val) in enumerate(zip(joint_names, solution)):
                    print(f"    {jname}: {val:.3f} rad ({np.degrees(val):.1f}°)")
            else:
                print(f"  ✗ IK FAILED")

def main():
    rclpy.init()
    tester = IKTest()
    try:
        tester.test_poses()
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
