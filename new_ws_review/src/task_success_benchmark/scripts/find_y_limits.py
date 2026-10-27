#!/usr/bin/env python3
"""
Find the maximum y-offset reachable at x=0.20, z=0.20
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, Constraints
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np

class YLimitFinder(Node):
    def __init__(self):
        super().__init__('y_limit_finder')
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
        ik_request = PositionIKRequest()
        ik_request.group_name = 'arm'
        ik_request.pose_stamped.header.frame_id = 'world'
        ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik_request.pose_stamped.pose.position = Point(x=x, y=y, z=z)
        ik_request.pose_stamped.pose.orientation = self.quaternion_from_euler(roll)
        ik_request.timeout.sec = 1
        ik_request.avoid_collisions = False
        ik_request.constraints = Constraints()
        request.ik_request = ik_request
        
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() and future.result().error_code.val == 1:
            return True
        return False
    
    def find_limits(self):
        print("\n" + "="*70)
        print("FINDING Y-OFFSET LIMITS AT X=0.20, Z=0.20")
        print("="*70)
        
        x_fixed = 0.20
        z_fixed = 0.20
        
        print(f"\nTesting positive y-offsets:")
        for y in np.arange(0.01, 0.11, 0.01):
            success = self.check_ik(x_fixed, y, z_fixed)
            print(f"  y=+{y:.2f}: {'✓' if success else '✗'}")
            if not success and y > 0.05:
                print(f"    → Maximum positive y-offset: {y-0.01:.2f}m")
                break
        
        print(f"\nTesting negative y-offsets:")
        for y in np.arange(-0.01, -0.11, -0.01):
            success = self.check_ik(x_fixed, y, z_fixed)
            print(f"  y={y:.2f}: {'✓' if success else '✗'}")
            if not success and y < -0.05:
                print(f"    → Maximum negative y-offset: {y+0.01:.2f}m")
                break

def main():
    rclpy.init()
    finder = YLimitFinder()
    finder.find_limits()
    finder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
