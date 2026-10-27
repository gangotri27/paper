#!/usr/bin/env python3
"""
Detailed reachability analysis using /compute_ik service
"""

import rclpy
import sys
import os
import numpy as np
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from task_success_benchmark.multi_stage_planner import MultiStagePlanner
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, Constraints, JointConstraint
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
import time

class ReachabilityTester:
    def __init__(self, robot_type='6dof'):
        self.planner = MultiStagePlanner(robot_type=robot_type)
        
        # Create IK service client
        self.ik_client = self.planner.create_client(
            GetPositionIK, 
            '/compute_ik'
        )
        
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.planner.get_logger().error("IK service not available")
        else:
            self.planner.get_logger().info("✓ Connected to IK service")
    
    def quaternion_from_euler(self, roll, pitch=0.0, yaw=0.0):
        """Convert euler angles to quaternion"""
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
    
    def check_ik(self, position, roll=0.0, pitch=0.0, yaw=0.0):
        """Check if pose has an IK solution"""
        request = GetPositionIK.Request()
        
        # Create IK request
        ik_request = PositionIKRequest()
        ik_request.group_name = 'arm'
        ik_request.ik_link_name = self.planner.ee_link
        ik_request.pose_stamped.header.frame_id = 'world'
        ik_request.pose_stamped.header.stamp = self.planner.get_clock().now().to_msg()
        
        # Set target pose
        ik_request.pose_stamped.pose.position = Point(
            x=position[0],
            y=position[1], 
            z=position[2]
        )
        ik_request.pose_stamped.pose.orientation = self.quaternion_from_euler(roll, pitch, yaw)
        
        # Set timeout
        ik_request.timeout.sec = 1
        
        # Set robot state (current state)
        ik_request.robot_state.joint_state.name = self.planner.joint_names
        ik_request.robot_state.joint_state.position = self.planner.current_joint_state
        
        # Avoid collisions
        ik_request.avoid_collisions = True
        
        request.ik_request = ik_request
        
        # Call service
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self.planner, future, timeout_sec=2.0)
        
        if future.result() is not None:
            response = future.result()
            if response.error_code.val == 1:  # SUCCESS
                solution = response.solution.joint_state.position
                return True, solution
        
        return False, None
    
    def test_poses(self):
        """Test various poses"""
        print("\n" + "="*70)
        print("DETAILED REACHABILITY ANALYSIS")
        print("="*70)
        
        # Test poses: (name, position, roll, pitch, yaw)
        test_poses = [
            ("Start State", (0.0, 0.0, 0.0), 0.0, 0.0, 0.0),
            ("Approach Pick (0.25m)", (0.20, -0.10, 0.25), 0.0, 0.0, 0.0),
            ("Pick (0.15m)", (0.20, -0.10, 0.15), 0.0, 0.0, 0.0),
            ("Place (0.15m)", (0.20, 0.10, 0.15), 0.0, 0.0, 0.0),
            ("Approach Place (0.25m)", (0.20, 0.10, 0.25), 0.0, 0.0, 0.0),
            ("Higher (0.30m)", (0.20, -0.10, 0.30), 0.0, 0.0, 0.0),
            ("With Roll 45°", (0.20, 0.0, 0.20), 0.785, 0.0, 0.0),
            ("With Roll 90°", (0.20, 0.0, 0.20), 1.571, 0.0, 0.0),
        ]
        
        for name, pos, roll, pitch, yaw in test_poses:
            print(f"\n{name}:")
            print(f"  Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            print(f"  Rotation: roll={roll:.2f}rad ({np.degrees(roll):.1f}°), "
                  f"pitch={pitch:.1f}rad, yaw={yaw:.1f}rad")
            
            # Check IK
            has_ik, solution = self.check_ik(pos, roll, pitch, yaw)
            if has_ik:
                print(f"  ✓ IK SOLUTION FOUND")
                joint_names = self.planner.joint_names
                for i, (name, val) in enumerate(zip(joint_names, solution)):
                    print(f"    {name}: {val:.3f} rad ({np.degrees(val):.1f}°)")
                
                # Try planning if IK exists
                self.planner.reset_to_start()
                traj, error = self.planner.plan_to_pose(
                    (pos[0], pos[1], pos[2], roll),
                    seed=12345
                )
                if error == 1:
                    print(f"  ✓ PATH PLANNABLE")
                else:
                    error_map = {
                        -2: "PLANNING_FAILED",
                        -3: "INVALID_MOTION_PLAN", 
                        -7: "TIMED_OUT",
                        -10: "START_STATE_IN_COLLISION",
                        -12: "GOAL_IN_COLLISION",
                        -14: "GOAL_CONSTRAINTS_VIOLATED",
                        -31: "NO_IK_SOLUTION",
                        99999: "GOAL_REJECTED"
                    }
                    error_name = error_map.get(error, f"UNKNOWN_{error}")
                    print(f"  ✗ PATH PLANNING FAILED: {error_name}")
            else:
                print(f"  ✗ NO IK SOLUTION - Pose unreachable")
    
    def cleanup(self):
        self.planner.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    tester = ReachabilityTester('6dof')
    try:
        tester.test_poses()
    finally:
        tester.cleanup()

if __name__ == '__main__':
    main()
