#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

from service_based_control_interface.srv import MoveArm

TRAJ_TOPIC = "/arm_controller/joint_trajectory"
JOINTS = ["joint1","joint2","joint3","joint4"]

class ArmService(Node):
    def __init__(self):
        super().__init__('arm_service')
        self.srv = self.create_service(MoveArm, 'move_arm', self.handle_request)
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self.poses = {
            'left': [1.57,0.0,0.0,0.0],
            'right': [-1.57,0.0,0.0,0.0],
            'top': [0.0,0.0,0.0,-1.0],
            'bottom': [0.0,0.0,0.0,1.0],
            'reset': [0.0,0.0,0.0,0.0]
        }
        self.get_logger().info(f"Service ready. Publishing to {TRAJ_TOPIC} | joints={JOINTS}")

    def tick(self, pose):
        msg = JointTrajectory(joint_names=JOINTS)
        point = JointTrajectoryPoint(positions=pose, time_from_start=Duration(sec=2))
        msg.points.append(point)
        self.pub.publish(msg)
        self.get_logger().info(f"Moved arm to {pose}")

    def handle_request(self, request, response):
        direction = request.direction.lower()
        if direction not in self.poses:
            response.success = False
            response.message = f"Invalid direction: {direction}"
            return response

        self.tick(self.poses[direction])
        time.sleep(2.0)  # ensure the robot has time to move
        response.success = True
        response.message = f"Moved arm {direction}"
        return response

def main():
    rclpy.init()
    node = ArmService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
