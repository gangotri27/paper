#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TRAJ_TOPIC = "/arm_controller/joint_trajectory"
JOINTS = ["joint1", "joint2", "joint3", "joint4"]

class Demo(Node):
    def __init__(self):
        super().__init__("omx_traj_demo")
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        
        # Each direction corresponds to one pose
        self.poses = {
            'left':  [1.57, 0.0, 0.0, 0.0],
            'right': [-1.57, 0.0, 0.0, 0.0],
            'up':    [0.0, 0.0, 0.0, -1.0],
            'down':  [0.0, 0.0, 0.0, 1.0],
            'reset': [0.0, 0.0, 0.0, 0.0]
        }

    def tick(self, pose):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = pose
        point.time_from_start = Duration(sec=2)
        msg.points.append(point)
        self.pub.publish(msg)
        self.get_logger().info(f"Moving to: {pose}")

def main():
    rclpy.init()
    node = Demo()
    try:
        while True:
            posi = input("\nEnter direction (left, right, up, down, reset) or 'exit': ").lower()
            
            if posi == 'exit':
                print("Exiting...")
                break
            elif posi in node.poses:
                node.tick(node.poses[posi])
                time.sleep(3)
            else:
                print("Invalid input — try again.")

    except KeyboardInterrupt:
        print("Ctrl+C — shutting down cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
