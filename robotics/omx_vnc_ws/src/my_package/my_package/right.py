#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Topic where trajectory commands are published
TRAJ_TOPIC = "/arm_controller/joint_trajectory"

# List of joint names in your robot arm
JOINTS = ["joint1", "joint2", "joint3", "joint4"]


class Demo(Node):
    def __init__(self):
        super().__init__("omx_traj_demo")
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self.poses = [
            [0.0, 0.0, 0.0, 0.0],  # Initial position
            [-1.57, 0.0, 0.0, 0.0], # Move to the right
            [0.0, 0.0, 0.0, 0.0],  # Return to start
        ]

    def tick(self, pose, index):
        msg = JointTrajectory()
        msg.joint_names = JOINTS

        p = JointTrajectoryPoint()
        p.positions = pose
        p.time_from_start = Duration(sec=2)

        msg.points.append(p)
        self.pub.publish(msg)
        self.get_logger().info(f"Phase {index} → positions {pose}")


def main():
    rclpy.init()
    node = Demo()
    try:
        for i in range(len(node.poses)):
            node.tick(node.poses[i], i)
            time.sleep(3)
    except KeyboardInterrupt:
        print("Ctrl+C — shutting down cleanly.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
