#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from srvcli.srv import PerformTask

TRAJ_TOPIC = "/arm_controller/joint_trajectory"
JOINTS = ["joint1", "joint2", "joint3", "joint4"]

# Define joint angles for each pose
POSES = {
    "home":  [0.0, 0.0, 0.0, 0.0],
    "ready": [0.0, -0.5, 0.5, 0.0],
    "pick":  [0.0, 1.2, -1.1, 0.5],
    "place": [0.0, -0.3, 0.8, -0.2],
}

class TaskServer(Node):
    def __init__(self):
        super().__init__("pick_place_task_server")
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self.srv = self.create_service(PerformTask, "perform_task", self.handle_task)
        self.get_logger().info("Service /perform_task ready to execute pick-and-place sequence!")

    def handle_task(self, request, response):
        if not request.start:
            response.result = "Task not started (set start=True)."
            return response

        for pose_name in ["home", "ready", "pick", "place", "home"]:
            self.move_to_pose(pose_name)
            time.sleep(2.0)  # wait for motion to complete

        response.result = "Pick-and-Place task completed successfully!"
        self.get_logger().info(response.result)
        return response

    def move_to_pose(self, pose_name):
        traj = JointTrajectory()
        traj.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = POSES[pose_name]
        point.time_from_start.sec = 2

        traj.points.append(point)
        self.pub.publish(traj)
        self.get_logger().info(f"Moved to {pose_name} pose.")

def main(args=None):
    rclpy.init(args=args)
    node = TaskServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
