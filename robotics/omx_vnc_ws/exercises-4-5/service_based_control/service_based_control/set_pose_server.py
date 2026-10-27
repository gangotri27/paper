#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from service_based_control_interface.srv import SetPose  # custom service type

TRAJ_TOPIC = "/arm_controller/joint_trajectory"
JOINTS = ["joint1", "joint2", "joint3", "joint4"]

# Define predefined joint angles (in radians)
POSES = {
    "home":  [0.0, 0.0, 0.0, 0.0],
    "ready": [0.0, -0.5, 0.5, 0.0],
    "pick":  [0.0, 0.7, -1.0, 0.5],
    "place": [0.0, -0.3, 0.8, -0.2],
}

class ArmPoseService(Node):
    def __init__(self):
        super().__init__("arm_pose_service")
        self.publisher = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self.srv = self.create_service(SetPose, "set_pose", self.pose_callback)
        self.get_logger().info("Service '/set_pose' is ready...")

    def pose_callback(self, request, response):
        pose_name = request.pose_name.lower().strip()
        if pose_name not in POSES:
            response.success = False
            response.message = f"Invalid pose '{pose_name}'. Try home/ready/pick/place."
            self.get_logger().warn(response.message)
            return response

        joint_positions = POSES[pose_name]
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 2
        traj.points.append(point)

        self.publisher.publish(traj)
        self.get_logger().info(f"Moving to {pose_name} pose...")
        response.success = True
        response.message = f"Arm moved to {pose_name} pose."
        return response


def main():
    rclpy.init()
    node = ArmPoseService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
