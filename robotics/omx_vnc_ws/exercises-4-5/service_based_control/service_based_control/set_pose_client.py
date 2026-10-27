#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from service_based_control_interface.srv import SetPose

class ArmPoseClient(Node):
    def __init__(self):
        super().__init__("arm_pose_client")
        self.client = self.create_client(SetPose, "set_pose")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /set_pose service...")

    def send_pose_request(self, pose_name):
        request = SetPose.Request()
        request.pose_name = pose_name
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    node = ArmPoseClient()
    while True:
        pose = input("Enter pose (home/ready/pick/place/quit): ").strip().lower()
        if pose == "quit":
            print("Exiting client...")
            break
        response = node.send_pose_request(pose)
        print(f"Response: success={response.success}, message={response.message}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
