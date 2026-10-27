#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from feedback_control_interface.srv import FeedbackControl

class CancelPickPlaceClient(Node):
    def __init__(self):
        super().__init__('cancel_pick_place_client')
        self.cli = self.create_client(FeedbackControl, 'cancel_pick_place')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /cancel_pick_place service...")

    def send_cancel_request(self):
        req = FeedbackControl.Request()  # empty request
        self.get_logger().info("Sending cancel request...")
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Cancel response: success={future.result().success}, message='{future.result().message}'")
        else:
            self.get_logger().error("Service call failed")

def main(args=None):
    rclpy.init(args=args)
    node = CancelPickPlaceClient()
    node.send_cancel_request()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
