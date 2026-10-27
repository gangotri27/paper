#!/usr/bin/env python3
"""
feedback_control_client.py

Simple client: just press Enter, pick-and-place runs on server.
No step-wise output on client.
"""

import rclpy
from rclpy.node import Node
from feedback_control_interface.srv import FeedbackControl

class FeedbackControlClient(Node):
    def __init__(self):
        super().__init__("feedback_control_client")
        self.cli = self.create_client(FeedbackControl, "perform_feedback")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /perform_feedback service...")

    def send_request(self):
        req = FeedbackControl.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackControlClient()
    try:
        input("Press Enter to start pick-and-place...")
        resp = node.send_request()
        print(f"Request sent: {resp.message}")  # prints only once
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
