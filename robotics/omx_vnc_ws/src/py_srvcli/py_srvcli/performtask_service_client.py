#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from srvcli.srv import PerformTask

class TaskClient(Node):
    def __init__(self):
        super().__init__("task_client")
        self.cli = self.create_client(PerformTask, "perform_task")

        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for service /perform_task...")

        self.request = PerformTask.Request()

    def send_request(self):
        self.request.start = True
        future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = TaskClient()
    response = client.send_request()
    client.get_logger().info(f"Response: {response.result}")
    client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
