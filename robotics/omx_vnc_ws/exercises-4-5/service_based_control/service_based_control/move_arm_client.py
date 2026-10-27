#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from service_based_control_interface.srv import MoveArm

class ArmClient(Node):
    def __init__(self):
        super().__init__('arm_client')
        self.cli = self.create_client(MoveArm, 'move_arm')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, direction):
        req = MoveArm.Request()
        req.direction = direction
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = ArmClient()
    try:
        while True:
            direction = input("Type: left | right | top | bottom | reset | exit --> ").lower()
            if direction == "exit":
                break
            resp = node.send_request(direction)
            print(f"Response: success={resp.success}, message='{resp.message}'")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()