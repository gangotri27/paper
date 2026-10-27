#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from perform_task_interface.srv import GripMove

class PickPlaceClient(Node):
    def __init__(self):
        super().__init__('pick_place_client')
        self.cli = self.create_client(GripMove, '/perform_task')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /perform_task service...')

    def send_request(self):
        req = GripMove.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("Service call failed")
            return None

def main(args=None):
    rclpy.init(args=args)
    client = PickPlaceClient()

    print("Pick-and-place client ready. Press Enter to start automatic task...")

    try:
        input()
        res = client.send_request()
        if res:
            print(f"Service response: success={res.success}, message='{res.message}'")
            print("Pick-and-Place task executed successfully!")
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()












# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from perform_task_interface.srv import GripMove

# class PickPlaceClient(Node):
#     def __init__(self):
#         super().__init__('pick_place_client')
#         self.cli = self.create_client(GripMove, '/perform_task')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Waiting for pick-and-place service...')

#     def send_request(self, action_name):
#         """Send a single action request to the server"""
#         req = GripMove.Request()
#         req.action = action_name
#         future = self.cli.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         if future.result() is not None:
#             return future.result()
#         else:
#             self.get_logger().error('Service call failed!')
#             return None


# def main(args=None):
#     rclpy.init(args=args)
#     client = PickPlaceClient()

#     print("Pick-and-place client ready. Available commands:")
#     print("home, ready, pick, close, lift, place, open, finish, reverse_pick, quit")

#     try:
#         while True:
#             action = input("Enter command: ").strip().lower()
#             if action == "quit":
#                 print("Exiting client...")
#                 break
#             elif action not in ["home", "ready", "pick", "close", "lift", "place", "open", "finish", "reverse_pick"]:
#                 print(f"Invalid command '{action}', try again.")
#                 continue

#             response = client.send_request(action)
#             if response:
#                 print(f"Response: success={response.success}, message={response.message}")

#     except KeyboardInterrupt:
#         print("\nClient interrupted. Exiting...")

#     finally:
#         client.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
