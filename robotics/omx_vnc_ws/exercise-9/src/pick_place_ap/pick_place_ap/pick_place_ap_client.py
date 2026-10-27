#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pick_place_ap_interface.action import PickPlace
from rclpy.action import ActionClient
import sys
import select

class PickPlaceClient(Node):
    def __init__(self):
        super().__init__("pick_place_client")
        self.client = ActionClient(self, PickPlace, "pick_place_action")
        self.goal_handle = None

    def send_goal(self):
        if not self.client.wait_for_server(timeout_sec=2.0):
            print("Server not found")
            return

        goal = PickPlace.Goal()
        print("-> Sending Goal...")
        
        # Send goal
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, future)
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            print("-> Goal Rejected")
            return

        print("-> Goal Accepted. Type 'cancel' to stop.")
        
        # Monitor Loop for Result + Cancel
        res_future = self.goal_handle.get_result_async()
        
        while rclpy.ok() and not res_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Check for user input (Non-blocking)
            if select.select([sys.stdin], [], [], 0.0)[0]:
                cmd = sys.stdin.readline().strip()
                if cmd == 'cancel':
                    print("-> Requesting Cancel...")
                    self.goal_handle.cancel_goal_async()
        
        if res_future.done():
            res = res_future.result().result
            print(f"-> Result: {res.success} | {res.message}")

    def feedback_cb(self, msg):
        fb = msg.feedback
        # We use getattr to be safe if 'status' is still missing during debug
        step = getattr(fb, "current_step", "Unknown")
        stat = getattr(fb, "status", "Unknown") 
        print(f"   [FB] {step} : {stat}")

def main():
    rclpy.init()
    node = PickPlaceClient()
    
    print("Commands: send | exit")
    try:
        while rclpy.ok():
            print("> ", end='', flush=True)
            if select.select([sys.stdin], [], [], 0.1)[0]:
                cmd = sys.stdin.readline().strip()
                if cmd == "send":
                    node.send_goal()
                elif cmd == "exit":
                    break
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()