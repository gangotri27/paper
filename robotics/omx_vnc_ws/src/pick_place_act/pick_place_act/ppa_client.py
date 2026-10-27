#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pick_place_act_interface.action import PickPlace
from rclpy.action import ActionClient
import sys
import select

class PickPlaceClient(Node):
    def __init__(self):
        super().__init__("pick_place_client")
        self.client = ActionClient(self, PickPlace, "pick_place_action")
        self.current_goal_handle = None
        self._active_cancel_monitor = False # monitor flag
        
        print("\n── Pick & Place Client Ready ───────────────────────────")
        print("Type: send | cancel | exit")
        print("────────────────────────────────────────────────────────\n")

    # -------------------------
    # SEND GOAL
    # -------------------------
    def send_goal(self):
        print("\n⏳ Waiting for action server...")
        if not self.client.wait_for_server(timeout_sec=2.0):
            print("❌ Action server not available.\n")
            return

        goal = PickPlace.Goal()
        print("🚀 Sending goal...\n")
        
        # We attach the feedback callback here
        send_goal_fut = self.client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_goal_fut)

        self.current_goal_handle = send_goal_fut.result()
        if not self.current_goal_handle.accepted:
            print("❌ Goal was rejected by server.\n")
            self.current_goal_handle = None
            return

        print("✅ Goal accepted.\n")

        # Start active cancel monitor
        self._active_cancel_monitor = True
        
        # Get result asynchronously
        result_future = self.current_goal_handle.get_result_async()

        # ----------------------------
        # MAIN LOOP (wait for result + monitor cancel)
        # ----------------------------
        while rclpy.ok() and self._active_cancel_monitor:
            # Pump ROS callbacks (this allows _feedback_cb to fire)
            rclpy.spin_once(self, timeout_sec=0.1)

            # Check if result arrived
            if result_future.done():
                result = result_future.result().result
                self._print_result(result)
                break

            # Check keyboard input
            if self._keyboard_pressed():
                ch = sys.stdin.readline().strip().lower()
                if ch == "cancel":
                    print("🛑 Sending cancel request...\n")
                    cancel_fut = self.current_goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_fut)
                    print("✅ Goal canceled (server acknowledged)\n")
                    break

        # Disable cancel monitor
        self._active_cancel_monitor = False
        self.current_goal_handle = None

    # -------------------------
    # FEEDBACK CALLBACK
    # -------------------------
    def _feedback_cb(self, feedback_msg):
        # 'feedback_msg' is the container. 'feedback_msg.feedback' is your actual message
        fb = feedback_msg.feedback
        step = getattr(fb, "current_step", "(no step)")
        status = getattr(fb, "status", getattr(fb, "status_msg", "(no status)"))
        
        print(f"🔄 Feedback: step={step}, status={status}")

    # -------------------------
    # RESULT PRINT
    # -------------------------
    def _print_result(self, result):
        print("\n🎯 RESULT:")
        success = getattr(result, "success", "(unknown)")
        message = getattr(result, "message", getattr(result, "final_status", "(no message)"))
        print(f"  Success: {success}")
        print(f"  Message: {message}")
        print("────────────────────────────────────────────────────────\n")

    # -------------------------
    # KEYBOARD CHECK
    # -------------------------
    def _keyboard_pressed(self):
        """Non-blocking keyboard check"""
        return select.select([sys.stdin], [], [], 0.0)[0]

# -------------------------
# MAIN LOOP
# -------------------------
def main():
    rclpy.init()
    node = PickPlaceClient()

    try:
        while rclpy.ok():
            cmd = input().strip().lower()
            if cmd == "send":
                node.send_goal()
            elif cmd == "cancel":
                print("⚠️ Cancel is only available while a goal is active inside 'send'\n")
            elif cmd == "exit":
                print("\n👋 Shutting down client...\n")
                break
            else:
                print("⚠️ Unknown command. Use: send / cancel / exit\n")
            
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\n👋 Client interrupted by user.\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()