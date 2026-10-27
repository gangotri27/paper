#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import sys
import termios
import tty

class KeyboardTrigger(Node):
    def __init__(self):
        super().__init__("keyboard_trigger")
        self.pub = self.create_publisher(Empty, "/listen", 10)
        print("\n" + "="*40)
        print("🎹 KEYBOARD TRIGGER STARTED")
        print("👉 Press [ENTER] to start listening")
        print("👉 Press [Ctrl+C] to quit")
        print("="*40 + "\n")

    def trigger(self):
        msg = Empty()
        self.pub.publish(msg)
        print("🔔 Signal sent! Speak now...")

def main():
    rclpy.init()
    node = KeyboardTrigger()
    
    try:
        while rclpy.ok():
            # Wait for user input
            input("Press [ENTER] to Listen > ")
            node.trigger()
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()