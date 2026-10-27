#!/usr/bin/env python3
import rclpy
import sys
import pyttsx3  # <--- IMPORT LIBRARY
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ArmMover(Node):
    def __init__(self):
        super().__init__("arm_mover")
        
        # --- TTS INIT ---
        # Initialize the Text-to-Speech engine once on startup
        try:
            self.engine = pyttsx3.init()
            self.engine.setProperty('rate', 150)    # Speed of speech
            self.engine.setProperty('volume', 1.0)  # Volume (0.0 to 1.0)
        except Exception as e:
            self.get_logger().error(f"Failed to init TTS: {e}")

        # Listen to the text from the Audio Node
        self.sub = self.create_subscription(String, "/prompt", self.command_callback, 10)
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        
        # --- DEFINING POSES ---
        self.poses = {
            "right": [-1.57, 0.0, 0.0, 0.0],
            "left":  [1.57, 0.0, 0.0, 0.0],
            "up":    [0.0, 0.0, 0.0, -1.0],
            "down":  [0.0, 0.0, 0.0, 1.0],
            "reset": [0.0, 0.0, 0.0, 0.0], 
            "home":  [0.0, 0.0, 0.0, 0.0]
        }
        
        self.get_logger().info("💪 Arm Mover Ready. Waiting for commands...")

    def speak(self, text):
        """Helper function to make the robot speak"""
        try:
            self.get_logger().info(f"🗣️ Saying: '{text}'")
            self.engine.say(text)
            self.engine.runAndWait()
        except Exception as e:
            self.get_logger().warn(f"TTS Error: {e}")

    def command_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"📩 Received: '{command}'")
        
        # 1. Check for EXIT
        if "exit" in command:
            self.speak("Shutting down system.")
            self.get_logger().warn("🛑 EXIT COMMAND RECEIVED.")
            rclpy.shutdown()
            sys.exit(0)
            return

        # 2. Check for STOP
        if "stop" in command:
            self.speak("Stopping movement.")
            self.move_arm([0.0, 0.0, 0.0, 0.0]) 
            return

        # 3. Check for MOVEMENT commands
        found_pose = None
        target_name = ""

        for key in self.poses:
            if key in command:
                found_pose = self.poses[key]
                target_name = key
                self.get_logger().info(f"✅ Matched command: {key.upper()}")
                break
        
        if found_pose:
            # First, send the physical command so the robot starts moving
            self.move_arm(found_pose)
            
            # Immediately say what we are doing
            # e.g., "Moving towards right"
            self.speak(f"Moving towards {target_name}")
        else:
            self.get_logger().info("🤷 Command not recognized in dictionary.")
            # Optional: Tell the user you didn't understand
            # self.speak("Command not recognized")

    def move_arm(self, positions):
        traj = JointTrajectory()
        traj.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=2)
        traj.points.append(point)
        self.pub.publish(traj)

def main():
    rclpy.init()
    node = ArmMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()