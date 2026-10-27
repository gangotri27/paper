#!/usr/bin/env python3
import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ArmMover(Node):
    def __init__(self):
        super().__init__("arm_mover")
        # Listen to the text from the Audio Node
        self.sub = self.create_subscription(String, "/prompt", self.command_callback, 10)
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        
        # --- DEFINING POSES ---
        self.poses = {
            "right": [-1.57, 0.0, 0.0, 0.0],
            "left":  [1.57, 0.0, 0.0, 0.0],
            "up":    [0.0, 0.0, 0.0, -1.0],
            "down":  [0.0, 0.0, 0.0, 1.0],
            "reset": [0.0, 0.0, 0.0, 0.0], # Reset goes to Home
            "home":  [0.0, 0.0, 0.0, 0.0]
        }
        
        self.get_logger().info("💪 Arm Mover Ready. Waiting for commands...")
        self.get_logger().info(f"Supported commands: {list(self.poses.keys())}, exit")

    def command_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"📩 Received: '{command}'")
        
        # 1. Check for EXIT
        if "exit" in command:
            self.get_logger().warn("🛑 EXIT COMMAND RECEIVED. Shutting down node.")
            rclpy.shutdown()
            sys.exit(0)
            return

        # 2. Check for STOP
        if "stop" in command:
            self.get_logger().warn("⚠️ STOPPING (Moving to current position/Home)")
            # For safety, sending it home or you could send current joint states
            self.move_arm([0.0, 0.0, 0.0, 0.0]) 
            return

        # 3. Check for MOVEMENT commands
        found_pose = None
        for key in self.poses:
            if key in command:
                found_pose = self.poses[key]
                self.get_logger().info(f"✅ Matched command: {key.upper()}")
                break
        
        if found_pose:
            self.move_arm(found_pose)
        else:
            self.get_logger().info("🤷 Command not recognized in dictionary.")

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





















# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from builtin_interfaces.msg import Duration

# class ArmMover(Node):
#     def __init__(self):
#         super().__init__("arm_mover")
#         # Listen to the text from the Audio Node
#         self.sub = self.create_subscription(String, "/prompt", self.command_callback, 10)
#         self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
#         self.get_logger().info("💪 Arm Mover Ready. Waiting for text on /prompt...")

#     def command_callback(self, msg):
#         command = msg.data
#         self.get_logger().info(f"Received Command: {command}")
        
#         # Simple Logic
#         if "right" in command:
#             self.move_arm([-1.57, 0.0, 0.0, 0.0])
#         elif "left" in command:
#             self.move_arm([1.57, 0.0, 0.0, 0.0])
#         elif "stop" in command:
#             self.get_logger().warn("STOPPING")
#             # Send stop logic here

#     def move_arm(self, positions):
#         traj = JointTrajectory()
#         traj.joint_names = ["joint1", "joint2", "joint3", "joint4"]
#         point = JointTrajectoryPoint()
#         point.positions = positions
#         point.time_from_start = Duration(sec=2)
#         traj.points.append(point)
#         self.pub.publish(traj)

# def main():
#     rclpy.init()
#     node = ArmMover()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()