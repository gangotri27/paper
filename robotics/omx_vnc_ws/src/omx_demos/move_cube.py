# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from std_msgs.msg import Float64
# import time

# class PickPlaceController(Node):
#     def __init__(self):
#         super().__init__('pick_place_controller')

#         # Publisher for joint trajectory
#         self.joint_pub = self.create_publisher(
#             JointTrajectory,
#             '/arm_controller/joint_trajectory',
#             10
#         )

#         # Publisher for gripper
#         self.gripper_pub = self.create_publisher(
#             Float64,
#             '/gripper_controller/transition_event',
#             10
#         )

#         # Joint names
#         self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

#     def send_joint_positions(self, positions, duration=2.0):
#         traj = JointTrajectory()
#         traj.joint_names = self.joint_names
#         point = JointTrajectoryPoint()
#         point.positions = positions
#         point.time_from_start.sec = int(duration)
#         traj.points.append(point)
#         self.joint_pub.publish(traj)
#         time.sleep(duration)

#     def control_gripper(self, value):
#         msg = Float64()
#         msg.data = value
#         self.gripper_pub.publish(msg)
#         time.sleep(1.0)

#     def execute_pick_and_place(self):
        
#         self.get_logger().info("Starting pick and place sequence...")

#         # Move to home
#         self.send_joint_positions([0.0, 0.0, 0.0, 0.0])

#         # Move arm above cube (pre-pick, EE ~ z=0.15)
#         self.send_joint_positions([0.0, 0.8, -0.6, 0.0], duration=3.0)

#         # Lower arm to cube height (EE ~ z=0.05)
#         self.send_joint_positions([0.0, 1.0, -0.9, 0.0], duration=3.0)

#         # Close gripper to grasp cube
#         self.control_gripper(0.01)

#         # Lift cube up a bit
#         self.send_joint_positions([0.0, 0.7, -0.5, 0.0], duration=3.0)

#         # Move to place position (shift x, y)
#         self.send_joint_positions([0.4, 0.6, -0.5, 0.2], duration=3.0)

#         # Open gripper to release cube
#         self.control_gripper(0.04)

#         # Return to home
#         self.send_joint_positions([0.0, 0.0, 0.0, 0.0], duration=3.0)

#         self.get_logger().info("Pick and place completed.")

# def main(args=None):
#     rclpy.init(args=args)
#     node = PickPlaceController()
#     node.execute_pick_and_place()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()






















































# # #!/usr/bin/env python3
# # import rclpy
# # import time
# # from rclpy.node import Node
# # from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# # from std_msgs.msg import Float64

# # # Topics
# # ARM_TOPIC = "/arm_controller/joint_trajectory"
# # GRIPPER_TOPIC = "/gripper_controller/command"

# # # Joint names for the manipulator
# # JOINTS = ["joint1", "joint2", "joint3", "joint4"]

# # # Poses for the arm
# # POSES = {
# #     "home":  [0.0, 0.0, 0.0, 0.0],
# #     "ready": [0.0, -0.3, 0.6, 0.0],
# #     "pick":  [0.0, 0.8, 0.0, 0.0],
# #     "place": [0.5, 0.7, -1.1, 0.2],
# # }

# # class PickPlaceDemo(Node):
# #     def __init__(self):
# #         super().__init__("pick_place_with_gripper_demo")

# #         # Publishers
# #         self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
# #         self.gripper_pub = self.create_publisher(Float64, GRIPPER_TOPIC, 10)

# #         self.get_logger().info("🤖 Pick & Place Simulation Node with Gripper started!")
# #         time.sleep(1.0)

# #     def move_arm(self, pose_name, duration=2.5):
# #         """Moves arm to the given pose"""
# #         traj = JointTrajectory()
# #         traj.joint_names = JOINTS

# #         point = JointTrajectoryPoint()
# #         point.positions = POSES[pose_name]
# #         point.time_from_start.sec = int(duration)
# #         traj.points.append(point)

# #         self.arm_pub.publish(traj)
# #         self.get_logger().info(f"🦾 Moving arm to '{pose_name}' pose: {POSES[pose_name]}")
# #         time.sleep(duration + 0.5)

# #     def move_gripper(self, value, action="open"):
# #         """Moves gripper (open or close)"""
# #         msg = Float64()
# #         msg.data = value
# #         self.gripper_pub.publish(msg)
# #         if action == "open":
# #             self.get_logger().info("👐 Gripper opening...")
# #         else:
# #             self.get_logger().info("✊ Gripper closing...")
# #         time.sleep(1.0)

# # def main():
# #     rclpy.init()
# #     node = PickPlaceDemo()

# #     try:
# #         # Move sequence
# #         node.move_arm("home")
# #         node.move_gripper(0.02, "open")    # Open gripper before starting
# #         node.move_arm("ready")

# #         node.move_arm("pick")
# #         node.move_gripper(0.0, "close")    # Close gripper at pick

# #         node.move_arm("place")
# #         node.move_gripper(0.02, "open")    # Open gripper to release

# #         node.move_arm("home")
# #         node.get_logger().info("✅ Pick & Place simulation complete!")

# #     finally:
# #         node.destroy_node()
# #         rclpy.shutdown()

# # if __name__ == "__main__":
# #     main()































# # #!/usr/bin/env python3
# # import rclpy
# # import time
# # from rclpy.node import Node
# # from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# # TRAJ_TOPIC = "/arm_controller/joint_trajectory"
# # JOINTS = ["joint1", "joint2", "joint3", "joint4"]

# # POSES = {
# #     "home":  [0.0, 0.0, 0.0, 0.0],
# #     "ready": [0.0, -0.3, 0.6, 0.0],
# #     "pick":  [0.0, 0.8, 0.0, 0.0],
# #     "place": [0.5, 0.7, -1.1, 0.2],
# # }

# # class PickPlaceDemo(Node):
# #     def __init__(self):
# #         super().__init__("pick_place_demo")
# #         self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
# #         self.get_logger().info("Pick & Place Simulation Node started!")
# #         time.sleep(1.0)

# #     def move_arm(self, pose_name, duration=2.5):
# #         traj = JointTrajectory()
# #         traj.joint_names = JOINTS

# #         point = JointTrajectoryPoint()
# #         point.positions = POSES[pose_name]
# #         point.time_from_start.sec = int(duration)

# #         traj.points.append(point)
# #         self.pub.publish(traj)
# #         self.get_logger().info(f"Moving to {pose_name} pose: {POSES[pose_name]}")
# #         time.sleep(duration + 1.0)

# # def main():
# #     rclpy.init()
# #     node = PickPlaceDemo()

# #     try:
# #         node.move_arm("home")
# #         node.move_arm("ready")
# #         node.move_arm("pick")
# #         node.move_arm("place")
# #         node.move_arm("home")
# #         node.get_logger().info("✅ Pick & Place simulation complete!")
# #     finally:
# #         node.destroy_node()
# #         rclpy.shutdown()

# # if __name__ == "__main__":
# #     main()



#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import time

# Define joint names
joint_names = ["joint1", "joint2", "joint3", "joint4"]

# Pick and place joint positions (example values, adjust for your cube)
pick_joints = [0.0, -0.5, 0.5, 0.0]
place_joints = [0.2, -0.3, 0.3, 0.0]

class PickPlace(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        # Publisher to arm trajectory
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Gripper action client
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        while not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper action server...')

    def move_arm(self, positions, duration=2.0):
        traj = JointTrajectory()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        traj.points.append(point)
        self.arm_pub.publish(traj)
        self.get_logger().info(f'Moving arm to {positions}')
        time.sleep(duration + 0.5)  # wait for motion to finish

    def move_gripper(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 1.0
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'Gripper moved to {position}')

def main(args=None):
    rclpy.init(args=args)
    node = PickPlace()

    # Step 1: Move above cube
    node.move_arm(pick_joints)

    # Step 2: Close gripper
    node.move_gripper(0.0)

    # Step 3: Move to place location
    node.move_arm(place_joints)

    # Step 4: Open gripper
    node.move_gripper(0.015)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
