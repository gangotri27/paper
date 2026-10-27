# import rclpy
# from rclpy.node import Node
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from std_msgs.msg import Float64
# import time
# from ros_gz_interfaces.srv import CreateJoint, DeleteJoint


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

#     # -------------------
#     # Attach / Detach Hacks
#     # -------------------
#     def attach_object(self, object_model="cube", object_link="link",
#                       ee_model="my_robot", ee_link="gripper_link"):
#         client = self.create_client(CreateJoint, "/world/default/create_joint")
#         while not client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info("Waiting for create_joint service...")

#         req = CreateJoint.Request()
#         req.joint_name = "grasp_joint"
#         req.parent_name = f"{ee_model}::{ee_link}"
#         req.child_name = f"{object_model}::{object_link}"
#         req.type = "fixed"

#         future = client.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         if future.result() is not None:
#             self.get_logger().info("Cube attached to gripper!")
#         else:
#             self.get_logger().error("Failed to attach cube")

#     def detach_object(self):
#         client = self.create_client(DeleteJoint, "/world/default/remove_joint")
#         while not client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info("Waiting for remove_joint service...")

#         req = DeleteJoint.Request()
#         req.name = "grasp_joint"

#         future = client.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         if future.result() is not None:
#             self.get_logger().info("Cube detached!")
#         else:
#             self.get_logger().error("Failed to detach cube")

#     # -------------------
#     # Pick and Place Sequence
#     # -------------------
#     def execute_pick_and_place(self):
#         self.get_logger().info("Starting pick and place sequence...")

#         # Move to home
#         self.send_joint_positions([0.0, 0.0, 0.0, 0.0])

#         # Move arm above cube (pre-pick, EE ~ z=0.15)
#         self.send_joint_positions([0.0, 0.8, -0.6, 0.0], duration=3.0)

#         # Lower arm to cube height (EE ~ z=0.05)
#         self.send_joint_positions([0.0, 1.0, -0.9, 0.0], duration=3.0)

#         # Close gripper to grasp cube + attach
#         self.control_gripper(0.01)
#         self.attach_object()

#         # Lift cube up a bit
#         self.send_joint_positions([0.0, 0.7, -0.5, 0.0], duration=3.0)

#         # Move to place position (shift x, y)
#         self.send_joint_positions([0.4, 0.6, -0.5, 0.2], duration=3.0)

#         # Detach + open gripper
#         self.detach_object()
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



# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from control_msgs.action import GripperCommand
# from rclpy.action import ActionClient
# import time

# class PickPlace(Node):
#     def __init__(self):
#         super().__init__('pick_place_node')

#         # Arm publisher
#         self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

#         # Gripper action client
#         self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
#         while not self.gripper_client.wait_for_server(timeout_sec=1.0):
#             self.get_logger().info('Waiting for gripper action server...')

#         # Define joint names
#         self.joint_names = ["joint1", "joint2", "joint3", "joint4"]

#     def send_joint_positions(self, positions, duration=2.0):
#         traj = JointTrajectory()
#         traj.joint_names = self.joint_names
#         point = JointTrajectoryPoint()
#         point.positions = positions
#         point.time_from_start.sec = int(duration)
#         traj.points.append(point)
#         self.arm_pub.publish(traj)
#         self.get_logger().info(f'Moving arm to {positions}')
#         time.sleep(duration + 0.5)  # wait for motion to complete

#     def move_gripper(self, position):
#         goal = GripperCommand.Goal()
#         goal.command.position = position
#         goal.command.max_effort = 1.0
#         future = self.gripper_client.send_goal_async(goal)
#         rclpy.spin_until_future_complete(self, future)
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error('Gripper goal rejected!')
#             return
#         result_future = goal_handle.get_result_async()
#         rclpy.spin_until_future_complete(self, result_future)
#         self.get_logger().info(f'Gripper moved to {position}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = PickPlace()

#     # ---- FIRST PICK AND PLACE ----

#     # Move to home
#     node.send_joint_positions([0.0, 0.0, 0.0, 0.0])

#     # Move above cube (pre-pick)
#     node.send_joint_positions([0.0, 0.8, -0.6, 0.0], duration=3.0)

#     # Lower arm to cube
#     node.send_joint_positions([0.0, 1.0, -0.7, 0.0], duration=3.0)

#     # Close gripper to pick cube
#     node.move_gripper(0.0)  # closed

#     # Lift cube
#     node.send_joint_positions([0.0, 0.7, -0.5, 0.0], duration=3.0)

#     # Move to place position
#     node.send_joint_positions([0.4, 0.6, -0.5, 0.2], duration=3.0)

#     # Open gripper to release cube
#     node.move_gripper(0.015)  # open

#     # Return home
#     node.send_joint_positions([0.0, 0.0, 0.0, 0.0], duration=2.0)

#     # ---- SECOND PICK AND PLACE (FROM DROPPED POSITION) ----
#     # Cube dropped slightly, so lower Z joint a little more
#     # Adjusted "pre-pick" and "pick" positions
#     node.send_joint_positions([0.3, 0.8, -0.6, 0.0], duration=3.0)  # pre-pick
#     node.send_joint_positions([0.3, 1.0, -0.5, 0.0], duration=3.0)  # lower to dropped cube

#     # Close gripper to pick cube
#     node.move_gripper(0.0)  # closed

#     # Lift cube
#     node.send_joint_positions([0.0, 0.75, -0.5, 0.0], duration=3.0)

#     # Move back to original place position
#     node.send_joint_positions([0.0, 1.0, -0.7, 0.0], duration=3.0)

#     # Open gripper to release cube
#     node.move_gripper(0.015)  # open

#     # Return home finally
#     node.send_joint_positions([0.0, 0.0, 0.0, 0.0], duration=2.0)

#     node.get_logger().info("Pick → Place → Reverse Pick → Place completed successfully.")

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

















#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
import time

class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')

        # Publisher for arm joint trajectory
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Action client for gripper
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

    def send_joint_positions(self, positions, duration=2.0):
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        traj.points.append(point)
        self.arm_pub.publish(traj)
        self.get_logger().info(f"Moving arm to {positions}")
        time.sleep(duration + 0.5)  # wait for movement to finish

    def control_gripper(self, position=0.0, max_effort=1.0):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.gripper_client.wait_for_server()
        # Send goal asynchronously
        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Gripper goal rejected')
            return

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result
        self.get_logger().info(f"Gripper set to {position}, result: {result}")


    def pick_and_place_reverse(self):
        # Define joint positions sequence
        positions_sequence = [
            [0.0, 0.0, 0.0, 0.0],        # Home
            [0.0, 0.8, -0.6, 0.0],       # Pre-pick
            [0.0, 1.0, -0.7, 0.0],       # Lower to cube
            [0.0, 0.7, -0.5, 0.0],       # Lift
            [0.4, 0.6, -0.5, 0.2],       # Place
        ]

        # 1) Move to home
        self.send_joint_positions(positions_sequence[0])

        # 2) Move above cube (pre-pick)
        self.send_joint_positions(positions_sequence[1], duration=3.0)

        # 3) Lower to cube
        self.send_joint_positions(positions_sequence[2], duration=3.0)

        # 4) Close gripper to pick
        self.control_gripper(position=0.0, max_effort=1.0)
        time.sleep(1.0)

        # 5) Lift cube
        self.send_joint_positions(positions_sequence[3], duration=3.0)

        # 6) Move to place
        self.send_joint_positions(positions_sequence[4], duration=3.0)

        # 7) Open gripper to release
        self.control_gripper(position=0.015, max_effort=1.0)
        time.sleep(1.0)

        # 8) Reverse path to original pick position
        reverse_positions = positions_sequence[::-1]
        for pos in reverse_positions:
            self.send_joint_positions(pos, duration=2.5)

        # 9) Open gripper fully just in case
        self.control_gripper(position=0.015, max_effort=1.0)

        self.get_logger().info("Pick -> Place -> Return to original position completed!")

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    node.pick_and_place_reverse()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

