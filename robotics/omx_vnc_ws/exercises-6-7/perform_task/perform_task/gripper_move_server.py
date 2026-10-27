#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from perform_task_interface.srv import GripMove
from std_msgs.msg import Float64  # Fallback if gripper action not found

TRAJ_TOPIC = "/arm_controller/joint_trajectory"
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"
JOINTS = ["joint1", "joint2", "joint3", "joint4"]

# Predefined poses
POSE_HOME = [0.0, 0.0, 0.0, 0.0]
POSE_ABOVE = [0.0, 0.8, -0.6, 0.0]
POSE_LOWER = [0.0, 1.0, -0.7, 0.0]
POSE_LIFT = [0.0, 0.7, -0.5, 0.0]
POSE_PLACE = [0.4, 0.6, -0.5, 0.2]

# Gripper positions
GRIP_CLOSE = 0.0
GRIP_OPEN = 0.015

# Timeout for gripper actions
TIMEOUT_SEC_ACTION = 8.0

class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_service_node')

        # Arm publisher
        self.arm_pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)

        # Gripper action client
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)
        self.gripper_action_available = self.gripper_client.wait_for_server(timeout_sec=2.0)
        if self.gripper_action_available:
            self.get_logger().info("Gripper action server found.")
        else:
            self.get_logger().warn("Gripper action server NOT found. Using direct topic fallback.")
            self.gripper_pub = self.create_publisher(Float64, '/gripper_controller/command', 10)

        # Service
        self.srv = self.create_service(GripMove, '/perform_task', self.handle_pick_place_request)

        self.get_logger().info(f"PickPlaceServer ready. Publishing arm commands to {TRAJ_TOPIC}")

    # ----------------- ARM MOVEMENT -----------------
    def publish_arm_pose(self, pose, duration=2.0):
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = pose
        sec = int(duration)
        nsec = int((duration - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points.append(pt)
        self.arm_pub.publish(traj)
        self.get_logger().info(f"[ARM] Moving to {pose} (duration {duration}s)")
        time.sleep(duration + 0.5)

    # ----------------- GRIPPER MOVEMENT -----------------
    def move_gripper(self, position, timeout_sec=TIMEOUT_SEC_ACTION):
        if self.gripper_action_available:
            goal_msg = GripperCommand.Goal()
            goal_msg.command.position = position
            goal_msg.command.max_effort = 1.0

            future = self.gripper_client.send_goal_async(goal_msg)

            start_time = self.get_clock().now()
            while not future.done():
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed > timeout_sec:
                    self.get_logger().error(f"Timed out waiting for gripper goal acceptance (pos={position})")
                    return False
                rclpy.spin_once(self, timeout_sec=0.05)

            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f"Gripper goal rejected (pos={position})")
                return False

            result_future = goal_handle.get_result_async()
            start_time = self.get_clock().now()
            while not result_future.done():
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed > timeout_sec:
                    self.get_logger().error(f"Timed out waiting for gripper result (pos={position})")
                    return False
                rclpy.spin_once(self, timeout_sec=0.05)

            result = result_future.result()
            status = result.status
            reached_goal = result.result.reached_goal

            if status == 3:
                self.get_logger().info(f"Gripper moved to {position} successfully")
                return True
            elif status == 4 and reached_goal:
                self.get_logger().warn(f"Gripper action aborted but physically reached goal at {position}")
                return True
            else:
                self.get_logger().error(f"Gripper failed: status={status}, reached_goal={reached_goal}")
                return False
        else:
            msg = Float64()
            msg.data = position
            self.gripper_pub.publish(msg)
            self.get_logger().info(f"Gripper moved to {position} via topic fallback")
            time.sleep(1.0)
            return True

    # ----------------- TASK SEQUENCE -----------------
    def _execute_task_sequence(self):
        try:
            # Home
            self.publish_arm_pose(POSE_HOME, 2.0)
            # Ready above cube
            self.publish_arm_pose(POSE_ABOVE, 3.0)
            # Lower
            self.publish_arm_pose(POSE_LOWER, 3.0)
            # Close gripper
            self.move_gripper(GRIP_CLOSE)
            time.sleep(0.8)
            # Lift
            self.publish_arm_pose(POSE_LIFT, 3.0)
            # Move to place
            self.publish_arm_pose(POSE_PLACE, 3.0)
            # Open gripper
            self.move_gripper(GRIP_OPEN)
            time.sleep(0.8)
            # Return home
            self.publish_arm_pose(POSE_HOME, 2.0)
            self.get_logger().info("Pick-and-place sequence completed.")
        except Exception as e:
            self.get_logger().error(f"Exception in task sequence: {e}")

    # ----------------- SERVICE CALLBACK -----------------
    def handle_pick_place_request(self, request, response):
        self.get_logger().info("Received /perform_task request, starting background sequence")
        t = threading.Thread(target=self._execute_task_sequence, daemon=True)
        t.start()
        response.success = True
        response.message = "Pick-and-place task started"
        return response

# ----------------- MAIN -----------------
def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()










# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from control_msgs.action import GripperCommand
# from rclpy.action import ActionClient
# from perform_task_interface.srv import GripMove
# import time
# from std_msgs.msg import Float64  # For fallback

# class PickPlaceServer(Node):
#     def __init__(self):
#         super().__init__('pick_place_service_node')

#         # Arm publisher
#         self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

#         # Gripper action client
#         self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
#         self.gripper_action_available = self.gripper_client.wait_for_server(timeout_sec=2.0)
#         if self.gripper_action_available:
#             self.get_logger().info("Gripper action server found and ready.")
#         else:
#             self.get_logger().warn("Gripper action server NOT found. Using direct topic control as fallback.")
#             self.gripper_pub = self.create_publisher(Float64, '/gripper_controller/command', 10)

#         # Define joint names
#         self.joint_names = ["joint1", "joint2", "joint3", "joint4"]

#         # Predefined positions
#         self.actions = {
#             "home": [0.0, 0.0, 0.0, 0.0],
#             "ready": [0.0, 0.8, -0.6, 0.0],
#             "pick": [0.0, 1.0, -0.7, 0.0],
#             "place": [0.4, 0.6, -0.5, 0.2],
#         }

#         # Gripper positions
#         self.gripper_open = 0.015
#         self.gripper_closed = 0.0

#         # Initialize gripper to open safely
#         self.move_gripper(self.gripper_open)
#         self.get_logger().info("Gripper initialized to open position")

#         # Service
#         self.srv = self.create_service(GripMove, '/perform_task', self.handle_pick_place_request)

#     # ----------------- ARM MOVEMENT -----------------
#     def send_joint_positions(self, positions, duration=3.0):
#         traj = JointTrajectory()
#         traj.joint_names = self.joint_names
#         point = JointTrajectoryPoint()
#         point.positions = positions
#         point.time_from_start.sec = int(duration)
#         traj.points.append(point)
#         self.arm_pub.publish(traj)
#         self.get_logger().info(f"Moving arm to {positions}")
#         time.sleep(duration + 0.5)

#     # ----------------- GRIPPER MOVEMENT -----------------
#     def move_gripper(self, position, timeout_sec=5.0):
#         """
#         Move the gripper safely.
#         - Uses action client if available.
#         - Falls back to direct topic if action server is missing.
#         - Returns True if gripper moved.
#         """
#         if self.gripper_action_available:
#             # Use ActionClient
#             goal_msg = GripperCommand.Goal()
#             goal_msg.command.position = position
#             goal_msg.command.max_effort = 1.0

#             future = self.gripper_client.send_goal_async(goal_msg)

#             start_time = self.get_clock().now()
#             while not future.done():
#                 elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
#                 if elapsed > timeout_sec:
#                     self.get_logger().error(f"Timed out waiting for goal acceptance (position={position})")
#                     return False
#                 rclpy.spin_once(self, timeout_sec=0.1)

#             goal_handle = future.result()
#             if not goal_handle.accepted:
#                 self.get_logger().error(f"Gripper goal rejected (position={position})")
#                 return False

#             # Wait for result
#             result_future = goal_handle.get_result_async()
#             start_time = self.get_clock().now()
#             while not result_future.done():
#                 elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
#                 if elapsed > timeout_sec:
#                     self.get_logger().error(f"Timed out waiting for result (position={position})")
#                     return False
#                 rclpy.spin_once(self, timeout_sec=0.1)

#             result = result_future.result()
#             status = result.status
#             reached_goal = result.result.reached_goal

#             if status == 3:  # SUCCEEDED
#                 self.get_logger().info(f"Gripper moved to {position} successfully")
#                 return True
#             elif status == 4 and reached_goal:  # ABORTED but physically reached goal
#                 self.get_logger().warn(f"Gripper action aborted but reached goal at {position}")
#                 return True
#             else:
#                 self.get_logger().error(f"Gripper movement failed: status={status}, reached_goal={reached_goal}")
#                 return False
#         else:
#             # Fallback: publish directly
#             msg = Float64()
#             msg.data = position
#             self.gripper_pub.publish(msg)
#             self.get_logger().info(f"Gripper moved to {position} via direct topic")
#             time.sleep(1.0)  # small delay for Gazebo to apply
#             return True

#     # ----------------- SERVICE CALLBACK -----------------
#     def handle_pick_place_request(self, request, response):
#         action = request.action.lower().strip()
#         try:
#             self.get_logger().info(f"Executing action: {action}")

#             if action == "home":
#                 self.send_joint_positions(self.actions["home"])
#             elif action == "ready":
#                 self.send_joint_positions(self.actions["ready"])
#             elif action == "pick":
#                 self.send_joint_positions(self.actions["pick"])
#             elif action == "close":
#                 success = self.move_gripper(self.gripper_closed)
#                 if not success:
#                     raise Exception("Gripper failed to close.")
#             elif action == "lift":
#                 self.send_joint_positions([0.0, 0.7, -0.5, 0.0])
#             elif action == "place":
#                 self.send_joint_positions(self.actions["place"])
#             elif action == "open":
#                 success = self.move_gripper(self.gripper_open)
#                 if not success:
#                     raise Exception("Gripper failed to open.")
#             elif action == "finish":
#                 self.send_joint_positions(self.actions["home"])
#             else:
#                 response.success = False
#                 response.message = f"Unknown command '{action}'"
#                 return response

#             response.success = True
#             response.message = f"Action '{action}' completed successfully."
#             return response

#         except Exception as e:
#             response.success = False
#             response.message = str(e)
#             self.get_logger().error(str(e))
#             return response

# # ----------------- MAIN -----------------
# def main(args=None):
#     rclpy.init(args=args)
#     node = PickPlaceServer()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
