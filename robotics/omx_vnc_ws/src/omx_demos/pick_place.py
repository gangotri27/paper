#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from collections import deque

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand as GripperCommandAction
from control_msgs.msg import GripperCommand as GripperCommandMsg
from rclpy.action import ActionClient


# ================================
# CONFIG (YOUR SAME VALUES)
# ================================

TRAJ_TOPIC = "/arm_controller/joint_trajectory"
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"
JOINTS = ["joint1", "joint2", "joint3", "joint4"]

POSE_HOME  = [0.0, 0.0, 0.0, 0.0]
POSE_ABOVE = [0.0, 0.8, -0.6, 0.0]
POSE_LOWER = [0.0, 1.0, -0.7, 0.0]
POSE_LIFT  = [0.0, 0.7, -0.5, 0.0]
POSE_PLACE = [0.4, 0.6, -0.5, 0.2]

GRIP_CLOSE = 0.0
GRIP_OPEN  = 0.015

EPS = 0.03
CYCLE = 0.05
N_CONSEC = 5


# ================================
# SIMPLE PICK & PLACE NODE
# ================================

class SimplePickPlace(Node):

    def __init__(self):
        super().__init__("simple_pick_place")

        self.arm_pub = self.create_publisher(
            JointTrajectory, TRAJ_TOPIC, 10)

        self.js_sub = self.create_subscription(
            JointState, "/joint_states",
            self.joint_state_cb, 10)

        self.gripper_client = ActionClient(
            self, GripperCommandAction, GRIPPER_ACTION)

        self.latest_js_map = None
        self.js_hist = deque(maxlen=50)

        self.get_logger().info("Simple Pick & Place Started")

        # Wait a bit for joint states
        self.wait_for_joint_state()

        # Execute sequence
        self.run_sequence()

    # --------------------------------
    # Joint State Callback
    # --------------------------------
    def joint_state_cb(self, msg):
        self.latest_js_map = {
            n: p for n, p in zip(msg.name, msg.position)
        }

    def wait_for_joint_state(self):
        self.get_logger().info("Waiting for joint states...")
        while rclpy.ok() and self.latest_js_map is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Joint states received.")

    def current_positions(self):
        try:
            return [self.latest_js_map[j] for j in JOINTS]
        except:
            return None

    # --------------------------------
    # Publish Pose
    # --------------------------------
    def move_to(self, positions, duration=2.0):

        self.get_logger().info(f"Moving to: {positions}")

        msg = JointTrajectory()
        msg.joint_names = JOINTS

        pt = JointTrajectoryPoint()
        pt.positions = positions

        sec = int(math.floor(duration))
        nsec = int((duration - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)

        msg.points = [pt]
        self.arm_pub.publish(msg)

        self.wait_until_reached(positions)

    # --------------------------------
    # Convergence Check
    # --------------------------------
    def wait_until_reached(self, target):

        consec = 0

        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=CYCLE)

            cur = self.current_positions()
            if cur is None:
                continue

            errors = [abs(cur[i] - target[i]) for i in range(4)]
            max_err = max(errors)

            consec = consec + 1 if max_err <= EPS else 0

            if consec >= N_CONSEC:
                self.get_logger().info("Reached target.")
                break

    # --------------------------------
    # Gripper Control
    # --------------------------------
    def set_gripper(self, position):

        self.get_logger().info(f"Setting gripper: {position}")

        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Gripper server not available")
            return

        goal = GripperCommandAction.Goal()
        goal.command = GripperCommandMsg()
        goal.command.position = float(position)
        goal.command.max_effort = 1.0

        send_fut = self.gripper_client.send_goal_async(goal)

        while not send_fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        goal_handle = send_fut.result()
        result_fut = goal_handle.get_result_async()

        while not result_fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info("Gripper done.")

    # --------------------------------
    # Full Pick & Place Sequence
    # --------------------------------
    def run_sequence(self):

        self.get_logger().info("Starting Pick & Place Sequence")

        self.move_to(POSE_HOME)
        self.move_to(POSE_ABOVE)
        self.move_to(POSE_LOWER)
        self.set_gripper(GRIP_CLOSE)
        self.move_to(POSE_LIFT)
        self.move_to(POSE_PLACE)
        self.set_gripper(GRIP_OPEN)
        self.move_to(POSE_HOME)

        self.get_logger().info("Pick & Place Completed ✅")


# ================================
# MAIN
# ================================

def main(args=None):
    rclpy.init(args=args)
    node = SimplePickPlace()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
