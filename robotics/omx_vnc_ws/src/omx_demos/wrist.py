#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading

# MoveIt
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint
from moveit_msgs.msg import MoveItErrorCodes, JointConstraint

# Gripper
from control_msgs.action import GripperCommand

# Geometry
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

# Joint states
from sensor_msgs.msg import JointState


class GraspContainer(Node):
    def __init__(self):
        super().__init__("grasp_container")

        # ------------------------
        # STATE
        # ------------------------
        self.robot_busy = False
        self.side = 1.0
        self.current_joints = {}

        # ------------------------
        # SUBSCRIBER (THE MISSING PIECE)
        # ------------------------
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10
        )

        # ------------------------
        # ACTION CLIENTS
        # ------------------------
        self.moveit_client = ActionClient(self, MoveGroup, "/move_action")
        self.gripper_client = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )

        self.get_logger().info("Waiting for MoveIt...")
        self.moveit_client.wait_for_server()

        self.get_logger().info("Waiting for gripper...")
        self.gripper_client.wait_for_server()

        # ------------------------
        # TERMINAL THREAD
        # ------------------------
        thread = threading.Thread(target=self.terminal_loop, daemon=True)
        thread.start()

    # ======================================================
    # JOINT STATE CALLBACK
    # ======================================================
    def joint_state_cb(self, msg):
        self.current_joints = dict(zip(msg.name, msg.position))

    # ======================================================
    # TERMINAL INPUT LOOP
    # ======================================================
    def terminal_loop(self):
        while rclpy.ok():
            user_input = input(
                "\nEnter color (blue / yellow) or 'exit': "
            ).strip().lower()

            if user_input == "exit":
                self.get_logger().info("Exiting node.")
                rclpy.shutdown()
                return

            if user_input not in ["blue", "yellow"]:
                self.get_logger().warn("Invalid input.")
                continue

            if self.robot_busy:
                self.get_logger().warn("Robot busy.")
                continue

            self.side = 1.0 if user_input == "blue" else -1.0
            self.robot_busy = True

            self.get_logger().info(f"Executing grasp on {user_input.upper()} side")
            success = self.execute_grasp()

            if success:
                self.get_logger().info("Grasp completed successfully.")
            else:
                self.get_logger().error("Grasp failed.")

            self.robot_busy = False

    # ======================================================
    # MAIN SEQUENCE (UNCHANGED)
    # ======================================================
    def execute_grasp(self):
        self.get_logger().info("STEP 1: Going to ready position")
        if not self.move_to_grasp_pose(0.20, 0.09, 0.12):
            return False
        rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info("STEP 2: Opening gripper")
        if not self.control_gripper(open=True):
            return False
        rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info("STEP 3: Moving to grasp pose")
        if not self.move_to_grasp_pose(0.24, 0.13, 0.12):
            return False
        rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info("STEP 4: Closing gripper")
        if not self.control_gripper(open=False):
            return False
        rclpy.spin_once(self, timeout_sec=0.5)

        print("Grasp completed successfully.")

        return self.pour_liquid()

    # ======================================================
    # POUR LIQUID
    # ======================================================
    def pour_liquid(self):
        self.get_logger().info("STEP 5: Moving to pour position")
        if not self.move_to_grasp_pose(0.24, 0.0, 0.12):
            return False
        rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info("STEP 6: Rotating wrist to pour")
        if not self.move_wrist(1.57,15.0):
            return False
        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("STEP 6: Rotating wrist back to init")
        if not self.move_wrist(0.0,15.0):
            return False
        rclpy.spin_once(self, timeout_sec=0.5)
        return self.release_container()
    
    def release_container(self):
        if not self.move_to_grasp_pose(0.24, 0.13, 0.12):
            return False
        rclpy.spin_once(self, timeout_sec=0.5)
        if not self.control_gripper(open = True):
            return False
        rclpy.spin_once(self, timeout_sec=0.3)
        return True
    
    # ======================================================
    # MOVEIT CARTESIAN MOTION
    # ======================================================
    def move_to_grasp_pose(self, a, b, c):
        x, y, z = a, b * self.side, c
        self.get_logger().info(f"Target → x={x}, y={y}, z={z}")

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        pos_constraint = PositionConstraint()
        pos_constraint.header = pose.header
        pos_constraint.link_name = "end_effector_link"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]

        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(pose.pose)
        pos_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)

        req = MotionPlanRequest()
        req.group_name = "arm"
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = 0.03
        req.max_acceleration_scaling_factor = 0.03
        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req

        send_goal = self.moveit_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal)

        goal_handle = send_goal.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("MoveIt goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return result_future.result().result.error_code.val == MoveItErrorCodes.SUCCESS

    # ======================================================
    # GRIPPER CONTROL
    # ======================================================
    def control_gripper(self, open=True):
        goal = GripperCommand.Goal()
        goal.command.position = 0.010 if open else -0.09
        goal.command.max_effort = 8.0

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        return result.reached_goal or (not open and result.stalled)

    # ======================================================
    # MOVE WRIST (FIXED, LOGGED)
    # ======================================================
    def move_wrist(self, angle, time_sec):
        self.get_logger().info(f"Rotating wrist to {angle} rad")

        joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5_roll"
        ]

        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while not all(j in self.current_joints for j in joint_names):
            if self.get_clock().now().seconds_nanoseconds()[0] - start_time > 2:
                self.get_logger().error("Joint states not available")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        constraints = Constraints()

        for j in joint_names:
            jc = JointConstraint()
            jc.joint_name = j
            jc.position = angle if j == "joint5_roll" else self.current_joints[j]
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = "arm"
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 5

        scale = min(1.0, max(0.05, 1.0 / time_sec))
        req.max_velocity_scaling_factor = scale
        req.max_acceleration_scaling_factor = scale

        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req

        send_goal = self.moveit_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal)

        goal_handle = send_goal.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Wrist goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        success = (
            result_future.result().result.error_code.val
            == MoveItErrorCodes.SUCCESS
        )

        self.get_logger().info("Wrist motion done" if success else "Wrist motion failed")
        return success


def main():
    rclpy.init()
    node = GraspContainer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()