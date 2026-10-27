#!/usr/bin/env python3
import math
import threading
import time
from collections import deque
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Action / service imports
from control_msgs.action import GripperCommand as GripperCommandAction
from control_msgs.msg import GripperCommand as GripperCommandMsg
from pick_place_ap_interface.action import PickPlace

# -----------------------------
# Config
# -----------------------------
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
STEP_TIMEOUT = 12.0
GRIPPER_TIMEOUT = 8.0

class FeedbackControlActionWrapper(Node):
    def __init__(self):
        super().__init__("feedback_control_action_wrapper")
        self.get_logger().error(">>> I AM THE NEW CODE - VERSION 2.0 <<<") 

        # Use ReentrantCallbackGroup so action execution doesn't block cancel callbacks
        self.cb_group = ReentrantCallbackGroup()

        # Publisher/subscriber
        self.arm_pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_cb, 10, callback_group=self.cb_group
        )

        # Gripper action client
        self.gripper_client = ActionClient(self, GripperCommandAction, GRIPPER_ACTION, callback_group=self.cb_group)

        # Action server
        self._action_server = ActionServer(
            self,
            PickPlace,
            "pick_place_action",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.latest_js_map = None
        self._cancel_event = threading.Event()
        self.get_logger().info("Action Server Ready. Use MultiThreadedExecutor.")

    def joint_state_cb(self, msg: JointState):
        self.latest_js_map = {n: p for n, p in zip(msg.name, msg.position)}

    def current_positions(self) -> Optional[List[float]]:
        if not self.latest_js_map: return None
        return [self.latest_js_map.get(j, 0.0) for j in JOINTS]

    # ----------------------------
    # Helpers
    # ----------------------------
    def publish_pose(self, positions: List[float], duration: float = 2.0):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        msg.points = [pt]
        self.arm_pub.publish(msg)

    def wait_for_convergence(self, target: List[float], timeout_sec: float = STEP_TIMEOUT) -> Tuple[bool, str]:
        """
        Waits for joints to reach target. 
        Note: No rclpy.spin_once() here! We rely on the Executor in main() to update joint_states.
        """
        start_time = self.get_clock().now()
        rate = self.create_rate(20) # 20Hz check

        while rclpy.ok():
            # 1. Check Cancel
            if self._cancel_event.is_set():
                return False, "canceled"

            # 2. Check Timeout
            now = self.get_clock().now()
            if (now - start_time).nanoseconds / 1e9 > timeout_sec:
                return False, "timeout"

            # 3. Check Position
            cur = self.current_positions()
            if cur:
                errs = [abs(cur[i] - target[i]) for i in range(len(JOINTS))]
                if max(errs) <= EPS:
                    return True, "converged"
            
            # Sleep to yield to other threads (like Cancel Callback)
            rate.sleep()
            
        return False, "shutdown"

    def gripper_set(self, position: float) -> Tuple[bool, str]:
        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            return False, "no_server"

        goal = GripperCommandAction.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 1.0

        send_fut = self.gripper_client.send_goal_async(goal)
        
        # Wait for accept
        while not send_fut.done():
            if self._cancel_event.is_set(): return False, "canceled"
            time.sleep(0.05)
            
        goal_handle = send_fut.result()
        if not goal_handle.accepted: return False, "rejected"

        # Wait for result
        res_fut = goal_handle.get_result_async()
        while not res_fut.done():
            if self._cancel_event.is_set(): return False, "canceled"
            time.sleep(0.05)
            
        return True, "done"

    # ----------------------------
    # Action Callbacks
    # ----------------------------
    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Received Cancel Request!")
        self._cancel_event.set() # Flag the running thread
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self._cancel_event.clear()
        feedback = PickPlace.Feedback()
        result = PickPlace.Result()
        
        steps = [
            ("home", "pose", POSE_HOME),
            ("ready", "pose", POSE_ABOVE),
            ("pick", "pose", POSE_LOWER),
            ("close", "grip", GRIP_CLOSE),
            ("lift", "pose", POSE_LIFT),
            ("place", "pose", POSE_PLACE),
            ("open", "grip", GRIP_OPEN),
            ("home", "pose", POSE_HOME),
        ]

        success = True

        for idx, (name, stype, target) in enumerate(steps, 1):
            # Check cancel before starting step
            if self._cancel_event.is_set():
                success = False
                break

            # Publish Feedback
            feedback.current_step = f"[{idx}/8] {name}"
            feedback.status = "running"
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Executing: {feedback.current_step}")

            # Execute Step
            if stype == "pose":
                self.publish_pose(target)
                ok, reason = self.wait_for_convergence(target)
            else:
                ok, reason = self.gripper_set(target)

            # Check result
            if not ok or self._cancel_event.is_set():
                self.get_logger().warn(f"Step failed or canceled: {reason}")
                success = False
                break

        if success:
            result.success = True
            result.message = "Complete"
            goal_handle.succeed()
        else:
            # UNWIND SEQUENCE
            self.get_logger().info("Performing Safe Unwind to HOME...")
            feedback.current_step = "UNWINDING"
            feedback.status = "moving_home"
            goal_handle.publish_feedback(feedback)
            
            # Reset cancel event locally so we can move again
            # (Note: In a complex system, you might process a hard-stop differently)
            # self._cancel_event.clear() 
            # Force move home ignores checking _cancel_event in this simple demo
            self.publish_pose(POSE_HOME, duration=4.0)
            
            # Simple wait (blocking) for safety
            time.sleep(4.0)
            
            result.success = False
            result.message = "Aborted/Canceled -> Safe Pose Reached"
            goal_handle.canceled()

        return result

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackControlActionWrapper()
    
    # CRITICAL: Use MultiThreadedExecutor so callbacks (Cancel/JointState) 
    # run in parallel to the execute_callback loop.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()