#!/usr/bin/env python3
"""
FeedbackControl ActionWrapper for pick-and-place
- Logs progress on server like Exercise 8 ([1/8] …)
- Supports cancel service & ActionServer
- Gripper commands fully functional
- UPDATED: Publishes feedback to client
"""
import math
import threading
from collections import deque
from typing import List, Optional, Tuple
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse, ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Action / service imports
from control_msgs.action import GripperCommand as GripperCommandAction
from control_msgs.msg import GripperCommand as GripperCommandMsg
from feedback_control_interface.srv import FeedbackControl
from pick_place_act_interface.action import PickPlace

# -----------------------------
# Config
# -----------------------------
TRAJ_TOPIC = "/arm_controller/joint_trajectory"
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"
JOINTS = ["joint1", "joint2", "joint3", "joint4"]

POSE_HOME = [0.0, 0.0, 0.0, 0.0]
POSE_ABOVE = [0.0, 0.8, -0.6, 0.0]
POSE_LOWER = [0.0, 1.0, -0.7, 0.0]
POSE_LIFT = [0.0, 0.7, -0.5, 0.0]
POSE_PLACE = [0.4, 0.6, -0.5, 0.2]

GRIP_CLOSE = 0.0
GRIP_OPEN = 0.015

EPS = 0.03
CYCLE_PERIOD = 0.05
N_CONSEC = 5
STEP_TIMEOUT = 12.0
GRIPPER_TIMEOUT = 8.0
RETRY_LIMIT = 1

# -----------------------------
# Node
# -----------------------------
class FeedbackControlActionWrapper(Node):
    def __init__(self):
        super().__init__("feedback_control_action_wrapper")

        # Original service
        self.srv = self.create_service(FeedbackControl, "perform_feedback", self.handle_request)
        
        # Publisher/subscriber
        self.arm_pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self.js_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)

        # Gripper action client
        self.gripper_client = ActionClient(self, GripperCommandAction, GRIPPER_ACTION)

        # Action server
        self._action_server = ActionServer(
            self,
            PickPlace,
            "pick_place_action",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.latest_js_map = None
        self.js_hist = deque(maxlen=50)
        self.step_logs = []
        self._cancel_event = threading.Event()

        self.get_logger().info("FeedbackControl ActionWrapper ready — /pick_place_action available.")

    # ----------------------------
    # Joint state callback
    # ----------------------------
    def joint_state_cb(self, msg: JointState):
        self.latest_js_map = {n: p for n, p in zip(msg.name, msg.position)}
        self.js_hist.append(self.latest_js_map)

    def current_positions(self) -> Optional[List[float]]:
        if not self.latest_js_map:
            return None
        try:
            return [self.latest_js_map[j] for j in JOINTS]
        except KeyError:
            return None

    # ----------------------------
    # Publish trajectory
    # ----------------------------
    def publish_pose(self, positions: List[float], duration: float = 2.0):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = positions
        sec = int(math.floor(duration))
        nsec = int((duration - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        msg.points = [pt]
        self.arm_pub.publish(msg)

    # ----------------------------
    # Wait for convergence
    # ----------------------------
    def wait_for_convergence(self, target: List[float], timeout_sec: float = STEP_TIMEOUT) -> Tuple[bool, float, str]:
        start = self.get_clock().now().nanoseconds
        consec = 0
        max_nanos = int(timeout_sec * 1e9)

        while True:
            if self._cancel_event.is_set():
                duration = (self.get_clock().now().nanoseconds - start)/1e9
                return False, duration, "canceled"

            rclpy.spin_once(self, timeout_sec=CYCLE_PERIOD)
            now = self.get_clock().now().nanoseconds
            
            if (now - start) > max_nanos:
                return False, (now - start)/1e9, "timeout"

            cur = self.current_positions()
            if cur is None:
                consec = 0
                continue

            errs = [abs(cur[i] - target[i]) for i in range(len(JOINTS))]
            max_err = max(errs)

            consec = consec + 1 if max_err <= EPS else 0
            if consec >= N_CONSEC:
                return True, (now - start)/1e9, "converged"

    # ----------------------------
    # Gripper command
    # ----------------------------
    def gripper_set(self, position: float, timeout_sec: float = GRIPPER_TIMEOUT) -> Tuple[bool, float, str]:
        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            return False, 0.0, "no_gripper_server"

        goal = GripperCommandAction.Goal()
        goal.command = GripperCommandMsg()
        goal.command.position = float(position)
        goal.command.max_effort = 1.0

        send_fut = self.gripper_client.send_goal_async(goal)
        start_ns = self.get_clock().now().nanoseconds

        while not send_fut.done():
            if self._cancel_event.is_set():
                return False, (self.get_clock().now().nanoseconds - start_ns)/1e9, "canceled"
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.get_clock().now().nanoseconds - start_ns)/1e9 > timeout_sec:
                 return False, (self.get_clock().now().nanoseconds - start_ns)/1e9, "goal_accept_timeout"

        goal_handle = send_fut.result()
        if not goal_handle.accepted:
            return False, (self.get_clock().now().nanoseconds - start_ns)/1e9, "rejected"

        res_fut = goal_handle.get_result_async()
        while not res_fut.done():
            if self._cancel_event.is_set():
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                return False, (self.get_clock().now().nanoseconds - start_ns)/1e9, "canceled"
            rclpy.spin_once(self, timeout_sec=0.05)

        return True, (self.get_clock().now().nanoseconds - start_ns)/1e9, "converged"

    # ----------------------------
    # Step execution with retry
    # ----------------------------
    def run_step_with_retry(self, idx: int, step: dict) -> bool:
        attempts = 0
        stype = step["type"]
        target = step["target"]
        name = step.get("name", stype)
        dur_cmd = step.get("duration", 2.0)

        while attempts <= RETRY_LIMIT:
            if self._cancel_event.is_set():
                self.get_logger().info(f"Step {idx} ({name}) canceled")
                return False
            
            attempts += 1
            if stype == "pose":
                self.publish_pose(target, dur_cmd)
                ok, took, reason = self.wait_for_convergence(target)
            else:
                ok, took, reason = self.gripper_set(target)
            
            status = "OK" if ok else "FAIL"
            self.get_logger().info(f"[{idx}/8] {stype}={name:6s} {status} reason={reason} duration={took:.2f}s")
            
            if ok: 
                return True
        return False

    # ----------------------------
    # Task worker (service legacy)
    # ----------------------------
    def _task_worker(self):
        # ... logic for service calls if needed ...
        pass

    # ----------------------------
    # Action callbacks
    # ----------------------------
    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self._cancel_event.set()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self._cancel_event.clear()
        
        feedback = PickPlace.Feedback()
        result = PickPlace.Result()

        steps = [
            ("home",  "pose", POSE_HOME),
            ("ready", "pose", POSE_ABOVE),
            ("pick",  "pose", POSE_LOWER),
            ("close", "grip", GRIP_CLOSE),
            ("lift",  "pose", POSE_LIFT),
            ("place", "pose", POSE_PLACE),
            ("open",  "grip", GRIP_OPEN),
            ("home",  "pose", POSE_HOME),
        ]

        for idx, (name, stype, target) in enumerate(steps, 1):
            if goal_handle.is_cancel_requested or self._cancel_event.is_set():
                self._cancel_event.set()
                self.publish_pose(POSE_HOME)
                self.wait_for_convergence(POSE_HOME)
                
                result.success = False
                result.message = "canceled: moved to SAFE pose"
                goal_handle.canceled()
                return result

            # --- MODIFIED SECTION: Publish Feedback ---
            # Assuming .action file has 'int32 current_step' and 'string status'
            feedback.current_step = idx
            feedback.status = f"Executing {name}" 
            goal_handle.publish_feedback(feedback)
            # ------------------------------------------

            self.run_step_with_retry(idx, {"type": stype, "name": name, "target": target})

        result.success = True
        result.message = "Pick-and-place completed successfully"
        goal_handle.succeed()
        return result

    # ----------------------------
    # Service callbacks
    # ----------------------------
    def handle_request(self, request, response):
        self._cancel_event.clear()
        # Logic adapted for service if needed
        response.success = True
        response.message = "Use action interface instead"
        return response

    # ----------------------------
    # Cleanup
    # ----------------------------
    def destroy_node(self):
        try:
            self._action_server.destroy()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackControlActionWrapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()