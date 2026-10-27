#!/usr/bin/env python3
"""
moveit_basic.py - Feedback-driven pick & place
- Ensures gripper closes after pick and opens after place
- Waits for convergence of joints including gripper
"""

import math
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Optional imports (may fail if not installed)
try:
    from control_msgs.msg import GripperCommand as GripperCommandMsg
except Exception:
    GripperCommandMsg = None

try:
    from control_msgs.action import GripperCommand as GripperCommandAction
except Exception:
    GripperCommandAction = None

# --------------------------
# Parameters
# --------------------------
ARM_TOPIC = '/arm_controller/joint_trajectory'
JOINTS_ARM = ['joint1', 'joint2', 'joint3', 'joint4']

EPSILON = 0.05  # increased tolerance (~3°)
STABLE_CYCLES = 10
CONTROL_RATE = 20.0
PER_STEP_TIMEOUT = 12.0
RECOVERY_TIMEOUT = 12.0
MAX_RETRIES = 1

POSES = {
    'home':  [0.0, 0.0, 0.0, 0.0],
    'ready': [0.0, -0.5, 0.5, 0.0],
    'pick':  [0.0, 0.9, -0.45, 0.0],  # safer pick angles
    'place': [0.4, 0.6, -0.5, 0.2],
}

GRIP_OPEN = 0.0
GRIP_CLOSED = 0.8

SEQUENCE = [
    ('pose', 'home'),
    ('pose', 'ready'),
    ('pose', 'pick'),
    ('grip', 'close'),
    ('pose', 'place'),
    ('grip', 'open'),
    ('pose', 'home'),
]

# --------------------------
# Utilities
# --------------------------
def monotonic_time() -> float:
    return time.monotonic()


class FeedbackPickPlaceNode(Node):
    def __init__(self):
        super().__init__('exercise8_feedback_pick_place')
        self.get_logger().info("Starting Exercise 8 feedback pick & place node")

        # Publishers
        self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)

        # Gripper interfaces
        self.gripper_mode: Optional[str] = None
        self.gripper_pub = None
        self.gripper_action_client: Optional[ActionClient] = None
        self.gripper_action_name: Optional[str] = None

        # Joint states
        self.latest_js: Optional[JointState] = None
        self.jname_to_index = {}
        self.create_subscription(JointState, '/joint_states', self.cb_joint_states, 10)

        # Summary storage
        self.summary: List[Tuple[int, str, str, bool, str, float]] = []

        # Wait for first joint_states
        self._wait_for_first_joint_states(timeout=5.0)

        # Setup gripper interface
        self.detect_and_setup_gripper()

    # ----------------------------
    # Joint states
    # ----------------------------
    def cb_joint_states(self, msg: JointState):
        self.latest_js = msg
        if not self.jname_to_index and msg.name:
            self.jname_to_index = {n: i for i, n in enumerate(msg.name)}
            self.get_logger().info(f"Discovered joint names: {msg.name}")

    def _wait_for_first_joint_states(self, timeout: float = 5.0):
        start = monotonic_time()
        while self.latest_js is None and (monotonic_time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.latest_js:
            self.get_logger().info("Received initial /joint_states.")
        else:
            self.get_logger().warning("No /joint_states received yet; convergence checks will wait.")

    # ----------------------------
    # Gripper detection
    # ----------------------------
    def detect_and_setup_gripper(self):
        self.get_logger().info("Detecting gripper interface...")
        topics = self.get_topic_names_and_types()
        topic_map = {name: types for name, types in topics}

        # Try GripperCommand topic
        if GripperCommandMsg:
            for name, types in topic_map.items():
                for t in types:
                    if 'GripperCommand' in t:
                        from control_msgs.msg import GripperCommand as GCmsg
                        self.gripper_pub = self.create_publisher(GCmsg, name, 10)
                        self.gripper_mode = 'gripper_msg_topic'
                        self.get_logger().info(f"Gripper: using GripperCommand topic {name}")
                        return

        # Try Float64 topic
        for name, types in topic_map.items():
            for t in types:
                if 'Float64' in t:
                    from std_msgs.msg import Float64
                    self.gripper_pub = self.create_publisher(Float64, name, 10)
                    self.gripper_mode = 'float_topic'
                    self.get_logger().info(f"Gripper: using Float64 topic {name}")
                    return

        # Fallback: try action detection
        action_root = None
        for name in topic_map.keys():
            if '/_action/' in name and 'gripper' in name.lower():
                action_root = name.split('/_action/')[0]
                break

        if action_root:
            self.gripper_mode = 'action'
            self.gripper_action_name = action_root
            self.get_logger().info(f"Gripper: detected action root '{action_root}'")
            return

        self.gripper_mode = None
        self.get_logger().warning("No gripper interface detected; gripper commands will be skipped.")

    # ----------------------------
    # Joint positions & tolerance
    # ----------------------------
    def current_positions_for(self, names: List[str]) -> Optional[List[float]]:
        if self.latest_js is None:
            return None
        try:
            return [self.latest_js.position[self.jname_to_index[n]] for n in names]
        except KeyError:
            return None

    def within_tolerance(self, target_positions: List[float], controlled_joints: List[str]) -> bool:
        cur = self.current_positions_for(controlled_joints)
        if cur is None:
            return False
        return all(abs(c - t) <= EPSILON for c, t in zip(cur, target_positions))

    # ----------------------------
    # Arm commands
    # ----------------------------
    def build_and_publish_arm_trajectory(self, positions: List[float], time_from_start: float = 1.0):
        arm_joint_names = [n for n in JOINTS_ARM if n in self.jname_to_index]
        if not arm_joint_names and self.latest_js:
            names = [n for n in self.latest_js.name if not n.startswith('gripper')]
            arm_joint_names = names[-4:]
        jt = JointTrajectory()
        jt.joint_names = arm_joint_names
        p = JointTrajectoryPoint()
        p.positions = positions[:len(arm_joint_names)]
        sec = int(math.floor(time_from_start))
        nsec = int((time_from_start - sec) * 1e9)
        p.time_from_start.sec = sec
        p.time_from_start.nanosec = nsec
        jt.points = [p]
        self.arm_pub.publish(jt)

    # ----------------------------
    # Gripper commands
    # ----------------------------
    def send_gripper_command(self, value: float) -> bool:
        if self.gripper_mode == 'gripper_msg_topic' and self.gripper_pub:
            msg = GripperCommandMsg()
            msg.position = float(value)
            msg.max_effort = 0.0
            self.gripper_pub.publish(msg)
            return True

        if self.gripper_mode == 'float_topic' and self.gripper_pub:
            from std_msgs.msg import Float64
            m = Float64()
            m.data = float(value)
            self.gripper_pub.publish(m)
            return True

        if self.gripper_mode == 'action' and self.gripper_action_name:
            if GripperCommandAction is None or GripperCommandMsg is None:
                return False
            if self.gripper_action_client is None:
                self.gripper_action_client = ActionClient(self, GripperCommandAction, self.gripper_action_name)
            if not self.gripper_action_client.wait_for_server(timeout_sec=2.0):
                return False
            goal_msg = GripperCommandAction.Goal()
            goal_msg.command = GripperCommandMsg()
            goal_msg.command.position = float(value)
            goal_msg.command.max_effort = 0.0
            send_future = self.gripper_action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=2.0)
            goal_handle = send_future.result()
            if not goal_handle.accepted:
                return False
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=PER_STEP_TIMEOUT)
            return True
        return False

    # ----------------------------
    # Convergence
    # ----------------------------
    def wait_for_convergence(self, target_positions: List[float], controlled_joints: List[str], timeout: float) -> Tuple[bool, float, str]:
        start = monotonic_time()
        consecutive = 0
        spin_timeout = 1.0 / max(1.0, CONTROL_RATE)

        while True:
            rclpy.spin_once(self, timeout_sec=spin_timeout)
            elapsed = monotonic_time() - start

            if self.latest_js is None:
                if elapsed > timeout:
                    return False, elapsed, 'timeout_no_joint_states'
                continue

            if self.within_tolerance(target_positions, controlled_joints):
                consecutive += 1
            else:
                consecutive = 0

            if consecutive >= STABLE_CYCLES:
                return True, monotonic_time() - start, 'converged'
            if elapsed > timeout:
                return False, elapsed, 'timeout'

    # ----------------------------
    # Recovery
    # ----------------------------
    def recover_to_home(self) -> Tuple[bool, float, str]:
        if 'home' not in POSES:
            return False, 0.0, 'no_home_pose'
        tgt = POSES['home']
        self.build_and_publish_arm_trajectory(tgt, time_from_start=1.5)
        arm_joints = [j for j in JOINTS_ARM if j in self.jname_to_index]
        if not arm_joints and self.latest_js:
            names = [n for n in self.latest_js.name if not n.startswith('gripper')]
            arm_joints = names[-4:]
        return self.wait_for_convergence(tgt, arm_joints, RECOVERY_TIMEOUT)

    # ----------------------------
    # Main sequence runner
    # ----------------------------
    def run_sequence(self):
        self.get_logger().info("Ready. Running sequence.")

        # Compute arm joints
        arm_joints = [j for j in JOINTS_ARM if j in self.jname_to_index]
        if not arm_joints and self.latest_js:
            names = [n for n in self.latest_js.name if not n.startswith('gripper')]
            arm_joints = names[-4:]
        if not arm_joints:
            arm_joints = JOINTS_ARM[:]
        self.get_logger().info(f"Arm-controlled joints: {arm_joints}")

        # Detect real gripper joint
        gripper_joint = None
        for gj in ('gripper_left_joint', 'gripper_right_joint', 'gripper_finger_joint'):
            if gj in self.jname_to_index:
                gripper_joint = gj
                break

        for step_idx, (stype, target) in enumerate(SEQUENCE, 1):
            step_tag = f"[{step_idx}/{len(SEQUENCE)}]"
            duration = 0.0
            ok = False
            reason = ''

            if stype == 'pose':
                if target not in POSES:
                    reason = 'unknown_pose'
                    self.summary.append((step_idx, stype, target, False, reason, 0.0))
                    continue
                tgt = POSES[target]

                # adjust time for bigger moves
                time_from_start = 6.0 if target == 'pick' else 4.0 if target == 'place' else 1.5

                attempt = 0
                while attempt <= MAX_RETRIES:
                    self.build_and_publish_arm_trajectory(tgt, time_from_start=time_from_start)
                    ok, dur, reason = self.wait_for_convergence(tgt, arm_joints, PER_STEP_TIMEOUT)
                    duration += dur
                    if ok:
                        break
                    attempt += 1
                if not ok:
                    rec_ok, rec_dur, rec_reason = self.recover_to_home()
                    duration += rec_dur
                    reason = f"{reason};recovery_{rec_reason}"

            elif stype == 'grip':
                gval = GRIP_OPEN if target == 'open' else GRIP_CLOSED
                sent = self.send_gripper_command(gval)
                if gripper_joint and sent:
                    ok, dur, reason = self.wait_for_convergence([gval], [gripper_joint], PER_STEP_TIMEOUT)
                    duration += dur
                else:
                    # fallback: assume STABLE_CYCLES
                    spin_timeout = 1.0 / max(1.0, CONTROL_RATE)
                    cycles = 0
                    start_wait = monotonic_time()
                    while cycles < STABLE_CYCLES:
                        rclpy.spin_once(self, timeout_sec=spin_timeout)
                        cycles += 1
                    duration += monotonic_time() - start_wait
                    ok = True
                    reason = 'assumed'

            status = "OK" if ok else "FAIL"
            self.get_logger().info(f"{step_tag} {stype}={target} {status} duration={duration:.2f}s reason={reason}")
            self.summary.append((step_idx, stype, target, ok, reason, duration))

        self.print_summary()

    # ----------------------------
    # Summary
    # ----------------------------
    def print_summary(self):
        self.get_logger().info("=== Final summary ===")
        self.get_logger().info(" idx | type  | target  | OK/FAIL | reason                  | duration(s)")
        for row in self.summary:
            idx, typ, target, ok, reason, dur = row
            flag = "OK" if ok else "FAIL"
            self.get_logger().info(f" {idx:2d}  | {typ:5s} | {str(target):7s} | {flag:6s} | {reason:22s} | {dur:7.2f}")


# ----------------------------
# Main entry
# ----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = FeedbackPickPlaceNode()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


























# #!/usr/bin/env python3
# """
# moveit_basic_fixed.py - Feedback-driven pick & place
# - Ensures gripper closes after pick and opens after place
# - Waits for joint convergence or action completion
# - Works with topic or action-based grippers
# """

# import math
# import time
# from typing import List, Optional, Tuple

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from sensor_msgs.msg import JointState
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# # Optional imports (may fail if not installed)
# try:
#     from control_msgs.msg import GripperCommand as GripperCommandMsg
# except Exception:
#     GripperCommandMsg = None

# try:
#     from control_msgs.action import GripperCommand as GripperCommandAction
# except Exception:
#     GripperCommandAction = None

# # --------------------------
# # Parameters
# # --------------------------
# ARM_TOPIC = '/arm_controller/joint_trajectory'
# JOINTS_ARM = ['joint1', 'joint2', 'joint3', 'joint4']

# EPSILON = 0.02
# STABLE_CYCLES = 10
# CONTROL_RATE = 20.0
# PER_STEP_TIMEOUT = 12.0
# RECOVERY_TIMEOUT = 12.0
# MAX_RETRIES = 1

# POSES = {
#     'home':  [0.0, 0.0, 0.0, 0.0],
#     'ready': [0.0, -0.5, 0.5, 0.0],
#     'pick':  [0.0, 1.0, -0.5, 0.0],
#     'place': [0.4, 0.6, -0.5, 0.2],
# }

# GRIP_OPEN = 0.0
# GRIP_CLOSED = 0.8

# SEQUENCE = [
#     ('pose', 'home'),
#     ('pose', 'ready'),
#     ('pose', 'pick'),
#     ('grip', 'close'),
#     ('pose', 'place'),
#     ('grip', 'open'),
#     ('pose', 'home'),
# ]

# # --------------------------
# # Utilities
# # --------------------------
# def monotonic_time() -> float:
#     return time.monotonic()


# class FeedbackPickPlaceNode(Node):
#     def __init__(self):
#         super().__init__('exercise8_feedback_pick_place')
#         self.get_logger().info("Starting Exercise 8 feedback pick & place node")

#         # Publishers
#         self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)

#         # Gripper interfaces
#         self.gripper_mode: Optional[str] = None
#         self.gripper_pub = None
#         self.gripper_action_client: Optional[ActionClient] = None
#         self.gripper_action_name: Optional[str] = None

#         # Joint states
#         self.latest_js: Optional[JointState] = None
#         self.jname_to_index = {}
#         self.create_subscription(JointState, '/joint_states', self.cb_joint_states, 10)

#         # Summary storage
#         self.summary: List[Tuple[int, str, str, bool, str, float]] = []

#         # Wait for first joint_states
#         self._wait_for_first_joint_states(timeout=5.0)

#         # Setup gripper interface
#         self.detect_and_setup_gripper()

#     # ----------------------------
#     # Joint states
#     # ----------------------------
#     def cb_joint_states(self, msg: JointState):
#         self.latest_js = msg
#         if not self.jname_to_index and msg.name:
#             self.jname_to_index = {n: i for i, n in enumerate(msg.name)}
#             self.get_logger().info(f"Discovered joint names: {msg.name}")

#     def _wait_for_first_joint_states(self, timeout: float = 5.0):
#         start = monotonic_time()
#         while self.latest_js is None and (monotonic_time() - start) < timeout:
#             rclpy.spin_once(self, timeout_sec=0.05)
#         if self.latest_js:
#             self.get_logger().info("Received initial /joint_states.")
#         else:
#             self.get_logger().warning("No /joint_states received yet; convergence checks will wait.")

#     # ----------------------------
#     # Gripper detection
#     # ----------------------------
#     def detect_and_setup_gripper(self):
#         self.get_logger().info("Detecting gripper interface...")
#         topics = self.get_topic_names_and_types()
#         topic_map = {name: types for name, types in topics}

#         # Try GripperCommand topic
#         if GripperCommandMsg:
#             for name, types in topic_map.items():
#                 for t in types:
#                     if 'GripperCommand' in t:
#                         from control_msgs.msg import GripperCommand as GCmsg
#                         self.gripper_pub = self.create_publisher(GCmsg, name, 10)
#                         self.gripper_mode = 'gripper_msg_topic'
#                         self.get_logger().info(f"Gripper: using GripperCommand topic {name}")
#                         return

#         # Try Float64 topic
#         for name, types in topic_map.items():
#             for t in types:
#                 if 'Float64' in t:
#                     from std_msgs.msg import Float64
#                     self.gripper_pub = self.create_publisher(Float64, name, 10)
#                     self.gripper_mode = 'float_topic'
#                     self.get_logger().info(f"Gripper: using Float64 topic {name}")
#                     return

#         # Fallback: try action detection
#         action_root = None
#         for name in topic_map.keys():
#             if '/_action/' in name and 'gripper' in name.lower():
#                 action_root = name.split('/_action/')[0]
#                 break

#         if action_root:
#             self.gripper_mode = 'action'
#             self.gripper_action_name = action_root
#             self.get_logger().info(f"Gripper: detected action root '{action_root}'")
#             return

#         self.gripper_mode = None
#         self.get_logger().warning("No gripper interface detected; gripper commands will be skipped.")

#     # ----------------------------
#     # Joint positions & tolerance
#     # ----------------------------
#     def current_positions_for(self, names: List[str]) -> Optional[List[float]]:
#         if self.latest_js is None:
#             return None
#         try:
#             return [self.latest_js.position[self.jname_to_index[n]] for n in names]
#         except KeyError:
#             return None

#     def within_tolerance(self, target_positions: List[float], controlled_joints: List[str]) -> bool:
#         cur = self.current_positions_for(controlled_joints)
#         if cur is None:
#             return False
#         return all(abs(c - t) <= EPSILON for c, t in zip(cur, target_positions))

#     # ----------------------------
#     # Arm commands
#     # ----------------------------
#     def build_and_publish_arm_trajectory(self, positions: List[float], time_from_start: float = 3.0):
#         arm_joint_names = [n for n in JOINTS_ARM if n in self.jname_to_index]
#         if not arm_joint_names and self.latest_js:
#             names = [n for n in self.latest_js.name if not n.startswith('gripper')]
#             arm_joint_names = names[-4:]
#         jt = JointTrajectory()
#         jt.joint_names = arm_joint_names
#         p = JointTrajectoryPoint()
#         p.positions = positions[:len(arm_joint_names)]
#         sec = int(math.floor(time_from_start))
#         nsec = int((time_from_start - sec) * 1e9)
#         p.time_from_start.sec = sec
#         p.time_from_start.nanosec = nsec
#         jt.points = [p]
#         self.arm_pub.publish(jt)

#     # ----------------------------
#     # Gripper commands
#     # ----------------------------
#     def send_gripper_command(self, value: float) -> bool:
#         if self.gripper_mode == 'action' and self.gripper_action_name:
#             if GripperCommandAction is None or GripperCommandMsg is None:
#                 return False
#             if self.gripper_action_client is None:
#                 self.gripper_action_client = ActionClient(self, GripperCommandAction, self.gripper_action_name)
#             if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
#                 self.get_logger().error("Gripper action server not available")
#                 return False

#             goal_msg = GripperCommandAction.Goal()
#             goal_msg.command = GripperCommandMsg()
#             goal_msg.command.position = float(value)
#             goal_msg.command.max_effort = 0.0

#             send_future = self.gripper_action_client.send_goal_async(goal_msg)
#             rclpy.spin_until_future_complete(self, send_future)
#             goal_handle = send_future.result()
#             if not goal_handle.accepted:
#                 self.get_logger().error("Gripper goal rejected")
#                 return False
#             get_result_future = goal_handle.get_result_async()
#             rclpy.spin_until_future_complete(self, get_result_future)
#             return True

#         if self.gripper_mode == 'gripper_msg_topic' and self.gripper_pub:
#             msg = GripperCommandMsg()
#             msg.position = float(value)
#             msg.max_effort = 0.0
#             self.gripper_pub.publish(msg)
#             return True

#         if self.gripper_mode == 'float_topic' and self.gripper_pub:
#             from std_msgs.msg import Float64
#             m = Float64()
#             m.data = float(value)
#             self.gripper_pub.publish(m)
#             return True

#         return False

#     # ----------------------------
#     # Convergence
#     # ----------------------------
#     def wait_for_convergence(self, target_positions: List[float], controlled_joints: List[str], timeout: float) -> Tuple[bool, float, str]:
#         start = monotonic_time()
#         consecutive = 0
#         spin_timeout = 1.0 / max(1.0, CONTROL_RATE)

#         while True:
#             rclpy.spin_once(self, timeout_sec=spin_timeout)
#             elapsed = monotonic_time() - start

#             if self.latest_js is None:
#                 if elapsed > timeout:
#                     return False, elapsed, 'timeout_no_joint_states'
#                 continue

#             if self.within_tolerance(target_positions, controlled_joints):
#                 consecutive += 1
#             else:
#                 consecutive = 0

#             if consecutive >= STABLE_CYCLES:
#                 return True, monotonic_time() - start, 'converged'
#             if elapsed > timeout:
#                 return False, elapsed, 'timeout'

#     # ----------------------------
#     # Recovery
#     # ----------------------------
#     def recover_to_home(self) -> Tuple[bool, float, str]:
#         if 'home' not in POSES:
#             return False, 0.0, 'no_home_pose'
#         tgt = POSES['home']
#         self.build_and_publish_arm_trajectory(tgt, time_from_start=3.0)
#         arm_joints = [j for j in JOINTS_ARM if j in self.jname_to_index]
#         if not arm_joints and self.latest_js:
#             names = [n for n in self.latest_js.name if not n.startswith('gripper')]
#             arm_joints = names[-4:]
#         return self.wait_for_convergence(tgt, arm_joints, RECOVERY_TIMEOUT)

#     # ----------------------------
#     # Main sequence runner
#     # ----------------------------
#     def run_sequence(self):
#         self.get_logger().info("Ready. Running sequence.")
#         arm_joints = [j for j in JOINTS_ARM if j in self.jname_to_index]
#         if not arm_joints and self.latest_js:
#             names = [n for n in self.latest_js.name if not n.startswith('gripper')]
#             arm_joints = names[-4:]
#         if not arm_joints:
#             arm_joints = JOINTS_ARM[:]
#         self.get_logger().info(f"Arm-controlled joints: {arm_joints}")

#         gripper_joint = None
#         for gj in ('gripper_left_joint', 'gripper_right_joint', 'gripper_finger_joint'):
#             if gj in self.jname_to_index:
#                 gripper_joint = gj
#                 break

#         for step_idx, (stype, target) in enumerate(SEQUENCE, 1):
#             step_tag = f"[{step_idx}/{len(SEQUENCE)}]"
#             duration = 0.0
#             ok = False
#             reason = ''

#             if stype == 'pose':
#                 if target not in POSES:
#                     reason = 'unknown_pose'
#                     self.summary.append((step_idx, stype, target, False, reason, 0.0))
#                     continue
#                 tgt = POSES[target]
#                 attempt = 0
#                 while attempt <= MAX_RETRIES:
#                     time_from_start = 3.0 if target in ['pick', 'place'] else 1.5
#                     self.build_and_publish_arm_trajectory(tgt, time_from_start=time_from_start)
#                     ok, dur, reason = self.wait_for_convergence(tgt, arm_joints, PER_STEP_TIMEOUT)
#                     duration += dur
#                     if ok:
#                         break
#                     attempt += 1
#                 if not ok:
#                     rec_ok, rec_dur, rec_reason = self.recover_to_home()
#                     duration += rec_dur
#                     reason = f"{reason};recovery_{rec_reason}"

#             elif stype == 'grip':
#                 gval = GRIP_OPEN if target == 'open' else GRIP_CLOSED
#                 sent = self.send_gripper_command(gval)
#                 if sent:
#                     # Wait a short time for action completion if gripper is action
#                     if self.gripper_mode == 'action':
#                         spin_timeout = 1.0 / max(1.0, CONTROL_RATE)
#                         cycles = 0
#                         start_wait = monotonic_time()
#                         while cycles < STABLE_CYCLES:
#                             rclpy.spin_once(self, timeout_sec=spin_timeout)
#                             cycles += 1
#                         duration += monotonic_time() - start_wait
#                     ok = True
#                     reason = 'converged'
#                 else:
#                     ok = False
#                     reason = 'no_gripper_interface'
#                     duration = 0.0

#             status = "OK" if ok else "FAIL"
#             self.get_logger().info(f"{step_tag} {stype}={target} {status} duration={duration:.2f}s reason={reason}")
#             self.summary.append((step_idx, stype, target, ok, reason, duration))

#         self.print_summary()

#     # ----------------------------
#     # Summary
#     # ----------------------------
#     def print_summary(self):
#         self.get_logger().info("=== Final summary ===")
#         self.get_logger().info(" idx | type  | target  | OK/FAIL | reason                  | duration(s)")
#         for row in self.summary:
#             idx, typ, target, ok, reason, dur = row
#             flag = "OK" if ok else "FAIL"
#             self.get_logger().info(f" {idx:2d}  | {typ:5s} | {str(target):7s} | {flag:6s} | {reason:22s} | {dur:7.2f}")


# # ----------------------------
# # Main entry
# # ----------------------------
# def main(args=None):
#     rclpy.init(args=args)
#     node = FeedbackPickPlaceNode()
#     try:
#         node.run_sequence()
#     except KeyboardInterrupt:
#         node.get_logger().info("Interrupted by user")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
























# #!/usr/bin/env python3
# """
# moveit_basic.py - Clean feedback-driven pick & place
# - Ensures gripper closes after pick and opens only after place
# - Waits for convergence of joints including gripper
# - Minimal imports, MoveIt-free
# """

# import math
# import time
# from typing import List, Optional, Tuple

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from sensor_msgs.msg import JointState
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# # Optional imports (may fail if not installed)
# try:
#     from control_msgs.msg import GripperCommand as GripperCommandMsg
# except Exception:
#     GripperCommandMsg = None

# try:
#     from control_msgs.action import GripperCommand as GripperCommandAction
# except Exception:
#     GripperCommandAction = None

# # --------------------------
# # Parameters
# # --------------------------
# ARM_TOPIC = '/arm_controller/joint_trajectory'
# JOINTS_ARM = ['joint1', 'joint2', 'joint3', 'joint4']

# EPSILON = 0.02
# STABLE_CYCLES = 10
# CONTROL_RATE = 20.0
# PER_STEP_TIMEOUT = 8.0
# RECOVERY_TIMEOUT = 12.0
# MAX_RETRIES = 1

# POSES = {
#     'home':  [0.0, 0.0, 0.0, 0.0],
#     'ready': [0.0, -0.5, 0.5, 0.0],
#     'pick':  [0.0, 1.0, -0.5, 0.0],
#     'place': [0.4, 0.6, -0.5, 0.2],
# }

# GRIP_OPEN = 0.0
# GRIP_CLOSED = 0.8

# SEQUENCE = [
#     ('pose', 'home'),
#     ('pose', 'ready'),
#     ('pose', 'pick'),
#     ('grip', 'close'),
#     ('pose', 'place'),
#     ('grip', 'open'),
#     ('pose', 'home'),
# ]

# # --------------------------
# # Utilities
# # --------------------------
# def monotonic_time() -> float:
#     return time.monotonic()


# class FeedbackPickPlaceNode(Node):
#     def __init__(self):
#         super().__init__('exercise8_feedback_pick_place')
#         self.get_logger().info("Starting Exercise 8 feedback pick & place node")

#         # Publishers
#         self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)

#         # Gripper interfaces
#         self.gripper_mode: Optional[str] = None
#         self.gripper_pub = None
#         self.gripper_action_client: Optional[ActionClient] = None
#         self.gripper_action_name: Optional[str] = None

#         # Joint states
#         self.latest_js: Optional[JointState] = None
#         self.jname_to_index = {}
#         self.create_subscription(JointState, '/joint_states', self.cb_joint_states, 10)

#         # Summary storage
#         self.summary: List[Tuple[int, str, str, bool, str, float]] = []

#         # Wait for first joint_states
#         self._wait_for_first_joint_states(timeout=5.0)

#         # Setup gripper interface
#         self.detect_and_setup_gripper()

#     # ----------------------------
#     # Joint states
#     # ----------------------------
#     def cb_joint_states(self, msg: JointState):
#         self.latest_js = msg
#         if not self.jname_to_index and msg.name:
#             self.jname_to_index = {n: i for i, n in enumerate(msg.name)}
#             self.get_logger().info(f"Discovered joint names: {msg.name}")

#     def _wait_for_first_joint_states(self, timeout: float = 5.0):
#         start = monotonic_time()
#         while self.latest_js is None and (monotonic_time() - start) < timeout:
#             rclpy.spin_once(self, timeout_sec=0.05)
#         if self.latest_js:
#             self.get_logger().info("Received initial /joint_states.")
#         else:
#             self.get_logger().warning("No /joint_states received yet; convergence checks will wait.")

#     # ----------------------------
#     # Gripper detection
#     # ----------------------------
#     def detect_and_setup_gripper(self):
#         self.get_logger().info("Detecting gripper interface...")
#         topics = self.get_topic_names_and_types()
#         topic_map = {name: types for name, types in topics}

#         # Try GripperCommand topic
#         if GripperCommandMsg:
#             for name, types in topic_map.items():
#                 for t in types:
#                     if 'GripperCommand' in t:
#                         from control_msgs.msg import GripperCommand as GCmsg
#                         self.gripper_pub = self.create_publisher(GCmsg, name, 10)
#                         self.gripper_mode = 'gripper_msg_topic'
#                         self.get_logger().info(f"Gripper: using GripperCommand topic {name}")
#                         return

#         # Try Float64 topic
#         for name, types in topic_map.items():
#             for t in types:
#                 if 'Float64' in t:
#                     from std_msgs.msg import Float64
#                     self.gripper_pub = self.create_publisher(Float64, name, 10)
#                     self.gripper_mode = 'float_topic'
#                     self.get_logger().info(f"Gripper: using Float64 topic {name}")
#                     return

#         # Fallback: try action detection
#         action_root = None
#         for name in topic_map.keys():
#             if '/_action/' in name and 'gripper' in name.lower():
#                 action_root = name.split('/_action/')[0]
#                 break

#         if action_root:
#             self.gripper_mode = 'action'
#             self.gripper_action_name = action_root
#             self.get_logger().info(f"Gripper: detected action root '{action_root}'")
#             return

#         self.gripper_mode = None
#         self.get_logger().warning("No gripper interface detected; gripper commands will be skipped.")

#     # ----------------------------
#     # Joint positions & tolerance
#     # ----------------------------
#     def current_positions_for(self, names: List[str]) -> Optional[List[float]]:
#         if self.latest_js is None:
#             return None
#         try:
#             return [self.latest_js.position[self.jname_to_index[n]] for n in names]
#         except KeyError:
#             return None

#     def within_tolerance(self, target_positions: List[float], controlled_joints: List[str]) -> bool:
#         cur = self.current_positions_for(controlled_joints)
#         if cur is None:
#             return False
#         return all(abs(c - t) <= EPSILON for c, t in zip(cur, target_positions))

#     # ----------------------------
#     # Arm commands
#     # ----------------------------
#     def build_and_publish_arm_trajectory(self, positions: List[float], time_from_start: float = 1.0):
#         arm_joint_names = [n for n in JOINTS_ARM if n in self.jname_to_index]
#         if not arm_joint_names and self.latest_js:
#             names = [n for n in self.latest_js.name if not n.startswith('gripper')]
#             arm_joint_names = names[-4:]
#         jt = JointTrajectory()
#         jt.joint_names = arm_joint_names
#         p = JointTrajectoryPoint()
#         p.positions = positions[:len(arm_joint_names)]
#         sec = int(math.floor(time_from_start))
#         nsec = int((time_from_start - sec) * 1e9)
#         p.time_from_start.sec = sec
#         p.time_from_start.nanosec = nsec
#         jt.points = [p]
#         self.arm_pub.publish(jt)

#     # ----------------------------
#     # Gripper commands
#     # ----------------------------
#     def send_gripper_command(self, value: float) -> bool:
#         if self.gripper_mode == 'gripper_msg_topic' and self.gripper_pub:
#             msg = GripperCommandMsg()
#             msg.position = float(value)
#             msg.max_effort = 0.0
#             self.gripper_pub.publish(msg)
#             return True

#         if self.gripper_mode == 'float_topic' and self.gripper_pub:
#             from std_msgs.msg import Float64
#             m = Float64()
#             m.data = float(value)
#             self.gripper_pub.publish(m)
#             return True

#         if self.gripper_mode == 'action' and self.gripper_action_name:
#             if GripperCommandAction is None or GripperCommandMsg is None:
#                 return False
#             if self.gripper_action_client is None:
#                 self.gripper_action_client = ActionClient(self, GripperCommandAction, self.gripper_action_name)
#             if not self.gripper_action_client.wait_for_server(timeout_sec=2.0):
#                 return False
#             goal_msg = GripperCommandAction.Goal()
#             goal_msg.command = GripperCommandMsg()
#             goal_msg.command.position = float(value)
#             goal_msg.command.max_effort = 0.0
#             send_future = self.gripper_action_client.send_goal_async(goal_msg)
#             rclpy.spin_until_future_complete(self, send_future, timeout_sec=2.0)
#             goal_handle = send_future.result()
#             if not goal_handle.accepted:
#                 return False
#             get_result_future = goal_handle.get_result_async()
#             rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=PER_STEP_TIMEOUT)
#             return True
#         return False

#     # ----------------------------
#     # Convergence
#     # ----------------------------
#     def wait_for_convergence(self, target_positions: List[float], controlled_joints: List[str], timeout: float) -> Tuple[bool, float, str]:
#         start = monotonic_time()
#         consecutive = 0
#         spin_timeout = 1.0 / max(1.0, CONTROL_RATE)

#         while True:
#             rclpy.spin_once(self, timeout_sec=spin_timeout)
#             elapsed = monotonic_time() - start

#             if self.latest_js is None:
#                 if elapsed > timeout:
#                     return False, elapsed, 'timeout_no_joint_states'
#                 continue

#             if self.within_tolerance(target_positions, controlled_joints):
#                 consecutive += 1
#             else:
#                 consecutive = 0

#             if consecutive >= STABLE_CYCLES:
#                 return True, monotonic_time() - start, 'converged'
#             if elapsed > timeout:
#                 return False, elapsed, 'timeout'

#     # ----------------------------
#     # Recovery
#     # ----------------------------
#     def recover_to_home(self) -> Tuple[bool, float, str]:
#         if 'home' not in POSES:
#             return False, 0.0, 'no_home_pose'
#         tgt = POSES['home']
#         self.build_and_publish_arm_trajectory(tgt, time_from_start=1.0)
#         arm_joints = [j for j in JOINTS_ARM if j in self.jname_to_index]
#         if not arm_joints and self.latest_js:
#             names = [n for n in self.latest_js.name if not n.startswith('gripper')]
#             arm_joints = names[-4:]
#         return self.wait_for_convergence(tgt, arm_joints, RECOVERY_TIMEOUT)

#     # ----------------------------
#     # Main sequence runner
#     # ----------------------------
#     def run_sequence(self):
#         self.get_logger().info("Ready. Running sequence.")
#         # Compute arm joints
#         arm_joints = [j for j in JOINTS_ARM if j in self.jname_to_index]
#         if not arm_joints and self.latest_js:
#             names = [n for n in self.latest_js.name if not n.startswith('gripper')]
#             arm_joints = names[-4:]
#         if not arm_joints:
#             arm_joints = JOINTS_ARM[:]
#         self.get_logger().info(f"Arm-controlled joints: {arm_joints}")

#         # Detect real gripper joint
#         gripper_joint = None
#         for gj in ('gripper_left_joint', 'gripper_right_joint', 'gripper_finger_joint'):
#             if gj in self.jname_to_index:
#                 gripper_joint = gj
#                 break

#         for step_idx, (stype, target) in enumerate(SEQUENCE, 1):
#             step_tag = f"[{step_idx}/{len(SEQUENCE)}]"
#             duration = 0.0
#             ok = False
#             reason = ''

#             if stype == 'pose':
#                 if target not in POSES:
#                     reason = 'unknown_pose'
#                     self.summary.append((step_idx, stype, target, False, reason, 0.0))
#                     continue
#                 tgt = POSES[target]
#                 attempt = 0
#                 while attempt <= MAX_RETRIES:
#                     self.build_and_publish_arm_trajectory(tgt, time_from_start=1.0)
#                     ok, dur, reason = self.wait_for_convergence(tgt, arm_joints, PER_STEP_TIMEOUT)
#                     duration += dur
#                     if ok:
#                         break
#                     attempt += 1
#                 if not ok:
#                     rec_ok, rec_dur, rec_reason = self.recover_to_home()
#                     duration += rec_dur
#                     reason = f"{reason};recovery_{rec_reason}"

#             elif stype == 'grip':
#                 gval = GRIP_OPEN if target == 'open' else GRIP_CLOSED
#                 if gripper_joint:
#                     sent = self.send_gripper_command(gval)
#                     if sent:
#                         ok, dur, reason = self.wait_for_convergence([gval], [gripper_joint], PER_STEP_TIMEOUT)
#                         duration += dur
#                     else:
#                         reason = 'no_gripper_interface'
#                         ok = False
#                         duration = 0.0
#                 else:
#                     sent = self.send_gripper_command(gval)
#                     # fallback: just assume STABLE_CYCLES
#                     spin_timeout = 1.0 / max(1.0, CONTROL_RATE)
#                     cycles = 0
#                     start_wait = monotonic_time()
#                     while cycles < STABLE_CYCLES:
#                         rclpy.spin_once(self, timeout_sec=spin_timeout)
#                         cycles += 1
#                     duration += monotonic_time() - start_wait
#                     ok = True
#                     reason = 'assumed'

#             status = "OK" if ok else "FAIL"
#             self.get_logger().info(f"{step_tag} {stype}={target} {status} duration={duration:.2f}s reason={reason}")
#             self.summary.append((step_idx, stype, target, ok, reason, duration))

#         self.print_summary()

#     # ----------------------------
#     # Summary
#     # ----------------------------
#     def print_summary(self):
#         self.get_logger().info("=== Final summary ===")
#         self.get_logger().info(" idx | type  | target  | OK/FAIL | reason                  | duration(s)")
#         for row in self.summary:
#             idx, typ, target, ok, reason, dur = row
#             flag = "OK" if ok else "FAIL"
#             self.get_logger().info(f" {idx:2d}  | {typ:5s} | {str(target):7s} | {flag:6s} | {reason:22s} | {dur:7.2f}")


# # ----------------------------
# # Main entry
# # ----------------------------
# def main(args=None):
#     rclpy.init(args=args)
#     node = FeedbackPickPlaceNode()
#     try:
#         node.run_sequence()
#     except KeyboardInterrupt:
#         node.get_logger().info("Interrupted by user")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
